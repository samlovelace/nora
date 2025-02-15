
#include "ConfigManager.h"
#include <yaml-cpp/yaml.h>
#include "plog/Log.h"

void ConfigManager::loadConfig(const std::string& filename) 
{
    LOGD << R"(
        __    __   _______  __       __        ______          .__   __.   ______   .______          ___      
       |  |  |  | |   ____||  |     |  |      /  __  \         |  \ |  |  /  __  \  |   _  \        /   \     
       |  |__|  | |  |__   |  |     |  |     |  |  |  |        |   \|  | |  |  |  | |  |_)  |      /  ^  \    
       |   __   | |   __|  |  |     |  |     |  |  |  |        |  . `  | |  |  |  | |      /      /  /_\  \   
       |  |  |  | |  |____ |  `----.|  `----.|  `--'  |  __    |  |\   | |  `--'  | |  |\  \----./  _____  \  
       |__|  |__| |_______||_______||_______| \______/  (_ )   |__| \__|  \______/  | _| `._____/__/     \__\ 
                                                         |/                                                   
       )";

    try {
        YAML::Node yamlConfig = YAML::LoadFile(filename);

        // Parse Robot Info
        mConfig.robot_name = yamlConfig["robot"]["name"].as<std::string>();
        mConfig.robot_ip = yamlConfig["robot"]["ip_address"].as<std::string>();
        mConfig.rigid_body_id = yamlConfig["robot"]["rigid_body_id"].as<int>();
        mConfig.mStateTrackerConfig.mNetwork.Local.IP = mConfig.robot_ip; 

        // Parse State Vector
        mConfig.position = yamlConfig["state_vector"]["position"].as<bool>();
        mConfig.velocity = yamlConfig["state_vector"]["velocity"].as<bool>();
        mConfig.acceleration = yamlConfig["state_vector"]["acceleration"].as<bool>();
        mConfig.orientation = yamlConfig["state_vector"]["orientation"].as<bool>();
        mConfig.angular_velocity = yamlConfig["state_vector"]["angular_velocity"].as<bool>();
        mConfig.angular_acceleration = yamlConfig["state_vector"]["angular_acceleration"].as<bool>();

        // Parse ROS Topics
        mConfig.state_topic = yamlConfig["topics"]["state"].as<std::string>();

        // Parse Motion Capture Settings
        mConfig.mStateTrackerConfig.mInterface = yamlConfig["motion_capture_system"].as<std::string>();
        mConfig.mStateTrackerConfig.mUpdateRate = yamlConfig["update_rate"].as<int>();

        // Parse Network Configuration
        if (mConfig.mStateTrackerConfig.mInterface == "optitrack") {
            mConfig.mStateTrackerConfig.mNetwork.Server.IP = yamlConfig["network"]["optitrack"]["server_address"].as<std::string>(); 
            mConfig.mStateTrackerConfig.mNetwork.Multicast.IP = yamlConfig["network"]["optitrack"]["multicast_address"].as<std::string>();
            mConfig.mStateTrackerConfig.mNetwork.Multicast.CmdPort = yamlConfig["network"]["optitrack"]["command_port"].as<int>();
            mConfig.mStateTrackerConfig.mNetwork.Multicast.DataPort = yamlConfig["network"]["optitrack"]["data_port"].as<int>();
        } else if (mConfig.mStateTrackerConfig.mInterface == "vicon") {
            mConfig.mStateTrackerConfig.mNetwork.Server.IP = yamlConfig["network"]["vicon"]["server_address"].as<std::string>();
            mConfig.mStateTrackerConfig.mNetwork.Multicast.DataPort = yamlConfig["network"]["vicon"]["port"].as<int>();
            mConfig.mStateTrackerConfig.mNetwork.Multicast.CmdPort = mConfig.mStateTrackerConfig.mNetwork.Multicast.DataPort; 
        }

        mConfig.mStateTrackerConfig.mNetwork.Server.CmdPort = mConfig.mStateTrackerConfig.mNetwork.Multicast.CmdPort;
        mConfig.mStateTrackerConfig.mNetwork.Server.DataPort = mConfig.mStateTrackerConfig.mNetwork.Multicast.DataPort;

        printConfig();
    } catch (const YAML::Exception& e) {
        LOGE << "[ERROR] Failed to load YAML: " << e.what();
    }
}

void ConfigManager::printConfig() const {
    LOGD << "===== Configuration Loaded =====";
    LOGD << "Robot Name: " << mConfig.robot_name;
    LOGD << "Robot IP: " << mConfig.robot_ip;
    LOGD << "Rigid Body ID: " << mConfig.rigid_body_id;
    LOGD << "--------------------------------";
    LOGD << "State Vector Configuration:";
    LOGD << "  Position: " << (mConfig.position ? "Enabled" : "Disabled");
    LOGD << "  Velocity: " << (mConfig.velocity ? "Enabled" : "Disabled");
    LOGD << "  Acceleration: " << (mConfig.acceleration ? "Enabled" : "Disabled");
    LOGD << "  Orientation: " << (mConfig.orientation ? "Enabled" : "Disabled");
    LOGD << "  Angular Velocity: " << (mConfig.angular_velocity ? "Enabled" : "Disabled");
    LOGD << "  Angular Acceleration: " << (mConfig.angular_acceleration ? "Enabled" : "Disabled");
    LOGD << "--------------------------------";
    LOGD << "ROS Topics:";
    LOGD << "  State Topic: " << mConfig.state_topic;
    LOGD << "--------------------------------";
    LOGD << "Motion Capture System: " << mConfig.mStateTrackerConfig.mInterface;
    LOGD << "Update Rate: " << mConfig.mStateTrackerConfig.mUpdateRate << " Hz";
    LOGD << "--------------------------------";
    LOGD << "Network Configuration:";
    if (mConfig.mStateTrackerConfig.mInterface == "optitrack") {
        LOGD << "  OptiTrack:";
        LOGD << "    Server Address: " << mConfig.mStateTrackerConfig.mNetwork.Server.IP; 
        LOGD << "    Multicast Address: " << mConfig.mStateTrackerConfig.mNetwork.Multicast.IP;
        LOGD << "    Command Port: " << mConfig.mStateTrackerConfig.mNetwork.Multicast.CmdPort;
        LOGD << "    Data Port: " << mConfig.mStateTrackerConfig.mNetwork.Multicast.DataPort;
    } else if (mConfig.mStateTrackerConfig.mInterface == "vicon") {
        LOGD << "  Vicon:";
        LOGD << "    Server Address: " << mConfig.mStateTrackerConfig.mNetwork.Multicast.IP;
        LOGD << "    Port: " << mConfig.mStateTrackerConfig.mNetwork.Multicast.DataPort;
    }
    LOGD << "================================";
}
