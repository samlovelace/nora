
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
        mConfig.motion_capture_system = yamlConfig["motion_capture_system"].as<std::string>();
        mConfig.update_rate = yamlConfig["update_rate"].as<int>();

        // Parse Network Configuration
        if (mConfig.motion_capture_system == "optitrack") {
            mConfig.optitrack_multicast_address = yamlConfig["network"]["optitrack"]["multicast_address"].as<std::string>();
            mConfig.optitrack_command_port = yamlConfig["network"]["optitrack"]["command_port"].as<int>();
            mConfig.optitrack_data_port = yamlConfig["network"]["optitrack"]["data_port"].as<int>();
        } else if (mConfig.motion_capture_system == "vicon") {
            mConfig.vicon_server_address = yamlConfig["network"]["vicon"]["server_address"].as<std::string>();
            mConfig.vicon_port = yamlConfig["network"]["vicon"]["port"].as<int>();
        }

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
    LOGD << "Motion Capture System: " << mConfig.motion_capture_system;
    LOGD << "Update Rate: " << mConfig.update_rate << " Hz";
    LOGD << "--------------------------------";
    LOGD << "Network Configuration:";
    if (mConfig.motion_capture_system == "optitrack") {
        LOGD << "  OptiTrack:";
        LOGD << "    Multicast Address: " << mConfig.optitrack_multicast_address;
        LOGD << "    Command Port: " << mConfig.optitrack_command_port;
        LOGD << "    Data Port: " << mConfig.optitrack_data_port;
    } else if (mConfig.motion_capture_system == "vicon") {
        LOGD << "  Vicon:";
        LOGD << "    Server Address: " << mConfig.vicon_server_address;
        LOGD << "    Port: " << mConfig.vicon_port;
    }
    LOGD << "================================";
}
