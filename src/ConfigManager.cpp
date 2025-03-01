
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

        std::stringstream s;
        s << "Configuration: \n"; 
        s << YAML::Dump(yamlConfig);
        LOGD << s.str();  

        // Parse Robot Info
        mConfig.robot_name = yamlConfig["robot"]["name"].as<std::string>();
        mConfig.robot_ip = yamlConfig["robot"]["ip_address"].as<std::string>();
        mConfig.rigid_body_id = yamlConfig["robot"]["rigid_body_id"].as<int>();
        mConfig.mStateTrackerConfig.mNetwork.Local.IP = mConfig.robot_ip; 

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

    } catch (const YAML::Exception& e) {
        LOGE << "[ERROR] Failed to load YAML: " << e.what();
    }
}
