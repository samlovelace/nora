
#include <cstdio> 
#include "Logger.h"
#include "ConfigManager.h"
#include "RobotStateTracker.h"
#include "RosTopicManager.h"
#include "nora_idl/msg/robot_state.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main()
{
    std::signal(SIGINT, signalHandler); 
	//std::string configFilePath = "../../configuration/config.yaml";
	std::string configFilePath = ament_index_cpp::get_package_share_directory("nora") + "/configuration/config.yaml"; 

	createLogger();
	auto configManager = ConfigManager::getInstance(); 
	configManager->loadConfig(configFilePath); 
	
	rclcpp::init(0, nullptr); 
	RosTopicManager::getInstance()->createPublisher<nora_idl::msg::RobotState>(configManager->getConfig().state_topic); 

	RobotStateTracker robot;
	robot.run(); 
	
	rclcpp::shutdown(); 
}
