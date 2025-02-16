
#include <cstdio> 
#include "Logger.h"
#include "ConfigManager.h"
#include "RobotStateTracker.h"
#include "RosTopicManager.h"
#include "nora_idl/msg/robot_state.hpp"

// Signal handler function
void signalHandler(int signal) {

    LOGD  << "\n" << "\t\t"
	  R"(_________________________
		|                       |
		|   SHUTTING DOWN...    |
		|_______________________|
               __   /
              / o) /
     _.----._/ /
    /         /
 __/ (  | (  |
/__.-'|_|--|_|
)";
    exit(0); // Exit the program
}
int main()
{
    std::signal(SIGINT, signalHandler); 
	std::string configFilePath = "./src/nora/configuration/config.yaml";
	
	createLogger();
	auto configManager = ConfigManager::getInstance(); 
	configManager->loadConfig(configFilePath); 
	
	rclcpp::init(0, nullptr); 
	RosTopicManager::getInstance()->createPublisher<nora_idl::msg::RobotState>(configManager->getConfig().state_topic); 

	RobotStateTracker robot;
	robot.run(); 
}
