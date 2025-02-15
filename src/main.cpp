
#include <cstdio> 
#include "Logger.h"
#include "ConfigManager.h"
#include "RobotStateTracker.h"

int main()
{
	std::string configFilePath = "./src/nora/configuration/config.yaml";
	
	createLogger();
	ConfigManager::getInstance()->loadConfig(configFilePath); 

	RobotStateTracker robot;
}
