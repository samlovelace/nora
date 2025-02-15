#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <yaml-cpp/yaml.h>
#include "Config.h"
#include "plog/Log.h"

class ConfigManager {
public:
    static ConfigManager* getInstance() {
        static ConfigManager instance;
        return &instance;
    }

    void loadConfig(const std::string& filename);
    void printConfig() const;
    const Config& getConfig() const { return mConfig; }
    const StateTrackerConfig& getStateTrackerConfig() const {return mConfig.mStateTrackerConfig;}

private:
    ConfigManager() = default;
    Config mConfig;  // Use mConfig as the member variable

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

};

#endif  // CONFIG_MANAGER_H
