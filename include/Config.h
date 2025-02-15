#ifndef CONFIG_H
#define CONFIG_H

#include <string>

struct SocketConfig {
    std::string IP;
    int CmdPort;
    int DataPort;
};

struct NetworkConfig {
    SocketConfig Server;
    SocketConfig Local;
    SocketConfig Multicast;
};

struct StateTrackerConfig {
    std::string mInterface;
    int mUpdateRate;
    NetworkConfig mNetwork;   
};

struct Config {
    std::string robot_name;
    std::string robot_ip;
    int rigid_body_id;

    bool position;
    bool velocity;
    bool acceleration;
    bool orientation;
    bool angular_velocity;
    bool angular_acceleration;

    std::string state_topic;

    StateTrackerConfig mStateTrackerConfig; 

};

#endif  // CONFIG_H
