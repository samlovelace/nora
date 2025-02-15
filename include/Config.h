#ifndef CONFIG_H
#define CONFIG_H

#include <string>

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

    std::string motion_capture_system;
    int update_rate;

    std::string optitrack_multicast_address;
    int optitrack_command_port;
    int optitrack_data_port;

    std::string vicon_server_address;
    int vicon_port;
};

#endif  // CONFIG_H
