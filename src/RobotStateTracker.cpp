#include "ConfigManager.h"
#include "RobotStateTracker.h"
#include "OptitrackStateFetcher.h"
#include "plog/Log.h"
#include <thread>
#include <chrono>
#include "RosTopicManager.h"
#include "builtin_interfaces/msg/time.hpp"

RobotStateTracker::RobotStateTracker() : mConfig(ConfigManager::getInstance()->getConfig()), mDoStateTracking(true)
{
    FetcherType typeToMake = toEnum(mConfig.mStateTrackerConfig.mInterface); 

    switch (typeToMake)
    {
    case FetcherType::OPTITRACK: 
        mStateFetcher = std::make_shared<OptitrackStateFetcher>(mConfig.mStateTrackerConfig.mNetwork); 
        LOGD << "Configuring NORA to use OptiTrack for state feedback"; 
        break;
    default:
        break;
    } 

}

RobotStateTracker::~RobotStateTracker()
{
    setStateTracking(false); 

    if(mStateTrackingThread.joinable())
    {
        LOGD << "Joining state tracking thread"; 
        mStateTrackingThread.join(); 
    }

}

RobotStateTracker::FetcherType RobotStateTracker::toEnum(std::string aTrackerType)
{
    RobotStateTracker::FetcherType enumToReturn;
    
    // convert all letters to lowercase
    std::transform(aTrackerType.begin(), aTrackerType.end(), aTrackerType.begin(), 
                   [](unsigned char c){ return std::tolower(c); });

    if ("optitrack" == aTrackerType)
    {
        enumToReturn = RobotStateTracker::FetcherType::OPTITRACK; 
    }
    else
    {
        LOGE << "Unsupported state fetcher type: " << aTrackerType; 
    }

    return enumToReturn; 
}

// TODO: figure out what behavior we want for this. Constantly state tracking or able to stop and restart?
void RobotStateTracker::run()
{   
    LOGD << "Starting state tracking thread"; 

    auto statePublisher = std::dynamic_pointer_cast<rclcpp::Publisher<nora_idl::msg::RobotState>>(
                                                    RosTopicManager::getInstance()->getPublisher(mConfig.state_topic));
    
    const std::chrono::duration<double> loop_duration(1.0 / mConfig.mStateTrackerConfig.mUpdateRate);
    
    if(!mStateFetcher->init())
    {
        LOGE << "Could not initialize state fetcher of type: " << mConfig.mStateTrackerConfig.mInterface; 
        //return; 
    }

    while(doStateTracking())
    {
        auto start = std::chrono::steady_clock::now(); 

        LOGD << "Fetching robot state..."; 
        Eigen::Matrix<double, 13,1> state = mStateFetcher->fetchState();   

        statePublisher->publish(toIDL(state)); 

        // Calculate the time taken for the loop iteration
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = loop_end - start;

        // Sleep for the remaining time to maintain the frequency
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        } else {
            LOGE << "Loop overrun! Elapsed time: " 
                      << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
                      << " ms\n";
        }
    }
}

nora_idl::msg::RobotState RobotStateTracker::toIDL(Eigen::Matrix<double, 13, 1> aState)
{
    nora_idl::msg::Vec3 pos;
    nora_idl::msg::Vec3 vel;
    nora_idl::msg::Vec3 ang_vel;

    nora_idl::msg::Quaternion quat; 
    nora_idl::msg::Euler eul; 

    // position
    pos.x = aState[0]; 
    pos.y = aState[1]; 
    pos.z = aState[2]; 

    // velocity
    vel.x = aState[3]; 
    vel.y = aState[4]; 
    vel.z = aState[5];
    
    quat.w = aState[6]; 
    quat.x = aState[7]; 
    quat.y = aState[8]; 
    quat.z = aState[9];

    //TODO: convert EulerToQuat 
    eul.roll = 46.0; 
    eul.pitch = 69.0; 
    eul.yaw = 42.0;

    //angular_velocity
    ang_vel.x = aState[10]; 
    ang_vel.y = aState[11]; 
    ang_vel.z = aState[12];

    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();

    builtin_interfaces::msg::Time timestamp;
    timestamp.sec = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    timestamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1'000'000'000;

    nora_idl::msg::RobotState state; 
    state.set__timestamp(timestamp); 
    state.set__position(pos); 
    state.set__velocity(vel);  
    state.set__euler(eul); 
    state.set__quat(quat);  
    state.set__angular_velocity(ang_vel); 

    return state; 
}




