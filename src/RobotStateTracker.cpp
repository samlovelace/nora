#include "ConfigManager.h"
#include "RobotStateTracker.h"
#include "OptitrackStateFetcher.h"
#include "plog/Log.h"
#include <thread>

RobotStateTracker::RobotStateTracker() : mConfig(ConfigManager::getInstance()->getStateTrackerConfig())
{
    FetcherType typeToMake = toEnum(mConfig.mInterface); 

    switch (typeToMake)
    {
    case FetcherType::OPTITRACK: 
        mStateFetcher = std::make_shared<OptitrackStateFetcher>(mConfig.mNetwork); 
        LOGD << "Configuring NORA to use OptiTrack for state feedback"; 
        break;
    default:
        break;
    } 

    mStateTrackingThread = std::thread(&RobotStateTracker::stateTrackerLoop, this); 
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
void RobotStateTracker::stateTrackerLoop()
{   
    LOGD << "Starting state tracking thread"; 

    const std::chrono::duration<double> loop_duration(1.0 / mConfig.mUpdateRate);
    
    if(!mStateFetcher->init())
    {
        LOGE << "Could not initialize state fetcher of type: " << mConfig.mInterface; 
        return; 
    }

    while(doStateTracking())
    {
        auto start = std::chrono::steady_clock::now(); 

        LOGD << "Fetching robot state..."; 
        mStateFetcher->fetchState(); 

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




