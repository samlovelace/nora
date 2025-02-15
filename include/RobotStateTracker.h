#ifndef ROBOTSTATETRACKER_H
#define ROBOTSTATETRACKER_H

#include "IStateFetcher.h"
//#include "Configurations.h"
//#include "ConfigurationManager.h"
#include <memory>
#include <thread> 

class RobotStateTracker
{
public:
    RobotStateTracker(/* args */);
    ~RobotStateTracker();

    enum class FetcherType
    { 
        OPTITRACK, 
        VICON, 
        NUM_TYPES
    };

    void stateTrackerLoop(); 

    Eigen::Vector3d getCurrentPose() { std::lock_guard<std::mutex> lock(mCurrentPoseMutex); return mCurrentPose; }
    Eigen::Vector3d getCurrentVelocity() {std::lock_guard<std::mutex> lock(mCurrentVelocityMutex); return mCurrentVelocity; }

    Eigen::Matrix<double, 6, 1> getCurrentState() { std::scoped_lock lock(mCurrentStateMutex); return mCurrentState; }

    bool doStateTracking() {std::lock_guard<std::mutex> lock(mStateTrackingMutex); return mDoStateTracking; }
    void setStateTracking(bool aFlag) {std::lock_guard<std::mutex> lock(mStateTrackingMutex); mDoStateTracking = aFlag; }

private:
    // polymorphic state fetcher interface so we arent tied to optiTrack
    std::shared_ptr<IStateFetcher> mStateFetcher; // state fetcher interface class 
    StateTrackerConfig mConfig; 

    bool mDoStateTracking;

    Eigen::Vector3d mCurrentPose;
    Eigen::Vector3d mCurrentVelocity;
    Eigen::VectorXd mCurrentState;

    std::mutex mCurrentStateMutex;
    std::mutex mCurrentPoseMutex;
    std::mutex mCurrentVelocityMutex;
    std::mutex mStateTrackingMutex; 

    std::thread mStateTrackingThread;  

    RobotStateTracker::FetcherType toEnum(std::string aTrackerType);
    void setCurrentState(const Eigen::VectorXd& aState) { std::scoped_lock lock(mCurrentStateMutex); mCurrentState = aState; }

};
#endif // RobotStateTracker_H