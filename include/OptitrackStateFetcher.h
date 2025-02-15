#ifndef OPTITRACKSTATEFETCHER_H
#define OPTITRACKSTATEFETCHER_H

#include "IStateFetcher.h"
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "NatNetCAPI.h"
#include <memory>
//#include "ConfigurationManager.h"
//#include "Configurations.h" 

class OptitrackStateFetcher : public IStateFetcher
{
public:
    OptitrackStateFetcher(Network aConfig);
    ~OptitrackStateFetcher() override; 

    bool init() override; 
    Eigen::Matrix<double, 6, 1> fetchState() override; 

    void setLatestState(Eigen::Matrix<double, 6, 1> aLatestState) {std::lock_guard<std::mutex> lock(mStateMutex); mLatestState = aLatestState; }

private:
    std::unique_ptr<NatNetClient> mNatNetClient;
    Network mConfig; 
    Eigen::Matrix<double, 6, 1> mLatestState; 
    int32_t mID; 

    std::mutex mStateMutex; 
    
    // callback function when frame is recvd
    void frameRecvdCallback(sFrameOfMocapData* data, void* pUserData);

    // helper/debug function to print connection info for optitrack
    void printConnectionInfo(); 
};
#endif //OPTITRACKSTATEFETCHER_H
    