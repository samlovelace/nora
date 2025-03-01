
#include "OptitrackStateFetcher.h"
#include "plog/Log.h"

OptitrackStateFetcher::OptitrackStateFetcher(NetworkConfig aConfig) : 
        mNatNetClient(nullptr), mConfig(aConfig), mID(1)
{
    ////// HACK FOR TESTING 
    dummyState();  
}

OptitrackStateFetcher::~OptitrackStateFetcher()
{
    
}

void OptitrackStateFetcher::dummyState()
{
    Eigen::Matrix<double, 13, 1> state;
    for(int j = 0; j < 12; j++)
    {
        state[j] = 0.0;
    }

    LOGD << "state: " << state; 
    setLatestState(state);

}

bool OptitrackStateFetcher::init() 
{
    if(mNatNetClient)
    {
        mNatNetClient->Disconnect(); 
    }

    // Properly re-instantiate the NatNetClient before using it
    mNatNetClient = std::make_unique<NatNetClient>();

    // Bind the member function using a lambda
    mNatNetClient->SetFrameReceivedCallback(
        [](sFrameOfMocapData* data, void* pUserContext) {
            if (pUserContext) {
                // Convert the user context pointer back to an instance of OptitrackStateFetcher
                static_cast<OptitrackStateFetcher*>(pUserContext)->frameRecvdCallback(data, pUserContext);
            }
        },
        this  // Pass the current instance as the user context
    );

    // Setup connection parameters from config
    sNatNetClientConnectParams connectParams;
    connectParams.serverAddress = mConfig.Server.IP.c_str();
    connectParams.localAddress = mConfig.Local.IP.c_str();
    connectParams.multicastAddress = mConfig.Multicast.IP.c_str();
    connectParams.serverCommandPort = (uint16_t)mConfig.Server.CmdPort;
    connectParams.serverDataPort = (uint16_t)mConfig.Server.DataPort;   

    if(ErrorCode_OK != mNatNetClient->Connect(connectParams))
    {
        LOGE << "Failed to connect to OptiTrack";
        return false;  
    }

    // TODO: Move this to a config file
    bool printOptiConnectInfo = true;
    if(printOptiConnectInfo)
    {
        printConnectionInfo();
    }

    return true; 
}

Eigen::Matrix<double, 13, 1> OptitrackStateFetcher::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mLatestState; 
}

// TODO: calculate lin/ang velocity
void OptitrackStateFetcher::frameRecvdCallback(sFrameOfMocapData* data, void* pUserData)
{
    auto dt = std::chrono::steady_clock::now() - mPrevRecvdTime; 

    for(size_t i = 0; i < data->nRigidBodies; i++)
    {
        if(data->RigidBodies[i].ID == mID)
        {
            ////////// TODO: Dummy values for most of these, implement actual calcuation
            Eigen::Matrix<double, 13, 1> state;
            for(int j = 0; j < 12; j++)
            {
                state[j] = 0.0;
            }

            // rigid body data from OptiTrack
            auto rb = data->RigidBodies[i]; 
            
            // position 
            state[0] = rb.x; 
            state[1] = rb.y; 
            state[2] = rb.z; 

            // linear velocity, derivative approximation 
            for(int i = 0; i <2; i++)
            {
                state[i+3] = (state[i] - mPrevState[i]) / dt.count(); 
            }

            // Quaternion
            Eigen::Quaterniond q(rb.qw, rb.qx, rb.qy, rb.qz); 
            q.normalize();

            // quaternion in core state vector
            state[6] = q.w();  
            state[7] = q.x(); 
            state[8] = q.y(); 
            state[9] = q.z(); 

            // angular velocity
            // previous quaternion
            Eigen::Quaterniond prev(mPrevState[6], mPrevState[7], mPrevState[8], mPrevState[9]); 
            Eigen::Quaterniond qDelta = q * prev.inverse();
            Eigen::Vector3d angularVelocity = 2.0 * qDelta.vec() / dt.count(); 

            state[10] = angularVelocity.x(); 
            state[11] = angularVelocity.y(); 
            state[12] = angularVelocity.z(); 

            // set latest state and update previous values
            setLatestState(state); 
            mPrevState = state; 
            mPrevRecvdTime = std::chrono::steady_clock::now(); 
        }
    }
}

void OptitrackStateFetcher::printConnectionInfo()
{
    // Print version info
    unsigned char ver[4];
    NatNet_GetVersion(ver);
    LOGD << "NatNet Sample Client (NatNet ver. " << ver << ")";
    
    sDataDescriptions* pDataDefs = NULL;

    // Retrieve RigidBody description from server
    if (ErrorCode_OK != mNatNetClient->GetDataDescriptionList(&pDataDefs))
    {
        // Unable to retrieve RigidBody description
        LOGE << "Unable to retrieve Data Descriptions";
        return;
    }
    else
    {
        std::stringstream s; 
        s << "[NatNetComm] Received %d Data Descriptions:" << pDataDefs->nDataDescriptions << "\n"; 
        for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
        {
            s.clear(); 
            s << "Data Description # " << i << "type: " << pDataDefs->arrDataDescriptions[i].type << "\n";
            if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                s << "RigidBody Name: " << pRB->szName << "\n";
                s << "RigidBody ID: " << pRB->ID << "\n";
                s << "RigidBody Parent ID: " << pRB->parentID << "\n"; 
                s << "Parent Offset (xyz): " << pRB->offsetx << " " << pRB->offsety << " " << pRB->offsetz << "\n";  
            }

            LOGD << s.str(); 
        }
    }
    NatNet_FreeDescriptions(pDataDefs);
    pDataDefs = NULL;
}

