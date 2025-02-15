
#include "OptitrackStateFetcher.h"
#include "plog/Log.h"

OptitrackStateFetcher::OptitrackStateFetcher(NetworkConfig aConfig) : 
        mNatNetClient(nullptr), mConfig(aConfig), mID(1)
{

}

OptitrackStateFetcher::~OptitrackStateFetcher()
{
    
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

Eigen::Matrix<double, 6, 1> OptitrackStateFetcher::fetchState()
{
    std::lock_guard<std::mutex> lock(mStateMutex); 
    return mLatestState; 
}


void OptitrackStateFetcher::frameRecvdCallback(sFrameOfMocapData* data, void* pUserData)
{
    //TODO: put the robot's optitrack ID in a config file
    for(size_t i = 0; i < data->nRigidBodies; i++)
    {
        if(data->RigidBodies[i].ID == mID)
        {
            auto rb = data->RigidBodies[i]; 
            
            // take the xyz
            Eigen::Matrix<double, 6, 1> state; 
            state[0] = rb.x; 
            state[1] = rb.y; 
            state[2] = rb.z; 

            // convert the quaternion to Euler angles
            Eigen::Quaternion q(rb.qw, rb.qx, rb.qy, rb.qz); 
            q.normalize(); 

            Eigen::Matrix3f rotationMatrix = q.toRotationMatrix(); 
            Eigen::Vector3f angles = rotationMatrix.eulerAngles(2, 1, 0); 

            state[3] = (double)angles[0]; 
            state[4] = (double)angles[1]; 
            state[5] = (double)angles[2]; 

            setLatestState(state); 
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

