#ifndef ISTATEFETCHER_H
#define ISTATEFETCHER_H

#include <eigen3/Eigen/Dense>
#include <mutex>

class IStateFetcher
{
public:
    virtual ~IStateFetcher() = default; 
    // [x y z vx vy vz qx qy qz qw wx wy wz]
    virtual Eigen::Matrix<double, 13, 1> fetchState() = 0; 
    virtual bool init() = 0; 

protected: 
    Eigen::Matrix<double, 6, 1> mState; 

};
#endif // ISTATEFETCHER_H