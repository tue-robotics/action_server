#include "joint_trajectory.h"

// ----------------------------------------------------------------------------------------------------

JointTrajectory::JointTrajectory()
{
}

// ----------------------------------------------------------------------------------------------------

JointTrajectory::~JointTrajectory()
{
}

// ----------------------------------------------------------------------------------------------------

void JointTrajectory::addPoint(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
{
    msg_.trajectory.points.push_back(trajectory_msgs::JointTrajectoryPoint());

    trajectory_msgs::JointTrajectoryPoint& p = msg_.trajectory.points.back();
    p.positions.resize(7);
    p.positions[0] = q1;
    p.positions[1] = q2;
    p.positions[2] = q3;
    p.positions[3] = q4;
    p.positions[4] = q5;
    p.positions[5] = q6;
    p.positions[6] = q7;
}
