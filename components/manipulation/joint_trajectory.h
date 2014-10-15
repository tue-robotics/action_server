#ifndef ACTION_SERVER_JOINT_TRAJECTORY_H_
#define ACTION_SERVER_JOINT_TRAJECTORY_H_

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ----------------------------------------------------------------------------------------------------

class JointTrajectory
{

public:

    JointTrajectory();

    virtual ~JointTrajectory();

    void addPoint(double q1, double q2, double q3, double q4, double q5, double q6, double q7);

    const control_msgs::FollowJointTrajectoryGoal& message() const { return msg_; }

private:

    control_msgs::FollowJointTrajectoryGoal msg_;

};

#endif
