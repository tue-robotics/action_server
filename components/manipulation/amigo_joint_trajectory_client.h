#ifndef ACTION_SERVER_AMIGO_JOINT_TRAJECTORY_CLIENT_H_
#define ACTION_SERVER_AMIGO_JOINT_TRAJECTORY_CLIENT_H_

#include "joint_trajectory.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class AmigoJointTrajectoryClient
{

public:

    AmigoJointTrajectoryClient();

    virtual ~AmigoJointTrajectoryClient();

    void initialize(const std::string& side);

    void execute(const JointTrajectory& trajectory);

private:

    std::vector<std::string> joint_names_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* client_;

};

#endif
