#include "amigo_joint_trajectory_client.h"

// ----------------------------------------------------------------------------------------------------

AmigoJointTrajectoryClient::AmigoJointTrajectoryClient() : client_(0)
{
}

// ----------------------------------------------------------------------------------------------------

AmigoJointTrajectoryClient::~AmigoJointTrajectoryClient()
{
    delete client_;
}

// ----------------------------------------------------------------------------------------------------


void AmigoJointTrajectoryClient::initialize(const std::string& side)
{
    joint_names_.push_back("shoulder_yaw_joint_" + side);
    joint_names_.push_back("shoulder_pitch_joint_" + side);
    joint_names_.push_back("shoulder_roll_joint_" + side);
    joint_names_.push_back("elbow_pitch_joint_" + side);
    joint_names_.push_back("elbow_roll_joint_" + side);
    joint_names_.push_back("wrist_pitch_joint_" + side);
    joint_names_.push_back("wrist_yaw_joint_" + side);

    std::string server_name = "/amigo/" + side + "_arm/joint_trajectory_action";

    client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(server_name, true);
    while(ros::ok())
    {
        if (client_->waitForServer(ros::Duration(1.0)))
            break;
        else
            std::cout << "Waiting for actionlib server '" << server_name << "'." << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

void AmigoJointTrajectoryClient::execute(const JointTrajectory& trajectory)
{
    control_msgs::FollowJointTrajectoryGoal goal = trajectory.message();
    goal.trajectory.joint_names = joint_names_;

    client_->sendGoal(goal);
    client_->waitForResult();
}

