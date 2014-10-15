#ifndef ACTION_SERVER_PICK_UP_H_
#define ACTION_SERVER_PICK_UP_H_

#include <action_server/action_factory.h>
#include <action_server/action.h>

#include "manipulation/amigo_joint_trajectory_client.h"

class PickUpAction : public act::Action
{

    void tick();

};

class PickUp : public act::ActionFactory
{

public:

    PickUp();

    act::ActionPtr createAction(const std::string& type, tue::Configuration config);

private:

    AmigoJointTrajectoryClient joint_client_left_;
    AmigoJointTrajectoryClient joint_client_right_;

};

#endif
