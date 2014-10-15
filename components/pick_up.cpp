#include "pick_up.h"

// ----------------------------------------------------------------------------------------------------

void PickUpAction::tick()
{
//    std::cout << "NavigationAction::tick " << id() << std::endl;
}

// ----------------------------------------------------------------------------------------------------

PickUp::PickUp()
{
    registerActionType("pick-up");

    joint_client_left_.initialize("left");
    joint_client_right_.initialize("right");
}

// ----------------------------------------------------------------------------------------------------

act::ActionPtr PickUp::createAction(const std::string& type, tue::Configuration config)
{
    act::ActionPtr action;

    std::string side;
    if (!config.value("side", side))
        return action;

    AmigoJointTrajectoryClient* client;
    if (side == "left")
        client = &joint_client_left_;
    else if (side == "right")
        client = &joint_client_right_;
    else
    {
        config.addError("Unknown arm side: '" + side + "'.");
        return action;
    }

    // Prepare grasp safe
    JointTrajectory t;
    t.addPoint(-0.1, -0.6, 0.1, 1.2, 0.0, 0.1, 0.0);
    t.addPoint(-0.1, -0.8, 0.1, 1.6, 0.0, 0.2, 0.0);
    t.addPoint(-0.1, -1.0, 0.1, 2.0, 0.0, 0.3, 0.0);
    t.addPoint(-0.1, -0.5, 0.1, 2.0, 0.0, 0.3, 0.0);
    client->execute(t);

    // SetGripper(open)

    // PRE_GRASP: ArmToQueryPoint (to grab point, with pre-grasp, first joint pos only)

    // GRASP: Grab (to grab point)

    // SetGripper(close)

    // LIFT: ArmToUserPose (cartesian: 0.0, 0.0, 0.1, 0.0, 0.0, 0.0)

    // RETRACT: ArmToUserPose (cartesian: -0.1, 0.0, 0.0, 0.0, 0.0, 0.0)

    // Carrying_pose

    // SetGripper(close) (if picking up failed)


    action.reset(new PickUpAction);

    return action;
}
