#include "navigate_to.h"

#include <cb_planner_msgs_srvs/GetPlan.h>
#include <cb_planner_msgs_srvs/OrientationConstraint.h>
#include <cb_planner_msgs_srvs/LocalPlannerActionGoal.h>

// ----------------------------------------------------------------------------------------------------

void NavigationAction::tick()
{
//    std::cout << "NavigationAction::tick " << id() << std::endl;
}

// ----------------------------------------------------------------------------------------------------

NavigateTo::NavigateTo()
{
    registerActionType("navigate-to");

    ros::NodeHandle nh;
    client_global_plan_ = nh.serviceClient<cb_planner_msgs_srvs::GetPlan>("/cb_base_navigation/global_planner_interface/get_plan_srv");
    pub_global_plan_ = nh.advertise<cb_planner_msgs_srvs::LocalPlannerActionGoal>("/cb_base_navigation/local_planner_interface/action_server/goal", 1);
}

// ----------------------------------------------------------------------------------------------------

act::ActionPtr NavigateTo::createAction(const std::string& type, tue::Configuration config)
{
    cb_planner_msgs_srvs::PositionConstraint pc;
    if (config.readGroup("position_constraint", tue::REQUIRED))
    {
        config.value("constraint", pc.constraint);
//        config.value("frame", pc.frame);

        config.endGroup();
    }
    config.value("entity", pc.frame);

    cb_planner_msgs_srvs::OrientationConstraint oc;
    if (config.readGroup("orientation_constraint", tue::OPTIONAL))
    {
        config.value("angle_offset", oc.angle_offset, tue::OPTIONAL);
//        config.value("frame", oc.frame);
        config.endGroup();
    }
    config.value("entity", oc.frame);

    if (config.hasError())
    {
        std::cout << "ERROR: " << config.error() << std::endl;
        return act::ActionPtr();
    }

    std::cout << pc << std::endl;
    std::cout << oc << std::endl;

    // Call for local plan
    cb_planner_msgs_srvs::GetPlan srv;
    srv.request.goal_position_constraints.push_back(pc);
    if (client_global_plan_.call(srv))
    {
        std::cout << "[navigate-to] Found global plan." << std::endl;
//        std::cout << srv.response << std::endl;

        cb_planner_msgs_srvs::LocalPlannerActionGoal msg;
        msg.goal.plan = srv.response.plan;
        msg.goal.orientation_constraint = oc;
        pub_global_plan_.publish(msg);
    }
    else
    {
        std::cout << "Global plan service could not be called" << std::endl;
        return act::ActionPtr();
    }

    return act::ActionPtr(new NavigationAction);
}
