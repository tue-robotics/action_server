#include <action_server/server.h>

// ROS communication
#include <ros/ros.h>
#include <action_server/AddAction.h>
#include <action_server/GetActionStatus.h>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>

// Modules
#include "navigate_to.h"

act::Server server;

// ----------------------------------------------------------------------------------------------------

bool srvAddAction(action_server::AddAction::Request& req, action_server::AddAction::Response& res)
{
    tue::Configuration action_cfg;
    tue::config::loadFromYAMLString(req.parameters, action_cfg);

    if (!action_cfg.hasError())
    {
        // Check for 'special' fields.

        std::string entity_id;
        if (action_cfg.value("object", entity_id, tue::OPTIONAL))
        {
            // TODO: ask ED for entity information regarding given action, and add this to action_cfg
        }

        act::ActionConstPtr action = server.addAction(req.action, action_cfg);
        if (action)
        {
            res.action_uuid = action->id().string();
        }
    }
    else
    {
        res.error_msg = action_cfg.error();
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvGetActionStatus(action_server::GetActionStatus::Request& req, action_server::GetActionStatus::Response& res)
{
    return true;
}


// ----------------------------------------------------------------------------------------------------


int main(int argc, char **argv)
{
    ros::init(argc, argv, "constraint_server");

    act::ActionFactoryPtr navigate_to(new NavigateTo);

    // Register component
    server.registerActionFactory(navigate_to);

    ros::NodeHandle nh;
    ros::ServiceServer srv_add_action = nh.advertiseService("/action_server/add_action", srvAddAction);
    ros::ServiceServer srv_get_action_status = nh.advertiseService("/action_server/get_action_status", srvGetActionStatus);

    ros::Rate r(20);
    while (ros::ok())
    {
        ros::spinOnce();
        server.tick();
        r.sleep();
    }

    return 0;
}
