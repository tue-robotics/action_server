#include <action_server/server.h>

// ROS communication
#include <ros/ros.h>
#include <action_server/AddAction.h>
#include <action_server/GetActionStatus.h>
#include <action_server/RegisterActionServer.h>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include <tue/config/reader.h>
#include <ed/SimpleQuery.h>

// Modules
#include "navigate_to.h"
#include "pick_up.h"

//#include <ed/models/models.h>

act::Server server;
ros::ServiceClient client_ed;

// List of additional action servers (chained to this action server)
std::map<std::string, ros::ServiceClient> action_server_clients;

// ----------------------------------------------------------------------------------------------------

bool srvAddAction(action_server::AddAction::Request& req, action_server::AddAction::Response& res)
{
    tue::Configuration action_cfg;
    tue::config::loadFromYAMLString(req.parameters, action_cfg);

    std::string entity_type;

    if (!action_cfg.hasError())
    {
        // Check for 'special' fields.

        std::string entity_id;
        if (action_cfg.value("entity", entity_id, tue::OPTIONAL))
        {
            // TODO: ask ED for entity information regarding given action, and add this to action_cfg
            ed::SimpleQuery srv;
            srv.request.id = entity_id;

            if (client_ed.call(srv))
            {
                if (srv.response.entities.empty())
                {
                    res.error_msg = "No such entity: '" + entity_id + "'";
                    std::cout << res.error_msg << std::endl;
                    return true;
                }


                const ed::EntityInfo& e_info = srv.response.entities.front();

                entity_type = e_info.type;

                // Load entity configuration
                tue::config::ReaderWriter r;
                tue::config::loadFromYAMLString(e_info.data, r);

                bool affordance_found = false;
                if (r.readGroup("affordances"))
                {
                    if (r.readGroup(req.action))
                    {
                        action_cfg.data().add(r.data());
                        affordance_found = true;
                        r.endGroup();
                    }
                    r.endGroup();
                }

                if (!affordance_found)
                {
                    if (req.action == "navigate-to")
                    {
                        // HACK: add navigation constraint for easy picking up. TODO: make nice
                        action_cfg.writeGroup("position_constraint");
                        action_cfg.setValue("constraint", "x^2 + y^2 < 0.59^2 and x^2 + y^2 > 0.30");
                        action_cfg.endGroup();

                        action_cfg.writeGroup("orientation_constraint");
                        action_cfg.setValue("angle_offset", 0.3805063771123649); // Default for right arm
                        action_cfg.endGroup();
                    }
                    else if (req.action == "pick-up")
                    {
                    }
                    else
                    {
                        res.error_msg = "No affordance '" + req.action + "' for entity type '" + e_info.type + "'.";
                    }
                }
            }
            else
            {
                res.error_msg = "Could not call /ed/simple_query";
            }
        }

        if (!res.error_msg.empty())
        {
            std::cout << res.error_msg << std::endl;
            return true;
        }

        std::cout << req.action << std::endl;
        std::cout << action_cfg << std::endl;

        act::ActionConstPtr action = server.addAction(req.action, action_cfg);
        if (action_cfg.hasError())
        {
            res.error_msg = action_cfg.error();
            std::cout << res.error_msg << std::endl;
        }
        else if (action)
        {
            res.action_uuid = action->id().string();
        }
        else
        {
            std::cout << "Action type '" + req.action + "' is unknown, but I'm going to try other registered action servers" << std::endl;

            action_server::AddAction srv;
            srv.request = req;
            srv.request.parameters = action_cfg.toYAMLString();
            srv.request.parameters += "\nentity_type: " + entity_type; // DIRTY HACK, PLEASE FIX (Sjoerd)! (TODO)
            bool succeeded = false;
            for(std::map<std::string, ros::ServiceClient>::iterator it = action_server_clients.begin(); it != action_server_clients.end(); ++it)
            {
                if (it->second.call(srv))
                {
                    if (srv.response.error_msg.empty())
                    {
                        // Succeeded!
                        succeeded = true;
                        break;
                    }
                }
                else
                {
                    std::cout << "Failed to call '" << it->second.getService() << "'." << std::endl;
                }
            }

            if (!succeeded)
            {
                // None of the registered action servers could handle the action
                res.error_msg = "Action of type '" + req.action +"' could not be created.";
                std::cout << res.error_msg << std::endl;
            }

        }
    }
    else
    {
        res.error_msg = action_cfg.error();
        std::cout << res.error_msg << std::endl;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvGetActionStatus(action_server::GetActionStatus::Request& req, action_server::GetActionStatus::Response& res)
{
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvRegisterActionServer(action_server::RegisterActionServer::Request& req, action_server::RegisterActionServer::Response& res)
{
    ros::NodeHandle nh;
    action_server_clients[req.add_action_service] = nh.serviceClient<action_server::AddAction>(req.add_action_service);
    return true;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        std::cout << "Please specify which robot to use" << std::endl;
        return 1;
    }

    std::string robot_name = argv[1];

    ros::init(argc, argv, "action_server");

    // Create components
    //    act::ActionFactoryPtr navigate_to(new NavigateTo);
    //    act::ActionFactoryPtr pick_up(new PickUp);

    // Register components
    //    server.registerActionFactory(navigate_to);
    //    server.registerActionFactory(pick_up);

    ros::NodeHandle nh_private("~");
    ros::ServiceServer srv_add_action = nh_private.advertiseService("add_action", srvAddAction);
    ros::ServiceServer srv_get_action_status = nh_private.advertiseService("get_action_status", srvGetActionStatus);
    ros::ServiceServer srv_register_action_server = nh_private.advertiseService("register_action_server", srvRegisterActionServer);

    ros::NodeHandle nh;
    client_ed = nh.serviceClient<ed::SimpleQuery>("ed/simple_query");

    ros::Rate r(20);
    while (ros::ok())
    {
        ros::spinOnce();
        server.tick();
        r.sleep();
    }

    return 0;
}
