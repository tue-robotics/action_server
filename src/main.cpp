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

// Mapping from action types (e.g. 'pick-up', 'navigate-to') to a client that serves it
std::map<std::string, ros::ServiceClient*> action_type_to_client_;

// List of additional action servers (chained to this action server)
std::map<std::string, ros::ServiceClient*> action_server_clients;

// ----------------------------------------------------------------------------------------------------

bool srvAddAction(action_server::AddAction::Request& req, action_server::AddAction::Response& res)
{
    tue::Configuration action_cfg;
    tue::config::loadFromYAMLString(req.parameters, action_cfg);

    std::string entity_type;

    if (!action_cfg.hasError())
    {
        // Check for 'special' fields.

//        std::string entity_id;
//        if (action_cfg.value("entity", entity_id, tue::OPTIONAL))
//        {
//            // TODO: ask ED for entity information regarding given action, and add this to action_cfg
//            ed::SimpleQuery srv;
//            srv.request.id = entity_id;

//            if (client_ed.call(srv))
//            {
//                if (srv.response.entities.empty())
//                {
//                    res.error_msg = "No such entity: '" + entity_id + "'";
//                    ROS_ERROR_STREAM(res.error_msg);
//                    return true;
//                }


//                const ed::EntityInfo& e_info = srv.response.entities.front();

//                entity_type = e_info.type;

//                // Load entity configuration
//                tue::config::ReaderWriter r;
//                tue::config::loadFromYAMLString(e_info.data, r);

//                bool affordance_found = false;
//                if (r.readGroup("affordances"))
//                {
//                    if (r.readGroup(req.action))
//                    {
//                        action_cfg.data().add(r.data());
//                        affordance_found = true;
//                        r.endGroup();
//                    }
//                    r.endGroup();
//                }

//                if (!affordance_found)
//                {
//                    if (req.action == "navigate-to")
//                    {
//                        // HACK: add navigation constraint for easy picking up. TODO: make nice
//                        action_cfg.writeGroup("position_constraint");
//                        action_cfg.setValue("constraint", "x^2 + y^2 < 0.59^2 and x^2 + y^2 > 0.30");
//                        action_cfg.endGroup();

//                        action_cfg.writeGroup("orientation_constraint");
//                        action_cfg.setValue("angle_offset", 0.3805063771123649); // Default for right arm
//                        action_cfg.endGroup();
//                    }
//                    else if (req.action == "pick-up")
//                    {
//                    }
//                    else
//                    {
//                        res.error_msg += "No affordance '" + req.action + "' for entity type '" + e_info.type + "'.\n";
//                    }
//                }
//            }
//            else
//            {
//                res.error_msg += "Could not call /ed/simple_query\n";
//            }
//        }

        if (!res.error_msg.empty())
        {
            ROS_ERROR_STREAM(res.error_msg);
            return true;
        }

        act::ActionConstPtr action = server.addAction(req.action, action_cfg);
        if (action_cfg.hasError())
        {
            res.error_msg = action_cfg.error();
        }
        else if (action)
        {
            res.action_uuid = action->id().string();
        }
        else
        {
            action_server::AddAction srv;
            srv.request = req;
//            srv.request.parameters = //action_cfg.toYAMLString();
//            srv.request.parameters += "\nentity_type: " + entity_type; // DIRTY HACK, PLEASE FIX (Sjoerd)! (TODO)

            std::map<std::string, ros::ServiceClient*>::iterator it_client = action_type_to_client_.find(req.action);
            if (it_client != action_type_to_client_.end())
            {
                ros::ServiceClient* client = it_client->second;

                if (client->call(srv))
                {
                    if (!srv.response.error_msg.empty())
                        res.error_msg += srv.response.error_msg + "\n";
                    else
                        res.action_uuid = srv.response.action_uuid;
                }
                else
                {
                    res.error_msg += "Failed to call '" + client->getService() + "'\n";
                }
            }
            else
            {
                // None of the registered action servers could handle the action
                res.error_msg = "No registered servers that serve action of type '" + req.action +"'.\n";
            }
        }
    }
    else
    {
        res.error_msg = action_cfg.error();
    }

    if (!res.error_msg.empty())
        ROS_ERROR_STREAM(res.error_msg);

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

    ros::ServiceClient* client;
    std::map<std::string, ros::ServiceClient*>::iterator it_client = action_server_clients.find(req.add_action_service);

    if (it_client == action_server_clients.end())
    {
        client = new ros::ServiceClient(nh.serviceClient<action_server::AddAction>(req.add_action_service));
        action_server_clients[req.add_action_service] = client;
    }
    else
    {
        client = it_client->second;
    }

    for(std::vector<std::string>::const_iterator it = req.action_types.begin(); it != req.action_types.end(); ++it)
        action_type_to_client_[*it] = client;

    return true;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        ROS_ERROR("Please specify which robot to use");
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
