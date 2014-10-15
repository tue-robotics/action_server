#ifndef ACTION_SERVER_NAVIGATE_TO_H_
#define ACTION_SERVER_NAVIGATE_TO_H_

#include <action_server/action_factory.h>
#include <action_server/action.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>

class NavigationAction : public act::Action
{

    void tick();

};

class NavigateTo : public act::ActionFactory
{

public:

    NavigateTo();

    act::ActionPtr createAction(const std::string& type, tue::Configuration config);

private:

    ros::Publisher pub_global_plan_;
    ros::ServiceClient client_global_plan_;

};

#endif
