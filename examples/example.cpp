#include <action_server/server.h>
#include "simple_action_factory.h"

#include <set>

int main(int argc, char **argv)
{
    // Create the action server
    act::Server server;

    // Create an action factory, that will be used to create 'simple' actions
    act::ActionFactoryPtr simple_af(new SimpleActionFactory);

    // Register the factory to the server
    server.registerActionFactory(simple_af);

    // Ask the server to create an action of type 'simple' with a specific configuration
    // The server will deduce which action factory is used to generate the action.
    tue::Configuration action_cfg;
    action_cfg.setValue("message", "This is a simple test");
    server.addAction("simple", action_cfg);

    // Loop the server and tick at approx. 10 hz
    while(true)
    {
        server.tick();
        usleep(100000);
    }

    return 0;
}
