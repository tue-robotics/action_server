#include "action_server/server.h"

#include "action_server/action.h"
#include "action_server/action_factory.h"

namespace act
{

// ----------------------------------------------------------------------------------------------------

Server::Server()
{
}

// ----------------------------------------------------------------------------------------------------

Server::~Server()
{
}

// ----------------------------------------------------------------------------------------------------

void Server::run()
{

}

// ----------------------------------------------------------------------------------------------------

void Server::tick()
{
    for(std::map<UUID, ActionPtr>::iterator it = actions_.begin(); it != actions_.end(); ++it)
    {
        it->second->tick();
    }
}

// ----------------------------------------------------------------------------------------------------

void Server::registerActionFactory(const ActionFactoryPtr& action_factory)
{
    const std::vector<std::string>& action_types = action_factory->supportedActionTypes();
    for(std::vector<std::string>::const_iterator it = action_types.begin(); it != action_types.end(); ++it)
    {
        action_type_to_factory_[*it] = action_factory;
    }

}

// ----------------------------------------------------------------------------------------------------

ActionConstPtr Server::addAction(const std::string& type, tue::Configuration config)
{
    ActionPtr action;

    std::map<std::string, ActionFactoryPtr>::const_iterator it = action_type_to_factory_.find(type);
    if (it != action_type_to_factory_.end())
    {
        ActionFactoryPtr af = it->second;
        action = af->createAction(type, config);

        if (config.hasError())
        {
            std::cout << "act::Server Error while creating an action of type '" << type << "':" << std::endl;
            std::cout << config.error() << std::endl;
        }
        else if (action)
        {
            action->initialize(config);
            actions_[action->id()] = action;
        }
        else
        {
            std::cout << "act::Server ERROR: action of type '" << type << "' could not be created." << std::endl;
        }
    }
    else
    {
        std::cout << "Unknown action type: " << type << std::endl;
    }

    return action;
}

// ----------------------------------------------------------------------------------------------------

void Server::stopAction(const UUID& id)
{

}

}
