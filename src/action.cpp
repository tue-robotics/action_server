#include "action_server/action.h"

#include "action_server/server.h"

namespace act
{

// ----------------------------------------------------------------------------------------------------

Action::Action() : id_(UUID::generate())
{
}

// ----------------------------------------------------------------------------------------------------

Action::~Action()
{
}

// ----------------------------------------------------------------------------------------------------

ActionConstPtr Action::addAction(const std::string& type, tue::Configuration config)
{
    return server_->addAction(type, config);
}

// ----------------------------------------------------------------------------------------------------

void Action::stopAction(const UUID& id)
{
    server_->stopAction(id);
}

}
