#ifndef ACTION_SERVER_TYPES_H_
#define ACTION_SERVER_TYPES_H_

#include <boost/shared_ptr.hpp>

#include "action_server/uuid.h"

namespace act
{

class Server;

class Action;
typedef boost::shared_ptr<Action> ActionPtr;
typedef boost::shared_ptr<const Action> ActionConstPtr;

class ActionFactory;
typedef boost::shared_ptr<ActionFactory> ActionFactoryPtr;
typedef boost::shared_ptr<const ActionFactory> ActionFactoryConstPtr;

}

#endif
