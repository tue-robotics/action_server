#ifndef ACTION_SERVER_SERVER_H_
#define ACTION_SERVER_SERVER_H_

#include "action_server/types.h"

#include <map>
#include <tue/config/configuration.h>

namespace act
{

class Server
{

public:

    Server();

    virtual ~Server();

    void run();

    void tick();

    void registerActionFactory(const ActionFactoryPtr& action_factory);

    ActionConstPtr addAction(const std::string& type, tue::Configuration config);

    void stopAction(const UUID& id);

private:

    std::map<UUID, ActionPtr> actions_;

    std::map<std::string, ActionFactoryPtr> action_type_to_factory_;

};

}

#endif
