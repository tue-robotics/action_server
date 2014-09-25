#ifndef ACTION_SERVER_ACTION_H_
#define ACTION_SERVER_ACTION_H_

#include "action_server/types.h"
#include <tue/config/configuration.h>

namespace act
{

class Action
{

public:

    Action();

    virtual ~Action();

    virtual void initialize(tue::Configuration config) = 0;

    virtual void tick() = 0;

    virtual void stop() = 0;

    const UUID& id() const { return id_; }

protected:

    UUID id_;

    Server* server_;

    ActionConstPtr addAction(const std::string& type, tue::Configuration config);

    void stopAction(const ActionConstPtr& action) { stopAction(action->id()); }

    void stopAction(const UUID& id);

};

}

#endif
