#ifndef ACTION_SERVER_ACTION_H_
#define ACTION_SERVER_ACTION_H_

#include "action_server/types.h"
#include <tue/config/configuration.h>

namespace act
{

enum ActionStatus
{
    SUCCEEDED,
    FAILED,
    RUNNING
};

class Action
{

public:

    Action();

    virtual ~Action();

    virtual void initialize(tue::Configuration config) {}

    virtual void tick() {}

    virtual void stop() {}

    const UUID& id() const { return id_; }

    ActionStatus status() const { return status_; }

    const tue::Configuration& feedback() const { return feedback_; }

protected:

    UUID id_;

    Server* server_;

    ActionStatus status_;

    tue::Configuration feedback_;

    ActionConstPtr addAction(const std::string& type, tue::Configuration config);

    void stopAction(const ActionConstPtr& action) { stopAction(action->id()); }

    void stopAction(const UUID& id);

    void setSucceeded(tue::Configuration feedback = tue::Configuration());

    void setFailed(tue::Configuration feedback = tue::Configuration());

    void setFeedback(tue::Configuration feedback) { feedback_ = feedback; }

};

}

#endif
