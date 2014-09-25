#ifndef ACTION_SERVER_ACTION_FACTORY_H_
#define ACTION_SERVER_ACTION_FACTORY_H_

#include "action_server/types.h"
#include <tue/config/configuration.h>

namespace act
{

class ActionFactory
{

public:

    ActionFactory();

    virtual ~ActionFactory();

    virtual ActionPtr createAction(const std::string& type, tue::Configuration config) = 0;

    const std::vector<std::string>& supportedActionTypes() { return action_types_; }

protected:

    void registerActionType(const std::string& action_type) { action_types_.push_back(action_type); }

private:

    std::vector<std::string> action_types_;

};

}

#endif
