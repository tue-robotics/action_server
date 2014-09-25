#ifndef ACTION_SERVER_EXAMPLES_SIMPLE_ACTION_FACTORY_H_
#define ACTION_SERVER_EXAMPLES_SIMPLE_ACTION_FACTORY_H_

#include <action_server/action_factory.h>

class SimpleActionFactory : public act::ActionFactory
{

public:

    SimpleActionFactory();

    act::ActionPtr createAction(const std::string& type, tue::Configuration config);


};

#endif
