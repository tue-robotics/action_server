#ifndef ACTION_SERVER_EXAMPLES_SIMPLE_ACTION_H_
#define ACTION_SERVER_EXAMPLES_SIMPLE_ACTION_H_

#include <action_server/action.h>

class SimpleAction : public act::Action
{

public:

    void initialize(tue::Configuration config);

    void tick();

    void stop();

private:

    std::string message_;

};

#endif
