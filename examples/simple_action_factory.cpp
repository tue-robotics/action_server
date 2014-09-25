#include "simple_action_factory.h"

#include "simple_action.h"

// ----------------------------------------------------------------------------------------------------

SimpleActionFactory::SimpleActionFactory()
{
    registerActionType("simple");
}

// ----------------------------------------------------------------------------------------------------

act::ActionPtr SimpleActionFactory::createAction(const std::string& type, tue::Configuration config)
{
    if (type == "simple")
    {
        return act::ActionPtr(new SimpleAction);
    }

    return act::ActionPtr();
}
