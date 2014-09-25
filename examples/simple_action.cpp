#include "simple_action.h"

// ----------------------------------------------------------------------------------------------------

void SimpleAction::initialize(tue::Configuration config)
{
    if (!config.value("message", message_, tue::OPTIONAL))
        message_ = "Hello world!";
}

// ----------------------------------------------------------------------------------------------------

void SimpleAction::tick()
{
    std::cout << message_ << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void SimpleAction::stop()
{

}

