# Action Server
The action server is an actionlib server for managing the execution of high
level tasks and their semantic chaining. It takes actionlib goals describing
series of high level tasks in json form, checks the consistency of the
semantics, and executes them. This readme explains the architecture of the
action server, the procedure of handling a goal and the lifecycle of an action.

## Architecture

![Action Server architecture](doc/action_server_architecture.jpg)

The action server uses an actionlib interface for communication with its
clients. A client implementation is provided as abstraction from the actionlib
client. Current examples of clients to the action server are:
 - The GPSR challenge
 - The Natural Language Console
Consult one of these implementations for example usage of the Client class.

Aside from the actionlib SimpleActionServer, the Action Server holds an
instance of the Task Manager. As the name suggests, this is the component that
does the actual managing of the task (including its subtasks) for us.

## Procedure

Let's see what happens when a client sends a goal to the action server. Let's
assume our client takes the high level natural language task *"Go to the
kitchen, find a coke, and bring it to me."*
With a good natural language parser, we can parse this to the following json
object:
```
{
  'actions' :
  [
    {
      'action': 'navigate-to',
      'object':
      {
        'type' : 'room',
        'name' : 'kitchen'
      }
    },
    {
      'action': 'find',
      'object':
      {
        'type': 'coke'
      }
    },
    {
      'action': 'bring',
      'object':
      {
        'type': 'reference'
      },
      'target-location':
      {
        'type': 'person',
        'id': 'operator'
      }
    }
  ]
}
```
This json object can be sent to the action server.

### Configuration
The server parses the object and passes the resulting Python dictionary (the
`recipe`) to the task manager (`set_up_state_machine(recipe)`). The task manager
then goes through the list of actions, instantiates actions and tries to chain
their semantics. It does this by calling the `configure` method on every action.

For example, the second action (*find a coke*) results in knowledge of a coke.
We don't know anything about this object yet, but we expect that there will be a
coke in the world model.
This knowledge is part of the `ConfigurationResult` returned by
`Find.configure(configuration_data)`.
It is added as knowledge to the configuration data that is passed to the next
action.
The next action, (*bring it to me*), receives this knowledge, but does not
necessarily need it.
However, the `reference` (*it*) in the `bring` action configuration data lets
the `bring` action know that there must be knowledge of an object
that can be brought somewhere.
Therefore, it will try to find this in the knowledge in its configuration data.
In our example, this knowledge is available, so the `bring` action will grab
the found coke and take it to the operator.
When the required knowledge is not available, the action may return a
ConfigurationResult specifying that information is missing. The server will
notify the client of this result so that it can ask the user for more
information.

### Execution
When all actions are successfully configured, the task manager is ready to
start executing the actions. To do this, the action server calls
`task_manager.execute_next_action()` while there are still remaining actions.
It will return early if an action fails.
In our example: if the `find` action fails (so no coke is found), the `bring`
action will not be executed.

## The Action life cycle
The Action life cycle consists of the following phases:
 - Instantiation:
    - checking of resources independent of task semantics
 - Configuration
    - checking information in the task semantics
    - passing knowledge from one action to another
 - Execution
    - performing the actual action

For a detailed guide on the internals of an Action implementation and
instructions on how to implement your own Action, take a look at the
`example_action`, which contains a lot of documentation and explanation on the
details of Actions.
