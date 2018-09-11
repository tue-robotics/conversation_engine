# conversation_engine
The conversation_engine package provides a way to conversate with a robot to give it commands

The ConversationEngine processes text and generates text, which together form a conversation. 

The ConversationEngine start idle and then waits for input from the user. 
When text (for example: 'go to the kitchen') is received, it is parsed by the [grammar_parser](https://github.com/tue-robotics/grammar_parser).

The result is a so-called 'semantics dictionary' that describes an action and parametrizes an action. For example, a 'go-to' action is parametrized with the destination (e.g. 'kitchen'). 

That action description is sent (as a [action_server_msgs/Task](https://github.com/tue-robotics/action_server/blob/master/action_server_msgs/action/Task.action)) to the [action_server](https://github.com/tue-robotics/action_server), which then tries to execute the action. 
The ConversationEngine is then waiting for the underlying robot to execute the action. 

There are several ways in which execution can fail (all specified in [action_server_msgs/Task](https://github.com/tue-robotics/action_server/blob/master/action_server_msgs/action/Task.action)). The most interesting for the conversation_engine is failing due to missing information. 
In that case, the action description was under-specified and needs more information. For example, if the command is "Find a person", the action_server misses the information of where to find a person and who to find. 

When this happens, the action_server specifies what field is missing. The ConversationEngine is configured with some mapping that specifies what subtree of the grammar to use for what field of missing information. For example, if the missing field is a location, the grammar target can try to parse the next text as a LOCATION. 

The ConversationEngine then generates a string asking for a location and wait for the user to reply with the missing information. 

When the additional information is received, it is parsed again, now using the e.g. LOCATION as the target for the parsing. If that gives a useable result, the semantics of the current action are updated and extended with the new info and the extended description is again sent to the action_server and the cycle begins again.
When an action is actually starting, the action_server provides some feedback about it's current (sub)task and the ConversationEngine passes that on to the user. 
