# System
import json
import os
import random
import yaml
from copy import deepcopy

# ROS
import rospy
from std_msgs.msg import String

# TU/e Robotics
from action_server import Client, TaskOutcome
from grammar_parser import cfgparser


def sanitize_text(txt):
    stripped = "".join(c for c in txt if c not in """!.,:'?`~@#$%^&*()+=-/\></*-+""")
    lowered = stripped.lower()

    mapping = {"dining table": "dining_table",
               "display case": "display_case",
               "storage shelf": "storage_shelf",
               "couch table": "couch_table",
               "tv table": "tv_table",
               "kitchen table": "kitchen_table",
               "kitchen cabinet": "kitchen_cabinet",
               "side table": "side_table",
               "living room": "living_room",
               "dining room": "dining_room",
               "storage table": "storage_table",
               "end table": "end_table",
               "chocolate drink": "chocolate_drink",
               "grape juice": "grape_juice",
               "orange juice": "orange_juice",
               "potato chips": "potato_chips",
               "cleaning stuff": "cleaning_stuff"}

    for key, value in mapping.iteritems():
        lowered = lowered.replace(key, value)

    return lowered


def describe_current_subtask(subtask, prefix=True):
    """
    Make a 'natural' language description of subtask name
    """
    to_verb = {"AnswerQuestion": "answering a question",
               "ArmGoal": "moving my arm",
               "DemoPresentation": "giving a demo",
               "Find": "finding",
               "Follow": "following",
               "Guide": "guiding",
               "GripperGoal": "moving my gripper",
               "HandOver": "handing something over",
               "Inspect": "inspecting",
               "LookAt": "looking",
               "NavigateTo": "navigating",
               "PickUp": "picking up",
               "Place": "placing",
               "ResetWM": "resetting my world model",
               "Say": "speaking",
               "SendPicture": "sending a picture",
               "TurnTowardSound": "turning towards a sound"}
    description = to_verb.get(subtask, subtask + "ing")

    if prefix:
        description = random.choice(["I'm busy", "I'm"]) + " " + description

    return description


class ConversationState(object):
    """Encapsulate all conversation state.
    This makes it impossible to transition to a next state without setting the correct fields

    When a new conversation is started, the ConversationState is IDLE.
    Then the user can initiate the conversation by supplying text, which then initializes the semantics of an action.
    The ConversationEngine then sends this to the action_server and we wait for the robot to execute the action or
    think about why the action cannot be performed yet.
    When calling the action_server, we must indicate that we are waiting for the robot via the the wait_for_robot()
    method.

    In the case of missing info, the robot misses some field from the semantics.
    The ConversationEngine figures out which subtree of the grammar to use to parse the user's eventual input and
    passes this in the wait_for_user() method.
    The missing field can, for example, be a location to do something with.
    This means the grammar should not try to parse the reply to come as a type of drink or as a verb, for example, but
    as a location.
    What rule to parse text with is stored in .target

    The ConversationEngine then says something to the user and waits for the user to reply.
    When this is received, the ConversationEngine reads the grammar target from the ConversationState before it parses
    the replied text for additional info.

    The current semantics are updated using update_semantics() and the action_server is called again with the now
    updated semantics.
    When this happens, the ConversationEngine must call wait_for_robot() again.

    The user can also ask to abort() the conversation to stop.
    """
    IDLE = "IDLE"
    WAIT_FOR_USER = "WAIT_FOR_USER"  # Waiting for info from user
    WAIT_FOR_ROBOT = "WAIT_FOR_ROBOT"  # Waiting for the action server to report success/aborted (missing info or fail)
    ABORTING = "ABORTING"

    def __init__(self):
        rospy.loginfo("New ConversationState")
        self._state = ConversationState.IDLE
        self._target = None
        self._missing_field = None

        self._current_semantics = {}

    @property
    def state(self):
        """
        Part of the conversation we're in
        """
        return self._state

    @property
    def target(self):
        """
        Subtree of the grammar tree to use when parsing text
        """
        return self._target

    @property
    def missing_field(self):
        """
        What field is missing in the current_semantics before the action_server can execute it
        The user must provide useful information to fill this field.
        """
        return self._missing_field

    @property
    def current_semantics(self):
        """
        A dict containing an (incomplete) action description for the action_server
        """
        return deepcopy(self._current_semantics)

    def wait_for_user(self, target, missing_field):
        """
        Indicate that the ConversationEngine is waiting for the user's input.

        This is to wait for additional info, that must be parsed according to target in order to fill some missing field
        :param target: name of the grammar rule to parse the user's reply with
        :type target: str
        :param missing_field: a 'path' indicating where to insert the additional info from the user into the
        current_semantics dictionary
        :type missing_field: str
        """
        rospy.loginfo("ConversationState: {old} -> {new}. Target='{t}', missing_field='{mf}'"
                      .format(old=self._state, new=ConversationState.WAIT_FOR_USER,
                              t=target, mf=missing_field))
        self._state = ConversationState.WAIT_FOR_USER
        self._target = target
        self._missing_field = missing_field

    def wait_for_robot(self):
        """
        Indicate that the ConversationEngine is waiting for the robot to either finish (planning) the action
        """
        rospy.loginfo("ConversationState: {old} -> {new}".format(old=self._state, new=ConversationState.WAIT_FOR_ROBOT))
        self._state = ConversationState.WAIT_FOR_ROBOT
        self._target = None
        self._missing_field = None

    def aborting(self, timeout, timeout_callback):
        """
        Set the state to ABORTING. If the action server hasn't aborted the action after the given timeout,
        call the callback to deal with that

        :param timeout: duration to wait before killing
        :type timeout: rospy.Duration
        :param timeout_callback: callback is called when the aborting takes too long
        :type timeout_callback: (event: rospy.TimerEvent)
        """
        rospy.loginfo("ConversationState: {old} -> {new}".format(old=self._state, new=ConversationState.ABORTING))
        self._state = ConversationState.ABORTING

        rospy.Timer(timeout,
                    timeout_callback,
                    oneshot=True)

    def initialize_semantics(self, semantics):
        """
        Initialize an action description for the action_server. This gets updated as the conversation progresses, via
        update_semantics()
        """
        self._current_semantics = semantics
        rospy.loginfo("Initialized semantics: {}".format(self._current_semantics))

    def update_semantics(self, semantics, missing_field_path):
        """
        Update the current action description for the action_server

        The semantics will be filled into current_semantics at the missing_field_path
        :param semantics: dictionary with additional info
        :type semantics: dict
        :param missing_field_path: a 'path' indicating where to insert the additional info from the user into the
        current_semantics dictionary
        :type missing_field_path: str
        """

        # I assume semantics is the exact information requested and supposed to go in place of the field indicated by
        # the missing information path

        # fill semantics in existing semantics
        path_list = missing_field_path.split('.')
        action_index = int(path_list[0].strip('actions[').rstrip(']'))
        fields = path_list[1:]

        elem = self._current_semantics['actions'][action_index]
        for field in fields:
            try:
                elem = elem[field]
            except KeyError:
                elem[field] = semantics

        rospy.loginfo("Updated semantics: {}".format(self._current_semantics))
        return


class ConversationEngine(object):
    """ConversationEngine provides the bridge between the user and the action_server.
    It accepts text and parses it to a 'semantics' dictionary that the action_server can interpret.

    The action_server then tries to formulate a plan based on the semantics and either:
    - starts execution and succeeds or fails
    - indicates it is missing information.
    In the case of missing information, the action_server indicates what field of information it is missing.

    Based on that, the ConversationEngine must converse with the user to obtain more information.
    There is some logic to determine with grammar rules/target to use to parse the additional information with.

    Once the information is obtained, the semantics of the current conversation state are updated
    and sent to the action_server again, in hope the semantics is now complete enough for execution.

    ConversationEngine is set up as a base class, that leaves the implementation of talking with the user to subclasses.
    Implement
    - _say_to_user to say something to the user
    - user_to_robot_text to accept text from the user
    """

    def __init__(self, action_client, grammar, command_target, give_examples=True):
        """
        Initialize a new ConversationEngine for the given robot, using some grammar with a command_target.
        Indicate whether to give examples of thins to say to the user via give_examples
        :param action_client: interface to the action server
        :type action_client: Client
        :param grammar: string to initialize a CFGParser with see https://github.com/tue-robotics/grammar_parser/
        :type grammar: str
        :param grammar: the root of the grammar's parse tree
        :type grammar: str
        :param give_examples: Include examples when talking with the user.
            These are randomly generated from the active part of the grammar
        :type give_examples: bool
        """

        self._state = ConversationState()

        self._action_client = action_client  # ToDo: update (GPSR) dependency before merging!

        self._parser = cfgparser.CFGParser.fromstring(grammar)
        self._grammar = grammar
        self._command_target = command_target

        self.give_examples = give_examples

        self._latest_feedback = None

        rospy.logdebug("Started conversation engine")

    def user_to_robot_text(self, text):
        """Accept raw text from the user.

        :param text: what the user typed or said
        :type text str"""
        self._handle_user_to_robot(text)

    def _handle_user_to_robot(self, text):
        """Start processing text from the user. This handles sanitation of the strings and
        any special commands that affect the conversation flow rather than the action (like aborting etc)
        :param text: what the user typed or said
        :type text str
        """
        rospy.loginfo("_handle_user_to_robot('{}')".format(text))

        text = sanitize_text(text)

        if self._handle_special_commands(text):
            return

        if self._state.state == ConversationState.IDLE:
            # parse command and send_goal
            self._handle_command(text)
        elif self._state.state == ConversationState.WAIT_FOR_USER:
            # Update semantics and send updated goal
            self._handle_additional_info(text)
        elif self._state.state == ConversationState.WAIT_FOR_ROBOT:
            # User must wait for robot/action server to reply, cannot handle user input now
            self._handle_user_while_waiting_for_robot(text)
        elif self._state.state == ConversationState.ABORTING:
            self._handle_user_while_aborting(text)

    def _handle_special_commands(self, text):
        """Check for special commands that should not be parsed further
        @returns bool indicating if the text is a special command"""

        stop_words = ["stop", "cancel", "quit", "reset"]
        if any([word for word in stop_words if word in text]):
            rospy.loginfo("_handle_special_commands('{}'):".format(text))
            if self._state.state == ConversationState.IDLE:
                self._say_to_user(random.choice(["I'm not busy",
                                                 "I'm not doing anything"]))
                self._say_ready_for_command()
            elif self._state.state == ConversationState.WAIT_FOR_ROBOT:
                self._stop()
            elif self._state.state == ConversationState.WAIT_FOR_USER:
                self._say_to_user(random.choice(["I'm waiting for you, there's nothing to stop",
                                                 "I can't stop, you stop!"]))

                self._start_new_conversation()
            elif self._state.state == ConversationState.ABORTING:
                self._say_to_user(random.choice(["I'm already stopping, gimme some time!"]))
            return True

        kill_words = ["sudo kill"]
        if any([word for word in kill_words if word in text]):
            rospy.loginfo("_handle_special_commands('{}'): kill action server".format(text))

            self._action_client.cancel_all_async()

            self._say_to_user(random.choice(["Woah, sorry dude for not stopping fast enough!"]))
            os.system("rosnode kill /state_machine")
            self._say_to_user(random.choice(["Killed the action_server, pray for resurrection"]))

            # This is assuming the state machine is back online when a command is received
            self._start_new_conversation()

            return True

    def _stop(self):
        rospy.loginfo("_stop(): Cancelling goals, resetting state")

        def notify_user(event):
            self._say_to_user(
                random.choice(["State machine takes a long time to abort, you can kill it with 'sudo kill'"]))

        self._state.aborting(rospy.Duration(20), notify_user)
        self._action_client.cancel_all_async()
        self._latest_feedback = None

        self._say_to_user(random.choice(["Stop! Hammer time",
                                         "Oops, sorry",
                                         "OK, I'll stop"]))

    def _start_new_conversation(self):
        self._state = ConversationState()
        self._latest_feedback = None
        self._say_ready_for_command()  # This is assuming the state machine is back online when a command is received
        self._start_wait_for_command(self._grammar, self._command_target)

    def _handle_command(self, text):
        """
        Parse text into goal semantics, send to action_server
        """
        rospy.loginfo("_handle_command('{}')".format(text))

        words = text.strip().split(" ")

        # The command the user gave is being parsed towards the command_target in the grammar
        # The parse returns a task description dictionary
        try:
            semantics = self._parser.parse_raw(self._command_target, words, debug=True)
            self._state.initialize_semantics(semantics)
            self._action_client.send_async_task(str(self._state.current_semantics),
                                                done_cb=self._done_cb,
                                                feedback_cb=self._feedback_cb)
            rospy.loginfo("Task sent: {}".format(self._state.current_semantics))

            self._state.wait_for_robot()
        except (cfgparser.GrammarError, cfgparser.ParseError) as e:
            rospy.logerr("Parsing '{}' failed: {}".format(text, e))
            self._log_invalid_command(text)

            if 'sandwich' in text:
                result_sentence = "Try 'sudo {}'.".format(text)
            else:
                result_sentence = random.choice(["You're not making sense.",
                                                 "Don't give me this shit.",
                                                 "Tell me something useful.",
                                                 "This is useless input. Thanks, but no thanks.",
                                                 "Make sense to me, fool!",
                                                 "Talk to the gripper, the PC is too good for you.",
                                                 "Something went terribly wrong.",
                                                 "Would your mother in law understand?",
                                                 "Try 'sudo {}'.".format(text)])
            self._say_to_user(result_sentence)
            self._start_new_conversation()

    def _handle_additional_info(self, text):
        """
        Parse text into additional info according to grammar & target received from action_server result
        """
        rospy.loginfo("_handle_additional_info('{}')".format(text))

        words = text.strip().split(" ")
        try:
            additional_semantics = self._parser.parse_raw(self._state.target, words, debug=True)
        except (cfgparser.GrammarError, cfgparser.ParseError) as e:
            rospy.logerr("Parsing '{}' failed: {}".format(text, e))
            sentence = random.choice(["Give me something useful"])
            if self.give_examples:
                example = self._parser.get_random_sentence(self._state.target)
                sentence += ", like '{}'".format(example)
            self._say_to_user(sentence)
            self._state.wait_for_user(missing_field=self._state.missing_field, target=self._state.target)
            return

        rospy.loginfo("additional_semantics: {}".format(additional_semantics))
        sem_str = json.dumps(additional_semantics)
        sem_dict = yaml.load(sem_str)
        rospy.logdebug("parsed: {}".format(sem_dict))

        try:
            rospy.loginfo("Additional_semantics: {}".format(additional_semantics))
            self._state.update_semantics(sem_dict, self._state.missing_field)
            self._say_to_user(random.choice(["OK, that helps!",
                                             "Allright, thanks!"]))

            self._action_client.send_async_task(str(self._state.current_semantics),
                                                done_cb=self._done_cb,
                                                feedback_cb=self._feedback_cb)
            rospy.loginfo("Updated task sent: {}".format(self._state.current_semantics))

            self._state.wait_for_robot()
        except (KeyError, IndexError) as e:
            rospy.logerr("Could not update semantics: {}".format(e))
            self._say_to_user(
                random.choice(["Something went terribly wrong, can we try a new command?",
                               "I didn't understand that, what do you want me to do?",
                               "What would you like me to do? Could you please rephrase you command?"]))
            self._stop()

    def _handle_user_while_waiting_for_robot(self, text):
        """
        Talk with the user while the robot is busy doing stuff
        """
        sentence = random.choice(["I'm busy, give me a sec.",
                                  "Hold on, "])

        if self._latest_feedback:
            sentence += " " + describe_current_subtask(self._latest_feedback.current_subtask)

        self._say_to_user(sentence)

    def _handle_user_while_aborting(self, text):
        """
        Talk with the user while the robot is busy aborting the action
        """
        sentence = random.choice(["Yes, I'll stop ",
                                  "I'm quitting "])

        if self._latest_feedback:
            sentence += describe_current_subtask(self._latest_feedback.current_subtask, prefix=False)

        self._say_to_user(sentence)

    def _say_ready_for_command(self):
        sentence = random.choice(["I'm ready for a command.",
                                  "Your wish is my command.",
                                  "Do you have an assignment for me?",
                                  "Do you have a command for me?",
                                  "Please tell me what to do.",
                                  "Anything to do boss?"])

        if self.give_examples:
            example = self._parser.get_random_sentence(self._command_target)

            sentence += " An example command is: '{}'. ".format(example)

        self._say_to_user(sentence)

    def _done_cb(self, task_outcome):
        """
        The action_server's action is done, which can mean the action is finished (successfully or failed) or
        needs additional info. This last option is handled by _on_request_missing_information and
        the other cases by a starting a new conversation
        """
        rospy.loginfo("_done_cb: Task done -> {to}".format(to=task_outcome))
        assert isinstance(task_outcome, TaskOutcome)

        self._latest_feedback = None

        if task_outcome.succeeded:
            rospy.loginfo("Action succeeded")
            self._on_task_successful(" ".join(task_outcome.messages))

            self._start_new_conversation()
        elif task_outcome.result == TaskOutcome.RESULT_MISSING_INFORMATION:
            rospy.loginfo("Action needs more info from user")

            sentence = "".join(task_outcome.messages)
            target = self._get_grammar_target(task_outcome.missing_field)
            self._state.wait_for_user(target=target,
                                      missing_field=task_outcome.missing_field)
            self._on_request_missing_information(sentence, self._grammar, target)

        elif task_outcome.result == TaskOutcome.RESULT_TASK_EXECUTION_FAILED:
            rospy.loginfo("Action execution failed")

            self._on_task_outcome_failed("".join(task_outcome.messages))

            self._start_new_conversation()

        elif task_outcome.result == TaskOutcome.RESULT_UNKNOWN:
            rospy.loginfo("Action result: unknown")
            self._on_task_outcome_unknown("".join(task_outcome.messages))

            self._start_new_conversation()

        else:
            rospy.loginfo("Action result: other")
            self._say_to_user("".join(task_outcome.messages))

            self._start_new_conversation()

    def _feedback_cb(self, feedback):
        self._latest_feedback = feedback

        rospy.loginfo(feedback.current_subtask)

    @staticmethod
    def _get_grammar_target(missing_field_path):
        """
        Determine which grammar target to use when a particular field of info is missing.
        """
        deepest_field_name = missing_field_path.split('.')[-1]

        grammar_target = "T"

        if 'location' in deepest_field_name:
            grammar_target = 'ROOM_OR_LOCATION'
        elif 'entity' in deepest_field_name:
            grammar_target = 'ROOM_OR_LOCATION'
        elif 'target' in deepest_field_name:
            grammar_target = 'NAMED_PERSON'
        elif 'sentence' in deepest_field_name:
            grammar_target = 'SAY_SENTENCE'
        elif 'object' in deepest_field_name:
            grammar_target = 'NAMED_OBJECT'

        rospy.loginfo("Missing information '{}' must match '{}' in grammar".format(missing_field_path, grammar_target))
        return grammar_target

    @staticmethod
    def _log_invalid_command(text):
        with open("invalid_commands.txt", "a") as dump:
            dump.writelines([text + "\n"])

    def _say_to_user(self, message):
        raise NotImplementedError(
            "How to say something to the user must be implemented in the subclass. "
            "Signature: ```_say_to_user(message: str)```")

    def _on_task_successful(self, message):
        self._say_to_user(message)

    def _on_request_missing_information(self, description, grammar, target):
        rospy.loginfo("_request_missing_information('{}', '{}...', '{}')".format(description, grammar[:10], target))

        if self.give_examples:
            example = self._parser.get_random_sentence(self._state.target)
            description += " For example: '{}'".format(example)
        self._say_to_user(description)

    def _on_task_outcome_failed(self, message):
        self._say_to_user(message)

    def _on_task_outcome_unknown(self, message):
        self._say_to_user(message)

    def _start_wait_for_command(self, grammar, target):
        pass

    def is_text_valid_input(self, text):
        """
        Checks if the provided text input is valid by parsing it

        :param text: input text
        :type text: str
        :return: whether the parsing succeeded or failed
        :returns: bool
        """
        sanitized = sanitize_text(text)
        words = sanitized.strip().split(" ")
        target = self._state.target if self._state.target else self._command_target
        try:
            self._parser.parse_raw(target, words, debug=True)
            return True
        except (cfgparser.GrammarError, cfgparser.ParseError) as e:
            rospy.logerr("Text input '{}' is not valid: {}".format(text, e))
            return False


class ConversationEngineUsingTopic(ConversationEngine):
    def __init__(self, robot_name, grammar, command_target):

        client = Client(robot_name)

        super(ConversationEngineUsingTopic, self).__init__(client, grammar, command_target)

        self._user_to_robot_sub = rospy.Subscriber("user_to_robot", String, self.user_to_robot_msg)
        self._robot_to_user_pub = rospy.Publisher("robot_to_user", String, queue_size=10)

    def user_to_robot_msg(self, msg):
        self._handle_user_to_robot(msg.data)

    def _say_to_user(self, message):
        self._robot_to_user_pub.publish(message)
