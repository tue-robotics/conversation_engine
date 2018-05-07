# System
import os
from copy import deepcopy

# ROS
import actionlib
import random
import rospy
import json
import yaml

# TU/e Robotics
from action_server import Client, TaskOutcome
from grammar_parser import cfgparser
from std_msgs.msg import String

# Conversation engine
from conversation_engine.msg import ConverseAction, ConverseResult, ConverseFeedback

def sanitize_text(txt):
    stripped = "".join(c for c in txt if c not in """!.,:'?`~@#$%^&*()+=-/\></*-+""")
    lowered = stripped.lower()

    mapping = { "dining table": "dining_table",
                "display case": "display_case",
                "storage shelf": "storage_shelf",
                "couch table": "couch_table",
                "tv table": "tv_table",
                "kitchen table": "kitchen_table",
                "kitchen cabinet": "kitchen_cabinet",
                "side table": "side_table",
                "living room": "living_room",
                "dining room": "dining_room"}

    for key, value in mapping.iteritems():
        lowered = lowered.replace(key, value)

    return lowered


def describe_current_subtask(subtask, prefix=True):
    """Make a 'natural' language description of subtask name"""
    to_verb = { "AnswerQuestion": "answering a question",
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
    This makes it impossible to transition to a next state without setting the correct fields"""
    IDLE = "IDLE"
    WAIT_FOR_USER = "WAIT_FOR_USER"  # Waiting for info from user
    WAIT_FOR_ROBOT = "WAIT_FOR_ROBOT"  # Waiting for the action server to reply with success/aborted (missing info or fail)
    ABORTING = "ABORTING"

    def __init__(self):
        rospy.loginfo("New ConversationState")
        self._state = ConversationState.IDLE
        self._target = None
        self._missing_field = None

        self._current_semantics = {}

    @property
    def state(self):
        return self._state

    @property
    def target(self):
        return self._target

    @property
    def missing_field(self):
        return self._missing_field

    @property
    def current_semantics(self):
        return deepcopy(self._current_semantics)

    def wait_for_user(self, target, missing_field):
        rospy.loginfo("ConversationState: {old} -> {new}. Target='{t}', missing_field='{mf}'"
                      .format(old=self._state, new=ConversationState.WAIT_FOR_USER,
                              t=target, mf=missing_field))
        self._state = ConversationState.WAIT_FOR_USER
        self._target = target
        self._missing_field = missing_field

    def wait_for_robot(self):
        rospy.loginfo("ConversationState: {old} -> {new}".format(old=self._state, new=ConversationState.WAIT_FOR_ROBOT))
        self._state = ConversationState.WAIT_FOR_ROBOT
        self._target = None
        self._missing_field = None

    def aborting(self, timeout, timeout_callback):
        """Set the state to ABORTING. If the action server hasn't aborted the action after the given timeout,
        call the callback to deal with that
        :param timeout duration to wait before killing
        :type timeout rospy.Duration
        :param timeout_callback callback is called when the aborting takes too long
        :type callable(event: rospy.TimerEvent)"""
        rospy.loginfo("ConversationState: {old} -> {new}".format(old=self._state, new=ConversationState.ABORTING))
        self._state = ConversationState.ABORTING

        rospy.Timer(timeout,
                    timeout_callback,
                    oneshot=True)

    def initialize_semantics(self, semantics):
        self._current_semantics = semantics
        rospy.loginfo("Initialized semantics: {}".format(self._current_semantics))

    def update_semantics(self, semantics, missing_field_path):
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
    def __init__(self, robot_name, grammar, command_target):

        self._state = ConversationState()

        self._action_client = Client(robot_name)

        self._parser = cfgparser.CFGParser.fromstring(grammar)
        self._robot_name = robot_name
        self._grammar = grammar
        self._command_target = command_target

        self._user_to_robot_sub = rospy.Subscriber("user_to_robot", String, self._handle_user_to_robot)
        self._robot_to_user_pub = rospy.Publisher("robot_to_user", String, queue_size=10)

        self._latest_feedback = None

        rospy.logdebug("Started conversation engine")

    def _handle_user_to_robot(self, msg):
        rospy.loginfo("_handle_user_to_robot('{}')".format(msg))

        text = sanitize_text(msg.data)

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
                self._robot_to_user_pub.publish(random.choice(["I'm not busy",
                                                               "I'm not doing anything"]))
                self._say_ready_for_command()
            elif self._state.state == ConversationState.WAIT_FOR_ROBOT:
                self._stop()
            elif self._state.state == ConversationState.WAIT_FOR_USER:
                self._robot_to_user_pub.publish(random.choice(["I'm waiting for you, there's nothing to stop",
                                                               "I can't stop, you stop!"]))

                self._start_new_conversation()
            elif self._state.state == ConversationState.ABORTING:
                self._robot_to_user_pub.publish(random.choice(["I'm already stopping, gimme some time!"]))
            return True

        kill_words = ["sudo kill"]
        if any([word for word in kill_words if word in text]):
            rospy.loginfo("_handle_special_commands('{}'): kill action server".format(text))

            self._action_client.cancel_all_async()

            self._robot_to_user_pub.publish(random.choice(["Woah, sorry dude for not stopping fast enough!"]))
            os.system("rosnode kill /state_machine")
            self._robot_to_user_pub.publish(random.choice(["Killed the action_server, pray for resurrection"]))
            self._start_new_conversation()  # This is assuming the state machine is back online when a command is received

            return True

    def _stop(self):
        rospy.loginfo("_stop(): Cancelling goals, resetting state")

        def hard_kill(event):
            os.system("rosnode kill /state_machine")
            self._robot_to_user_pub.publish(random.choice(["Headshot to the action_server, it had enough time to comply"]))

            self._start_new_conversation()  # This is assuming the state machine is back online when a command is received

        self._state.aborting(rospy.Duration(20), hard_kill)
        self._action_client.cancel_all_async()
        self._latest_feedback = None

        self._robot_to_user_pub.publish(random.choice(["Stop! Hammer time",
                                                       "Oops, sorry",
                                                       "OK, I'll stop"]))

    def _start_new_conversation(self):
        self._state = ConversationState()
        self._latest_feedback = None
        self._say_ready_for_command()  # This is assuming the state machine is back online when a command is received

    def _handle_command(self, text):
        """Parse text into goal semantics, send to action_server"""
        rospy.loginfo("_handle_command('{}')".format(text))

        words = text.strip().split(" ")

        # The command the user gave is being parsed towards the command_target in the grammar
        # The parse returns a task description dictionary
        self._state.initialize_semantics(self._parser.parse(self._command_target, words, debug=True))

        result_sentence = None

        if self._state.current_semantics:
            self._action_client.send_async_task(str(self._state.current_semantics),
                                                done_cb=self._done_cb,
                                                feedback_cb=self._feedback_cb)
            rospy.loginfo("Task sent: {}".format(self._state.current_semantics))

            self._state.wait_for_robot()
        else:
            rospy.loginfo("Could not parse '{}'".format(text))
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
            self._robot_to_user_pub.publish(result_sentence)
            self._start_new_conversation()

    def _handle_additional_info(self, text):
        """Parse text into additional info according to grammar & target received from action_server result"""
        rospy.loginfo("_handle_additional_info('{}')".format(text))

        words = text.strip().split(" ")
        additional_semantics = self._parser.parse(self._state.target, words, debug=True)

        rospy.loginfo("additional_semantics: {}".format(additional_semantics))
        sem_str = json.dumps(additional_semantics)
        sem_dict = yaml.load(sem_str)
        rospy.logdebug("parsed: {}".format(sem_dict))

        if additional_semantics:
            try:
                rospy.loginfo("Additional_semantics: {}".format(additional_semantics))
                self._state.update_semantics(sem_dict, self._state.missing_field)
                self._robot_to_user_pub.publish(random.choice(["OK, I can work with that",
                                                               "Allright, thanks!"]))

                self._action_client.send_async_task(str(self._state.current_semantics),
                                                    done_cb=self._done_cb,
                                                    feedback_cb=self._feedback_cb)
                rospy.loginfo("Updated task sent: {}".format(self._state.current_semantics))

                self._state.wait_for_robot()
            except Exception as e:
                rospy.logerr("Could not update semantics: {}".format(e))
                self._robot_to_user_pub.publish("Something went terribly wrong, can we try a new command?")
                self._stop()
        else:
            example = self._parser.get_random_sentence(self._state.target)
            sentence = random.choice(["Give me something useful"])
            sentence += ", like '{}'".format(example)
            self._robot_to_user_pub.publish(sentence)
            self._state.wait_for_user(missing_field=self._state.missing_field, target=self._state.target)

    def _handle_user_while_waiting_for_robot(self, text):
        sentence = random.choice(["I'm busy, give me a sec.",
                                  "Hold on, "])

        if self._latest_feedback:
            sentence += " " + describe_current_subtask(self._latest_feedback.current_subtask)

        self._robot_to_user_pub.publish(sentence)

    def _handle_user_while_aborting(self, text):
        sentence = random.choice(["Yes, I'll stop ",
                                  "I'm quitting "])

        if self._latest_feedback:
            sentence += describe_current_subtask(self._latest_feedback.current_subtask, prefix=False)

        self._robot_to_user_pub.publish(sentence)

    def _say_ready_for_command(self):
        sentence = random.choice(["I'm ready for a command.",
                                  "Your wish is my command.",
                                  "Do you have an assignment for me?",
                                  "Do you have a command for me?",
                                  "Please tell me what to do."
                                  "Anything to do boss?"])
        example = self._parser.get_random_sentence(self._command_target)

        sentence += " An example command is: '{}'. ".format(example)

        self._robot_to_user_pub.publish(sentence)

    def _done_cb(self, task_outcome):
        rospy.loginfo("_done_cb: Task done -> {to}".format(to=task_outcome))
        assert isinstance(task_outcome, TaskOutcome)

        self._latest_feedback = None

        if task_outcome.succeeded:
            rospy.loginfo("Action succeeded")
            self._robot_to_user_pub.publish(" ".join(task_outcome.messages))

            self._start_new_conversation()

        elif task_outcome.result == TaskOutcome.RESULT_MISSING_INFORMATION:
            rospy.loginfo("Action needs more info from user")

            sentence = "".join(task_outcome.messages)
            self._state.wait_for_user(target=self._get_grammar_target(task_outcome.missing_field),
                                      missing_field=task_outcome.missing_field)
            example = self._parser.get_random_sentence(self._state.target)
            sentence += " For example: '{}'".format(example)
            self._robot_to_user_pub.publish(sentence)

        elif task_outcome.result == TaskOutcome.RESULT_TASK_EXECUTION_FAILED:
            rospy.loginfo("Action execution failed")
            self._robot_to_user_pub.publish("".join(task_outcome.messages))

            self._start_new_conversation()

        elif task_outcome.result == TaskOutcome.RESULT_UNKNOWN:
            rospy.loginfo("Action result: unknown")
            self._robot_to_user_pub.publish("".join(task_outcome.messages))

            self._start_new_conversation()

        else:
            rospy.loginfo("Action result: other")
            self._robot_to_user_pub.publish("".join(task_outcome.messages))

            self._start_new_conversation()

    def _feedback_cb(self, feedback):
        self._latest_feedback = feedback

        rospy.loginfo(feedback.current_subtask)
        self._robot_to_user_pub.publish(describe_current_subtask(feedback.current_subtask))

    def _get_grammar_target(self, missing_field_path):
        deepest_field_name = missing_field_path.split('.')[-1]

        if 'location' in deepest_field_name:
            return 'ROOM_OR_LOCATION'
        else:
            return "T"

    def _log_invalid_command(self, text):
        with open("invalid_commands.txt", "a") as dump:
            dump.writelines([text+"\n"])