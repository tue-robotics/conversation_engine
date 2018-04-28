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
from robocup_knowledge import knowledge_loader

# Conversation engine
from conversation_engine.msg import ConverseAction, ConverseResult, ConverseFeedback

def sanitize_text(txt):
    stripped = "".join(c for c in txt if c not in """!.,:'?`~@#$%^&*()+=-></*-+""")
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

    for key, value in mapping:
        lowered = lowered.replace(key, value)

    return lowered


class ConversationState(object):
    IDLE = 0
    WAIT_FOR_USER = 1  # Waiting for info from user
    WAIT_FOR_ROBOT = 2  # Waiting for the action server to reply with success/aborted (missing info or fail)


class ConversationEngine(object):
    def __init__(self, robot_name):
        self.current_semantics = {}
        self._knowledge = knowledge_loader.load_knowledge('challenge_gpsr')

        self._state = ConversationState.IDLE

        self._action_client = Client(robot_name)

        self._parser = cfgparser.CFGParser.fromstring(self._knowledge.grammar)
        self._robot_name = robot_name

        self._user_to_robot_sub = rospy.Subscriber("chatter/user_to_robot", String, self._handle_user_to_robot)
        self._robot_to_user_pub = rospy.Publisher("chatter/robot_to_user", String)

        rospy.logdebug("Started conversation engine")

    def _handle_user_to_robot(self, msg):
        rospy.loginfo("_handle_user_to_robot({})".format(msg))

        text = sanitize_text(msg.data)

        # TODO: Check for special commands (like stop, start etc)

        if self._state == ConversationState.IDLE:
            # parse command and send_goal
            self._handle_command(text)
        elif self._state == ConversationState.WAIT_FOR_USER:
            pass
            # Update semantics and send updated goal
        elif self._state == ConversationState.WAIT_FOR_ROBOT:
            # User must wait for robot/action server to reply, cannot handle user input now
            pass

    def _handle_command(self, text):
        """Parse text into goal semantics, send to action_server"""
        # TODO parse and send_goal
        self._state = ConversationState.WAIT_FOR_ROBOT

    def _handle_addition_info(self, text):
        """Parse text into additional info according to grammar & target received from action_server result"""
        # TODO parse, update semantics, send_goal
        self._state = ConversationState.WAIT_FOR_ROBOT

    def _done_cb(self, task_outcome):
        rospy.loginfo("_done_cb: Task done -> {to}".format(to=task_outcome))

        assert isinstance(task_outcome, TaskOutcome)

        if task_outcome.succeeded:
            rospy.loginfo("Action succeeded")
            self._robot_to_user_pub.publish(" ".join(task_outcome.messages))
            self._state = ConversationState.IDLE

        elif task_outcome.result == TaskOutcome.RESULT_MISSING_INFORMATION:
            rospy.loginfo("Action needs more info")
            self._robot_to_user_pub.publish("".join(task_outcome.messages))
            self._state = ConversationState.WAIT_FOR_USER

        elif task_outcome.result == TaskOutcome.RESULT_TASK_EXECUTION_FAILED:
            rospy.loginfo("Action execution failed")
            self._robot_to_user_pub.publish("".join(task_outcome.messages))
            self._state = ConversationState.IDLE

        elif task_outcome.result == TaskOutcome.RESULT_UNKNOWN:
            rospy.loginfo("Action result: unknown")
            self._robot_to_user_pub.publish("".join(task_outcome.messages))
            self._state = ConversationState.IDLE

        else:
            rospy.loginfo("Action result: other")
            self._robot_to_user_pub.publish("".join(task_outcome.messages))
            self._state = ConversationState.IDLE


    def _feedback_cb(self, feedback):
        self._robot_to_user_pub.publish(feedback.current_subtask)  # TODO make natural language

    def command_goal_cb(self, goal):
        print "Received new goal: {}".format(goal)
        self.current_semantics = self._parser.parse(self._knowledge.grammar_target, goal.command.strip().split(" "))
        print "Example: {}".format(self._parser.get_random_sentence(self._knowledge.grammar_target))
        
        if self.current_semantics:

            while True:
                task_outcome = self._action_client.send_task(str(self.current_semantics))
                print task_outcome
                if task_outcome.result != TaskOutcome.RESULT_MISSING_INFORMATION:

                    break
                if not task_outcome.messages or task_outcome.result == TaskOutcome.RESULT_MISSING_INFORMATION and not task_outcome:
                    break

                target = self._get_grammar_target(task_outcome.missing_field)
                try:
                    result = self._hmi_client.query(description="".join(task_outcome.messages),
                                                    grammar=self._knowledge.grammar, target=target, timeout=100)
                    print "I received result: {}".format(result)
                    sem_str = json.dumps(result.semantics)
                    sem_dict = yaml.load(sem_str)
                    print "parsed: {}".format(sem_dict)
                    self.process_hmi_result(sem_dict, task_outcome.missing_field)
                    print "New semantics: {}".format(self.current_semantics)
                except hmi.TimeoutException:
                    pass

            if not task_outcome.succeeded:
                if task_outcome.messages:
                    self._action_server.set_aborted(ConverseResult(result_sentence="".join(task_outcome.messages)))
                else:
                    self._action_server.set_aborted(ConverseResult(result_sentence="I don't feel so good"))
            else:
                self._action_server.set_succeeded(ConverseResult(result_sentence="".join(task_outcome.messages)))
        else:
            result_sentence = random.choice(["You're not making sense.",
                                             "Don't give me this shit.",
                                             "Tell me something useful.",
                                             "This is useless input. Thanks, but no thanks.",
                                             "Make sense to me, fool!",
                                             "Talk to the gripper, the PC is too good for you.",
                                             "Something went terribly wrong.",
                                             "Would your mother in law understand?",
                                             "Try 'sudo {}'".format(goal.command)])
            self._action_server.set_aborted(ConverseResult(result_sentence=result_sentence))

    def reset(self):
        self._action_client.cancel_all()
        self.current_semantics = {}

    def process_hmi_result(self, semantics, missing_field_path):
        # I assume semantics is the exact information requested and supposed to go in place of the field indicated by
        # the missing information path

        # fill semantics in existing semantics
        path_list = missing_field_path.split('.')
        action_index = int(path_list[0].strip('actions[').rstrip(']'))
        fields = path_list[1:]

        elem = self.current_semantics['actions'][action_index]
        for field in fields:
            try:
                elem = elem[field]
            except KeyError:
                elem[field] = semantics
        return

    def _get_grammar_target(self, missing_field_path):
        deepest_field_name = missing_field_path.split('.')[-1]
        if 'location' in deepest_field_name:
            return 'ROOM_OR_LOCATION'
        else:
            return "T"
