# ROS
import actionlib
import random
import rospy
import json
import yaml

# TU/e Robotics
from action_server import Client, TaskOutcome
from grammar_parser import cfgparser
import hmi
from robocup_knowledge import knowledge_loader

# Conversation engine
from conversation_engine.msg import ConverseAction, ConverseResult, ConverseFeedback


class ConversationEngine(object):
    def __init__(self, robot_name):
        self.current_semantics = {}
        self._knowledge = knowledge_loader.load_knowledge('challenge_gpsr')

        self._action_client = Client(robot_name)
        self._action_client._action_client.feedback_cb = self.feedback_cb
        self._parser = cfgparser.CFGParser.fromstring(self._knowledge.grammar)
        self._robot_name = robot_name
        self._hmi_client = hmi.client.Client("/" + self._robot_name + '/conversation_engine_hmi')

        # Set up actionlib interface for clients to give a task to the robot.
        self._action_name = "/" + self._robot_name + "/conversation_engine"
        self._action_server = actionlib.SimpleActionServer(self._action_name, ConverseAction,
                                                           execute_cb=self.command_goal_cb, auto_start=False)

        self._action_server.register_preempt_callback(cb=self.reset)

        self._action_server.start()

        rospy.logdebug("Started conversation engine on {}".format(self._action_name))

    def feedback_cb(self, feedback):
        print feedback.current_subtask

    def command_goal_cb(self, goal):
        print "Received new goal: {}".format(goal)
        self.current_semantics = self._parser.parse(self._knowledge.grammar_target, goal.command.strip().split(" "))
        
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
