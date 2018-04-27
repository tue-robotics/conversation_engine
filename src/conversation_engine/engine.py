#!/usr/bin/env python
import rospy
from action_server import Client, TaskOutcome
import actionlib
import action_server_msgs
from conversation_engine.msg import ConverseAction
from grammar_parser import cfgparser
from robocup_knowledge import knowledge_loader


class ConversationEngine(object):
    def __init__(self, robot_name):
        self.current_semantics = {}
        self._path_of_missing_information = None
        self._knowledge = knowledge_loader.load_knowledge('challenge_gpsr')
        self._action_client = Client(robot_name)
        self._parser = cfgparser.CFGParser.fromstring(self._knowledge.grammar)
        self._robot_name = robot_name

        # Set up actionlib interface for clients to give a task to the robot.
        self._action_name = "/" + self._robot_name + "/conversation_engine/command"
        self._action_server = actionlib.SimpleActionServer(self._action_name, ConverseAction,
                                                           execute_cb=self.command_goal_cb, auto_start=False)

        self._action_server.register_preempt_callback(cb=self.reset)

        self._action_server.start()

        self._bla = False

        rospy.logdebug("Started conversation engine on {}".format(self._action_name))

    def command_goal_cb(self, goal):
        self.current_semantics = self._parser.parse(self._knowledge.grammar_target, goal.command.strip().split(" "))
        self._action_client.send_goal(self.current_semantics)
        r = rospy.Rate(1)
        while not self._bla:
            print "still waiting"
            r.sleep()

    def reset(self):
        self._bla = True
        self._action_client.cancel_all()
        self.current_semantics = {}

    def process(self, semantics):
        # I assume semantics is the exact information requested and supposed to go in place of the field indicated by
        # the missing information path

        if self._path_of_missing_information:
            # fill semantics in existing semantics
            path_list = self._path_of_missing_information.split('.')
            action_index = int(path_list[0].strip('actions[').rstrip(']'))
            fields = path_list[1:]

            elem = self.current_semantics['actions'][action_index]
            for field in fields:
                try:
                    elem = elem[field]
                except KeyError:
                    elem[field] = semantics

        return self.current_semantics


if __name__ == "__main__":
    rospy.init_node("test_bla")
    conv_engine = ConversationEngine("amigo")
    rospy.spin()