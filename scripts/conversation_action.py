#! /usr/bin/env python

import rospy
import actionlib

from conversation_engine.msg import ConverseAction, ConverseGoal, ConverseFeedback, ConverseResult


class ConverseActionServer(object):

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ConverseAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()
        rospy.loginfo("'{}' started".format(name))

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)

        rospy.loginfo(goal)

        # publish info to the console for the user
        rospy.loginfo('{an}: Executing, processing command {cmd}'.format(an=self._action_name, cmd=goal.command))

        # publish the feedback
        self._as.publish_feedback(ConverseFeedback(feedback="I'm working on {}".format(goal.command)))
        # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep()

        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(ConverseResult(result_sentence="I finished {}".format(goal.command)))


if __name__ == '__main__':
    rospy.init_node('conversation_engine')
    server = ConverseActionServer(rospy.get_name())
    rospy.spin()
