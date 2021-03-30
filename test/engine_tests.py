# System
import mock
print(mock.__version__)
import unittest

# TU/e Robotics
from action_server import TaskOutcome

# Conversation engine
from conversation_engine import ConversationEngine, ConversationState


class WrappedConversationEngine(ConversationEngine):
    """
    Wraps the conversation engine and implements the '_say_to_user' method
    """
    def __init__(self, grammar, target):
        self.client = mock.Mock()
        self.client.send_async_task = mock.MagicMock()
        super(WrappedConversationEngine, self).__init__(self.client, grammar, target)
        self.robot_to_user_messages = []

    def user_to_robot_msg(self, msg):
        self._handle_user_to_robot(msg)

    def _say_to_user(self, message):
        self.robot_to_user_messages.append(message)


class ConversationEngineTests(unittest.TestCase):
    def test_invalid_input(self):
        grammar = "T -> yes | no"
        target = "T"
        wce = WrappedConversationEngine(grammar, target)
        wce.user_to_robot_msg("foo")
        self.assertTrue(len(wce.robot_to_user_messages) > 0)  # Robot has responed
        self.assertEqual(wce._state._state, ConversationState.IDLE)  # Robot should still be idling

    def test_simple_assignment(self):
        sentence = "tell the time"
        target = "T"
        grammar = [
            "T[A] -> C[A]",
            "C[{'actions': <A1>}] -> VP[A1]",
            "VP[{'action': 'say', 'sentence': X}] -> V_SAY SAY_SENTENCE[X]",
            "V_SAY -> tell | say",
            "SAY_SENTENCE['time'] -> the time"]
        grammar = "\n".join(grammar)
        wce = WrappedConversationEngine(grammar, target)
        wce.user_to_robot_msg(sentence)
        self.assertEqual(wce._state._state, ConversationState.WAIT_FOR_ROBOT)  # State should have changed
        wce.client.send_async_task.assert_called()  # Check if the action has been sent to the robot

        # Simulate that the robot is done
        msg = "foo"
        outcome = TaskOutcome(TaskOutcome.RESULT_SUCCEEDED, [msg])
        wce._done_cb(outcome)

        self.assertEqual(wce._state._state, ConversationState.IDLE)  # Robot is done
        # Result message has been communicated to the user
        self.assertTrue(any([msg in msg_to_user for msg_to_user in wce.robot_to_user_messages]))

    def test_missing_information(self):
        """Test that the engine asks for missing information
        The conversation should go like this:
        User:  "bring me a drink"
        Robot: "Don't know where to look for the drink" (missing information: 'action[0].location')
        User:  "fridge"
        Robot: "here's your drink"
        """
        sentence = "bring me a drink"
        target = "T"
        location = "fridge"
        missing_field = "action[0].location"

        # ToDo: not nice
        grammar = "T[A] -> C[A]\n"
        grammar += "C[{'actions': <A1>}] -> VP[A1]\n"  # This 'actions' makes that the grammar _parser outputs a dict with 'actions'
        grammar += "V_BRING -> bring | give\n"
        grammar += "DET -> the | a\n"
        grammar += "OPERATOR[{'type': 'person', 'id': 'operator'}] -> me\n"
        grammar += "NAMED_OBJECT[{'type': 'drink'}] -> drink\n"

        grammar += "VP[{'action': 'hand-over', 'target-location': Y, 'object': Z}] -> V_BRING OPERATOR[Y] DET NAMED_OBJECT[Z]\n"

        grammar += "ROOM_OR_LOCATION[X] -> LOCATION[X]\n"
        grammar += "LOCATION[{'id': 'fridge'}] -> fridge\n"
        # grammar = "{} -> {}\nROOM_OR_LOCATION -> {}".format(target, sentence, location)
        wce = WrappedConversationEngine(grammar, target)
        wce.user_to_robot_msg(sentence)
        self.assertEqual(wce._state._state, ConversationState.WAIT_FOR_ROBOT)  # State should have changed
        wce.client.send_async_task.assert_called()  # Check if the action has been sent to the robot

        # Simulate that the action server has requested additional information
        msg = "Don't know where to look for the drink"
        outcome = TaskOutcome(TaskOutcome.RESULT_MISSING_INFORMATION, [msg], missing_field)
        wce._done_cb(outcome)

        # The TaskOutcome indicates that we are missing some info, which should be put in the missing_field
        # The missing field is mapped to a part of the grammar (ROOM_OR_LOCATION) by the ConversationEngine

        # Check result
        self.assertEqual(wce._state._state, ConversationState.WAIT_FOR_USER)
        self.assertEqual(wce._state.target, "ROOM_OR_LOCATION")
        self.assertTrue(any([msg in msg_to_user for msg_to_user in wce.robot_to_user_messages]))

        # Clear the user messages
        wce.robot_to_user_messages = []

        # Set the answer
        wce.user_to_robot_msg(location)
        print(wce.robot_to_user_messages)

        # Now: the robot should be busy once again
        self.assertEqual(wce._state._state, ConversationState.WAIT_FOR_ROBOT)

        # Set the taskoutcome to succeeded
        msg2 = "Here's your beer"
        outcome2 = TaskOutcome(TaskOutcome.RESULT_SUCCEEDED, [msg2])
        wce._done_cb(outcome2)

        # Check the result once again
        self.assertEqual(wce._state._state, ConversationState.IDLE)
        self.assertTrue(any([msg2 in msg_to_user for msg_to_user in wce.robot_to_user_messages]))


if __name__ == "__main__":
    unittest.main()
