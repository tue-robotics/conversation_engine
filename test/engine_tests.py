# System
import mock
import unittest

# Conversation engine
from conversation_engine import ConversationEngine, ConversationState


class WrappedConversationEngine(ConversationEngine):
    """
    Wraps the conversation engine and implements the '_say_to_user' method
    """
    def __init__(self):
        client = mock.MagicMock()
        grammar = "T -> yes | no"
        command_target = "T"
        super(WrappedConversationEngine, self).__init__(client, grammar, command_target)
        self.robot_to_user_messages = []

    def user_to_robot_msg(self, msg):
        self._handle_user_to_robot(msg)

    def _say_to_user(self, message):
        self.robot_to_user_messages.append(message)


class ConversationEngineTests(unittest.TestCase):
    def test_invalid_input(self):
        wce = WrappedConversationEngine()
        wce.user_to_robot_msg("foo")
        self.assertTrue(len(wce.robot_to_user_messages) > 0)  # Robot has responed
        self.assertEqual(wce._state._state, ConversationState.IDLE)  # Robot should still be idling


if __name__ == "__main__":
    unittest.main()
