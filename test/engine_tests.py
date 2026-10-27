# System
import mock
print(mock.__version__)
import sys
import unittest

# Conversation engine
from conversation_engine import ConversationEngine, ConversationState


class FakeCommandParser(object):
    """
    Stands in for the NER model, which needs torch and its downloaded weights. Parses every command into the same
    semantics.
    """
    def __init__(self, semantics):
        self._semantics = semantics

    def parse_raw(self, target, words, debug=False):
        return self._semantics


class WrappedConversationEngine(ConversationEngine):
    """
    Wraps the conversation engine and implements the '_say_to_user' method. The NER model, which needs torch and its
    downloaded weights, is replaced by command_parser.
    """
    def __init__(self, grammar, target, command_parser):
        self.client = mock.Mock()
        self.client.send_async_task = mock.MagicMock()
        ner_model_parser = mock.Mock(NERParser=mock.Mock(return_value=command_parser))
        with mock.patch.dict(sys.modules, {'ner_model': mock.Mock(), 'ner_model.parser': ner_model_parser}):
            super(WrappedConversationEngine, self).__init__(self.client, grammar, target)
        self.robot_to_user_messages = []

    def user_to_robot_msg(self, msg):
        self._handle_user_to_robot(msg)

    def _say_to_user(self, message):
        self.robot_to_user_messages.append(message)


class ConversationEngineTests(unittest.TestCase):
    def test_commands_are_parsed_with_the_ner_model(self):
        semantics = {'actions': [{'action': 'navigate-to', 'target-location': {'id': 'kitchen'}}]}
        wce = WrappedConversationEngine("T -> yes | no", "T", FakeCommandParser(semantics))
        wce.user_to_robot_msg("go to the kitchen")  # Not in the grammar
        self.assertEqual(wce._state._state, ConversationState.WAIT_FOR_ROBOT)  # State should have changed
        wce.client.send_async_task.assert_called()  # Check if the action has been sent to the robot
        self.assertEqual(wce.client.send_async_task.call_args[0][0], str(semantics))


if __name__ == "__main__":
    unittest.main()
