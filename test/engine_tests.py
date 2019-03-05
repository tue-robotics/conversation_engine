# System
import unittest

# Conversation engine
from conversation_engine import ConversationEngine, ConversationState


class WrappedConversationEngine(ConversationEngine):
    """
    Wraps the conversation engine and implements the '_say_to_user' method
    """
    def _say_to_user(self, message):
        pass


class ConversationEngineTests(unittest.TestCase):
    pass


if __name__ == "__main__":
    unittest.main()
