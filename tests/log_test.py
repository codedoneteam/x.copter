import unittest
from unittest.mock import Mock, patch
from src.xcopter.modules.log import Log


class TestLog(unittest.TestCase):
    def test_info_without_master_does_not_raise(self):
        logger = Log()

        with patch("logging.info") as mock_logging_info:
            logger.info("hello")

        mock_logging_info.assert_called_once_with("hello")

    def test_info_sends_statustext_when_master_available(self):
        logger = Log()
        logger.master = Mock()

        logger.info("hello")

        logger.master.mav.statustext_send.assert_called_once()

    def test_error_swallows_statustext_failures(self):
        logger = Log()
        logger.master = Mock()
        logger.master.mav.statustext_send.side_effect = RuntimeError("broken transport")

        with patch("logging.error") as mock_logging_error:
            logger.error("boom")

        mock_logging_error.assert_called_once_with("boom")


if __name__ == "__main__":
    unittest.main()
