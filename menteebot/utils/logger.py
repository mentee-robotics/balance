import logging
import os

from rich.logging import RichHandler


def configure_logging():
    logger = logging.getLogger("menteebot")

    # the handler determines where the logs go: stdout/file
    shell_handler = RichHandler(show_path=False, markup=True)

    logger.setLevel(logging.INFO)
    shell_handler.setLevel(logging.INFO)

    # the formatter determines what our logs will look like
    fmt_shell = "%(name)s | %(message)s"

    shell_formatter = logging.Formatter(fmt_shell)

    # here we hook everything together
    shell_handler.setFormatter(shell_formatter)

    logger.addHandler(shell_handler)

    return logger


def configure_logging_output_file(filename):
    # Check if directory exists
    d = os.path.dirname(filename)
    assert os.path.exists(d), f"Directory {d} does not exists!"

    logger = logging.getLogger("menteebot")

    file_handler = logging.FileHandler(filename)
    fmt_file = "%(levelname)s %(asctime)s [%(filename)s:%(funcName)s:%(lineno)d] %(message)s"
    file_formatter = logging.Formatter(fmt_file)
    file_handler.setFormatter(file_formatter)
    logger.addHandler(file_handler)
