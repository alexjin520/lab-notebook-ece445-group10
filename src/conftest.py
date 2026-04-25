"""
pytest configuration for OmniSense-Dual  |  ECE 445 Group 10
=============================================================

Activates the omnisense logging hierarchy during test runs so that
[VERIFY:*] log lines appear in the pytest output and in a log file,
without modifying any test files or requiring extra CLI flags.

Usage
-----
  python -m pytest tests/ -v                  ← INFO on console, DEBUG to file
  python -m pytest tests/ -v -s               ← same (stdout not captured)
  python -m pytest tests/ -v --log-cli-level=DEBUG  ← DEBUG on console too

Log file location
-----------------
  logs/test_<YYYYMMDD_HHMMSS>.log
"""

import logging
import logging.handlers
import os
import time

import pytest


def pytest_configure(config: pytest.Config) -> None:
    """
    Called once before any tests are collected.

    Sets up file + console handlers on the 'omnisense' logger so every
    [VERIFY:*] line emitted during the test session is captured.
    """
    log_dir = os.path.join(os.path.dirname(__file__), "logs")
    os.makedirs(log_dir, exist_ok=True)

    ts    = time.strftime("%Y%m%d_%H%M%S")
    fpath = os.path.join(log_dir, f"test_{ts}.log")

    root = logging.getLogger("omnisense")
    root.setLevel(logging.DEBUG)
    root.handlers.clear()

    # ── File handler – DEBUG (full detail for offline analysis) ──────────────
    fh = logging.handlers.RotatingFileHandler(
        fpath, maxBytes=10 * 1024 * 1024, backupCount=5, encoding="utf-8",
    )
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-8s %(name)s  %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    root.addHandler(fh)

    # ── Pytest live-log handler – INFO (shown inline with test output) ────────
    # Pytest also has its own log-capture mechanism; this StreamHandler ensures
    # output even when pytest's capture is active (-s or --log-cli-level).
    # Set to WARNING by default so normal test runs stay readable; override
    # with --log-cli-level=INFO or --log-cli-level=DEBUG when needed.
    ch = logging.StreamHandler()
    ch.setLevel(logging.WARNING)
    ch.setFormatter(logging.Formatter("%(levelname)-8s %(message)s"))
    root.addHandler(ch)

    root.info(
        "[VERIFY:TEST_SESSION_START] log_file=%s", fpath,
    )


def pytest_terminal_summary(terminalreporter, exitstatus: int, config: pytest.Config) -> None:
    """Print the log file path at the end of every test run."""
    log_dir = os.path.join(os.path.dirname(__file__), "logs")
    # Find the most recently created test log
    try:
        logs = sorted(
            (f for f in os.listdir(log_dir) if f.startswith("test_") and f.endswith(".log")),
            reverse=True,
        )
        if logs:
            terminalreporter.write_sep(
                "-",
                f"OmniSense log → logs/{logs[0]}  (grep [VERIFY:TAG] to filter by requirement)",
            )
    except FileNotFoundError:
        pass
