#!/usr/bin/env python3
"""Tests for full and single-parameter argparse help behavior."""

from __future__ import annotations

import argparse
import contextlib
import io
import unittest

from cli_help import show_requested_parameter_help


class CliHelpTest(unittest.TestCase):
    def test_launcher_environment_names_are_unique_and_resolve(self):
        from launcher_help import RECORDER_ENV, parameters
        self.assertEqual(len(RECORDER_ENV), len(set(RECORDER_ENV)))
        values = parameters()
        self.assertTrue(set(RECORDER_ENV).issubset(values))
        self.assertEqual(values['ENCODER_THREADS'][0], 2)
        self.assertEqual(values['BACKGROUND_SAVE'][0], 1)

    def test_launcher_help_distinguishes_seal_from_exit(self):
        from launcher_help import main
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            main([])
        self.assertIn('F=seal batch', output.getvalue())
        self.assertIn('Q=drain saves and exit', output.getvalue())

    def make_parser(self) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )
        parser.add_argument("--fps", type=int, default=30, help="Target recording rate.")
        parser.add_argument("--with-depth", action="store_true", help="Record depth.")
        parser.add_argument(
            "--depth-use-log",
            action=argparse.BooleanOptionalAction,
            default=True,
            help="Use logarithmic depth quantization.",
        )
        return parser

    def capture_single_help(self, argv: list[str]) -> str:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            with self.assertRaises(SystemExit) as result:
                show_requested_parameter_help(self.make_parser(), argv)
        self.assertEqual(result.exception.code, 0)
        return output.getvalue()

    def test_single_value_option_help(self) -> None:
        output = self.capture_single_help(["--fps", "--help"])
        self.assertIn("Usage:", output)
        self.assertIn("Target recording rate.", output)
        self.assertIn("Default: 30", output)
        self.assertNotIn("Record depth.", output)

    def test_single_boolean_option_help(self) -> None:
        output = self.capture_single_help(["--with-depth", "--help"])
        self.assertIn("--with-depth", output)
        self.assertIn("Record depth.", output)
        self.assertNotIn("Target recording rate.", output)

    def test_boolean_optional_alias_is_shown(self) -> None:
        output = self.capture_single_help(["--no-depth-use-log", "--help"])
        self.assertIn("--depth-use-log, --no-depth-use-log", output)


if __name__ == "__main__":
    unittest.main()
