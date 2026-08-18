#!/usr/bin/env python3
"""Small argparse helpers shared by collector command-line tools."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from typing import Any


def show_requested_parameter_help(
    parser: argparse.ArgumentParser,
    argv: Sequence[str] | None = None,
) -> None:
    """Print one option's help when ``OPTION --help`` is requested.

    ``argparse`` normally treats ``--help`` as a global action.  The collector
    has many configuration switches, so a command such as
    ``record_lerobot_official.py --max-sync-delta-sec --help`` should explain
    only that switch and should not require unrelated required arguments.
    Normal ``--help`` behavior remains unchanged when no option precedes it.
    """

    values = list(sys.argv[1:] if argv is None else argv)
    help_indexes = [index for index, value in enumerate(values) if value in {"-h", "--help"}]
    if not help_indexes:
        return

    help_index = help_indexes[0]
    option_actions = parser._option_string_actions
    requested = [
        value
        for value in values[:help_index]
        if value in option_actions and value not in {"-h", "--help"}
    ]
    unique_requested = list(dict.fromkeys(requested))
    if len(unique_requested) != 1:
        return

    action = option_actions[unique_requested[0]]
    _print_action_help(parser, action)
    parser.exit()


def _print_action_help(parser: argparse.ArgumentParser, action: argparse.Action) -> None:
    option_names = ", ".join(action.option_strings)
    primary_option = action.option_strings[0]
    value_text = _value_usage(action)
    usage = f"{parser.prog} {primary_option}{value_text}"

    print(f"{option_names}\n")
    print(f"Usage: {usage}")
    print(f"Description: {action.help or 'No description provided.'}")
    print(f"Destination: {action.dest}")
    if action.required:
        print("Required: yes")
    else:
        print(f"Default: {_display_default(action.default)}")
    if action.choices is not None:
        print("Choices: " + ", ".join(str(choice) for choice in action.choices))
    if action.type is not None:
        print(f"Value type: {_type_name(action.type)}")
    if action.nargs not in (None, 0):
        print(f"Number of values: {action.nargs}")
    print("\nUse --help without an option to display all collector parameters.")


def _value_usage(action: argparse.Action) -> str:
    """Render the value portion of one option's usage line."""

    if action.nargs == 0:
        return ""
    metavar = action.metavar
    if metavar is None:
        metavar = action.dest.upper().replace("_", "-")
    if action.nargs in (None, 1):
        return f" {metavar}"
    if action.nargs == "?":
        return f" [{metavar}]"
    if action.nargs == "*":
        return f" [{metavar} ...]"
    if action.nargs == "+":
        return f" {metavar} [{metavar} ...]"
    if isinstance(action.nargs, int):
        return " " + " ".join(metavar for _ in range(action.nargs))
    return f" {metavar}"


def _display_default(value: Any) -> str:
    if value is argparse.SUPPRESS:
        return "none"
    if value is None:
        return "none"
    return repr(value)


def _type_name(value: Any) -> str:
    return getattr(value, "__name__", str(value))
