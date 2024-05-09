#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

# Copyright 2024, University of Freiburg,
# Chair of Algorithms and Data Structures
# Author: Hannah Bast <bast@cs.uni-freiburg.de>

import argparse
import re
import subprocess
import time
from pathlib import Path

import argcomplete


def all_combinations(options: list[tuple[str, str]]) -> list[tuple[str, str]]:
    """
    Print all combinations of the options.

    >>> test_options = [('a', '--anna'), ('b', '--berta'), ('c', '--clara')]
    >>> all_combinations(test_options) # doctest: +NORMALIZE_WHITESPACE
    [('abc', '--anna --berta --clara'), ('Abc', '--berta --clara'), \
     ('aBc', '--anna --clara'), ('ABc', '--clara'), \
     ('abC', '--anna --berta'), ('AbC', '--berta'), \
     ('aBC', '--anna'), ('ABC', '')]
    """

    combinations = []
    for i in range(2 ** len(options)):
        combination = ""
        name = ""
        for j in range(len(options)):
            if i & (1 << j) == 0:
                name += options[j][0].lower()
                combination += options[j][1] + " "
            else:
                name += options[j][0].upper()
        combination = combination.strip()
        combinations.append((name, combination))
    return combinations


def evaluate_all(args: argparse.Namespace):
    """
    Evaluate `spatialjoin` for the given dataset, for all combinations of the
    `options` above.
    """

    # The five options and a represenative letter for each.
    options = [('b', '--no-box-ids'), ('s', '--no-surface-area'),
               ('c', '--no-cutouts'), ('d', '--no-diag-box'),
               ('o', '--no-oriented-envelope')]

    # Try all combinations of the options.
    for name, combination in all_combinations(options):
        cmd = (f"cat {args.basename}.spatialjoin-input.tsv |"
               f" spatialjoin {combination}"
               f" --contains \" ogc:sfContains \""
               f" --covers \" ogc:sfCovers \""
               f" --intersects \" ogc:sfIntersects \""
               f" --equals \" ogc:sfEquals \""
               f" --touches \" ogc:sfTouches \""
               f" --crosses \" ogc:sfCrosses \""
               f" --overlaps \" ogc:sfOverlaps \""
               f" --suffix $' .\\n'")

        # Only show the commands?
        if args.only_show_commands:
            print(f"{name}\t{cmd}")
            continue

        # Run the command and time it.
        start = time.time()
        result = subprocess.run(cmd, shell=True,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE)
        end = time.time()
        duration_seconds = f"{end - start:.3f}"
        if result.returncode != 0:
            # print(colored(
            #     f"Command failed with exit code {result.returncode}"
            #     f", and the following output to stderr:", "red"))
            # print()
            error_message = [line for line in
                             result.stderr.decode().split("\n")
                             if " INFO : " not in line]
            if len(error_message) > 0:
                error_message = error_message[0]
            else:
                error_message = "[no output to stderr except INFO messages]"
            duration_seconds = error_message
            # print(colored(error_message, "red"))
            # print()
            # print("The command was:")
            # print()
            # print(colored(f"{cmd}", "blue"))
            # print()
            # exit(1)
        print(f"{name}\t{duration_seconds}")


def analyze(args: argparse.Namespace):
    """
    Analyze the results from a previous run.
    """

    # Read the results from the file.
    results = []
    with open(f"{args.basename}.spatialjoin-evaluation.tsv") as file:
        for line in file:
            name, duration = line.strip().split("\t")
            results.append((name, float(duration)))

    # For each option, compute its maximal and minimal speedup relative
    # to all the other options.
    for option_index, description in \
            [0, "box ids"], [1, "surface area"], [2, "cutouts"], \
            [3, "diagonal boxes"], [4, "oriented envelopes"]:
        # Sort the results by name, with the option at `option_index` as
        # least significant.
        def sort_key(pair):
            name = pair[0]
            i = option_index
            key = name[:i] + name[i + 1:] + name[i]
            return key
        sorted_results = sorted(results, key=sort_key)
        # Iterate over pairs of consecutive items in the sorted list and keep
        # track of the minimal and maximal speedup.
        min_speedup, min_speedup_index = None, None
        max_speedup, max_speedup_index = None, None
        for i in range(0, len(sorted_results), 2):
            name1, duration1 = sorted_results[i]
            name2, duration2 = sorted_results[i + 1]
            letter = name1[option_index].upper()
            # Assert that the names differ only in the option at `option_index`
            # and that at `option_index` the difference is only in case.
            assert name1[:option_index] == name2[:option_index]
            assert name1[option_index + 1:] == name2[option_index + 1:]
            assert name1[option_index].upper() == letter
            assert name2[option_index].upper() == letter
            # Compute the speedup.
            speedup = duration2 / duration1
            # Update the minimal and maximal speedup.
            if min_speedup_index is None or speedup < min_speedup:
                min_speedup = speedup
                min_speedup_index = i
            if max_speedup_index is None or speedup > max_speedup:
                max_speedup = speedup
                max_speedup_index = i
        # Show the minimal and maximal speedup for this option.
        min_speedup_names = f"{sorted_results[min_speedup_index + 1][0]} -> " \
                            f"{sorted_results[min_speedup_index][0]}"
        max_speedup_names = f"{sorted_results[max_speedup_index + 1][0]} -> " \
                            f"{sorted_results[max_speedup_index][0]}"
        print(f"{description:20}: "
              f"max speedup {max_speedup:4.2f}x ({max_speedup_names}), "
              f"min speedup {min_speedup:4.2f}x ({min_speedup_names})")


if __name__ == "__main__":
    # Get the basenames of all files of the form `*.spatialjoin-input.tsv` or
    # `*.spatialjoin-evaluation.tsv` in the current directory.
    basenames = set()
    search_paths = (list(Path(".").glob("*.spatialjoin-input.tsv")) +
                    list(Path(".").glob("*.spatialjoin-evaluation.tsv")))
    for p in search_paths:
        basenames.add(re.sub("\\..*$", "", p.name))

    # Parse command line arguments.
    parser = argparse.ArgumentParser()
    parser.add_argument("basename",
                        help="basename of the dataset").completer = \
        argcomplete.ChoicesCompleter(list(basenames))
    parser.add_argument("--only-show-commands",
                        action="store_true", default=False,
                        help="Only show the commands that would be executed")
    parser.add_argument("--analyze", action="store_true", default=False,
                        help="Analyze the results from a previous run")
    argcomplete.autocomplete(parser, always_complete_options="long")
    args = parser.parse_args()

    # Evaluate `spatialjoin` for the given dataset, for all combinations of the
    # `options` above.
    if not args.analyze:
        evaluate_all(args)
    else:
        analyze(args)
