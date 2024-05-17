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

    # The five options and a representative letter for each.
    all_options = [('b', '--no-box-ids'), ('s', '--no-surface-area'),
                   ('c', '--no-cutouts'), ('d', '--no-diag-box'),
                   ('o', '--no-oriented-envelope')]
    options = [all_options[int(i)] for i in args.option_indexes.split(",")]

    # Try all combinations of the options.
    sweep_mode = " --no-fast-sweep-skip" if args.no_fast_sweep_skip else ""
    for name, combination in all_combinations(options):
        cmd = (f"cat {args.basename}.spatialjoin-input.tsv |"
               f" spatialjoin{sweep_mode} {combination}"
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
        total_time = f"{end - start:.3f}"

        # If non-zero exit code, print the error message
        if result.returncode != 0:
            error_message = [line for line in
                             result.stderr.decode().split("\n")
                             if " INFO : " not in line]
            if len(error_message) > 0:
                total_time = error_message[0]
            else:
                total_time = "[no output to stderr except INFO messages]"

        # Extract the separate times for parsing and sweeping from the log.
        parse_time = "[not found]"
        sweep_time = "[not found]"
        for line in result.stderr.decode().split("\n"):
            match = re.match(".*INFO : done \\(([0-9.]+)s\\)\\.", line)
            if match:
                if parse_time == "[not found]":
                    parse_time = f"{float(match.group(1)):.3f}"
                elif sweep_time == "[not found]":
                    sweep_time = f"{float(match.group(1)):.3f}"

        print(f"{name}\t{total_time}\t{parse_time}\t{sweep_time}", flush=True)


def analyze(args: argparse.Namespace):
    """
    Analyze the results from a previous run.
    """

    # Read the results from the file.
    results = []
    with open(f"{args.basename}.spatialjoin-evaluation.tsv") as file:
        for line in file:
            if line.startswith("#"):
                continue
            name, total_time, parse_time, sweep_time = line.strip().split("\t")
            time = total_time
            if args.time == "parse":
                time = parse_time
            elif args.time == "sweep":
                time = sweep_time
            results.append((name, float(time)))

    # First, show the maximal and minimal duration overall.
    sorted_results = sorted(results, key=lambda pair: pair[1])
    min_name, min_duration = sorted_results[0]
    max_name, max_duration = sorted_results[-1]
    med_name, med_duration = sorted_results[len(sorted_results) // 2]
    print(f"Overall stats  : "
          f"min {min_duration:.1f}s ({min_name}), "
          f"max {max_duration:.1f}s ({max_name}), "
          f"median {med_duration:.1f}s ({med_name}), "
          f"max/min = {max_duration / min_duration:.1f}x")
    print()

    # Best k options, for each fixed k.
    best = {}
    for name, time in results:
        k = sum(1 for c in name if c.isupper())
        if k not in best:
            best[k] = (name, time)
        else:
            if time < best[k][1]:
                best[k] = (name, time)
    baseline_name = best[0][0]
    baseline_time = best[0][1]
    print(f"Baseline      : {baseline_name} -> {baseline_time:6.1f}s")
    for k in sorted(best.keys()):
        if k > 0:
            name, time = best[k]
            previous_name, previous_time = best[k - 1]
            print(f"Best {k} {'option ' if k == 1 else 'options'}: "
                  f"{name} -> {time:6.1f}s, "
                  f"{baseline_time / time:4.1f}x speedup over baseline"
                  f" ({previous_time / time:3.1f}x over previous)")

    print()

    # A short human-readable description for each option.
    descriptions = {0: "box ids", 1: "surface area", 2: "cutouts",
                    3: "diagonal boxes", 4: "oriented boxes"}

    # For each option, compute its maximal and minimal speedup relative
    # to all the other options.
    for option_index in args.option_indexes.split(","):
        option_index = int(option_index)
        description = descriptions[option_index]

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
        print(f"{description:15}: "
              f"max speedup {max_speedup:4.2f}x ({max_speedup_names}), "
              f"min speedup {min_speedup:4.2f}x ({min_speedup_names})")


if __name__ == "__main__":
    # Get the basenames of all files of the form `*.spatialjoin-input.tsv` or
    # `*.spatialjoin-evaluation.tsv` in the current directory.
    basenames = set()
    search_paths = (list(Path(".").glob("*.spatialjoin-input.tsv")) +
                    list(Path(".").glob("*.spatialjoin-evaluation.tsv")))
    for p in search_paths:
        basenames.add(re.sub("\\.spatialjoin-[a-z]+\\.tsv*$", "", p.name))

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
    parser.add_argument("--time",
                        choices=["total", "parse", "sweep"],
                        default="total",
                        help="Time to analyze with --analyze (default: total)")
    parser.add_argument("--option-indexes", type=str,
                        default="0,1,2,3,4",
                        help="Comma-separated list of option indexes "
                        "(default: 0,1,2,3,4)")
    parser.add_argument("--no-fast-sweep-skip", action="store_true",
                        default=False,
                        help="Call spatialjoin with --no-fast-sweep-skip")
    argcomplete.autocomplete(parser, always_complete_options="long")
    args = parser.parse_args()

    # Evaluate `spatialjoin` for the given dataset, for all combinations of the
    # `options` above.
    if not args.analyze:
        evaluate_all(args)
    else:
        analyze(args)
