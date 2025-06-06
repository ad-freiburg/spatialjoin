#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

# Copyright 2024 - 2025, University of Freiburg
# Chair of Algorithms and Data Structures
# Author: Hannah Bast <bast@cs.uni-freiburg.de>

import argparse
import re
import subprocess
import time
from pathlib import Path

import argcomplete
from argcomplete.completers import ChoicesCompleter


def all_combinations(options: list[tuple[str, str]]) -> list[tuple[str, str]]:
    """
    Print all combinations of the options.

    >>> test_options = [('a', '--no-a'), ('b', '--no-b'), ('C', '--use-c')]
    >>> all_combinations(test_options) # doctest: +NORMALIZE_WHITESPACE
    [('abc', '--no-a --no-b'), ('Abc', '--no-b'), \
     ('aBc', '--no-a'), ('ABc', ''), \
     ('abC', '--no-a --no-b --use-c'), ('AbC', '--no-b --use-c'), \
     ('aBC', '--no-a --use-c'), ('ABC', '--use-c')]
    """

    combinations = []
    for i in range(2 ** len(options)):
        combination = ""
        name = ""
        for j in range(len(options)):
            if i & (1 << j) == 0:
                name += options[j][0].lower()
                if options[j][0].islower():
                    combination += options[j][1] + " "
            else:
                name += options[j][0].upper()
                if options[j][0].isupper():
                    combination += options[j][1] + " "
        combination = combination.strip()
        combinations.append((name, combination))
    return combinations


def compute(args: argparse.Namespace):
    """
    Compute `spatialjoin` for the given dataset, for all combinations in
    `args.combinations`.
    """

    # The six options and a representative letter for each.
    #
    # NOTE: Since May 2025, `spatialjoin` no longer supports cutouts because
    # they do not work easily with `libgeos`, and we want to compare with
    # `libgeos`.
    all_options = [
        ("b", "--no-box-ids"),
        # ("c", "--no-cutouts"),
        ("s", "--no-surface-area"),
        ("d", "--no-diag-box"),
        ("o", "--no-oriented-envelope"),
        ("I", "--use-inner-outer"),
    ]
    options = [all_options[int(i)] for i in args.option_indexes.split(",")]

    # The combinations from `--combinations` as a set.
    combinations = set(args.combinations.split(","))

    # Create empty log file (clear it if it already exists).
    if args.log_file:
        log_file_name = args.log_file
    else:
        log_file_name = f"{args.basename}.spatialjoin-evaluation.tsv"
    log_file = open(log_file_name, "w")
    log_file.close()

    # Try all combinations of the options.
    count = 0
    for name, combination_options in all_combinations(options):
        # Consider only combinations compatible with `args.combinations`.
        if args.combinations != "ALL" and name not in combinations:
            continue

        # Helper lambda that returns the option from the first arguments if
        # the second argument is `True`, otherwise returns an empty string.
        def option(name: str, condition: bool) -> str:
            return f" {name}" if condition else ""

        # The command line for this combination.
        cmd = (
            f"cat {args.basename}.spatialjoin-input.tsv | "
            f"{args.spatialjoin_binary}"
            f" --num-threads {args.num_threads}"
            f"{option('--de9im', args.use_de9im == 'true')}"
            f"{option('--no-fast-sweep-skip', args.no_fast_sweep_skip)}"
            f"{option('--libgeos', args.core_library == 'libgeos')}"
            f" {combination_options}"
        )

        # Optionally, generate RDF output.
        if args.rdf_output:
            cmd += (
                ' --contains " ogc:sfContains "'
                ' --covers " ogc:sfCovers "'
                ' --intersects " ogc:sfIntersects "'
                ' --equals " ogc:sfEquals "'
                ' --touches " ogc:sfTouches "'
                ' --crosses " ogc:sfCrosses "'
                ' --overlaps " ogc:sfOverlaps "'
                " --suffix $' .\\n'"
            )

        # Optionally, tee stderr to the log file.
        if log_file_name:
            count += 1
            with open(log_file_name, "a") as log_file:
                log_file.write(
                    f"\n"
                    f"### Combination {count}\n"
                    f"### {name}\n"
                    f"### {cmd} > /dev/null\n"
                    f"\n"
                )
            cmd += f" 2> >(tee -a {log_file_name} >&2)"

        # Only show the commands?
        if args.only_show_commands:
            print(f"{name}\t{cmd}")
            continue

        # Run the command and time it.
        start = time.time()
        result = subprocess.run(
            cmd,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        end = time.time()
        total_time = f"{end - start:.3f}"

        # If non-zero exit code, print the error message
        if result.returncode != 0:
            error_message = [
                line
                for line in result.stderr.decode().split("\n")
                if " INFO : " not in line
            ]
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


def read_results(args: argparse.Namespace) -> list[tuple[str, float]]:
    """
    Read the results from file `{args.basename}.spatialjoin-evaluation.tsv`.
    """

    results = []
    with open(f"{args.basename}.spatialjoin-evaluation.tsv") as file:
        for line in file:
            if line.startswith("#"):
                continue
            name, total_time, parse_time, sweep_time = line.strip().split("\t")
            time = total_time
            if args.analyze == "parse":
                time = parse_time
            elif args.analyze == "sweep":
                time = sweep_time
            results.append((name, float(time)))
    return results


def analyze_selected(args: argparse.Namespace):
    """
    Show results for the combinations given by `args.combinations`.
    """

    # Read the results from file and convert them to a dictionary.
    results = read_results(args)
    results = dict(results)

    first_name, first_time, previous_time = None, None, None
    names = args.combinations.split(",")
    for i, name in enumerate(names):
        if name not in results:
            print(f"ERROR: no result for {name}")
            continue
        time = results[name]
        if first_name is None:
            first_name, first_time = name, time
            if args.minutes:
                print(f"{name} -> {time/60:6.1f} min")
            else:
                print(f"{name} -> {time:6.1f} s")
        else:
            # Find previous to compare with (must have prefix in common,
            # depending on how you go back in the list of `names`).
            for j in range(i):
                if names[j][:j] == name[:j]:
                    previous_name = names[j]
            previous_time = results[previous_name]
            # Show the speedup.
            if args.minutes:
                print(f"{name} -> {time/60:6.1f} min, ", end="")
            else:
                print(f"{name} -> {time:6.1f} s, ", end="")
            print(
                f"{previous_time / time:5.2f}x speedup over {previous_name}"
                f" ({first_time / time:5.2f}x over {first_name})"
            )


def analyze_all(args: argparse.Namespace):
    """
    Analyze the results from a previous run.
    """

    # Read the results from file.
    results = read_results(args)

    # First, show the maximal and minimal duration overall.
    sorted_results = sorted(results, key=lambda pair: pair[1])
    min_name, min_duration = sorted_results[0]
    max_name, max_duration = sorted_results[-1]
    med_name, med_duration = sorted_results[len(sorted_results) // 2]
    print(
        f"Overall stats  : "
        f"min {min_duration:.1f}s ({min_name}), "
        f"max {max_duration:.1f}s ({max_name}), "
        f"median {med_duration:.1f}s ({med_name}), "
        f"max/min = {max_duration / min_duration:.1f}x"
    )
    print()

    # Best k options, for each fixed k.
    best = {}
    for name, seconds in results:
        k = sum(1 for c in name if c.isupper())
        if k not in best:
            best[k] = (name, seconds)
        else:
            if seconds < best[k][1]:
                best[k] = (name, seconds)
    baseline_name = best[0][0]
    baseline_time = best[0][1]
    print(f"Baseline      : {baseline_name} -> {baseline_time:6.1f}s")
    for k in sorted(best.keys()):
        if k > 0:
            name, seconds = best[k]
            previous_name, previous_time = best[k - 1]
            print(
                f"Best {k} {'option ' if k == 1 else 'options'}: "
                f"{name} -> {seconds:6.1f}s, "
                f"{baseline_time / seconds:4.1f}x speedup over baseline"
                f" ({previous_time / seconds:3.1f}x over previous)"
            )

    print()

    # A short human-readable description for each option.
    #
    # NOTE: See above, regarding the cutouts option.
    descriptions = {
        0: "box ids",
        1: "surface area",
        # 2: "cutouts",
        2: "diagonal boxes",
        3: "oriented boxes",
        4: "inner/outer boxes",
    }

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
            key = name[:i] + name[i + 1 :] + name[i]
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
            assert name1[option_index + 1 :] == name2[option_index + 1 :]
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
        min_speedup_names = (
            f"{sorted_results[min_speedup_index + 1][0]} -> "
            f"{sorted_results[min_speedup_index][0]}"
        )
        max_speedup_names = (
            f"{sorted_results[max_speedup_index + 1][0]} -> "
            f"{sorted_results[max_speedup_index][0]}"
        )
        print(
            f"{description:15}: "
            f"max speedup {max_speedup:4.2f}x ({max_speedup_names}), "
            f"min speedup {min_speedup:4.2f}x ({min_speedup_names})"
        )


if __name__ == "__main__":
    # Get the basenames of all files of the form `*.spatialjoin-input.tsv` or
    # `*.spatialjoin-evaluation.tsv` in the current directory.
    basenames = set()
    search_paths = list(Path(".").glob("*.spatialjoin-input.tsv")) + list(
        Path(".").glob("*.spatialjoin-evaluation.tsv")
    )
    for p in search_paths:
        basenames.add(re.sub("\\.spatialjoin-[a-z]+\\.tsv*$", "", p.name))

    # By default, assume that the `spatialjoin` repo is checked out, that the
    # script is run from `/path/to/repo/scripts/` and that the `spatialjoin`
    # binary is located at `/path/to/repo/build/spatialjoin`.
    script_path = Path(__file__).resolve()
    spatialjoin_binary_path = script_path.parent.parent / "build" / "spatialjoin"

    # Parse command line arguments.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "basename", help="basename of the dataset"
    ).completer = ChoicesCompleter(list(basenames))
    parser.add_argument(
        "--num-threads",
        type=int,
        default=28,
        help="Number of threads to use in call of `spatialjoin` (default: 28)",
    )
    parser.add_argument(
        "--use-de9im",
        choices=["true", "false"],
        default="true",
        help="Use DE-9IM for spatial relations (default: true)",
    )
    parser.add_argument(
        "--core-library",
        choices=["ours", "libgeos"],
        default="ours",
        help="Use our core library or `libgeos` (default: `ours`)",
    )
    parser.add_argument(
        "--spatialjoin-binary",
        type=str,
        default=str(spatialjoin_binary_path),
        help="Path to the `spatialjoin` binary, default is "
        f"{spatialjoin_binary_path}",
    )
    parser.add_argument(
        "--only-show-commands",
        action="store_true",
        default=False,
        help="Only show the commands that would be executed",
    )
    parser.add_argument(
        "--analyze",
        choices=["total", "parse", "sweep"],
        default=None,
        help="Analyze the specifed time" " (reads file produced by previous run)",
    )
    parser.add_argument(
        "--option-indexes",
        type=str,
        default="0,1,2,3,4",
        help="Comma-separated list of option indexes " "(default: 0,1,2,3,4,5)",
    )
    parser.add_argument(
        "--no-fast-sweep-skip",
        action="store_true",
        default=False,
        help="Call spatialjoin with --no-fast-sweep-skip",
    )
    combinations = ["ALL", "bsdoi,Bsdoi,BSdoi,BSDoi,BSdOi,BSdoI"]
    parser.add_argument(
        "--combinations",
        type=str,
        default="ALL",
        help="Compute / analyze only these " "combinations",
    ).completer = ChoicesCompleter(combinations)
    parser.add_argument(
        "--log-file",
        type=str,
        help="Name of log file (default: <basename>.spatialjoin-evaluation.log)",
    )
    parser.add_argument(
        "--rdf-output", action="store_true", default=False, help="Generate RDF output"
    )
    parser.add_argument(
        "--minutes",
        action="store_true",
        default=False,
        help="Show times in minutes instead of seconds",
    )
    argcomplete.autocomplete(parser, always_complete_options="long")
    args = parser.parse_args()

    # Evaluate `spatialjoin` for the given dataset, for all combinations of the
    # `options` above.
    if args.analyze:
        if args.combinations == "ALL":
            analyze_all(args)
        else:
            analyze_selected(args)
    else:
        compute(args)
