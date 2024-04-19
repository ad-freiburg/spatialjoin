// Copyright 2023, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include <iostream>

#include "BoxIds.h"
#include "Sweeper.h"
#include "WKTParse.h"
#include "util/Misc.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

using sj::Sweeper;
using util::geo::DLine;
using util::geo::DPoint;
using util::geo::I32Line;
using util::geo::I32MultiLine;
using util::geo::I32MultiPolygon;
using util::geo::I32Point;
using util::geo::I32Polygon;

static const char* YEAR = &__DATE__[7];
static const char* COPY =
    "University of Freiburg - Chair of Algorithms and Data Structures";
static const char* AUTHORS = "Patrick Brosi <brosi@informatik.uni-freiburg.de>";

// _____________________________________________________________________________
void printHelp(int argc, char** argv) {
  UNUSED(argc);
  std::cout
      << "\n"
      << "(C) 2023-" << YEAR << " " << COPY << "\n"
      << "Authors: " << AUTHORS << "\n\n"
      << "Usage: " << argv[0] << " [--help] [-h] <input>\n\n"
      << "Allowed options:\n\n"
      << std::setfill(' ') << std::left << "General:\n"
      << std::setw(41) << "  -h [ --help ]"
      << "show this help message\n"
      << std::setw(41) << "  -o [ --output ] (default: '')"
      << "output file (.bz2 or .gz supported), empty prints to stdout\n"
      << std::setw(41) << "  -c [ --cache ] (default: '.')"
      << "cache directory for intermediate files\n"
      << std::setw(41) << "  -C"
      << "don't parse input, re-use intermediate cache files\n"
      << std::setw(41) << "  --prefix (default: '')"
      << "prefix added at the beginning of every relation\n"
      << std::setw(41) << "  --intersects (default: ' intersects ')"
      << "separator between intersecting geometry IDs\n"
      << std::setw(41) << "  --contains (default: ' contains ')"
      << "separator between containing geometry IDs\n"
      << std::setw(41) << "  --covers (default: ' covers ')"
      << "separator between covering geometry IDs\n"
      << std::setw(41) << "  --suffix (default: '\\n')"
      << "suffix added at the beginning of every relation\n\n"
      << std::setfill(' ') << std::left << "Geometric computation:\n"
      << std::setw(41) << "  --no-box-ids"
      << "disable box id criteria for contains/covers/intersect computation\n"
      << std::setw(41) << "  --no-surface-area"
      << "disable surface area criteria for polygon contains/covers\n"
      << std::endl;
}

// _____________________________________________________________________________
int main(int argc, char** argv) {
  // disable output buffering for standard output
  setbuf(stdout, NULL);

  // initialize randomness
  srand(time(NULL) + rand());  // NOLINT

  bool useCache = false;

  int state = 0;

  std::string prefix = "";
  std::string output = "";
  std::string cache = ".";
  std::string contains = " contains ";
  std::string intersects = " intersects ";
  std::string covers = " covers ";
  std::string suffix = "\n";

  bool useBoxIds = true;
  bool useArea = true;

  for (int i = 1; i < argc; i++) {
    std::string cur = argv[i];
    switch (state) {
      case 0:
        if (cur == "-h" || cur == "--help") {
          printHelp(argc, argv);
          exit(0);
        }
        if (cur == "-C") {
          useCache = true;
        }
        if (cur == "--prefix") {
          state = 1;
        }
        if (cur == "--contains") {
          state = 2;
        }
        if (cur == "--intersects") {
          state = 3;
        }
        if (cur == "--suffix") {
          state = 4;
        }
        if (cur == "--output" || cur == "-o") {
          state = 5;
        }
        if (cur == "--cache" || cur == "-c") {
          state = 6;
        }
        if (cur == "--covers") {
          state = 7;
        }
        if (cur == "--no-box-ids") {
          useBoxIds = false;
        }
        if (cur == "--no-surface-area") {
          useArea = false;
        }
        break;
      case 1:
        prefix = cur;
        state = 0;
        break;
      case 2:
        contains = cur;
        state = 0;
        break;
      case 3:
        intersects = cur;
        state = 0;
        break;
      case 4:
        suffix = cur;
        state = 0;
        break;
      case 5:
        output = cur;
        state = 0;
        break;
      case 6:
        cache = cur;
        state = 0;
        break;
      case 7:
        covers = cur;
        state = 0;
        break;
    }
  }

  char* buf = new char[1024 * 1024 * 100];
  size_t len;
  std::string dangling;
  size_t gid = 0;

  size_t NUM_THREADS = std::thread::hardware_concurrency();

  Sweeper sweeper({NUM_THREADS, prefix, intersects, contains, covers, suffix,
                   useBoxIds, useArea},
                  useCache, cache, output);

  if (!useCache) {
    LOGTO(INFO, std::cerr) << "Parsing input geometries...";

    while ((len = read(0, buf, 1024 * 1024 * 100)) > 0) {
      parse(buf, len, dangling, &gid, sweeper);
    }

    sweeper.flush();
  }

  LOGTO(INFO, std::cerr) << "done.";

  LOGTO(INFO, std::cerr) << "Sweeping...";
  sweeper.sweep();
  LOGTO(INFO, std::cerr) << "done.";
}
