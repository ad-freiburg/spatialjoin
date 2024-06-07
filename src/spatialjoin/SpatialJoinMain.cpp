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

static const size_t NUM_THREADS = std::thread::hardware_concurrency();

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
      // << std::setw(41) << "  -C"
      // << "don't parse input, re-use intermediate cache files\n"
      << std::setw(41) << "  --prefix (default: '')"
      << "prefix added at the beginning of every relation\n"
      << std::setw(41) << "  --intersects (default: ' intersects ')"
      << "separator between intersecting geometry IDs\n"
      << std::setw(41) << "  --contains (default: ' contains ')"
      << "separator between containing geometry IDs\n"
      << std::setw(41) << "  --covers (default: ' covers ')"
      << "separator between covering geometry IDs\n"
      << std::setw(41) << "  --touches (default: ' touches ')"
      << "separator between touching geometry IDs\n"
      << std::setw(41) << "  --equals (default: ' equals ')"
      << "separator between equivalent geometry IDs\n"
      << std::setw(41) << "  --overlaps (default: ' overlaps ')"
      << "separator between overlapping geometry IDs\n"
      << std::setw(41) << "  --crosses (default: ' crosses ')"
      << "separator between crossing geometry IDs\n"
      << std::setw(41) << "  --suffix (default: '\\n')"
      << "suffix added at the beginning of every relation\n\n"
      << std::setfill(' ') << std::left << "Geometric computation:\n"
      << std::setw(41) << "  --no-box-ids"
      << "disable box id criteria for contains/covers/intersect computation\n"
      << std::setw(41) << "  --no-surface-area"
      << "disable surface area criteria for polygon contains/covers\n"
      << std::setw(41) << "  --no-oriented-envelope"
      << "disable oriented envelope cirteria for contains/intersect\n"
      << std::setw(41) << "  --no-cutouts"
      << "disable cutouts\n"
      << std::setw(41) << "  --no-diag-box"
      << "disable diagonal bounding-box based pre-filter\n"
      << std::setw(41) << "  --no-fast-sweep-skip"
      << "disable fast sweep skip using binary search\n"
      << std::setw(41) << "  --use-inner-outer"
      << "(experimental) use inner/outer geometries\n\n"
      << std::setfill(' ') << std::left << "Misc:\n"
      << std::setw(41) << "  --pre-sort-cache"
      << "sort cache by leftmost X coordinates for higher locality\n"
      << std::setw(41)
      << "  --num-threads (default: " + std::to_string(NUM_THREADS) + ")"
      << "number of threads for geometric computation\n"
      << std::setw(41)
      << "  --num-caches (default: " + std::to_string(NUM_THREADS) + ")"
      << "number of geometry caches (if < --num-threads, syncing\n"
      << std::setw(41) << " "
      << "is done via locks)\n"
      << std::setw(41) << "  --no-geometry-checks"
      << "do not compute geometric relations, only report number of "
         "candidates\n"
      << std::endl;
}

// _____________________________________________________________________________
int main(int argc, char** argv) {
  // disable output buffering for standard output
  setbuf(stdout, NULL);

  // initialize randomness
  srand(time(NULL) + rand());  // NOLINT

  int state = 0;

  std::string prefix = "";
  std::string output = "";
  std::string cache = ".";
  std::string contains = " contains ";
  std::string intersects = " intersects ";
  std::string covers = " covers ";
  std::string touches = " touches ";
  std::string equals = " equals ";
  std::string overlaps = " overlaps ";
  std::string crosses = " crosses ";
  std::string suffix = "\n";

  bool useBoxIds = true;
  bool useArea = true;
  bool useOBB = true;
  bool useCutouts = true;
  bool useDiagBox = true;
  bool useFastSweepSkip = true;
  bool useInnerOuter = false;
  bool noGeometryChecks = false;

  bool preSortCache = false;

  size_t numThreads = NUM_THREADS;
  size_t numCaches = NUM_THREADS;

  for (int i = 1; i < argc; i++) {
    std::string cur = argv[i];
    switch (state) {
      case 0:
        if (cur == "-h" || cur == "--help") {
          printHelp(argc, argv);
          exit(0);
        }
        if (cur == "--prefix") {
          state = 1;
        } else if (cur == "--contains") {
          state = 2;
        } else if (cur == "--intersects") {
          state = 3;
        } else if (cur == "--suffix") {
          state = 4;
        } else if (cur == "--output" || cur == "-o") {
          state = 5;
        } else if (cur == "--cache" || cur == "-c") {
          state = 6;
        } else if (cur == "--covers") {
          state = 7;
        } else if (cur == "--touches") {
          state = 8;
        } else if (cur == "--equals") {
          state = 9;
        } else if (cur == "--overlaps") {
          state = 10;
        } else if (cur == "--crosses") {
          state = 11;
        } else if (cur == "--num-caches") {
          state = 12;
        } else if (cur == "--num-threads") {
          state = 13;
        } else if (cur == "--no-box-ids") {
          useBoxIds = false;
        } else if (cur == "--no-surface-area") {
          useArea = false;
        } else if (cur == "--no-oriented-envelope") {
          useOBB = false;
        } else if (cur == "--no-cutouts") {
          useCutouts = false;
        } else if (cur == "--no-diag-box") {
          useDiagBox = false;
        } else if (cur == "--no-geometry-checks") {
          noGeometryChecks = true;
        } else if (cur == "--no-fast-sweep-skip") {
          useFastSweepSkip = false;
        } else if (cur == "--use-inner-outer") {
          useInnerOuter = true;
        } else if (cur == "--pre-sort-cache") {
          preSortCache = true;
        } else {
          std::cerr << "Unknown option '" << cur << "', see -h" << std::endl;
          exit(1);
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
      case 8:
        touches = cur;
        state = 0;
        break;
      case 9:
        equals = cur;
        state = 0;
        break;
      case 10:
        overlaps = cur;
        state = 0;
        break;
      case 11:
        crosses = cur;
        state = 0;
        break;
      case 12:
        numCaches = atoi(cur.c_str());
        state = 0;
        break;
      case 13:
        numThreads = atoi(cur.c_str());
        state = 0;
        break;
    }
  }

  const static size_t CACHE_SIZE = 1024 * 1024 * 100;
  unsigned char* buf = new unsigned char[CACHE_SIZE];
  size_t len;
  std::string dangling;
  size_t gid = 1;

  Sweeper sweeper({numThreads, numCaches, prefix, intersects, contains, covers,
                   touches, equals, overlaps, crosses, suffix, useBoxIds,
                   useArea, useOBB, useCutouts, useDiagBox, useFastSweepSkip,
                   useInnerOuter, noGeometryChecks},
                  cache, output);

  LOGTO(INFO, std::cerr) << "Parsing input geometries...";
  auto ts = TIME();

  util::JobQueue<ParseBatch> jobs(1000);
  std::vector<std::thread> thrds(NUM_THREADS);
  for (size_t i = 0; i < thrds.size(); i++)
    thrds[i] = std::thread(&processQueue, &jobs, i, &sweeper);

  while ((len = util::readAll(0, buf, CACHE_SIZE)) > 0) {
    parse(reinterpret_cast<char*>(buf), len, dangling, &gid, jobs, 0);
  }

  // end event
  jobs.add({});
  // wait for all workers to finish
  for (auto& thr : thrds) thr.join();

  sweeper.flush();

  LOGTO(INFO, std::cerr) << "done (" << TOOK(ts) / 1000000000.0 << "s).";

  if (preSortCache) {
    ts = TIME();
    LOGTO(INFO, std::cerr) << "Pre-sorting cache...";
    sweeper.sortCache();
    sweeper.flush();
    LOGTO(INFO, std::cerr) << "done (" << TOOK(ts) / 1000000000.0 << "s).";
  }

  LOGTO(INFO, std::cerr) << "Sweeping...";
  ts = TIME();
  sweeper.sweep();
  LOGTO(INFO, std::cerr) << "done (" << TOOK(ts) / 1000000000.0 << "s).";

  delete[] buf;
}
