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

using sj::ParseBatch;
using sj::Sweeper;
using util::geo::DLine;
using util::geo::DPoint;
using util::geo::I32Line;
using util::geo::I32MultiLine;
using util::geo::I32MultiPolygon;
using util::geo::I32Point;
using util::geo::I32Polygon;
using util::LogLevel::DEBUG;
using util::LogLevel::ERROR;
using util::LogLevel::INFO;
using util::LogLevel::VDEBUG;
using util::LogLevel::WARN;

static const char* YEAR = &__DATE__[7];
static const char* COPY =
    "University of Freiburg - Chair of Algorithms and Data Structures";
static const char* AUTHORS = "Patrick Brosi <brosi@informatik.uni-freiburg.de>";

static const size_t NUM_THREADS = std::thread::hardware_concurrency();
static const size_t DEFAULT_CACHE_SIZE = 300 * 1000 * 1000 * (NUM_THREADS * 3);

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
      << std::setw(42) << "  -h [ --help ]"
      << "show this help message\n"
      << std::setw(42) << "  -o [ --output ] (default: '')"
      << "output file (.bz2 or .gz supported), empty prints to stdout\n"
      << std::setw(42) << "  -c [ --cache ] (default: '.')"
      << "cache directory for intermediate files\n"
      << std::setw(42) << "  --de9im"
      << "output DE-9IM relationships\n"
      << std::setw(42) << "  --prefix (default: '')"
      << "prefix added at the beginning of every relation\n"
      << std::setw(42) << "  --intersects (default: ' intersects ')"
      << "separator between intersecting geometry IDs\n"
      << std::setw(42) << "  --contains (default: ' contains ')"
      << "separator between containing geometry IDs\n"
      << std::setw(42) << "  --covers (default: ' covers ')"
      << "separator between covering geometry IDs\n"
      << std::setw(42) << "  --touches (default: ' touches ')"
      << "separator between touching geometry IDs\n"
      << std::setw(42) << "  --equals (default: ' equals ')"
      << "separator between equivalent geometry IDs\n"
      << std::setw(42) << "  --overlaps (default: ' overlaps ')"
      << "separator between overlapping geometry IDs\n"
      << std::setw(42) << "  --crosses (default: ' crosses ')"
      << "separator between crossing geometry IDs\n"
      << std::setw(42) << "  --suffix (default: '\\n')"
      << "suffix added at the beginning of every relation\n\n"
      << std::setw(42) << "  --within-distance (default: '')"
      << "if set to non-negative value, only compute for each object the "
         "objects within the given distance\n\n"
      << std::setfill(' ') << std::left << "Geometric computation:\n"
      << std::setw(42) << "  --no-box-ids"
      << "disable box id criteria for contains/covers/intersect computation\n"
      << std::setw(42) << "  --no-surface-area"
      << "disable surface area criteria for polygon contains/covers\n"
      << std::setw(42) << "  --no-oriented-envelope"
      << "disable oriented envelope cirteria for contains/intersect\n"
      << std::setw(42) << "  --no-diag-box"
      << "disable diagonal bounding-box based pre-filter\n"
      << std::setw(42) << "  --no-fast-sweep-skip"
      << "disable fast sweep skip using binary search\n"
      << std::setw(42) << "  --use-inner-outer"
      << "(experimental) use inner/outer geometries\n\n"
      << std::setfill(' ') << std::left << "Misc:\n"
      << std::setw(42)
      << "  --num-threads (default: " + std::to_string(NUM_THREADS) + ")"
      << "number of threads for geometric computation\n"
      << std::setw(42)
      << "  --num-caches (default: " + std::to_string(NUM_THREADS) + ")"
      << "number of geometry caches (if < --num-threads, syncing\n"
      << std::setw(42)
      << "  --cache-max-size (default: " + std::to_string(DEFAULT_CACHE_SIZE) + ")"
      << "maximum approx. size in bytes of cache per type and thread\n"
      << std::setw(42) << "  --no-geometry-checks"
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
  double withinDist = -1;

  bool useBoxIds = true;
  bool useArea = true;
  bool useOBB = true;
  bool useDiagBox = true;
  bool useFastSweepSkip = true;
  bool useInnerOuter = false;
  bool noGeometryChecks = false;
  bool computeDE9IM = false;

  size_t numThreads = NUM_THREADS;
  size_t numCaches = NUM_THREADS;
  size_t geomCacheMaxSizeBytes = DEFAULT_CACHE_SIZE;

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
        } else if (cur == "--cache-max-size") {
          state = 14;
        } else if (cur == "--within-distance") {
          state = 15;
        } else if (cur == "--de9im") {
          computeDE9IM = true;
        } else if (cur == "--no-box-ids") {
          useBoxIds = false;
        } else if (cur == "--no-surface-area") {
          useArea = false;
        } else if (cur == "--no-oriented-envelope") {
          useOBB = false;
        } else if (cur == "--no-diag-box") {
          useDiagBox = false;
        } else if (cur == "--no-geometry-checks") {
          noGeometryChecks = true;
        } else if (cur == "--no-fast-sweep-skip") {
          useFastSweepSkip = false;
        } else if (cur == "--use-inner-outer") {
          useInnerOuter = true;
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
      case 14:
        std::stringstream(cur) >> geomCacheMaxSizeBytes;
        state = 0;
        break;
      case 15:
        withinDist = atof(cur.c_str());
        state = 0;
        break;
    }
  }

  const static size_t CACHE_SIZE = 1024 * 1024 * 100;
  unsigned char* buf = new unsigned char[CACHE_SIZE];
  size_t len;

  Sweeper sweeper({numThreads,
                   numCaches,
                   geomCacheMaxSizeBytes / (numThreads * 3),
                   prefix,
                   intersects,
                   contains,
                   covers,
                   touches,
                   equals,
                   overlaps,
                   crosses,
                   suffix,
                   useBoxIds,
                   useArea,
                   useOBB,
                   useDiagBox,
                   useFastSweepSkip,
                   useInnerOuter,
                   noGeometryChecks,
                   withinDist,
                   computeDE9IM,
                   {},
                   [](const std::string& s) { LOGTO(INFO, std::cerr) << s; },
                   [](const std::string& s) { std::cerr << s; },
                   {},
                   {}},
                  cache, output);

  LOGTO(INFO, std::cerr) << "Parsing input geometries...";
  auto ts = TIME();

  sj::WKTParser parser(&sweeper, NUM_THREADS);

  while ((len = util::readAll(0, buf, CACHE_SIZE)) > 0) {
    parser.parse(reinterpret_cast<char*>(buf), len, 0);
  }

  parser.done();

  LOGTO(INFO, std::cerr) << "Done parsing ("
                         << TOOK(ts) / 1000000000.0 << "s).";
  ts = TIME();

  LOGTO(INFO, std::cerr) << "Sorting sweep events...";

  sweeper.flush();

  LOGTO(INFO, std::cerr) << "done (" << TOOK(ts) / 1000000000.0 << "s).";

  LOGTO(INFO, std::cerr) << "Sweeping...";
  ts = TIME();
  sweeper.sweep();
  LOGTO(INFO, std::cerr) << "done (" << TOOK(ts) / 1000000000.0 << "s).";

  delete[] buf;
}
