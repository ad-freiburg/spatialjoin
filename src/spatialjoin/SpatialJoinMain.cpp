// Copyright 2023, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include <iostream>

#include "BoxIds.h"
#include "OutputWriter.h"
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
using util::geo::DE9IMFilter;
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
static const size_t DEFAULT_CACHE_SIZE = 1000 * 1000 * 1000;
static const size_t DEFAULT_CACHE_NUM_ELEMENTS = 10000;

// _____________________________________________________________________________
void printHelp(int argc, char** argv) {
  UNUSED(argc);
  std::cerr
      << "\n"
      << "(C) 2023-" << YEAR << " " << COPY << "\n"
      << "Authors: " << AUTHORS << "\n\n"
      << "Usage: " << argv[0] << " [OPTIONS] [INPUT1] [INPUT1]\n\n"
      << "With input from either stdin, or from file(s) [INPUT1] and [INPUT2] "
         "(.bz2 or .gz\nsupported). If "
      << "both [INPUT1] and [INPUT2] are given, compute a non-self join with\n"
      << "[INPUT1] on the left side, [INPUT2] on the right side.\n\n"
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
      << std::setw(42) << "  --within-distance (default: '-1')"
      << "if set to non-negative value, only compute for each object\n"
      << std::setw(42) << " "
      << "the objects within the given distance\n\n"
      << std::setfill(' ') << std::left << "Formatting:\n"
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
      << std::setfill(' ') << std::left << "Geometric computation:\n"
      << std::setw(42) << "  --no-box-ids"
      << "disable box id criteria for contains/covers/intersect\n"
      << std::setw(42) << " "
      << "computation\n"
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
      << "number of geometry caches (if < --num-threads, syncing)\n"
      << std::setw(42)
      << "  --cache-max-size (default: " + std::to_string(DEFAULT_CACHE_SIZE) +
             ")"
      << "maximum approx. size in bytes of cache per type and\n"
      << std::setw(42) << " " << "thread, 0 = unlimited\n"
      << std::setw(42)
      << "  --cache-max-elements (default: " +
             std::to_string(DEFAULT_CACHE_NUM_ELEMENTS) + ")"
      << "maximum number of elements per cache, type and thread,\n"
      << std::setw(42) << " " << "0 = unlimited\n"
      << std::setw(42) << "  --no-geometry-checks"
      << "do not compute geometric relations, only report number of\n"
      << std::setw(42) << " "
      << "candidates\n"
      << std::setw(42) << "  --stats"
      << "output stats\n"
      << std::setw(42) << "  -v [ --verbose ]"
      << "verbose logging\n"
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

  bool printStats = false;
  bool verbose = false;

  size_t numThreads = NUM_THREADS;
  size_t numCaches = NUM_THREADS;
  size_t geomCacheMaxSizeBytes = DEFAULT_CACHE_SIZE;
  size_t geomCacheMaxNumElements = DEFAULT_CACHE_NUM_ELEMENTS;
  DE9IMFilter de9imFilter;

  std::vector<std::string> inputFiles;

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
        } else if (cur == "--cache-max-elements") {
          state = 16;
        } else if (cur == "--de9im-filter") {
          state = 17;
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
        } else if (cur == "--stats") {
          printStats = true;
        } else if (cur == "--verbose" || cur == "-v") {
          verbose = true;
        } else {
          inputFiles.push_back(cur);
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
      case 16:
        std::stringstream(cur) >> geomCacheMaxNumElements;
        state = 0;
        break;
      case 17:
        if (cur.size() < 9) cur.insert(cur.size(), 9 - cur.size(), '*');
        std::cout << cur << std::endl;
        de9imFilter = cur.c_str();
        state = 0;
        break;
    }
  }

  const static size_t CACHE_SIZE = 1024 * 1024;
  unsigned char* buf = new unsigned char[CACHE_SIZE];
  size_t len;

  sj::OutputWriter outWriter(numThreads, prefix, suffix, output, cache);

  std::function<void(size_t, const char*, size_t, const char*, size_t,
                     const char*, size_t)>
      writeRelCb;

  if (outWriter.getOutMode() != sj::OutMode::NONE)
    writeRelCb = [&outWriter](size_t t, const char* a, size_t an, const char* b,
                              size_t bn, const char* pred, size_t predn) {
      outWriter.writeRelCb(t, a, an, b, bn, pred, predn);
    };

  sj::SweeperCfg sweeperCfg{numThreads,
                            numCaches,
                            geomCacheMaxSizeBytes,
                            geomCacheMaxNumElements,
                            intersects,
                            contains,
                            covers,
                            touches,
                            equals,
                            overlaps,
                            crosses,
                            useBoxIds,
                            useArea,
                            useOBB,
                            useDiagBox,
                            useFastSweepSkip,
                            useInnerOuter,
                            noGeometryChecks,
                            withinDist,
                            computeDE9IM,
                            de9imFilter,
                            writeRelCb,
                            {},
                            {},
                            {},
                            {}};

  if (printStats)
    sweeperCfg.statsCb = [](const std::string& s) { std::cerr << s; };

  if (verbose)
    sweeperCfg.logCb = [](const std::string& s) {
      LOGTO(INFO, std::cerr) << s;
    };

  Sweeper sweeper(sweeperCfg, cache);

  sweeper.log("Parsing input geometries...");
  auto ts = TIME();

  sj::WKTParser parser(&sweeper, NUM_THREADS);

  if (!inputFiles.empty()) {
    if (inputFiles.size() > 2) {
      std::cerr << "Either 1 input files (for self join), or 2 input files "
                   "(for non-self join) can be provided."
                << std::endl;
      exit(1);
    }
    for (size_t i = 0; i < inputFiles.size(); i++) {
      if (util::endsWith(inputFiles[i], ".bz2")) {
#ifndef SPATIALJOIN_NO_BZIP2
        auto fh = fopen(inputFiles[i].c_str(), "r");
        if (!fh) {
          std::cerr << "Could not open input file " << inputFiles[i]
                    << std::endl;
          exit(1);
        }
        int err;
        BZFILE* f = BZ2_bzReadOpen(&err, fh, 0, 0, NULL, 0);
        if (!f || err != BZ_OK) {
          std::cerr << "Could not open input file " << inputFiles[i]
                    << std::endl;
          exit(1);
        }
        while ((len = util::bz2readAll(f, buf, CACHE_SIZE)) > 0) {
          parser.parse(reinterpret_cast<char*>(buf), len, i != 0);
        }

        BZ2_bzReadClose(&err, f);
        fclose(fh);
#else
        std::cerr << "Could not open input file " << inputFiles[i]
                  << ", spatialjoin was compiled without BZip2 support"
                  << std::endl;
        exit(1);
#endif
      } else if (util::endsWith(inputFiles[i], ".gz")) {
#ifndef SPATIALJOIN_NO_ZLIB
        gzFile f = gzopen(inputFiles[i].c_str(), "r");
        if (f == Z_NULL) {
          std::cerr << "Could not open input file " << inputFiles[i]
                    << std::endl;
          exit(1);
        }
        while ((len = util::zreadAll(f, buf, CACHE_SIZE)) > 0) {
          parser.parse(reinterpret_cast<char*>(buf), len, i != 0);
        }

        gzclose(f);
#else
        std::cerr << "Could not open input file " << inputFiles[i]
                  << ", spatialjoin was compiled without gzip support"
                  << std::endl;
        exit(1);
#endif
      } else {
        int f = open(inputFiles[i].c_str(), O_RDONLY);

        if (f < 0) {
          std::cerr << "Could not open input file " << inputFiles[i]
                    << std::endl;
          exit(1);
        }

        while ((len = util::readAll(f, buf, CACHE_SIZE)) > 0) {
          parser.parse(reinterpret_cast<char*>(buf), len, i != 0);
        }

        close(f);
      }
    }
  } else {
    while ((len = util::readAll(0, buf, CACHE_SIZE)) > 0) {
      parser.parse(reinterpret_cast<char*>(buf), len, 0);
    }
  }

  parser.done();

  sweeper.log("Done parsing (" + std::to_string(TOOK(ts) / 1000000000.0) +
              "s).");
  ts = TIME();

  sweeper.log("Sorting sweep events...");

  sweeper.flush();

  sweeper.log("done (" + std::to_string(TOOK(ts) / 1000000000.0) + "s).");

  sweeper.log("Sweeping...");
  ts = TIME();
  sweeper.sweep();
  sweeper.log("done (" + std::to_string(TOOK(ts) / 1000000000.0) + "s).");

  delete[] buf;
}
