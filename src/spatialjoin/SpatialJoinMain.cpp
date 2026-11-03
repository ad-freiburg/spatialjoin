// Copyright 2023, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Authors: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#include <iostream>
#include <memory>
#include <mutex>

#include "BoxIds.h"
#include "GeometryCache.h"
#include "OutputWriter.h"
#include "SpatialJoinCfg.h"
#include "Sweeper.h"
#include "WKTParse.h"
#include "util/Misc.h"
#include "util/geo/Geo.h"
#include "util/log/Log.h"

using sj::GeometryCache;
using sj::ParseBatch;
using sj::Sweeper;
using util::geo::DE9IMFilter;
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

static const sj::SpatialJoinCfg DEFAULT_CFG;
static const size_t NUM_THREADS = DEFAULT_CFG.numThreads;
static const size_t DEFAULT_CACHE_SIZE = DEFAULT_CFG.geomCacheMaxSize;
static const size_t DEFAULT_CACHE_NUM_ELEMENTS =
    DEFAULT_CFG.geomCacheMaxNumElements;

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
      << std::setw(42) << "  --de9im-filter"
      << "only output relations which match given DE-9IM filter (has\n"
      << std::setw(42) << " "
      << "no effect for --within-distance)\n"
      << std::setw(42) << "  --within-dist[ance] (default: -1)"
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
      << std::setw(42) << "  --no-diag-box"
      << "disable diagonal bounding-box based pre-filter\n"
      << std::setw(42) << "  --no-fast-sweep-skip"
      << "disable fast sweep skip using binary search\n\n"
      << std::setfill(' ') << std::left << "Misc:\n"
      << std::setw(42) << "  --wkt-strict"
      << "use strict WKT parsing\n"
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
      << std::setw(42) << " "
      << "thread, 0 = unlimited\n"
      << std::setw(42)
      << "  --cache-max-elements (default: " +
             std::to_string(DEFAULT_CACHE_NUM_ELEMENTS) + ")"
      << "maximum number of elements per cache, type and thread,\n"
      << std::setw(42) << " "
      << "0 = unlimited\n"
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
  // initialize randomness
  srand(time(NULL) + rand());  // NOLINT

  int state = 0;

  std::string prefix = "";
  std::string output = "";
  std::string cacheDir = ".";
  std::string contains = DEFAULT_CFG.sepContains;
  std::string intersects = DEFAULT_CFG.sepIsect;
  std::string covers = DEFAULT_CFG.sepCovers;
  std::string touches = DEFAULT_CFG.sepTouches;
  std::string equals = DEFAULT_CFG.sepEquals;
  std::string overlaps = DEFAULT_CFG.sepOverlaps;
  std::string crosses = DEFAULT_CFG.sepCrosses;
  std::string suffix = "\n";
  double withinDist = DEFAULT_CFG.withinDist;
  bool euclideanDist = DEFAULT_CFG.euclideanDist;
  bool haversineApprox = DEFAULT_CFG.haversineApprox;

  bool useBoxIds = DEFAULT_CFG.useBoxIds;
  bool useArea = DEFAULT_CFG.useArea;
  bool useDiagBox = DEFAULT_CFG.useDiagBox;
  bool useFastSweepSkip = DEFAULT_CFG.useFastSweepSkip;
  bool noGeometryChecks = DEFAULT_CFG.noGeometryChecks;
  bool computeDE9IM = DEFAULT_CFG.computeDE9IM;
  bool wktStrict = false;

  bool printStats = false;
  bool verbose = false;

  size_t numThreads = NUM_THREADS;
  size_t numCaches = DEFAULT_CFG.numCacheThreads;
  size_t geomCacheMaxSizeBytes = DEFAULT_CACHE_SIZE;
  size_t geomCacheMaxNumElements = DEFAULT_CACHE_NUM_ELEMENTS;
  DE9IMFilter de9imFilter = DEFAULT_CFG.de9imFilter;

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
        } else if (cur == "--within-dist") {
          state = 15;
        } else if (cur == "--cache-max-elements") {
          state = 16;
        } else if (cur == "--de9im-filter") {
          state = 17;
        } else if (cur == "--de9im") {
          computeDE9IM = true;
        } else if (cur == "--wkt-strict") {
          wktStrict = true;
        } else if (cur == "--no-box-ids") {
          useBoxIds = false;
        } else if (cur == "--no-surface-area") {
          useArea = false;
        } else if (cur == "--no-diag-box") {
          useDiagBox = false;
        } else if (cur == "--no-geometry-checks") {
          noGeometryChecks = true;
        } else if (cur == "--no-fast-sweep-skip") {
          useFastSweepSkip = false;
        } else if (cur == "--euclidean-dist") {
          euclideanDist = true;
        } else if (cur == "--haversine-approx") {
          euclideanDist = true;
          haversineApprox = true;
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
        cacheDir = cur;
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
        de9imFilter = cur.c_str();
        state = 0;
        break;
    }
  }

  if (de9imFilter.maxExteriorDim() < 2) {
    if (verbose) {
      LOGTO(INFO, std::cerr) << "Skipping all comparisons because of DE-9IM "
                                "filter which will not match any pairs...";
      LOGTO(INFO, std::cerr)
          << " (max exterior dim=" << (int)de9imFilter.maxExteriorDim() << ")";
    }
    return 0;
  }

  if (de9imFilter.minLeftBoundaryDim() > 1) {
    if (verbose) {
      LOGTO(INFO, std::cerr) << "Skipping all comparisons because of DE-9IM "
                                "filter which will not match any pairs...";
      LOGTO(INFO, std::cerr)
          << " (min left boundary dim=" << (int)de9imFilter.minLeftBoundaryDim()
          << ")";
    }
    return 0;
  }

  if (de9imFilter.minRightBoundaryDim() > 1) {
    if (verbose) {
      LOGTO(INFO, std::cerr) << "Skipping all comparisons because of DE-9IM "
                                "filter which will not match any pairs...";
      LOGTO(INFO, std::cerr) << " (min right boundary dim="
                             << (int)de9imFilter.minRightBoundaryDim() << ")";
    }
    return 0;
  }

  if (de9imFilter.maxInteriorDim() < 0) {
    if (verbose) {
      LOGTO(INFO, std::cerr) << "Skipping all comparisons because of DE-9IM "
                                "filter which will not match any pairs...";
      LOGTO(INFO, std::cerr)
          << " (max interior dim=" << (int)de9imFilter.maxInteriorDim() << ")";
    }
    return 0;
  }


  sj::OutputWriter outWriter(numThreads, prefix, suffix, output, cacheDir);

  std::function<void(size_t, const char*, size_t, const char*, size_t,
                     const char*, size_t)>
      writeRelCb;

  if (outWriter.getOutMode() != sj::OutMode::NONE)
    writeRelCb = [&outWriter](size_t t, const char* a, size_t an, const char* b,
                              size_t bn, const char* pred, size_t predn) {
      outWriter.writeRelCb(t, a, an, b, bn, pred, predn);
    };

  sj::SpatialJoinCfg spatialJoinCfg{numThreads,
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
                                    useDiagBox,
                                    useFastSweepSkip,
                                    noGeometryChecks,
                                    withinDist,
                                    euclideanDist,
                                    haversineApprox,
                                    computeDE9IM,
                                    de9imFilter,
                                    writeRelCb,
                                    {},
                                    {},
                                    {},
                                    {}};

  if (printStats)
    spatialJoinCfg.statsCb = [](const std::string& s) { std::cerr << s; };

  if (verbose)
    spatialJoinCfg.logCb = [](const std::string& s) {
      LOGTO(INFO, std::cerr) << s;
    };

  GeometryCache geomCache(spatialJoinCfg);

  if (inputFiles.size() > 2) {
    std::cerr << "Either 1 input files (for self join), or 2 input files "
                 "(for non-self join) can be provided."
              << std::endl;
    exit(1);
  }

  // sweep event lists
  std::vector<std::unique_ptr<sj::SweepEventList>> sweepEvents;

  geomCache.log("Parsing input geometries...");
  auto ts = TIME();

  // geometry stores, here we use one store per side
  std::vector<std::unique_ptr<sj::GeometryStore>> geomStores;

  size_t numInvalid = 0;

  for (size_t i = 0; i < std::max<size_t>(inputFiles.size(), 1); i++) {
    sweepEvents.emplace_back(new sj::SweepEventList(cacheDir, ".spatialjoin"));

    geomStores.emplace_back(
        new sj::GeometryStore(spatialJoinCfg, cacheDir, ".spatialjoin", false));
    size_t storeId = geomCache.addStore(geomStores.back().get());

    auto* events = sweepEvents.back().get();
    std::mutex eventsMtx;
    sj::WKTParser parser(
        geomStores.back().get(),
        [events, storeId, &eventsMtx](size_t,
                                      const sj::PreparedGeomMeta& meta) {
          std::lock_guard<std::mutex> lock(eventsMtx);
          events->add(meta, storeId);
        },
        NUM_THREADS, wktStrict);

    try {
      if (inputFiles.empty()) {
        parser.parseFd(0);
      } else {
        parser.parseFile(inputFiles[i]);
      }
    } catch (const std::runtime_error& e) {
      std::cerr << e.what() << std::endl;
      exit(1);
    }

    parser.done();

    numInvalid += parser.getNumInvalid();
  }

  if (numInvalid > 0)
    geomCache.log("Note: skipped " + std::to_string(numInvalid) +
                  " invalid WKT literals");

  geomCache.log("Done parsing (" + std::to_string(TOOK(ts) / 1000000000.0) +
                "s).");

  for (auto& store : geomStores) store->flush();

  geomCache.log("Sorting events...");
  for (auto& events : sweepEvents) events->finalize(NUM_THREADS);
  geomCache.log("...done");

  for (auto& events : sweepEvents) events->duplicatesToReferences(geomCache);

  Sweeper sweeper(spatialJoinCfg, &geomCache);

  geomCache.log("Sweeping...");
  ts = TIME();
  sweeper.sweep(sweepEvents[0].get(),
                sweepEvents.size() > 1 ? sweepEvents[1].get() : 0);
  geomCache.log("done (" + std::to_string(TOOK(ts) / 1000000000.0) + "s).");


  return 0;
}
