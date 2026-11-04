// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

// Integration example for a non-self join between two parts of a single
// dataset, held in a single storage

#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "spatialjoin/GeometryCache.h"
#include "spatialjoin/GeometryStore.h"
#include "spatialjoin/OutputWriter.h"
#include "spatialjoin/SpatialJoinCfg.h"
#include "spatialjoin/SweepEventList.h"
#include "spatialjoin/Sweeper.h"
#include "spatialjoin/WKTParse.h"

// _____________________________________________________________________________
int main(int argc, char** argv) {
  const std::string input =
      argc > 1 ? argv[1] : EXAMPLE_DATASET_DIR "/freiburg";

  const std::string dir = ".";
  const std::string prefix = "spatialjoin";

  sj::SpatialJoinCfg cfg;

  sj::OutputWriter out(cfg.numThreads, "", "\n", "", ".");

  // called by sweeper for each relation found (by thread t)
  cfg.writeRelCb = [&out](size_t t, const char* a, size_t an, const char* b,
                          size_t bn, const char* pred, size_t predn) {
    out.writeRelCb(t, a, an, b, bn, pred, predn);
  };

  sj::GeometryStore store(cfg, dir, prefix, false);

  sj::GeometryCache cache(cfg);
  const size_t storeId = cache.addStore(&store);

  // collect the PreparedGeomMetas in our own vector first
  std::vector<sj::PreparedGeomMeta> metas;
  std::mutex metasMtx;
  sj::WKTParser parser(
      &store,
      [&metas, &metasMtx](size_t, const sj::PreparedGeomMeta& meta) {
        std::lock_guard<std::mutex> lock(metasMtx);
        metas.push_back(meta);
      },
      cfg.numThreads);

  try {
    parser.parseFile(input);
  } catch (const std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  store.flush();  // necessary!

  sj::SweepEventList eventsLeft(cfg, dir, prefix);
  sj::SweepEventList eventsRight(cfg, dir, prefix);

  // first half is left side, second half is right side
  for (size_t i = 0; i < metas.size(); i++) {
    if (i < metas.size() / 2) {
      eventsLeft.add(metas[i], storeId);
    } else {
      eventsRight.add(metas[i], storeId);
    }
  }

  // necessary, this sorts the event files
  eventsLeft.finalize(cfg.numThreads);
  eventsRight.finalize(cfg.numThreads);

  // this is optional
  eventsLeft.duplicatesToReferences(cache);
  eventsRight.duplicatesToReferences(cache);

  sj::Sweeper sweeper(cfg, &cache);
  sweeper.sweep(&eventsLeft, &eventsRight);
}
