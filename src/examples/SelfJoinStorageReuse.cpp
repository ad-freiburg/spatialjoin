// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

// Integration example for a self-join on a geometry store which was persisted
// on disk before

#include <cstdio>
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
std::vector<sj::PreparedGeomMeta> writePersistedStorage(
    const sj::SpatialJoinCfg& cfg, const std::string& input,
    const std::string& dir, const std::string& prefix) {
  // note: persistOnDisk is true
  sj::GeometryStore store(cfg, dir, prefix, true);

  std::vector<sj::PreparedGeomMeta> metas;
  std::mutex metasMtx;
  sj::WKTParser parser(
      &store,
      [&metas, &metasMtx](size_t, const std::string&,
                          const sj::PreparedGeomMeta& meta) {
        std::lock_guard<std::mutex> lock(metasMtx);
        // metas are collection in a simple vector
        metas.push_back(meta);
      },
      cfg.numThreads);
  parser.parseFile(input);

  store.flush();  // necessary!

  return metas;
}

// _____________________________________________________________________________
int main(int argc, char** argv) {
  const std::string input =
      argc > 1 ? argv[1] : EXAMPLE_DATASET_DIR "/freiburg";

  sj::SpatialJoinCfg cfg;

  std::vector<sj::PreparedGeomMeta> metas;

  metas = writePersistedStorage(cfg, input, ".", "spatialjoin");

  sj::OutputWriter out(cfg.numThreads, "", "\n", "", ".");

  // called by sweeper for each relation found (by thread t)
  cfg.writeRelCb = [&out](size_t t, const char* a, size_t an, const char* b,
                          size_t bn, const char* pred, size_t predn) {
    out.writeRelCb(t, a, an, b, bn, pred, predn);
  };

  {
    // use the persisted store.
    sj::GeometryStore store(cfg, ".", "spatialjoin");
    sj::GeometryCache cache(cfg);
    cache.addStore(&store);

    // build the sweep event list from the metas
    sj::SweepEventList events(cfg, ".", "spatialjoin");
    for (const auto& meta : metas) events.add(meta, 0);

    events.finalize(cfg.numThreads);       // necessary
    events.duplicatesToReferences(cache);  // this is optional

    sj::Sweeper sweeper(cfg, &cache);
    sweeper.sweep(&events);
  }

  // delete the persisted stores again
  for (const char* f :
       {"./spatialjoinpoints", "./spatialjoinareas", "./spatialjoinsimpleareas",
        "./spatialjoinlines", "./spatialjoinsimplelines",
        "./spatialjoinmultis"}) {
    std::remove(f);
  }
}
