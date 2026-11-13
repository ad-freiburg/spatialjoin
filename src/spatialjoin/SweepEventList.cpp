// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#include "SweepEventList.h"

#include <errno.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "GeometryCache.h"
#include "GeometryStore.h"
#include "util/Misc.h"

using sj::BoxVal;
using sj::GeometryCache;
using sj::SweepEventList;

// _____________________________________________________________________________
void sj::padSweepEventForDist(BoxVal* bv, const SpatialJoinCfg& cfg) {
  const util::geo::I32Box box{{bv->val(), bv->loY()}, {bv->val(), bv->upY()}};
  const auto padded = padBox(box, box, cfg);

  bv->setVal(bv->out() ? padded.getUpperRight().getX()
                       : padded.getLowerLeft().getX());
  bv->setLoY(padded.getLowerLeft().getY());
  bv->setUpY(padded.getUpperRight().getY());

  if (bv->hasB45())
    bv->setB45(padBox(bv->b45(), box, cfg));
  else
    bv->padB45(boxPadding(box, cfg));
}

// _____________________________________________________________________________
void SweepEventList::duplicatesToReferences(GeometryCache& cache) {
  cache.log("Removing duplicates...");

  DuplicateState state;

  if (!_onDisk) {
    // event list is held in memory, modify it in place
    if (duplicatesToReferences(_buffer.data(), _buffer.size(), cache, state) &&
        _persistOnDisk) {
      // the persisted copy written in finalize() is outdated now
      writeBufferToPersistFile();
    }
  } else {
    // on disk, modifications are written back to _file, which is the persisted
    // file if _persistOnDisk
    duplicatesOnDiskToReferences(cache, state);
  }

  if (_persistOnDisk) writeRefs(refsFName(persistFName()));

  cache.log("...done, " + std::to_string(numReferences()) + " references");
}

// _____________________________________________________________________________
void SweepEventList::duplicatesOnDiskToReferences(GeometryCache& cache,
                                                  DuplicateState& state) {
  std::vector<unsigned char> buf(DEFAULT_RBUF_S);

  ssize_t len;
  size_t pos = 0;

  while ((len = util::preadAll(_file, buf.data(), buf.size(), pos)) != 0) {
    size_t posOld = pos;
    if (len < 0) {
      std::stringstream ss;
      ss << "Could not read from events file\n";
      ss << strerror(errno) << std::endl;
      throw std::runtime_error(ss.str());
    }

    pos += len;

    if (len % sizeof(BoxVal))
      throw std::runtime_error("Corrupted events file");

    // if we changed something in this buffer, write it back
    if (duplicatesToReferences(reinterpret_cast<BoxVal*>(buf.data()),
                               len / sizeof(BoxVal), cache, state)) {
      if (util::pwriteAll(_file, buf.data(), len, posOld) < 0) {
        std::stringstream ss;
        ss << "Could not write to events file\n";
        ss << strerror(errno) << std::endl;
        throw std::runtime_error(ss.str());
      }
    }
  }
}

// _____________________________________________________________________________
bool SweepEventList::duplicatesToReferences(BoxVal* events, size_t n,
                                            GeometryCache& cache,
                                            DuplicateState& state) {
  const auto& cfg = cache.cfg();
  bool updated = false;

  for (size_t i = 0; i < n; i++) {
    auto cur = events + i;
    const size_t curId = duplicateId(cur->storeId(), getId(*cur));

    if (cfg.sweepCancellationCb && state.numEvents % 10000 == 0) {
      cfg.sweepCancellationCb();
    }

    state.numEvents++;

    if (cur->out()) {
      if (cur->type() == POLYGON) {
        if (state.deletedPolys.erase(curId)) {
          // erase it if present, to avoid unnecessary memory consumption
          cur->setType(DELETED);
          updated = true;
        }
        state.referencedPolys.erase(curId);
      }

      if (cur->type() == LINE) {
        if (state.deletedLines.erase(curId)) {
          // erase it if present, to avoid unnecessary memory consumption
          cur->setType(DELETED);
          updated = true;
        }
        state.referencedLines.erase(curId);
      }

      continue;
    }

    if (state.curX != cur->val()) {
      // new equal-X block
      state.duplicatePolys = {};
      state.duplicateLines = {};
      state.curX = cur->val();
    }

    if (cur->type() != POLYGON && cur->type() != LINE) continue;

    const auto& store = cache.getStore(cur->storeId());

    if (cur->type() == POLYGON &&
        cur->numAnchors() >= store.DUPLICATE_REMOVAL_MIN_SIZE) {
      size_t h = duplicateId(cur->storeId(), cur->numAnchors());
      const auto& existing = state.duplicatePolys.find(h);

      if (existing != state.duplicatePolys.end()) {
        auto a = cache.getArea(getId(*cur), cur->storeId());
        auto b = cache.getArea(existing->second, cur->storeId());

        if (a->geom == b->geom) {
          state.deletedPolys.insert(curId);
          if (state.referencedPolys
                  .insert(duplicateId(cur->storeId(), existing->second))
                  .second) {
            // for the first element referencing this, modify this
            // event to the self check of the referenced geom
            cur->setType(SELF_CHECK_AREA);
            // the self check event references the referenced geom in
            // its store
            cur->setId(typedId(existing->second, cur->type()));
          } else {
            cur->setType(DELETED);
          }

          updated = true;
          addReference(cur->storeId(), b->id, b->subId, a->id, a->subId);
        }
      } else {
        state.duplicatePolys[h] = getId(*cur);
      }
    }

    if (cur->type() == LINE &&
        cur->numAnchors() >= store.DUPLICATE_REMOVAL_MIN_SIZE) {
      size_t h = duplicateId(cur->storeId(), cur->numAnchors());
      const auto& existing = state.duplicateLines.find(h);

      if (existing != state.duplicateLines.end()) {
        auto a = cache.getLine(getId(*cur), cur->storeId());
        auto b = cache.getLine(existing->second, cur->storeId());

        if (a->geom == b->geom) {
          state.deletedLines.insert(curId);
          if (state.referencedLines
                  .insert(duplicateId(cur->storeId(), existing->second))
                  .second) {
            // for the first element referencing this, modify this
            // event to the self check of the referenced geom
            cur->setType(SELF_CHECK_LINE);
            // the self check event references the referenced geom in
            // its store
            cur->setId(typedId(existing->second, cur->type()));
          } else {
            cur->setType(DELETED);
          }
          updated = true;
          addReference(cur->storeId(), b->id, b->subId, a->id, a->subId);
        }
      } else {
        state.duplicateLines[h] = getId(*cur);
      }
    }
  }

  return updated;
}

// _____________________________________________________________________________
void SweepEventList::addReference(uint8_t storeId, const std::string& refGid,
                                  size_t refSubId, const std::string& gid,
                                  size_t subId) {
  if (_refs.size() <= storeId) _refs.resize(storeId + 1);
  _refs[storeId][refGid][refSubId][gid] = subId;
  _hasRefs = true;
}

// _____________________________________________________________________________
size_t SweepEventList::numReferences() const {
  size_t ret = 0;
  for (const auto& storeRefs : _refs) {
    for (const auto& subs : storeRefs) {
      for (const auto& refd : subs.second) ret += refd.second.size();
    }
  }
  return ret;
}

namespace {
// _____________________________________________________________________________
void writeSize(std::ostream& str, size_t v) {
  str.write(reinterpret_cast<const char*>(&v), sizeof(v));
}

// _____________________________________________________________________________
void writeString(std::ostream& str, const std::string& s) {
  writeSize(str, s.size());
  str.write(s.data(), s.size());
}

// _____________________________________________________________________________
size_t readSize(std::istream& str) {
  size_t v;
  if (!str.read(reinterpret_cast<char*>(&v), sizeof(v)))
    throw std::runtime_error("Corrupted references file");
  return v;
}

// _____________________________________________________________________________
std::string readString(std::istream& str) {
  std::string s(readSize(str), 0);
  if (!str.read(&s[0], s.size()))
    throw std::runtime_error("Corrupted references file");
  return s;
}
}  // namespace

// _____________________________________________________________________________
void SweepEventList::writeRefs(const std::string& fName) const {
  std::ofstream out(fName, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!out) throw std::runtime_error("Could not open references file " + fName);

  writeSize(out, numReferences());
  for (size_t storeId = 0; storeId < _refs.size(); storeId++) {
    for (const auto& refd : _refs[storeId]) {
      for (const auto& refdSub : refd.second) {
        for (const auto& referer : refdSub.second) {
          writeSize(out, storeId);
          writeString(out, refd.first);
          writeSize(out, refdSub.first);
          writeString(out, referer.first);
          writeSize(out, referer.second);
        }
      }
    }
  }

  if (!out) throw std::runtime_error("Could not write references file " + fName);
}

// _____________________________________________________________________________
void SweepEventList::readRefs(const std::string& fName) {
  std::ifstream in(fName, std::ios::in | std::ios::binary);
  if (!in) return;

  size_t numRefs = readSize(in);
  for (size_t i = 0; i < numRefs; i++) {
    size_t storeId = readSize(in);
    auto refGid = readString(in);
    size_t refSubId = readSize(in);
    auto gid = readString(in);
    size_t subId = readSize(in);
    addReference(storeId, refGid, refSubId, gid, subId);
  }
}
