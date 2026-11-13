// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#include "SweepEventListTest.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "spatialjoin/SweepEventList.h"
#include "util/Test.h"

using sj::BoxVal;
using util::geo::CRSType;
using util::geo::DBox;
using util::geo::DPoint;
using sj::SweepEventList;

namespace {

// a write buffer small enough to force writing the events to disk
const size_t SMALL_BUF = 2 * sizeof(BoxVal);

// _____________________________________________________________________________
BoxVal event(size_t id, int32_t x) {
  return BoxVal(sj::typedId(id, sj::POINT), 0, 0, x, false, 0, {0, 0});
}

// _____________________________________________________________________________
void fill(SweepEventList* events, size_t offset, size_t n) {
  for (size_t i = 0; i < n; i++)
    events->add(event(offset + i, static_cast<int32_t>(n - i)));
}

// _____________________________________________________________________________
std::vector<size_t> ids(const SweepEventList& events) {
  std::vector<size_t> ret;
  auto reader = events.newReader();
  const BoxVal* cur;
  while ((cur = reader.next())) ret.push_back(sj::getId(*cur));
  return ret;
}

// _____________________________________________________________________________
std::vector<BoxVal> boxVals(const SweepEventList& events) {
  std::vector<BoxVal> ret;
  auto reader = events.newReader();
  const BoxVal* cur;
  while ((cur = reader.next())) ret.push_back(*cur);
  return ret;
}

// _____________________________________________________________________________
template <typename F>
void testAddBox(F add, int32_t llx, int32_t lly, int32_t urx, int32_t ury) {
  SweepEventList list(sj::SpatialJoinCfg(), ".", ".spatialjoin");
  add(&list, sj::typedId(1, sj::POLYGON));
  list.finalize(1);

  auto res = boxVals(list);
  TEST(res.size(), ==, 2);
  TEST(res[0].out(), ==, false);
  TEST(res[0].val(), ==, llx);
  TEST(res[1].out(), ==, true);
  TEST(res[1].val(), ==, urx);
  for (const auto& bv : res) {
    TEST(sj::getId(bv), ==, 1);
    TEST(bv.loY(), ==, lly);
    TEST(bv.upY(), ==, ury);
  }
}

// _____________________________________________________________________________
std::vector<size_t> sorted(size_t offset, size_t n) {
  std::vector<size_t> ret;
  for (size_t i = n; i > 0; i--) ret.push_back(offset + i - 1);
  return ret;
}

// _____________________________________________________________________________
std::vector<std::string> files(const std::string& dir) {
  std::vector<std::string> ret;
  DIR* d = opendir(dir.c_str());
  TEST(d != 0);
  while (const dirent* entry = readdir(d)) {
    std::string name = entry->d_name;
    if (name != "." && name != "..") ret.push_back(dir + "/" + name);
  }
  closedir(d);
  return ret;
}

// _____________________________________________________________________________
void removeDir(const std::string& dir) {
  DIR* d = opendir(dir.c_str());
  if (!d) return;
  while (const dirent* entry = readdir(d)) {
    std::string name = entry->d_name;
    if (name != "." && name != "..") unlink((dir + "/" + name).c_str());
  }
  closedir(d);
  rmdir(dir.c_str());
}

}  // namespace

// _____________________________________________________________________________
void sweepEventListTest() {
  static_assert(!std::is_copy_constructible<SweepEventList>::value,
                "SweepEventList must not be copyable");
  static_assert(std::is_nothrow_move_constructible<SweepEventList>::value,
                "SweepEventList must be nothrow move constructible");
  static_assert(std::is_nothrow_move_assignable<SweepEventList>::value,
                "SweepEventList must be nothrow move assignable");

  // loading a non-existing file fails...
  TEST_THROWS(std::runtime_error,
              SweepEventList("./.spatialjoin-test-events-nonexisting"));

  // test with different buffer sizes
  for (size_t bufSize : {sj::DEFAULT_BUFFER_S, SMALL_BUF}) {
    {
      // move construction before finalize()
      SweepEventList a(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
      fill(&a, 0, 5);

      SweepEventList b(std::move(a));
      fill(&b, 5, 5);
      TEST(b.numEvents(), ==, 10);

      b.finalize(1);
      auto res = ids(b);
      TEST(res.size(), ==, 10);

      for (size_t i = 0; i < 5; i++) {
        TEST(res[2 * i], ==, 4 - i);
        TEST(res[2 * i + 1], ==, 9 - i);
      }
    }

    {
      // move construction after finalize()
      SweepEventList a(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
      fill(&a, 0, 7);
      a.finalize(1);

      SweepEventList b(std::move(a));
      TEST(b.numEvents(), ==, 7);
      TEST(ids(b) == sorted(0, 7));
    }

    {
      SweepEventList a(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
      fill(&a, 0, 3);
      a.finalize(1);
      auto reader = a.newReader();

      SweepEventList b(std::move(a));
      std::vector<size_t> res;
      const BoxVal* cur;
      while ((cur = reader.next())) res.push_back(sj::getId(*cur));
      TEST(res == sorted(0, 3));
    }

    {
      SweepEventList a(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
      fill(&a, 0, 4);
      SweepEventList b(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
      fill(&b, 100, 6);

      b = std::move(a);
      TEST(b.numEvents(), ==, 4);
      b.finalize(1);
      TEST(ids(b) == sorted(0, 4));
    }

    {
      SweepEventList a(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
      fill(&a, 0, 4);
      a.finalize(1);
      SweepEventList b(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
      fill(&b, 100, 6);
      b.finalize(1);

      b = std::move(a);
      TEST(b.numEvents(), ==, 4);
      TEST(ids(b) == sorted(0, 4));

      SweepEventList c(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
      fill(&c, 200, 2);
      c.finalize(1);
      a = std::move(c);
      TEST(ids(a) == sorted(200, 2));
    }

    {
      // persisted lists are written to the file tmpPrefix in the cache
      // directory
      const std::string dir = "./.spatialjoin-test-persist";
      const std::string path = dir + "/.spatialjoinevents";

      // left over from an aborted earlier run
      removeDir(dir);
      TEST(mkdir(dir.c_str(), 0777) == 0);

      {
        // a list which is not persisted leaves no file
        SweepEventList n(sj::SpatialJoinCfg(), dir, ".spatialjoin", bufSize);
        fill(&n, 0, 3);
        n.finalize(1);
        TEST(files(dir).empty());
      }

      {
        SweepEventList a(sj::SpatialJoinCfg(), dir, ".spatialjoin", bufSize,
                         true);
        fill(&a, 0, 7);
        a.finalize(1);
        TEST(files(dir) == std::vector<std::string>{path});

        SweepEventList b(path);
        TEST(b.numEvents(), ==, 7);
        TEST(ids(b) == sorted(0, 7));

        // the original is unchanged
        TEST(ids(a) == sorted(0, 7));

        // a loaded list can be moved
        SweepEventList c(std::move(b));
        TEST(ids(c) == sorted(0, 7));
      }

      {
        // the persisted file is kept after the list was destroyed
        TEST(files(dir) == std::vector<std::string>{path});
        TEST(ids(SweepEventList(path)) == sorted(0, 7));
      }

      {
        // an empty list, which overwrites the existing file
        SweepEventList e(sj::SpatialJoinCfg(), dir, ".spatialjoin", bufSize,
                         true);
        e.finalize(1);
        TEST(files(dir) == std::vector<std::string>{path});
        SweepEventList f(path);
        TEST(f.numEvents(), ==, 0);
        TEST(ids(f).empty());
        unlink(path.c_str());
      }

      TEST(rmdir(dir.c_str()) == 0);
    }

    {
      // event lists in a vector, which moves them when growing
      std::vector<SweepEventList> lists;
      for (size_t i = 0; i < 10; i++) {
        lists.emplace_back(sj::SpatialJoinCfg(), ".", ".spatialjoin", bufSize);
        fill(&lists.back(), i * 10, 3);
      }

      for (size_t i = 0; i < lists.size(); i++) {
        lists[i].finalize(1);
        TEST(ids(lists[i]) == sorted(i * 10, 3));
      }
    }
  }

  {
    testAddBox(
        [](SweepEventList* l, size_t id) {
          l->add(id, DBox({100.0, 200.0}, {300.0, 400.0}),
                 CRSType::WEB_MERCATOR, 0);
        },
        100 * PREC, 200 * PREC, 300 * PREC, 400 * PREC);
    testAddBox(
        [](SweepEventList* l, size_t id) {
          l->add(id, DBox({-1.06, -2.01}, {3.01, 4.06}),
                 CRSType::WEB_MERCATOR, 0);
        },
        -11, -21, 31, 41);
  }

  {
    auto ll = util::geo::latLngToWebMerc(DPoint(7.8, 47.9));
    auto ur = util::geo::latLngToWebMerc(DPoint(7.9, 48.1));
    testAddBox(
        [](SweepEventList* l, size_t id) {
          l->add(id, DBox({7.8, 47.9}, {7.9, 48.1}),
                 CRSType::CRS84, 0);
        },
        std::floor(ll.getX() * PREC), std::floor(ll.getY() * PREC),
        std::ceil(ur.getX() * PREC), std::ceil(ur.getY() * PREC));

    int32_t world = std::ceil(sj::boxids::WORLD_H / 2.0);
    testAddBox(
        [](SweepEventList* l, size_t id) {
          l->add(id, DBox({0.0, 0.0}, {0.0, 90.0}),
                 CRSType::CRS84, 0);
        },
        0, 0, 0, world);
  }

  {
    auto ll = util::geo::latLngToWebMerc(DPoint(7.8, 47.9));
    auto ur = util::geo::latLngToWebMerc(DPoint(7.9, 48.1));
    testAddBox(
        [](SweepEventList* l, size_t id) {
          l->add(id, DBox({47.9, 7.8}, {48.1, 7.9}),
                 CRSType::WGS84, 0);
        },
        std::floor(ll.getX() * PREC), std::floor(ll.getY() * PREC),
        std::ceil(ur.getX() * PREC), std::ceil(ur.getY() * PREC));

    int32_t world = std::ceil(sj::boxids::WORLD_H / 2.0);
    testAddBox(
        [](SweepEventList* l, size_t id) {
          l->add(id, DBox({-90.0, 0.0}, {0.0, 0.0}),
                 CRSType::WGS84, 0);
        },
        0, -world, 0, 0);
  }
}
