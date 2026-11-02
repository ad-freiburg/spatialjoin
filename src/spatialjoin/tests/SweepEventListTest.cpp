// Copyright 2026, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#include "SweepEventListTest.h"

#include <unistd.h>

#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "spatialjoin/SweepEventList.h"
#include "util/Test.h"

using sj::BoxVal;
using sj::SweepEventList;

namespace {

// a write buffer small enough to force writing the events to disk
const size_t SMALL_BUF = 2 * sizeof(BoxVal);

// _____________________________________________________________________________
BoxVal event(size_t id, int32_t x) {
  BoxVal ret{};
  ret.id = sj::typedId(id, sj::POINT);
  ret.loY = 0;
  ret.upY = 0;
  ret.val = x;
  ret.out = false;
  return ret;
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
std::vector<size_t> sorted(size_t offset, size_t n) {
  std::vector<size_t> ret;
  for (size_t i = n; i > 0; i--) ret.push_back(offset + i - 1);
  return ret;
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
      SweepEventList a(".", ".spatialjoin", bufSize);
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
      SweepEventList a(".", ".spatialjoin", bufSize);
      fill(&a, 0, 7);
      a.finalize(1);

      SweepEventList b(std::move(a));
      TEST(b.numEvents(), ==, 7);
      TEST(ids(b) == sorted(0, 7));
    }

    {
      SweepEventList a(".", ".spatialjoin", bufSize);
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
      SweepEventList a(".", ".spatialjoin", bufSize);
      fill(&a, 0, 4);
      SweepEventList b(".", ".spatialjoin", bufSize);
      fill(&b, 100, 6);

      b = std::move(a);
      TEST(b.numEvents(), ==, 4);
      b.finalize(1);
      TEST(ids(b) == sorted(0, 4));
    }

    {
      SweepEventList a(".", ".spatialjoin", bufSize);
      fill(&a, 0, 4);
      a.finalize(1);
      SweepEventList b(".", ".spatialjoin", bufSize);
      fill(&b, 100, 6);
      b.finalize(1);

      b = std::move(a);
      TEST(b.numEvents(), ==, 4);
      TEST(ids(b) == sorted(0, 4));

      SweepEventList c(".", ".spatialjoin", bufSize);
      fill(&c, 200, 2);
      c.finalize(1);
      a = std::move(c);
      TEST(ids(a) == sorted(200, 2));
    }

    {
      const std::string path = ".spatialjoin-test-events";
      const std::string path2 = ".spatialjoin-test-events2";

      SweepEventList a(".", ".spatialjoin", bufSize);
      fill(&a, 0, 7);
      a.finalize(1);
      a.serializeToDisk(path);

      SweepEventList b(path);
      TEST(b.numEvents(), ==, 7);
      TEST(ids(b) == sorted(0, 7));

      // the original is unchanged
      TEST(ids(a) == sorted(0, 7));

      // a loaded list can be serialized again, and moved
      b.serializeToDisk(path2);
      SweepEventList c(std::move(b));
      SweepEventList d(path2);
      TEST(d.numEvents(), ==, 7);
      TEST(ids(d) == sorted(0, 7));
      TEST(ids(c) == sorted(0, 7));

      // an empty list
      SweepEventList e(".", ".spatialjoin", bufSize);
      e.finalize(1);
      e.serializeToDisk(path);
      SweepEventList f(path);
      TEST(f.numEvents(), ==, 0);
      TEST(ids(f).empty());

      unlink(path.c_str());
      unlink(path2.c_str());
    }

    {
      // event lists in a vector, which moves them when growing
      std::vector<SweepEventList> lists;
      for (size_t i = 0; i < 10; i++) {
        lists.emplace_back(".", ".spatialjoin", bufSize);
        fill(&lists.back(), i * 10, 3);
      }

      for (size_t i = 0; i < lists.size(); i++) {
        lists[i].finalize(1);
        TEST(ids(lists[i]) == sorted(i * 10, 3));
      }
    }
  }
}
