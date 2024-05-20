// Copyright 2024, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_INTERVALIDX_H_
#define SPATIALJOINS_INTERVALIDX_H_

#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <unordered_map>

#include "BoxIds.h"
#include "util/geo/Geo.h"

namespace sj {

template <typename K, typename V>
struct IntervalVal {
  K l;
  K r;
  V v;
};

template <typename K, typename V>
inline bool operator<(const IntervalVal<K, V>& l, const IntervalVal<K, V>& r) {
  return l.l < r.l || (l.l == r.l && l.r < r.r) ||
         (l.l == r.l && l.r == r.r && l.v < r.v);
}

template <typename K, typename V>
class IntervalIdx {
 public:
  IntervalIdx() {
    _ts = {10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};
    _ivals.resize(_ts.size() + 1);
  }

  // insert an interval s = [a, b] with value val
  void insert(const std::pair<K, K> s, const V val) {
    const K span = s.second - s.first;

    for (size_t i = 0; i < _ts.size(); i++) {
      if (span < _ts[i]) {
        _ivals[i].insert({s.first, s.second, val});
        return;
      }
    }

    // largest
    _ivals.back().insert({s.first, s.second, val});
    if (span > _maxSpan) _maxSpan = span;
  }

  // erase an interval s = [a, b] with value val
  void erase(const std::pair<K, K> s, const V val) {
    const K span = s.second - s.first;

    for (size_t i = 0; i < _ts.size(); i++) {
      if (span < _ts[i]) {
        _ivals[i].erase({s.first, s.second, val});
        return;
      }
    }

    // largest
    _ivals.back().erase({s.first, s.second, val});
  }

  // find all intervals, with their values, which overlap s= [a, b]
  std::vector<IntervalVal<K, V>> overlap_find_all(
      const std::pair<K, K> s) const {
    std::vector<IntervalVal<K, V>> ret;
    ret.reserve(15);

    // retrieve from each sub-list
    for (size_t j = 0; j < _ts.size(); j++) get(s, _ivals[j], _ts[j], ret);

    // also retrieve from largest sub-list
    get(s, _ivals.back(), _maxSpan, ret);

    return ret;
  }

  // returns the size of the interval index
  size_t size() {
    size_t res = 0;
    for (const auto& ival : _ivals) res += ival.size();
    return res;
  }

 private:
  std::vector<K> _ts;
  std::vector<std::set<IntervalVal<K, V>>> _ivals;

  K _maxSpan = 0;

  // retrieve all overlapping intervals from an set of intervals, sorted by
  // left interval boundary, with a parameter t which specifies the guaranteed
  // maximum interval range in the set
  void get(const std::pair<K, K>& val, const std::set<IntervalVal<K, V>>& idx,
           K t, std::vector<IntervalVal<K, V>>& ret) const {
    // search for the first interval that might overlap with the queried one
    // we can do binary search here by exploiting the fact that we know that
    // no interval in the set is longer than t, so each interval with a
    // left bound < val[0] - t canno overlap with val
    auto i = idx.lower_bound({val.first - t, 0, {}});

    // explicitly check for overlaps as long as the left bound is not larger
    // than the right bound of val (from then on, no intersects are possible
    // any more)
    while (i != idx.end() && i->l <= val.second) {
      if ((val.first >= i->l && val.first <= i->r) || (val.second <= i->r) ||
          (i->l >= val.first) || (i->r >= val.first && i->r <= val.second)) {
        ret.push_back(*i);
      }
      i++;
    }
  }
};

}  // namespace sj
#endif
