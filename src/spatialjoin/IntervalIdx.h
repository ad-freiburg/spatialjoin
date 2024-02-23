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

  std::vector<IntervalVal<K, V>> overlap_find_all(const std::pair<K, K> s) const {
    std::vector<IntervalVal<K, V>> ret;

    for (size_t j = 0; j < _ts.size(); j++) {
      get(s, _ivals[j], _ts[j], ret);
    }
    get(s, _ivals.back(), _maxSpan, ret);

    return ret;
  }

  size_t size() {
    size_t res = 0;
    for (const auto& ival : _ivals) res += ival.size();
    return res;
  }

 private:
  std::vector<K> _ts;
  std::vector<std::set<IntervalVal<K, V>>> _ivals;

  K _maxSpan = 0;

  void get(const std::pair<K, K> val, const std::set<IntervalVal<K, V>>& idx, K t,
           std::vector<IntervalVal<K, V>>& ret) const {
    auto i = idx.lower_bound({val.first - t, 0, {}});

    while (i != idx.end() && i->l <= val.second) {
      if ((val.first >= i->l && val.first <= i->r) ||
          (val.second <= i->r) ||
          (i->l >= val.first) ||
          (i->r >= val.first && i->r <= val.second)) {
        ret.push_back(*i);
      }
      i++;
    }
  }
};

}  // namespace sj
#endif
