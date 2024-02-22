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

template <typename V>
class IntervalIdx {
 public:
  IntervalIdx() {}

  IntervalIdx(V len, V step) : _len(len), _step(step) {
    _ts = {10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};
    _ivals.resize(_ts.size() + 1);

  }

  void insert(std::pair<V, V> val) {
    V span = val.second - val.first;

    for (size_t i = 0; i < _ts.size(); i++) {
      if (span < _ts[i]) {
        _ivals[i].insert(val);
        return;
      }
    }

    // largest
    _ivals.back().insert(val);
    if (span > _maxSpan) _maxSpan = span;
  }

  void erase(std::pair<V, V> val) {
    V span = val.second - val.first;

    for (size_t i = 0; i < _ts.size(); i++) {
      if (span < _ts[i]) {
        _ivals[i].erase(val);
        return;
      }
    }

    // largest
    _ivals.back().erase(val);
  }

  std::vector<std::pair<V, V>> overlap_find_all(std::pair<V, V> val) const {
    std::vector<std::pair<V, V>> ret;
    ret.reserve(50);

    for (size_t j = 0; j < _ts.size(); j++) {
      auto i = _ivals[j].lower_bound({val.first - _ts[j], 0});

      while (i != _ivals[j].end() && i->first < val.second) {
        if ((val.first >= i->first && val.first <= i->second) ||
            (val.second >= i->first && val.second <= i->second) ||
            (i->first >= val.first && i->first <= val.second) ||
            (i->second >= val.first && i->second <= val.second)) {
          ret.push_back(*i);
        }
        i++;
      }
    }

    auto i = _ivals.back().lower_bound({val.first - _maxSpan, 0});

    while (i != _ivals.back().end() && i->first < val.second) {
      if ((val.first >= i->first && val.first <= i->second) ||
          (val.second >= i->first && val.second <= i->second) ||
          (i->first >= val.first && i->first <= val.second) ||
          (i->second >= val.first && i->second <= val.second)) {
        ret.push_back(*i);
      }
      i++;
    }

    return ret;
  }

  size_t size() {
    size_t res = 0 ;

    for (const auto& ival : _ivals) res += ival.size();

    return res;
  }

 private:
  V _len;
  V _step;

  std::vector<V> _ts;

  std::vector<std::set<std::pair<V, V>>> _ivals;

  V _maxSpan = 0;
};

}  // namespace sj
#endif
