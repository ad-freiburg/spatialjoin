// Copyright 2016, University of Freiburg,
// Chair of Algorithms and Data Structures.
// Author: Patrick Brosi <brosi@informatik.uni-freiburg.de>

#ifndef UTIL_GEO_LINE_H_
#define UTIL_GEO_LINE_H_

#include <vector>

#include "./Point.h"

namespace util {
namespace geo {

template <typename T>
class Line : public std::vector<Point<T>> {
  using std::vector<Point<T>>::vector;
};

template <typename T>
using LineSegment = std::pair<Point<T>, Point<T>>;

template <typename T>
struct XSortedTuple {
  // a tuple used in X-sorted linestrings and polygons.
  // we do not store the corresponding line segment explicitly, but only the
  // other point, with "vals" holding which of the two points comes first on
  // bit 1, and whether this element is an out event on bit 0. Bit 2 is 1 if
  // the line is completely empty.
  // for 64bit points, we thus safe 16 bytes (~29%), which let's more
  // XSortedTuples fit into the CPU caches during tests for intersect / contains
  // (in particular the binary search during startup), and also makes for faster
  // reading from disk
  Point<T> p;
  Point<T> other;
  uint8_t vals : 4;

  XSortedTuple() {}
  XSortedTuple(const Point<T>& p, bool out) : p(p), vals(0) {
    if (out) vals = 1;
    vals += 4;
  }
  XSortedTuple(const Point<T>& p, const LineSegment<T>& seg, bool out)
      : p(p), vals(0) {
    if (out) vals = 1;
    if (seg.first == p) {
      vals += 2;
      other = seg.second;
    } else if (seg.second == p) {
      other = seg.first;
    }
  }

  LineSegment<T> seg() const {
    if (vals & 4) return {{0, 0}, {0, 0}};
    if (vals & 2) return {p, other};
    return {other, p};
  }

  bool out() const { return vals & 1; }
};

template <typename T>
bool operator<(const XSortedTuple<T>& a, const XSortedTuple<T>& b) {
  return a.p.getX() < b.p.getX() ||
         (a.p.getX() == b.p.getX() && !a.out() && b.out());
}

template <typename T>
bool operator<(const LineSegment<T>& a, const LineSegment<T>& b) {
  // this should be implicitely catched by the comparisons below,
  // however, because of floating point imprecisions, we might decide
  // < for equivalent segments. This is a problem, as segments are only
  // identified via they coordinates, not by some ID, in the active sets
  if (a.first.getX() == b.first.getX() && a.first.getY() == b.first.getY() &&
      a.second.getX() == b.second.getX() &&
      a.second.getY() == b.second.getY()) {
    return false;
  }

  if (a.first.getX() < b.first.getX() || b.first.getX() == b.second.getX()) {
    // a was first in active set
    if (a.first.getX() != a.second.getX()) {
      // check whether first point of b is right of or left of a
      double d = -((b.first.getX() - a.first.getX()) *
                       (a.second.getY() - a.first.getY()) -
                   (b.first.getY() - a.first.getY()) *
                       (a.second.getX() - a.first.getX()));
      if (d < 0) return false;
      if (d > 0) return true;

      // if we arrive here, first point of b was EXACTLY on a, we have to decide
      // based on the second point of b
      d = -((b.second.getX() - a.first.getX()) *
                (a.second.getY() - a.first.getY()) -
            (b.second.getY() - a.first.getY()) *
                (a.second.getX() - a.first.getX()));
      if (d < 0) return false;
      if (d > 0) return true;
    }

  } else {
    // b was first in active set
    if (b.first.getX() != b.second.getX()) {
      double d = (a.first.getX() - b.first.getX()) *
                     (b.second.getY() - b.first.getY()) -
                 (a.first.getY() - b.first.getY()) *
                     (b.second.getX() - b.first.getX());
      if (d < 0) return false;
      if (d > 0) return true;

      // if we arrive here, first point of a was EXACTLY on b, we have to decide
      // based on the second point of a
      d = (a.second.getX() - b.first.getX()) *
              (b.second.getY() - b.first.getY()) -
          (a.second.getY() - b.first.getY()) *
              (b.second.getX() - b.first.getX());
      if (d < 0) return false;
      if (d > 0) return true;
    }
  }

  // a and b are colinear
  return a.first.getY() < b.first.getY() ||
         (a.first.getY() == b.first.getY() &&
          a.first.getX() < b.first.getX()) ||
         (a.first.getY() == b.first.getY() &&
          a.first.getX() == b.first.getX() &&
          a.second.getY() < b.second.getY()) ||
         (a.first.getY() == b.first.getY() &&
          a.first.getX() == b.first.getX() &&
          a.second.getY() == b.second.getY() &&
          a.second.getX() < b.second.getX());
}

template <typename T>
bool operator>(const LineSegment<T>& a, const LineSegment<T>& b) {
  return b < a;
}

template <typename T>
class XSortedLine {
 public:
  XSortedLine() {}

  XSortedLine(const Line<T>& line) {
    _line.reserve(line.size());
    for (size_t i = 1; i < line.size(); i++) {
      double len = fabs(line[i - 1].getX() - line[i].getX());
      if (len > _maxSegLen) _maxSegLen = len;

      if (line[i - 1].getX() < line[i].getX()) {
        _line.push_back({line[i - 1], {line[i - 1], line[i]}, false});
        _line.push_back({line[i], {line[i - 1], line[i]}, true});
      } else {
        _line.push_back({line[i], {line[i], line[i - 1]}, false});
        _line.push_back({line[i - 1], {line[i], line[i - 1]}, true});
      }
    }

    std::sort(_line.begin(), _line.end());
  }

  XSortedLine(const LineSegment<T>& line) {
    _line.resize(2);
    if (line.first.getX() < line.second.getX()) {
      _line[0] = {line.first, {line.first, line.second}, false};
      _line[1] = {line.second, {line.first, line.second}, true};
    } else {
      _line[0] = {line.second, {line.second, line.first}, false};
      _line[1] = {line.first, {line.second, line.first}, true};
    }
    _maxSegLen = fabs(line.first.getX() - line.second.getX());
  }

  double getMaxSegLen() const { return _maxSegLen; }
  void setMaxSegLen(double l) { _maxSegLen = l; }

  const std::vector<XSortedTuple<T>>& rawLine() const { return _line; }
  std::vector<XSortedTuple<T>>& rawLine() { return _line; }

 private:
  std::vector<XSortedTuple<T>> _line;
  double _maxSegLen = -1;
};

template <typename T>
using MultiLine = std::vector<Line<T>>;

}  // namespace geo
}  // namespace util

#endif  // UTIL_GEO_LINE_H_
