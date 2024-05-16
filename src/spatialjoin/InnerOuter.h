// Copyright 2024, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_INNEROUTER_H_
#define SPATIALJOINS_INNEROUTER_H_

#include "util/geo/Geo.h"

namespace sj {
namespace innerouter {

enum class Mode { INNER, OUTER };

// ____________________________________________________________________________
template <Mode MODE, typename T>
bool innerOuterDouglasPeucker(const util::geo::Ring<T>& inputPoints,
                              util::geo::Ring<T>& outputPoints, size_t l,
                              size_t r, double eps) {
  // this is basically a verbatim translation from Hannah's qlever map UI code

  assert(r >= l);
  assert(!inputPoints.empty());
  assert(r < inputPoints.size());

  if (l == r) {
    outputPoints.push_back(inputPoints[l]);
    return false;
  }

  if (l + 1 == r) {
    outputPoints.push_back(inputPoints[l]);
    outputPoints.push_back(inputPoints[r]);
    return false;
  }
  // Compute the position of the point m between l and r that is furthest aways
  // from the line segment connecting l and r. Note that l < m < r, that is, it
  // cannot (and must not) happen that m == l or m == r.
  //
  // NOTE: If all points happen to lie directly on the line segment, max_dist ==
  // 0 and we can simplify without loss, no matter which variant.

  size_t m;
  // double max_dist = -1;
  auto m_left = l;
  auto m_right = l;
  double max_dist_left = 0;
  double max_dist_right = 0;
  auto L = inputPoints[l];
  auto R = inputPoints[r];

  // L and R should be different points.
  if (L == R) {
    std::cerr << "DOUGLAS PEUCKER FAIL!" << std::endl;
    // TODO: handle
    return false;
  }

  // Compute point furthest to the left (negative value for
  // distanceFromPointToLine) and furthest to the right (positive value).
  for (auto k = l + 1; k <= r - 1; k++) {
    auto dist = signedDistanceFromPointToLine(L, R, inputPoints[k]);
    if (dist < 0 && -dist > max_dist_left) {
      m_left = k;
      max_dist_left = -dist;
    }
    if (dist > 0 && dist > max_dist_right) {
      m_right = k;
      max_dist_right = dist;
    }
  }

  bool simplify = false;

  // INNER Douglas-Peucker: Simplify iff there is no point to the *left* and the
  // rightmost point has distance <= eps. Otherwise m is the leftmost point or,
  // if there is no such point, the rightmost point.
  if (MODE == Mode::INNER) {
    simplify = (max_dist_left == 0 && max_dist_right <= eps);
    m = max_dist_left > 0 ? m_left : m_right;
  }

  // OUTER Douglas-Peucker: Simplify iff there is no point to the *right*
  // *and* the leftmost point has distance <= eps. Otherwise m is the rightmost
  // point or if there is no such point the leftmost point.
  if (MODE == Mode::OUTER) {
    simplify = (max_dist_right == 0 && max_dist_left <= eps);
    m = max_dist_right > 0 ? m_right : m_left;
  }

  // Simplification case: If m is at most eps away from the line segment
  // connecting l and r, we can simplify the part of the polygon from l to r by
  // the line segment that connects l and r.
  // assert(m > l);
  // assert(m < r);
  if (simplify) {
    outputPoints.push_back(L);
    outputPoints.push_back(R);
    return true;
  }

  // Recursion case: If we come here, we have a point at position m, where l < m
  // < r and that point is more than eps away from the line segment connecting l
  // and r. Then we call the algorithm recursively on the part to the left of m
  // and the part to the right of m. NOTE: It's a matter of taste whether we
  // include m in the left recursion or the right recursion, but we should not
  // include it in both.
  bool a = innerOuterDouglasPeucker<MODE>(inputPoints, outputPoints, l, m, eps);
  bool b =
      innerOuterDouglasPeucker<MODE>(inputPoints, outputPoints, m + 1, r, eps);

  return a || b;
}

// ____________________________________________________________________________
template <typename T>
double signedDistanceFromPointToLine(const util::geo::Point<T>& A,
                                     const util::geo::Point<T>& B,
                                     const util::geo::Point<T>& C) {
  // Check that the input is OK and not A == B.
  if (A == B) return 0;

  // The actual computation, see this Wikipedia article for the formula:
  // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  double distAB = sqrt((A.getX() - B.getX()) * (A.getX() - B.getX()) +
                       (A.getY() - B.getY()) * (A.getY() - B.getY()));
  double areaTriangleTimesTwo = (B.getY() - A.getY()) * (A.getX() - C.getX()) -
                                (B.getX() - A.getX()) * (A.getY() - C.getY());
  return areaTriangleTimesTwo / distAB;
}

}  // namespace innerouter
}  // namespace sj

#endif  // SPATIALJOINS_INNEROUTER_H_
