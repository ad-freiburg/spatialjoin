// Copyright 2023, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_BOXIDS_H_
#define SPATIALJOINS_BOXIDS_H_

#include <map>

#include "util/geo/Geo.h"

const static int PREC = 10;

namespace sj {
namespace boxids {

const static int NUM_GRID_CELLS = 45000;
// const static int NUM_GRID_CELLS = 5000;

const static double WORLD_W = 20037508.3427892 * PREC * 2.0;
const static double WORLD_H = 20037508.3427892 * PREC * 2.0;

const static double GRID_W = WORLD_W / (NUM_GRID_CELLS * 1.0);
const static double GRID_H = WORLD_H / (NUM_GRID_CELLS * 1.0);

const static double GRID_AREA = GRID_W * GRID_H;

typedef std::pair<int32_t, uint8_t> BoxId;

typedef std::vector<BoxId> BoxIdList;

struct BoxIdCmp {
  bool operator()(const BoxId& left, const BoxId& right) {
    return abs(left.first) < abs(right.first);
  }
  bool operator()(const BoxId& left, int32_t right) {
    return abs(left.first) < abs(right);
  }
};

// ____________________________________________________________________________
inline int32_t getBoxId(const util::geo::I32Point& p) {
  int32_t x = floor((1.0 * p.getX() + WORLD_W / 2.0) / GRID_W);
  int32_t y = floor((1.0 * p.getY() + WORLD_H / 2.0) / GRID_H);

  return y * NUM_GRID_CELLS + x + 1;
}

// ____________________________________________________________________________
inline void getBoxIds(const util::geo::I32XSortedLine& line,
                      const util::geo::I32Box& envelope, int xFrom, int xTo,
                      int yFrom, int yTo, int xWidth, int yHeight,
                      BoxIdList* ret, size_t startA, size_t startB) {
  for (int32_t y = yFrom; y < yTo; y += yHeight) {
    size_t firstInA = startA;
    size_t firstInB = startB;

    for (int32_t x = xFrom; x < xTo; x += xWidth) {
      int localXWidth = std::min(xTo - x, xWidth);
      int localYHeight = std::min(yTo - y, yHeight);

      util::geo::I32Box box(
          {static_cast<int>((x * GRID_W - WORLD_W / 2.0)),
           static_cast<int>((y * GRID_H - WORLD_H / 2.0))},
          {static_cast<int>((x + localXWidth) * GRID_W - WORLD_W / 2.0),
           static_cast<int>((y + localYHeight) * GRID_H - WORLD_H / 2.0)});

      if (!util::geo::intersects(box, envelope)) continue;

      const util::geo::I32XSortedPolygon boxPoly{util::geo::I32Polygon(box)};

      auto check = util::geo::intersectsContainsCovers(
          line, envelope, boxPoly, box, &firstInA, &firstInB);

      if (std::get<0>(check)) {
        if (localXWidth == 1 && localYHeight == 1) {
          int32_t newId = y * NUM_GRID_CELLS + x + 1;

          if (!ret->empty() && ret->back().second < 254 &&
              ret->back().first - ret->back().second == newId + 1) {
            ret->back().second++;
          } else {
            ret->push_back({newId, 0});
          }
        } else {
          // we need to check in detail on a smaller level!
          // recurse down...
          int newXWidth = (localXWidth + 1) / 2;
          int newYHeight = (localYHeight + 1) / 2;

          getBoxIds(line, envelope, x, x + localXWidth, y, y + localYHeight,
                    newXWidth, newYHeight, ret, firstInA, firstInB);
        }
      }
    }
  }
}

// ____________________________________________________________________________
inline void getBoxIds(const util::geo::I32XSortedPolygon& poly,
                      const util::geo::I32Box& envelope, double area, int xFrom,
                      int xTo, int yFrom, int yTo, int xWidth, int yHeight,
                      BoxIdList* ret, size_t startA, size_t startB) {
  for (int32_t y = yFrom; y < yTo; y += yHeight) {
    size_t firstInA = startA;
    size_t firstInB = startB;

    for (int32_t x = xFrom; x < xTo; x += xWidth) {
      int localXWidth = std::min(xTo - x, xWidth);
      int localYHeight = std::min(yTo - y, yHeight);

      util::geo::I32Box box(
          {static_cast<int>((x * GRID_W - WORLD_W / 2.0)),
           static_cast<int>((y * GRID_H - WORLD_H / 2.0))},
          {static_cast<int>((x + localXWidth) * GRID_W - WORLD_W / 2.0),
           static_cast<int>((y + localYHeight) * GRID_H - WORLD_H / 2.0)});

      if (!util::geo::intersects(box, envelope)) continue;

      const util::geo::I32XSortedPolygon boxPoly{util::geo::I32Polygon(box)};

      double boxArea = GRID_AREA * (localXWidth) * (localYHeight);

      auto check = util::geo::intersectsContainsCovers(
          boxPoly, box, boxArea, poly, envelope, area, &firstInA, &firstInB);

      if (std::get<1>(check)) {
        // we can insert all at once
        for (int32_t ly = y; ly < y + localYHeight; ly++) {
          int a = 1;
          int start = ly * NUM_GRID_CELLS + x;
          while (a <= (localXWidth + ((255 + 1) - 1)) / (255 + 1)) {
            int r = std::min(
                255, start + localXWidth - (start + (a - 1) * (255 + 1)) - 1);
            ret->push_back({start + (a - 1) * (255 + 1) + 1, r});
            a++;
          }
        }
      } else if (std::get<0>(check)) {
        if (localXWidth == 1 && localYHeight == 1) {
          // only intersecting
          int32_t newId = -(y * NUM_GRID_CELLS + x + 1);

          if (!ret->empty() && ret->back().second < 254 &&
              ret->back().first - ret->back().second == newId + 1) {
            ret->back().second++;
          } else {
            ret->push_back({newId, 0});
          }
        } else {
          // we need to check in detail on a smaller level!
          // recurse down...
          int newXWidth = (localXWidth + 1) / 2;
          int newYHeight = (localYHeight + 1) / 2;

          getBoxIds(poly, envelope, area, x, x + localXWidth, y,
                    y + localYHeight, newXWidth, newYHeight, ret, firstInA,
                    firstInB);
        }
      }
    }
  }
}

// ____________________________________________________________________________
inline BoxIdList getBoxIds(const util::geo::I32XSortedLine& line,
                           const util::geo::I32Box& envelope) {
  int32_t a = getBoxId(envelope.getLowerLeft());
  int32_t b = getBoxId(envelope.getUpperRight());
  if (a == b) return {{a, 0}};  // shortcut

  int32_t startX = std::floor(
      (1.0 * envelope.getLowerLeft().getX() + WORLD_W / 2.0) / GRID_W);
  int32_t startY = std::floor(
      (1.0 * envelope.getLowerLeft().getY() + WORLD_H / 2.0) / GRID_H);

  int32_t endX =
      std::floor((1.0 * envelope.getUpperRight().getX() + WORLD_W / 2.0) /
                 GRID_W) +
      1;
  int32_t endY =
      std::floor((1.0 * envelope.getUpperRight().getY() + WORLD_H / 2.0) /
                 GRID_H) +
      1;

  BoxIdList boxIds;

  getBoxIds(line, envelope, startX, endX, startY, endY, (endX - startX + 3) / 4,
            (endY - startY + 3) / 4, &boxIds, 0, 0);
  std::sort(boxIds.begin(), boxIds.end(), BoxIdCmp());

  return boxIds;
}

// ____________________________________________________________________________
inline BoxIdList getBoxIds(const util::geo::I32XSortedPolygon& poly,
                           const util::geo::I32Box& envelope, double area) {
  int32_t a = getBoxId(envelope.getLowerLeft());
  int32_t b = getBoxId(envelope.getUpperRight());
  if (a == b) return {{-a, 0}};  // shortcut

  int32_t startX = std::floor(
      (1.0 * envelope.getLowerLeft().getX() + WORLD_W / 2.0) / GRID_W);
  int32_t startY = std::floor(
      (1.0 * envelope.getLowerLeft().getY() + WORLD_H / 2.0) / GRID_H);

  int32_t endX =
      std::floor((1.0 * envelope.getUpperRight().getX() + WORLD_W / 2.0) /
                 GRID_W) +
      1;
  int32_t endY =
      std::floor((1.0 * envelope.getUpperRight().getY() + WORLD_H / 2.0) /
                 GRID_H) +
      1;

  BoxIdList boxIds;
  boxIds.reserve((area / (GRID_AREA)) / 10);

  getBoxIds(poly, envelope, area, startX, endX, startY, endY,
            (endX - startX + 3) / 4, (endY - startY + 3) / 4, &boxIds, 0, 0);
  std::sort(boxIds.begin(), boxIds.end(), BoxIdCmp());

  return boxIds;
}

// ____________________________________________________________________________
inline BoxIdList packBoxIds(const BoxIdList& ids) {
  if (ids.empty()) {
    return {{0, 0}};
  }

  if (ids.size() == 1) {
    return {{1 + ids[0].second, 0}, ids[0]};
  }

  // assume the list is sorted!

  BoxIdList ret;
  ret.reserve(ids.size() / 2);
  // dummy value, will later hold number of entries
  ret.push_back({ids.front().second + 1, 0});
  ret.push_back(ids.front());

  for (size_t i = 1; i < ids.size(); i++) {
    ret[0].first += ids[i].second + 1;
    if ((ret.back().second < 254 - ids[i].second && ids[i].first > 0 &&
         ret.back().first > 0 &&
         ret.back().first + ret.back().second == ids[i].first - 1) ||
        (ret.back().second < 254 - ids[i].second && ids[i].first < 0 &&
         ret.back().first < 0 &&
         ret.back().first - ret.back().second == ids[i].first + 1)) {
      ret.back().second += 1 + ids[i].second;
    } else {
      ret.push_back(ids[i]);
    }
  }

  return ret;
}

// ____________________________________________________________________________
inline std::pair<int32_t, int32_t> boxIdIsect(const BoxIdList& idsA,
                                              const BoxIdList& idsB) {
  size_t fullContained = 0;
  size_t partContained = 0;

  // catch empty box ids
  if (idsA.size() < 2 || idsB.size() < 2) return {0, 0};

  // shortcuts
  if (abs(idsA[1].first) > abs(idsB.back().first) + idsB.back().second) {
    return {fullContained, partContained};
  }
  if (abs(idsA.back().first) + idsA.back().second < idsB[1].first) {
    return {fullContained, partContained};
  }

  size_t i = 1;
  int32_t ii = 0;
  size_t j = 1;
  int32_t jj = 0;

  bool noContained = false;

  while (i < idsA.size() && j < idsB.size()) {
    if (abs(idsA[i].first) + ii == abs(idsB[j].first) + jj) {
      if (idsB[j].first > 0) {
        fullContained++;

        // we now know that we surely intersect. If we know already that
        // we cannot be contained, return here
        if (noContained) return {fullContained, partContained};
      }
      if (idsB[j].first < 0) partContained++;

      if (++ii > idsA[i].second) {
        i++;
        ii = 0;
      }
      if (++jj > idsB[j].second) {
        j++;
        jj = 0;
      }
    } else if (abs(idsA[i].first) + ii < abs(idsB[j].first) + jj) {
      // if we already know that we intersect, we are now sure that we
      // cannot be contained - it is irrelevant by how "much" we cannot be
      // contained, so just return
      if (fullContained > 0) return {fullContained, partContained};

      // set noContained marker to true for later
      noContained = true;

      if (abs(idsA[i].first) + idsA[i].second < abs(idsB[j].first) + jj) {
        // entire block smaller, jump it
        ii = 0;
        i++;
      } else {
        if (++ii > idsA[i].second) {
          i++;
          ii = 0;
        }
      }
    } else {
      size_t gallop = 1;
      do {
        auto end = idsB.end();
        if (j + gallop < idsB.size()) {
          end = idsB.begin() + j + gallop;
        }

        if (end == idsB.end() || abs(end->first) >= abs(idsA[i].first) + ii) {
          jj = 0;
          j = std::lower_bound(idsB.begin() + j + gallop / 2, end,
                               abs(idsA[i].first) + ii, BoxIdCmp()) -
              idsB.begin();
          if (j > 0 && abs(idsB[j - 1].first) < abs(idsA[i].first) + ii &&
              abs(idsB[j - 1].first) + idsB[j - 1].second >=
                  abs(idsA[i].first) + ii) {
            j--;
            jj = (abs(idsA[i].first) + ii) - abs(idsB[j].first);
          }
          break;
        }

        gallop *= 2;

      } while (true);
    }
  }

  return {fullContained, partContained};
}

}  // namespace boxids
}  // namespace sj

#endif  // SPATIALJOINS_BOXIDS_H_
