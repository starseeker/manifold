#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#if (MANIFOLD_PAR == 1) && __has_include(<pstl/glue_execution_defs.h>)
#include <execution>
#endif

#include "../src/utils.h"
#include "manifold/manifold.h"
#include "manifold/polygon.h"

using namespace manifold;

inline double cross(vec2 p, vec2 q) { return p.x * q.y - p.y * q.x; }

// return true if p intersects with q
// note that we don't care about collinear, intersection in the ends etc.
// https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
inline bool intersect(vec2 p0, vec2 p1, vec2 q0, vec2 q1, double precision) {
  vec2 r = p1 - p0;
  vec2 s = q1 - q0;
  // allow some error in the intersection point
  double epsilon_r = 2.0 * precision / la::length(r);
  double epsilon_s = 2.0 * precision / la::length(s);
  double rxs = cross(r, s);
  // in case they are nearly collinear, ignore them...
  // this is to avoid treating degenerate triangles as intersecting
  //
  // note that this does not correspond to a fixed angle,
  // but seems to work well enough
  if (rxs < kPrecision && rxs > -kPrecision) {
    return false;
  }
  double u = cross(q0 - p0, r) / rxs;
  double t = cross(q0 - p0, s) / rxs;
  // we only care about intersection in the middle of both lines, excluding
  // their ends
  if (epsilon_r <= u && u <= (1 - epsilon_r) &&  //
      epsilon_s <= t && t <= (1 - epsilon_s)) {
    // there can be cases which r and s are close to collinear,
    // so even though the lines are not intersecting,
    // perturbation along r/s is not enough.
    //
    // in that case, apply perturbation perpendicular to r
    vec2 r_orth = la::normalize(vec2(-r.y, r.x)) * precision;
    double u1 = cross(q0 - p0 + r_orth, r) / rxs;
    double u2 = cross(q0 - p0 - r_orth, r) / rxs;
    double t1 = cross(q0 - p0 + r_orth, s) / rxs;
    double t2 = cross(q0 - p0 - r_orth, s) / rxs;
    return 0 <= u1 && u1 <= 1 &&  //
           0 <= t1 && t1 <= 1 &&  //
           0 <= u2 && u2 <= 1 &&  //
           0 <= t2 && t2 <= 1;
    return true;
  } else {
    return false;
  }
}

// check if removing point j in polygon i will introduce self-intersection
// this checks if the new edge j-1 <-> j+1 intersects with any other edges,
// assuming the original polygon does not contain self-intersection
bool safeToRemove(const Polygons& polys, size_t i, size_t j, double precision) {
  if (polys[i].size() == 3) return false;
  vec2 prev = polys[i][j == 0 ? polys[i].size() - 1 : (j - 1)];
  vec2 next = polys[i][j == (polys[i].size() - 1) ? 0 : (j + 1)];
  for (size_t k = 0; k < polys.size(); k++) {
    auto ll = [&](size_t l) {
      return l == (polys[k].size() - 1) ? 0 : (l + 1);
    };
    const vec2* polysk = polys[k].data();
    if (!std::all_of(
#if (MANIFOLD_PAR == 1) && __has_include(<pstl/glue_execution_defs.h>)
            std::execution::par,
#endif
            countAt(0_uz), countAt(polys[k].size()), [=](size_t l) {
              if (i == k && (l == j || ll(l) == j)) return true;
              return !intersect(prev, next, polysk[l], polysk[ll(l)],
                                precision);
            }))
      return false;
  }
  return true;
}

std::pair<int, int> findIndex(const Polygons& polys, size_t i) {
  size_t outer = 0;
  while (i >= polys[outer].size()) i -= polys[outer++].size();
  return std::make_pair(outer, i);
}

void Dump(const Polygons& polys, const double epsilon) {
  std::cout << std::setprecision(19);
  std::cout << "Polygon 0 " << epsilon << " " << polys.size() << std::endl;
  for (auto poly : polys) {
    std::cout << poly.size() << std::endl;
    for (auto v : poly) {
      std::cout << v.x << " " << v.y << std::endl;
    }
  }
  std::cout << "# ... " << std::endl;
  for (auto poly : polys) {
    std::cout << "show(array([" << std::endl;
    for (auto v : poly) {
      std::cout << "  [" << v.x << ", " << v.y << "]," << std::endl;
    }
    std::cout << "]))" << std::endl;
  }
}

void DumpTriangulation(const Polygons& polys, double precision) {
  ExecutionParams oldParams = ManifoldParams();
  manifold::ManifoldParams().processOverlaps = true;
  std::vector<ivec3> result = Triangulate(polys);
  ManifoldParams() = oldParams;
  for (const ivec3& tri : result) {
    std::pair<int, int> xid = findIndex(polys, tri.x);
    std::pair<int, int> yid = findIndex(polys, tri.y);
    std::pair<int, int> zid = findIndex(polys, tri.z);
    vec2 x = polys[xid.first][xid.second];
    vec2 y = polys[yid.first][yid.second];
    vec2 z = polys[zid.first][zid.second];
    printf("show(array([[%.7f, %.7f], [%.7f, %.7f], [%.7f, %.7f]]))\n", x.x,
           x.y, y.x, y.y, z.x, z.y);
    if (CCW(x, y, z, precision) == -1) {
      printf("^ not CCW\n");
    }
  }
}

// x direction ray
// idk which direction, but it is a fixed direction...
// https://stackoverflow.com/questions/11716268/point-in-polygon-algorithm
bool rayHit(vec2 point, vec2 q0, vec2 q1) {
  return ((q0.y >= point.y) != (q1.y >= point.y)) &&
         (point.x <= (q1.x - q0.x) * (point.y - q0.y) / (q1.y - q0.y) + q0.x);
}

// get the list of polygons inside polygon i
// this enumerates over the first point in all other polygons,
// and check if they are inside the current polygon
// this algorithm assumes that polygons are non-intersecting
std::vector<int> getChildren(const Polygons& polys, size_t i) {
  std::vector<int> results;
  for (size_t j = 0; j < polys.size(); j++) {
    if (i == j) continue;
    vec2 point = polys[j][0];
    auto k1 = [&](size_t k) {
      return k == (polys[i].size() - 1) ? 0 : (k + 1);
    };
    int count = std::count_if(
#if (MANIFOLD_PAR == 1) && __has_include(<pstl/glue_execution_defs.h>)
        std::execution::par,
#endif
        countAt((size_t)0), countAt(polys[i].size()),
        [&](size_t k) { return rayHit(point, polys[i][k], polys[i][k1(k)]); });
    if (count % 2) results.push_back(j);
  }
  return results;
}

// assuming polys is valid
// we try to remove vertices from polys such that
// 1. the updated polys is still valid (no overlapping edges, correct winding
// direction)
// 2. same error in triangulation (either geometryErr or overlapping triangles)
void simplify(Polygons& polys, double precision = -1) {
  bool removedSomething = true;
  std::string msg;
  while (removedSomething) {
    int points = 0;
    for (const SimplePolygon& poly : polys) points += poly.size();
    std::cout << "trying to simplify " << points << " points" << std::endl;

    removedSomething = false;
    // try to remove simple polygons
    for (size_t i = 0; i < polys.size(); i++) {
      std::vector<int> children = getChildren(polys, i);
      // if there are children, we can't remove it or we will mess up with
      // winding direction...
      if (!children.empty()) continue;
      SimplePolygon poly = std::move(polys[i]);
      polys.erase(polys.begin() + i);
      try {
        std::vector<ivec3> result = Triangulate(polys, precision);
        polys.insert(polys.begin() + i, std::move(poly));
      } catch (geometryErr& e) {
        if (msg.size() > 0 && msg.compare(e.what()) != 0) {
          polys.insert(polys.begin() + i, std::move(poly));
        } else {
          removedSomething = true;
          msg = e.what();
        }
      } catch (topologyErr& e) {
        polys.insert(polys.begin() + i, std::move(poly));
      }
    }

    for (size_t i = 0; i < polys.size(); i++) {
      std::vector<int> children = getChildren(polys, i);
      for (size_t j = 0; j < polys[i].size(); j++) {
        // removed vertex cannot change inclusion relation
        // we just check if the vertex
        // x: intersects with j0, j (original edge 1)
        // y: intersects with j, j1 (original edge 2)
        // z: intersects with j0, j1 (new edge)
        // if (x ^ y ^ z) is true, it means that the count % 2 is changed,
        // and we changed inclusion relation, so vertex j cannot be removed
        size_t j0 = j == 0 ? polys[i].size() - 1 : j - 1;
        size_t j1 = j == polys[i].size() - 1 ? 0 : j + 1;
        if (std::any_of(children.begin(), children.end(), [&](int k) {
              return rayHit(polys[k][0], polys[i][j0], polys[i][j]) ^
                     rayHit(polys[k][0], polys[i][j], polys[i][j1]) ^
                     rayHit(polys[k][0], polys[i][j0], polys[i][j1]);
            }))
          continue;

        // removed vertex cannot introduce intersection
        if (!safeToRemove(polys, i, j, precision)) continue;
        vec2 removed = polys[i][j];
        polys[i].erase(polys[i].begin() + j);
        try {
          std::vector<ivec3> result = Triangulate(polys, precision);
          polys[i].insert(polys[i].begin() + j, removed);
        } catch (geometryErr& e) {
          if (msg.size() > 0 && msg.compare(e.what()) != 0) {
            polys[i].insert(polys[i].begin() + j, removed);
          } else {
            removedSomething = true;
            msg = e.what();
          }
        } catch (topologyErr& e) {
          polys[i].insert(polys[i].begin() + j, removed);
        }
      }
    }
  }
  std::cout << "Error message for triangulation:" << std::endl
            << msg.c_str() << std::endl;
}

struct Edge {
  vec2 west;    // -x side vertex
  vec2 east;    // +x side vertex
  size_t i, j;  // indices of origin vertex
  bool operator<(const Edge& other) const {
    return west.x < other.west.x ||
           (west.x == other.west.x && east.x < other.east.x);
  }
  Edge(const Polygons& polys, size_t i, size_t j) : i(i), j(j) {
    size_t j1 = j == polys[i].size() - 1 ? 0 : j + 1;
    east = polys[i][j];
    west = polys[i][j1];
    if (west.x > east.x) std::swap(east, west);
  }
};

int isValid(const Polygons& polys, double precision = -1) {
  size_t numEdge = 0;
  for (const SimplePolygon& poly : polys) numEdge += poly.size();
  std::vector<Edge> edges;
  edges.reserve(numEdge);
  for (size_t i = 0; i < polys.size(); i++)
    for (size_t j = 0; j < polys[i].size(); j++)
      edges.push_back(Edge(polys, i, j));
  std::sort(edges.begin(), edges.end());
  // check intersection
  std::mutex mutex;
  std::vector<std::pair<Edge, Edge>> overlappingPairs;

  std::for_each(  //
      countAt((size_t)0), countAt(edges.size()), [&](size_t i) {
        // check all subsequent edges e' until e'.west.x > e.east.x
        // this ensures we checked all edges e' where
        //      e.west.x < e'.west.x < e.east.x
        Edge e = edges[i];
        for (size_t j = i + 1; j < edges.size() && edges[j].west.x < e.east.x;
             j++) {
          if (intersect(e.west, e.east, edges[j].west, edges[j].east,
                        precision)) {
            mutex.lock();
            overlappingPairs.push_back(std::make_pair(e, edges[j]));
            mutex.unlock();
          }
        }
      });

  if (!overlappingPairs.empty()) {
    std::cout << "found " << overlappingPairs.size() << " self-intersection"
              << std::endl;
    for (const std::pair<Edge, Edge>& pairs : overlappingPairs) {
      std::cout << pairs.first.i << " " << pairs.first.j << "<->"  //
                << pairs.second.i << " " << pairs.second.j << std::endl;
      printf("array([[%.7f, %.7f], [%.7f, %.7f]])\n", pairs.first.west.x,
             pairs.first.west.y, pairs.first.east.x, pairs.first.east.y);
      printf("array([[%.7f, %.7f], [%.7f, %.7f]])\n", pairs.second.west.x,
             pairs.second.west.y, pairs.second.east.x, pairs.second.east.y);
    }
  } else {
    std::cout << "No self-intersection" << std::endl;
  }

  return 0;
}

int main(int argc, char** argv) {
  ManifoldParams().intermediateChecks = true;
  ManifoldParams().processOverlaps = false;
  ManifoldParams().suppressErrors = true;

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input file>" << std::endl;
    return 0;
  }
  std::ifstream f(argv[1]);

  std::string name;
  double epsilon, x, y;
  int expectedNumTri, numPolys, numPoints;

  f >> name >> expectedNumTri >> epsilon >> numPolys;
  Polygons polys;
  for (int i = 0; i < numPolys; i++) {
    polys.emplace_back();
    f >> numPoints;
    for (int j = 0; j < numPoints; j++) {
      f >> x >> y;
      polys.back().emplace_back(x, y);
    }
  }
  f.close();

  std::cout << "------------" << std::endl;

  isValid(polys, epsilon);

  simplify(polys, epsilon);

  std::cout << "------------" << std::endl;
  std::cout << "Final polygon:" << std::endl;
  Dump(polys, epsilon);

  std::cout << "------------" << std::endl;
  std::cout << "Erroneous triangulation:" << std::endl;
  DumpTriangulation(polys, epsilon);
  return 0;
}
