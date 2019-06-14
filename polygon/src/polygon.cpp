// Copyright 2019 Emmett Lalish
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <iostream>
#include <map>
#include <stack>

#include "polygon.h"

constexpr bool kVerbose = false;

constexpr float kTolerance = 1e5;

namespace {
using namespace manifold;

struct VertAdj {
  glm::vec2 pos;
  int mesh_idx;             // this is a global index into the mesh
  int right, left, across;  // these are local indices within this vector
  bool merge;
  bool Processed() const { return across >= 0; }
};

int next(int i, int n) { return ++i >= n ? 0 : i; }
int prev(int i, int n) { return --i < 0 ? n - 1 : i; }

int CCW(glm::vec2 p0, glm::vec2 p1, glm::vec2 p2) {
  glm::vec2 v1 = p1 - p0;
  glm::vec2 v2 = p2 - p0;
  float result = v1.x * v2.y - v1.y * v2.x;
  p0 = glm::abs(p0);
  p1 = glm::abs(p1);
  p2 = glm::abs(p2);
  float norm = p0.x * p0.y + p1.x * p1.y + p2.x * p2.y;
  if (std::abs(result) * kTolerance < norm)
    return 0;
  else
    return result > 0 ? 1 : -1;
}

class Monotones {
 public:
  const std::vector<VertAdj> &GetMonotones() { return monotones_; }

  enum VertType { START, END, RIGHTWARDS, LEFTWARDS, MERGE, SPLIT, REV_START };

  Monotones(const Polygons &polys) {
    std::vector<std::tuple<float, int, int>> sweep_line;
    for (const SimplePolygon &poly : polys) {
      int start = Num_verts();
      for (int i = 0; i < poly.size(); ++i) {
        monotones_.push_back({poly[i].pos,                   //
                              poly[i].idx,                   //
                              next(i, poly.size()) + start,  //
                              prev(i, poly.size()) + start,  //
                              -1, false});
        // Ensure sweep line is sorted identically here and in the Triangulator
        // below, including when the y-values are identical.
        sweep_line.push_back(
            std::make_tuple(monotones_.back().pos.y, poly[i].idx, start + i));
        if (kVerbose)
          std::cout << "idx = " << start + i
                    << ", mesh_idx = " << monotones_.back().mesh_idx
                    << std::endl;
      }
    }
    if (kVerbose) std::cout << "starting sweep" << std::endl;
    std::sort(sweep_line.begin(), sweep_line.end());
    VertType v_type = START;
    for (const auto &tmp : sweep_line) {
      v_type = ProcessVert(std::get<2>(tmp));
      if (kVerbose) std::cout << v_type << std::endl;
    }
    Check();
    ALWAYS_ASSERT(v_type == END, logicErr,
                  "Monotones did not finish with an END.");

    // for (int i = sweep_line.size(); i < monotones_.size(); ++i) {
    //   if(kVerbose) std::cout << "idx = " << i << ", mesh_idx = " <<
    //   monotones_[i].mesh_idx
    //             << std::endl;
    // }
  }

  void Check() {
    std::vector<EdgeVerts> edges;
    for (int i = 0; i < monotones_.size(); ++i) {
      edges.emplace_back(i, monotones_[i].right);
      ALWAYS_ASSERT(monotones_[monotones_[i].right].right != i, logicErr,
                    "two-edge monotone!");
      ALWAYS_ASSERT(monotones_[monotones_[i].right].left == i, logicErr,
                    "monotone vert neighbors don't agree!");
    }
    Polygons polys = Assemble(edges);
    Dump(polys);
  }

 private:
  std::vector<VertAdj> monotones_;

  VertAdj &Vert(int idx) { return monotones_[idx]; }
  VertAdj &Right(const VertAdj &v) { return monotones_[v.right]; }
  VertAdj &Left(const VertAdj &v) { return monotones_[v.left]; }
  VertAdj &Across(const VertAdj &v) { return monotones_[v.across]; }
  int Num_verts() const { return monotones_.size(); }

  void Match(int idx1, int idx2) {
    if (kVerbose)
      std::cout << "matched " << idx1 << " and " << idx2 << std::endl;
    Vert(idx1).across = idx2;
    Vert(idx2).across = idx1;
  }

  void Link(int left_idx, int right_idx) {
    Vert(left_idx).right = right_idx;
    Vert(right_idx).left = left_idx;
  }

  void Duplicate(int v_idx) {
    Vert(v_idx).merge = true;
    int v_right_idx = Num_verts();
    monotones_.push_back(Vert(v_idx));
    Right(Vert(v_idx)).left = v_right_idx;
    if (Left(Vert(v_idx)).Processed())
      Match(v_idx, Helper(v_idx, Vert(v_idx).left));
    else
      Vert(v_idx).across = v_idx;
    if (Right(Vert(v_idx)).Processed())
      Match(v_right_idx, Helper(v_idx, Vert(v_idx).right));
    else
      Vert(v_right_idx).across = v_right_idx;
    Link(v_idx, v_right_idx);
  }

  int SplitVerts(int v_idx, int left_dupe_idx) {
    // at split events, add duplicate vertices to end of list and reconnect
    if (kVerbose)
      std::cout << "split from " << v_idx << " to " << left_dupe_idx
                << std::endl;
    Vert(left_dupe_idx).merge = false;
    Right(Vert(left_dupe_idx)).merge = false;
    int newVert_idx = Num_verts();
    monotones_.push_back(Vert(v_idx));
    Left(Vert(newVert_idx)).right = newVert_idx;
    Link(newVert_idx, Vert(left_dupe_idx).right);
    Link(left_dupe_idx, v_idx);
    return newVert_idx;
  }

  int Helper(int v_idx, int neighbor_idx) {
    int helper_idx = Vert(neighbor_idx).across;
    if (helper_idx == v_idx) helper_idx = neighbor_idx;
    return helper_idx;
  }

  int PositiveExteriorHelper(int v_idx) {
    // find nearest sweep line crossing -X of this vertex
    float best_x = -std::numeric_limits<float>::infinity();
    int helper_idx = -1;
    int winding = 0;
    for (int i = 0; i < Num_verts(); ++i) {
      if (Vert(i).Processed() != Left(Vert(i)).Processed()) {  // active edge
        float a = (Vert(i).pos.y - Vert(v_idx).pos.y) /
                  (Vert(i).pos.y - Left(Vert(i)).pos.y);
        if (std::isnan(a)) a = 0.0f;
        a = std::max(std::min(a, 1.0f), 0.0f);
        float x = (1.0f - a) * Vert(i).pos.x + a * Left(Vert(i)).pos.x;
        if (kVerbose)
          std::cout << "x = " << x << ", v_x = " << Vert(v_idx).pos.x
                    << std::endl;
        if (x < Vert(v_idx).pos.x) {
          winding += Vert(i).Processed() ? 1 : -1;
          if (Vert(i).Processed() && x > best_x) {  // Rightward & nearest
            best_x = x;
            helper_idx = i;
          }
        }
      }
    }
    if (kVerbose) std::cout << "winding = " << winding << std::endl;
    // only return helper if geometrically valid
    return winding == 1 ? helper_idx : -1;
  }

  VertType ProcessVert(int idx) {
    auto &vert = Vert(idx);
    if (kVerbose)
      std::cout << "idx = " << idx << ", mesh_idx = " << vert.mesh_idx
                << std::endl;
    if (Right(vert).Processed()) {
      if (Left(vert).Processed()) {
        if (Right(vert).across == vert.left) {  // End
          return END;
        } else if (Across(Right(vert)).right == Left(vert).across &&
                   Across(Right(vert)).merge) {  // Split End
          SplitVerts(idx, Right(vert).across);
          return END;
        } else {  // Merge
          Duplicate(idx);
          if (Across(Vert(idx)).merge) {
            int helper_idx = Across(Vert(idx)).left;
            SplitVerts(idx, helper_idx);
            Match(idx, Vert(helper_idx).across);
          }
          if (Across(Right(Vert(idx))).merge) {
            int newVert_idx =
                SplitVerts(Vert(idx).right, Right(Vert(idx)).across);
            Match(newVert_idx, Right(Vert(newVert_idx)).across);
          }
          return MERGE;
        }
      } else {  // Leftwards
        int helper_idx = Helper(idx, vert.right);
        if (Vert(helper_idx).merge) {
          int newVert_idx = SplitVerts(idx, helper_idx);
          Match(newVert_idx, Right(Vert(newVert_idx)).across);
        } else
          Match(idx, helper_idx);
        return LEFTWARDS;
      }
    } else {
      if (Left(vert).Processed()) {  // Rightwards
        int helper_idx = Helper(idx, vert.left);
        if (Vert(helper_idx).merge) {
          helper_idx = Vert(helper_idx).left;
          SplitVerts(idx, helper_idx);
          Match(idx, Vert(helper_idx).across);
        } else
          Match(idx, helper_idx);
        return RIGHTWARDS;
      } else {
        if (CCW(vert.pos, Right(vert).pos, Left(vert).pos) >= 0) {  // Start
          vert.across = idx;
          return START;
        } else {
          int helper_idx = PositiveExteriorHelper(idx);
          if (helper_idx >= 0) {  // Split
            if (Vert(helper_idx).pos.y < Across(Vert(helper_idx)).pos.y)
              helper_idx = Vert(helper_idx).across;
            if (!Vert(helper_idx).merge) Duplicate(helper_idx);
            int newVert_idx = SplitVerts(idx, helper_idx);
            Match(newVert_idx, Right(Vert(newVert_idx)).across);
            Match(idx, Vert(helper_idx).across);
            return SPLIT;
          } else {  // Reversed start
            vert.across = idx;
            return REV_START;
          }
        }
      }
    }
  }
};

class Triangulator {
 public:
  Triangulator(const std::vector<VertAdj> &monotones, int v_idx)
      : monotones_(monotones) {
    reflex_chain_.push(v_idx);
    other_side_ = v_idx;
  }
  int NumTriangles() { return triangles_output; }

  bool ProcessVert(int vi_idx, std::vector<TriVerts> &triangles) {
    int attached = Attached(vi_idx);
    if (attached == 0)
      return 0;
    else {
      const VertAdj vi = monotones_[vi_idx];
      int v_top_idx = reflex_chain_.top();
      VertAdj v_top = monotones_[v_top_idx];
      if (reflex_chain_.size() < 2) {
        reflex_chain_.push(vi_idx);
        onRight_ = vi.left == v_top_idx;
        return 1;
      }
      reflex_chain_.pop();
      int vj_idx = reflex_chain_.top();
      VertAdj vj = monotones_[vj_idx];
      if (attached == 1) {
        if (kVerbose) std::cout << "same chain" << std::endl;
        while (CCW(vi.pos, vj.pos, v_top.pos) == (onRight_ ? 1 : -1)) {
          AddTriangle(triangles, vi.mesh_idx, vj.mesh_idx, v_top.mesh_idx);
          v_top_idx = vj_idx;
          reflex_chain_.pop();
          if (reflex_chain_.size() == 0) break;
          v_top = vj;
          vj_idx = reflex_chain_.top();
          vj = monotones_[vj_idx];
        }
        reflex_chain_.push(v_top_idx);
        reflex_chain_.push(vi_idx);
      } else {
        if (kVerbose) std::cout << "different chain" << std::endl;
        onRight_ = !onRight_;
        VertAdj v_last = v_top;
        while (!reflex_chain_.empty()) {
          vj = monotones_[reflex_chain_.top()];
          AddTriangle(triangles, vi.mesh_idx, v_last.mesh_idx, vj.mesh_idx);
          v_last = vj;
          reflex_chain_.pop();
        }
        reflex_chain_.push(v_top_idx);
        reflex_chain_.push(vi_idx);
        other_side_ = v_top_idx;
      }
      return 1;
    }
  }

 private:
  const std::vector<VertAdj> &monotones_;
  std::stack<int> reflex_chain_;
  int other_side_;
  int triangles_output = 0;
  bool onRight_;

  const VertAdj GetTop() { return monotones_[reflex_chain_.top()]; }
  const VertAdj GetOther() { return monotones_[other_side_]; }

  int Attached(int v_idx) {
    if (onRight_) {
      if (GetOther().left == v_idx)
        return -1;
      else if (GetTop().right == v_idx)
        return 1;
      else
        return 0;
    } else {
      if (GetOther().right == v_idx)
        return -1;
      else if (GetTop().left == v_idx)
        return 1;
      else
        return 0;
    }
  }

  void AddTriangle(std::vector<TriVerts> &triangles, int v0, int v1, int v2) {
    if (onRight_)
      triangles.emplace_back(v0, v1, v2);
    else
      triangles.emplace_back(v0, v2, v1);
    ++triangles_output;
  }
};

void TriangulateMonotones(const std::vector<VertAdj> &monotones,
                          std::vector<TriVerts> &triangles) {
  // make sorted index list to traverse the sweep line.
  std::vector<std::tuple<float, int, int>> sweep_line;
  for (int i = 0; i < monotones.size(); ++i) {
    // Ensure sweep line is sorted identically here and in Monotones
    // above, including when the y-values are identical.
    sweep_line.push_back(
        std::make_tuple(monotones[i].pos.y, monotones[i].mesh_idx, i));
  }
  std::sort(sweep_line.begin(), sweep_line.end());
  std::vector<Triangulator> triangulators;
  for (int i = 0; i < sweep_line.size(); ++i) {
    const int v_idx = std::get<2>(sweep_line[i]);
    if (kVerbose)
      std::cout << "i = " << i << ", v_idx = " << v_idx
                << ", mesh_idx = " << monotones[v_idx].mesh_idx << std::endl;
    bool found = false;
    for (int i = 0; i < triangulators.size(); ++i) {
      if (triangulators[i].ProcessVert(v_idx, triangles)) {
        found = true;
        if (kVerbose)
          std::cout << "in triangulator " << i << ", with "
                    << triangulators[i].NumTriangles() << " triangles so far"
                    << std::endl;
        break;
      }
    }
    if (!found) triangulators.emplace_back(monotones, v_idx);
  }
  // quick validation
  int triangles_left = monotones.size();
  for (auto &triangulator : triangulators) {
    triangles_left -= 2;
    // ALWAYS_ASSERT(triangulator.NumTriangles() > 0, logicErr,
    //               "Monotone produced no triangles.");
    triangles_left -= triangulator.NumTriangles();
  }
  ALWAYS_ASSERT(triangles_left == 0, logicErr,
                "Triangulation produced wrong number of triangles.");
}
}  // namespace

namespace manifold {

Polygons Assemble(const std::vector<EdgeVerts> &halfedges) {
  Polygons polys;
  std::map<int, int> vert_edge;
  for (int i = 0; i < halfedges.size(); ++i) {
    ALWAYS_ASSERT(
        vert_edge.emplace(std::make_pair(halfedges[i].first, i)).second,
        runtimeErr, "polygon has duplicate vertices.");
  }
  auto startEdge = halfedges.begin();
  auto thisEdge = halfedges.begin();
  for (;;) {
    if (thisEdge == startEdge) {
      if (vert_edge.empty()) break;
      startEdge = std::next(halfedges.begin(), vert_edge.begin()->second);
      thisEdge = startEdge;
      polys.push_back({});
    }
    polys.back().push_back({glm::vec2(1.0f / 0.0f), thisEdge->first});
    auto result = vert_edge.find(thisEdge->second);
    ALWAYS_ASSERT(result != vert_edge.end(), runtimeErr, "nonmanifold edge");
    thisEdge = std::next(halfedges.begin(), result->second);
    vert_edge.erase(result);
  }
  return polys;
}

void Triangulate(std::vector<TriVerts> &triangles, const Polygons &polys) {
  Monotones monotones(polys);
  TriangulateMonotones(monotones.GetMonotones(), triangles);
}

std::vector<EdgeVerts> Polygons2Edges(const Polygons &polys) {
  std::vector<EdgeVerts> halfedges;
  for (const auto &poly : polys) {
    for (int i = 1; i < poly.size(); ++i) {
      halfedges.emplace_back(poly[i - 1].idx, poly[i].idx);
    }
    halfedges.emplace_back(poly.back().idx, poly[0].idx);
  }
  return halfedges;
}

std::vector<EdgeVerts> Triangles2Edges(const std::vector<TriVerts> &triangles) {
  std::vector<EdgeVerts> halfedges;
  for (const TriVerts &tri : triangles) {
    halfedges.emplace_back(tri[0], tri[1]);
    halfedges.emplace_back(tri[1], tri[2]);
    halfedges.emplace_back(tri[2], tri[0]);
  }
  return halfedges;
}

void CheckManifold(const std::vector<EdgeVerts> &halfedges) {
  ALWAYS_ASSERT(halfedges.size() % 2 == 0, runtimeErr,
                "Odd number of halfedges.");
  size_t n_edges = halfedges.size() / 2;
  std::vector<EdgeVerts> forward(halfedges.size()), backward(halfedges.size());

  auto end = std::copy_if(halfedges.begin(), halfedges.end(), forward.begin(),
                          [](EdgeVerts e) { return e.second > e.first; });
  ALWAYS_ASSERT(std::distance(forward.begin(), end) == n_edges, runtimeErr,
                "Half of halfedges should be forward.");
  forward.resize(n_edges);

  end = std::copy_if(halfedges.begin(), halfedges.end(), backward.begin(),
                     [](EdgeVerts e) { return e.second < e.first; });
  ALWAYS_ASSERT(std::distance(backward.begin(), end) == n_edges, runtimeErr,
                "Half of halfedges should be backward.");
  backward.resize(n_edges);

  std::for_each(backward.begin(), backward.end(),
                [](EdgeVerts &e) { std::swap(e.first, e.second); });
  std::sort(forward.begin(), forward.end());
  std::sort(backward.begin(), backward.end());
  for (int i = 0; i < forward.size(); ++i) {
    ALWAYS_ASSERT(forward[i].first == backward[i].first &&
                      forward[i].second == backward[i].second,
                  runtimeErr, "Forward and backward edge do not match.");
  }
  ALWAYS_ASSERT(
      forward.end() == std::adjacent_find(forward.begin(), forward.end()),
      runtimeErr, "Not a 2-manifold.");
  ALWAYS_ASSERT(
      backward.end() == std::adjacent_find(backward.begin(), backward.end()),
      runtimeErr, "Not a 2-manifold.");
}

void CheckManifold(const std::vector<TriVerts> &triangles,
                   const Polygons &polys) {
  std::vector<EdgeVerts> halfedges = Triangles2Edges(triangles);
  std::vector<EdgeVerts> openEdges = Polygons2Edges(polys);
  for (EdgeVerts e : openEdges) {
    halfedges.emplace_back(e.second, e.first);
  }
  CheckManifold(halfedges);
}

void Dump(const Polygons &polys) {
  for (const SimplePolygon &poly : polys) {
    if (kVerbose) std::cout << "next poly" << std::endl;
    for (const PolyVert &v : poly) {
      if (kVerbose)
        std::cout << v.pos.x << ", " << v.pos.y << ", " << v.idx << std::endl;
    }
  }
}

}  // namespace manifold