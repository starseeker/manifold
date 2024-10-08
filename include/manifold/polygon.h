// Copyright 2021 The Manifold Authors.
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

#pragma once

#include<array>
#include<vector>

namespace manifold {

/**
 * Boolean operation type: Add (Union), Subtract (Difference), and Intersect.
 */
enum class OpType { Add, Subtract, Intersect };

/**
 * Global parameters that control debugging output. Only has an
 * effect when compiled with the MANIFOLD_DEBUG flag.
 */
struct ExecutionParams {
  /// Perform extra sanity checks and assertions on the intermediate data
  /// structures.
  bool intermediateChecks = false;
  /// Verbose output primarily of the Boolean, including timing info and vector
  /// sizes.
  bool verbose = false;
  /// If processOverlaps is false, a geometric check will be performed to assert
  /// all triangles are CCW.
  bool processOverlaps = true;
  /// Suppresses printed errors regarding CW triangles. Has no effect if
  /// processOverlaps is true.
  bool suppressErrors = false;
  /// Deterministic outputs. Will disable some parallel optimizations.
  bool deterministic = false;
  /// Perform optional but recommended triangle cleanups in SimplifyTopology()
  bool cleanupTriangles = true;
};

/** @addtogroup Private
 *  @{
 */

/* Single polygon contour, wound CCW. First and last point are implicitly
 * connected. Should ensure all input is
 * [&epsilon;-valid](https://github.com/elalish/manifold/wiki/Manifold-Library#definition-of-%CE%B5-valid).
 */
using SimplePolygon = std::vector<std::array<double,2>>;

/**
 * Set of polygons with holes. Order of contours is arbitrary. Can contain any
 * depth of nested holes and any number of separate polygons. Should ensure all
 * input is
 * [&epsilon;-valid](https://github.com/elalish/manifold/wiki/Manifold-Library#definition-of-%CE%B5-valid).
 */
using Polygons = std::vector<SimplePolygon>;

/**
 * Polygon vertex.
 */
struct PolyVert {
  /// X-Y position
  std::array<double,2> pos;
  /// ID or index into another vertex vector
  int idx;
};

using SimplePolygonIdx = std::vector<PolyVert>;
using PolygonsIdx = std::vector<SimplePolygonIdx>;

std::vector<std::array<int,3>> TriangulateIdx(const PolygonsIdx &polys,
                                  double precision = -1);
/** @} */

/** @ingroup Connections
 *  @{
 */
std::vector<std::array<int,3>> Triangulate(const Polygons &polygons, double precision = -1);

ExecutionParams &PolygonParams();
/** @} */
}  // namespace manifold
