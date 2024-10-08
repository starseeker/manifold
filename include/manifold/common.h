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
#define GLM_ENABLE_EXPERIMENTAL  // needed for glm/gtx/compatibility.hpp
#define GLM_FORCE_EXPLICIT_CTOR
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

namespace manifold {
/** @defgroup Math data structure definitions
 *  @brief Abstract away from glm.
 *  In the future the underlying data type can change.
 *  @{
 */
using vec2 = glm::dvec2;
using vec3 = glm::dvec3;
using vec4 = glm::dvec4;
using mat2 = glm::dmat2;
using mat2x3 = glm::dmat2x3;
using mat2x4 = glm::dmat2x4;
using mat3x2 = glm::dmat3x2;
using mat3 = glm::dmat3;
using mat3x4 = glm::dmat3x4;
using mat4x3 = glm::dmat4x3;
using mat4 = glm::dmat4;
using ivec2 = glm::vec<2, int>;
using ivec3 = glm::vec<3, int>;
using ivec4 = glm::vec<4, int>;
using quat = glm::dquat;
///@}

/**
 * Sine function where multiples of 90 degrees come out exact.
 *
 * @param x Angle in degrees.
 */
inline double sind(double x) {
  if (!std::isfinite(x)) return sin(x);
  if (x < 0.0) return -sind(-x);
  int quo;
  x = remquo(fabs(x), 90.0, &quo);
  switch (quo % 4) {
    case 0:
      return sin(glm::radians(x));
    case 1:
      return cos(glm::radians(x));
    case 2:
      return -sin(glm::radians(x));
    case 3:
      return -cos(glm::radians(x));
  }
  return 0.0;
}

/**
 * Cosine function where multiples of 90 degrees come out exact.
 *
 * @param x Angle in degrees.
 */
inline double cosd(double x) { return sind(x + 90.0); }

/**
 * Single polygon contour, wound CCW. First and last point are implicitly
 * connected. Should ensure all input is
 * [&epsilon;-valid](https://github.com/elalish/manifold/wiki/Manifold-Library#definition-of-%CE%B5-valid).
 */
using SimplePolygon = std::vector<vec2>;

/**
 * Set of polygons with holes. Order of contours is arbitrary. Can contain any
 * depth of nested holes and any number of separate polygons. Should ensure all
 * input is
 * [&epsilon;-valid](https://github.com/elalish/manifold/wiki/Manifold-Library#definition-of-%CE%B5-valid).
 */
using Polygons = std::vector<SimplePolygon>;

/**
 * Defines which edges to sharpen and how much for the Manifold.Smooth()
 * constructor.
 */
struct Smoothness {
  /// The halfedge index = 3 * tri + i, referring to Mesh.triVerts[tri][i].
  size_t halfedge;
  /// A value between 0 and 1, where 0 is sharp and 1 is the default and the
  /// curvature is interpolated between these values. The two paired halfedges
  /// can have different values while maintaining C-1 continuity (except for 0).
  double smoothness;
};

/**
 * Geometric properties of the manifold, created with Manifold.GetProperties().
 */
struct Properties {
  double surfaceArea, volume;
};

struct Box {
  vec3 min = vec3(std::numeric_limits<double>::infinity());
  vec3 max = vec3(-std::numeric_limits<double>::infinity());

  /**
   * Default constructor is an infinite box that contains all space.
   */
  Box() {}

  /**
   * Creates a box that contains the two given points.
   */
  Box(const vec3 p1, const vec3 p2) {
    min = glm::min(p1, p2);
    max = glm::max(p1, p2);
  }

  /**
   * Returns the dimensions of the Box.
   */
  vec3 Size() const { return max - min; }

  /**
   * Returns the center point of the Box.
   */
  vec3 Center() const { return 0.5 * (max + min); }

  /**
   * Returns the absolute-largest coordinate value of any contained
   * point.
   */
  double Scale() const {
    vec3 absMax = glm::max(glm::abs(min), glm::abs(max));
    return glm::max(absMax.x, glm::max(absMax.y, absMax.z));
  }

  /**
   * Does this box contain (includes equal) the given point?
   */
  bool Contains(const vec3& p) const {
    return glm::all(glm::greaterThanEqual(p, min)) &&
           glm::all(glm::greaterThanEqual(max, p));
  }

  /**
   * Does this box contain (includes equal) the given box?
   */
  bool Contains(const Box& box) const {
    return glm::all(glm::greaterThanEqual(box.min, min)) &&
           glm::all(glm::greaterThanEqual(max, box.max));
  }

  /**
   * Expand this box to include the given point.
   */
  void Union(const vec3 p) {
    min = glm::min(min, p);
    max = glm::max(max, p);
  }

  /**
   * Expand this box to include the given box.
   */
  Box Union(const Box& box) const {
    Box out;
    out.min = glm::min(min, box.min);
    out.max = glm::max(max, box.max);
    return out;
  }

  /**
   * Transform the given box by the given axis-aligned affine transform.
   *
   * Ensure the transform passed in is axis-aligned (rotations are all
   * multiples of 90 degrees), or else the resulting bounding box will no longer
   * bound properly.
   */
  Box Transform(const mat4x3& transform) const {
    Box out;
    vec3 minT = transform * vec4(min, 1.0);
    vec3 maxT = transform * vec4(max, 1.0);
    out.min = glm::min(minT, maxT);
    out.max = glm::max(minT, maxT);
    return out;
  }

  /**
   * Shift this box by the given vector.
   */
  Box operator+(vec3 shift) const {
    Box out;
    out.min = min + shift;
    out.max = max + shift;
    return out;
  }

  /**
   * Shift this box in-place by the given vector.
   */
  Box& operator+=(vec3 shift) {
    min += shift;
    max += shift;
    return *this;
  }

  /**
   * Scale this box by the given vector.
   */
  Box operator*(vec3 scale) const {
    Box out;
    out.min = min * scale;
    out.max = max * scale;
    return out;
  }

  /**
   * Scale this box in-place by the given vector.
   */
  Box& operator*=(vec3 scale) {
    min *= scale;
    max *= scale;
    return *this;
  }

  /**
   * Does this box overlap the one given (including equality)?
   */
  inline bool DoesOverlap(const Box& box) const {
    return min.x <= box.max.x && min.y <= box.max.y && min.z <= box.max.z &&
           max.x >= box.min.x && max.y >= box.min.y && max.z >= box.min.z;
  }

  /**
   * Does the given point project within the XY extent of this box
   * (including equality)?
   */
  inline bool DoesOverlap(vec3 p) const {  // projected in z
    return p.x <= max.x && p.x >= min.x && p.y <= max.y && p.y >= min.y;
  }

  /**
   * Does this box have finite bounds?
   */
  bool IsFinite() const {
    return glm::all(glm::isfinite(min)) && glm::all(glm::isfinite(max));
  }
};

/**
 * Axis-aligned rectangular bounds.
 */
struct Rect {
  vec2 min = vec2(std::numeric_limits<double>::infinity());
  vec2 max = vec2(-std::numeric_limits<double>::infinity());

  /**
   * Default constructor is an empty rectangle..
   */
  Rect() {}

  /**
   * Create a rectangle that contains the two given points.
   */
  Rect(const vec2 a, const vec2 b) {
    min = glm::min(a, b);
    max = glm::max(a, b);
  }

  /** @name Information
   *  Details of the rectangle
   */
  ///@{

  /**
   * Return the dimensions of the rectangle.
   */
  vec2 Size() const { return max - min; }

  /**
   * Return the area of the rectangle.
   */
  double Area() const {
    auto sz = Size();
    return sz.x * sz.y;
  }

  /**
   * Returns the absolute-largest coordinate value of any contained
   * point.
   */
  double Scale() const {
    vec2 absMax = glm::max(glm::abs(min), glm::abs(max));
    return glm::max(absMax.x, absMax.y);
  }

  /**
   * Returns the center point of the rectangle.
   */
  vec2 Center() const { return 0.5 * (max + min); }

  /**
   * Does this rectangle contain (includes on border) the given point?
   */
  bool Contains(const vec2& p) const {
    return glm::all(glm::greaterThanEqual(p, min)) &&
           glm::all(glm::greaterThanEqual(max, p));
  }

  /**
   * Does this rectangle contain (includes equal) the given rectangle?
   */
  bool Contains(const Rect& rect) const {
    return glm::all(glm::greaterThanEqual(rect.min, min)) &&
           glm::all(glm::greaterThanEqual(max, rect.max));
  }

  /**
   * Does this rectangle overlap the one given (including equality)?
   */
  bool DoesOverlap(const Rect& rect) const {
    return min.x <= rect.max.x && min.y <= rect.max.y && max.x >= rect.min.x &&
           max.y >= rect.min.y;
  }

  /**
   * Is the rectangle empty (containing no space)?
   */
  bool IsEmpty() const { return max.y <= min.y || max.x <= min.x; };

  /**
   * Does this recangle have finite bounds?
   */
  bool IsFinite() const {
    return glm::all(glm::isfinite(min)) && glm::all(glm::isfinite(max));
  }

  ///@}

  /** @name Modification
   */
  ///@{

  /**
   * Expand this rectangle (in place) to include the given point.
   */
  void Union(const vec2 p) {
    min = glm::min(min, p);
    max = glm::max(max, p);
  }

  /**
   * Expand this rectangle to include the given Rect.
   */
  Rect Union(const Rect& rect) const {
    Rect out;
    out.min = glm::min(min, rect.min);
    out.max = glm::max(max, rect.max);
    return out;
  }

  /**
   * Shift this rectangle by the given vector.
   */
  Rect operator+(const vec2 shift) const {
    Rect out;
    out.min = min + shift;
    out.max = max + shift;
    return out;
  }

  /**
   * Shift this rectangle in-place by the given vector.
   */
  Rect& operator+=(const vec2 shift) {
    min += shift;
    max += shift;
    return *this;
  }

  /**
   * Scale this rectangle by the given vector.
   */
  Rect operator*(const vec2 scale) const {
    Rect out;
    out.min = min * scale;
    out.max = max * scale;
    return out;
  }

  /**
   * Scale this rectangle in-place by the given vector.
   */
  Rect& operator*=(const vec2 scale) {
    min *= scale;
    max *= scale;
    return *this;
  }

  /**
   * Transform the rectangle by the given axis-aligned affine transform.
   *
   * Ensure the transform passed in is axis-aligned (rotations are all
   * multiples of 90 degrees), or else the resulting rectangle will no longer
   * bound properly.
   */
  Rect Transform(const mat3x2& m) const {
    Rect rect;
    rect.min = m * vec3(min, 1);
    rect.max = m * vec3(max, 1);
    return rect;
  }
  ///@}
};
/** @} */

/** @addtogroup Core
 *  @{
 */

/**
 * Boolean operation type: Add (Union), Subtract (Difference), and Intersect.
 */
enum class OpType { Add, Subtract, Intersect };

constexpr int DEFAULT_SEGMENTS = 0;
constexpr double DEFAULT_ANGLE = 10.0;
constexpr double DEFAULT_LENGTH = 1.0;
/**
 * These static properties control how circular shapes are quantized by
 * default on construction. If circularSegments is specified, it takes
 * precedence. If it is zero, then instead the minimum is used of the segments
 * calculated based on edge length and angle, rounded up to the nearest
 * multiple of four. To get numbers not divisible by four, circularSegments
 * must be specified.
 */
class Quality {
 private:
  inline static int circularSegments_ = DEFAULT_SEGMENTS;
  inline static double circularAngle_ = DEFAULT_ANGLE;
  inline static double circularEdgeLength_ = DEFAULT_LENGTH;

 public:
  /**
   * Sets an angle constraint the default number of circular segments for the
   * CrossSection::Circle(), Manifold::Cylinder(), Manifold::Sphere(), and
   * Manifold::Revolve() constructors. The number of segments will be rounded up
   * to the nearest factor of four.
   *
   * @param angle The minimum angle in degrees between consecutive segments. The
   * angle will increase if the the segments hit the minimum edge length.
   * Default is 10 degrees.
   */
  static void SetMinCircularAngle(double angle) {
    if (angle <= 0) return;
    circularAngle_ = angle;
  }

  /**
   * Sets a length constraint the default number of circular segments for the
   * CrossSection::Circle(), Manifold::Cylinder(), Manifold::Sphere(), and
   * Manifold::Revolve() constructors. The number of segments will be rounded up
   * to the nearest factor of four.
   *
   * @param length The minimum length of segments. The length will
   * increase if the the segments hit the minimum angle. Default is 1.0.
   */
  static void SetMinCircularEdgeLength(double length) {
    if (length <= 0) return;
    circularEdgeLength_ = length;
  }

  /**
   * Sets the default number of circular segments for the
   * CrossSection::Circle(), Manifold::Cylinder(), Manifold::Sphere(), and
   * Manifold::Revolve() constructors. Overrides the edge length and angle
   * constraints and sets the number of segments to exactly this value.
   *
   * @param number Number of circular segments. Default is 0, meaning no
   * constraint is applied.
   */
  static void SetCircularSegments(int number) {
    if (number < 3 && number != 0) return;
    circularSegments_ = number;
  }

  /**
   * Determine the result of the SetMinCircularAngle(),
   * SetMinCircularEdgeLength(), and SetCircularSegments() defaults.
   *
   * @param radius For a given radius of circle, determine how many default
   * segments there will be.
   */
  static int GetCircularSegments(double radius) {
    if (circularSegments_ > 0) return circularSegments_;
    int nSegA = 360.0 / circularAngle_;
    int nSegL = 2.0 * radius * glm::pi<double>() / circularEdgeLength_;
    int nSeg = fmin(nSegA, nSegL) + 3;
    nSeg -= nSeg % 4;
    return std::max(nSeg, 3);
  }

  /**
   * Resets the circular construction parameters to their defaults if
   * SetMinCircularAngle, SetMinCircularEdgeLength, or SetCircularSegments have
   * been called.
   */
  static void ResetToDefaults() {
    circularSegments_ = DEFAULT_SEGMENTS;
    circularAngle_ = DEFAULT_ANGLE;
    circularEdgeLength_ = DEFAULT_LENGTH;
  }
};
/** @} */

/** @defgroup Exceptions
 *  @brief Custom Exceptions
 *
 *  Exceptions are only thrown if the MANIFOLD_EXCEPTIONS flag is set.
 * @{
 */
struct userErr : public virtual std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct topologyErr : public virtual std::runtime_error {
  using std::runtime_error::runtime_error;
};
struct geometryErr : public virtual std::runtime_error {
  using std::runtime_error::runtime_error;
};
using logicErr = std::logic_error;
/** @} */

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

}  // namespace manifold

#undef HOST_DEVICE
