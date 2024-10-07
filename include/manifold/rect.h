// Copyright 2023 The Manifold Authors.
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

#include <array>
#include <limits>
#include <vector>

namespace manifold {

/**
 * Axis-aligned rectangular bounds.
 */
struct Rect {
  double min[2] = {std::numeric_limits<double>::infinity()};
  double max[2] = {-std::numeric_limits<double>::infinity()};

  /**
   * Default constructor is an empty rectangle..
   */
  Rect();
  ~Rect();

  /**
   * Create a rectangle that contains the two given points.
   */
  Rect(const std::array<double, 2> a, const std::array<double, 2> b);

  /** @name Information
   *  Details of the rectangle
   */
  ///@{

  /**
   * Return the dimensions of the rectangle.
   */
  std::array<double, 2> Size();

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
    std::array<double, 2> absMax = glm::max(glm::abs(min), glm::abs(max));
    return glm::max(absMax.x, absMax.y);
  }

  /**
   * Returns the center point of the rectangle.
   */
  std::array<double, 2> Center() const { return 0.5 * (max + min); }

  /**
   * Does this rectangle contain (includes on border) the given point?
   */
  bool Contains(const std::array<double, 2>& p) const {
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
  void Union(const std::array<double, 2> p) {
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
  Rect operator+(const std::array<double, 2> shift) const {
    Rect out;
    out.min = min + shift;
    out.max = max + shift;
    return out;
  }

  /**
   * Shift this rectangle in-place by the given vector.
   */
  Rect& operator+=(const std::array<double, 2> shift) {
    min += shift;
    max += shift;
    return *this;
  }

  /**
   * Scale this rectangle by the given vector.
   */
  Rect operator*(const std::array<double, 2> scale) const {
    Rect out;
    out.min = min * scale;
    out.max = max * scale;
    return out;
  }

  /**
   * Scale this rectangle in-place by the given vector.
   */
  Rect& operator*=(const std::array<double, 2> scale) {
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
    rect.min = m * std::array<double, 3>(min, 1);
    rect.max = m * std::array<double, 3>(max, 1);
    return rect;
  }
  ///@}
};

}  // namespace manifold
