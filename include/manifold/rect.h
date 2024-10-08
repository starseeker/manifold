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
class Rect {
  public:
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
  std::array<double, 2> const Size() const;

  /**
   * Return the area of the rectangle.
   */
  double Area() const;

  /**
   * Returns the absolute-largest coordinate value of any contained
   * point.
   */
  double Scale() const;

  /**
   * Returns the center point of the rectangle.
   */
  std::array<double, 2> Center() const;

  /**
   * Does this rectangle contain (includes on border) the given point?
   */
  bool Contains(const std::array<double, 2>& p) const;

  /**
   * Does this rectangle contain (includes equal) the given rectangle?
   */
  bool Contains(const Rect& rect) const;

  /**
   * Does this rectangle overlap the one given (including equality)?
   */
  bool DoesOverlap(const Rect& rect) const;

  /**
   * Is the rectangle empty (containing no space)?
   */
  bool IsEmpty() const;

  /**
   * Does this recangle have finite bounds?
   */
  bool IsFinite() const;

  ///@}

  /** @name Modification
   */
  ///@{

  /**
   * Expand this rectangle (in place) to include the given point.
   */
  void Union(const std::array<double, 2> p);

  /**
   * Expand this rectangle to include the given Rect.
   */
  Rect Union(const Rect& rect) const;

  /**
   * Shift this rectangle by the given vector.
   */
  Rect operator+(const std::array<double, 2> shift) const {
    Rect out;
    out.min[0] = min[0] + shift[0];
    out.min[1] = min[1] + shift[1];
    out.max[0] = max[0] + shift[0];
    out.max[1] = max[1] + shift[1];
    return out;
  }

  /**
   * Shift this rectangle in-place by the given vector.
   */
  Rect& operator+=(const std::array<double, 2> shift) {
    min[0] += shift[0];
    min[1] += shift[1];
    max[0] += shift[0];
    max[1] += shift[1];
    return *this;
  }

  /**
   * Scale this rectangle by the given vector.
   */
  Rect operator*(const std::array<double, 2> scale) const {
    Rect out;
    out.min[0] = min[0] * scale[0];
    out.min[1] = min[1] * scale[1];
    out.max[0] = max[0] * scale[0];
    out.max[1] = max[1] * scale[1];
    return out;
  }

  /**
   * Scale this rectangle in-place by the given vector.
   */
  Rect& operator*=(const std::array<double, 2> scale) {
    min[0] *= scale[0];
    min[1] *= scale[1];
    max[0] *= scale[0];
    max[1] *= scale[1];
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
