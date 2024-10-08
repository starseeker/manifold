// Copyright 2024 The Manifold Authors.
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

#include "./common.h"
#include "manifold/rect.h"

namespace manifold {

Rect::Rect() {};
Rect::~Rect() {};

Rect::Rect(const std::array<double, 2> a, const std::array<double, 2> b) {
  glm::vec2 va(a[0], a[1]);
  glm::vec2 vb(b[0], b[1]);
  glm::vec2 vmin = glm::min(va, vb);
  glm::vec2 vmax = glm::max(va, vb);
  min[0] = vmin.x;
  min[1] = vmin.y;
  max[0] = vmax.x;
  max[1] = vmax.y;
}

std::array<double, 2> const
Rect::Size() const {
  std::array<double, 2> ret;
  ret[0] = max[0] - min[0];
  ret[1] = max[1] - min[1];
  return ret;
}

double
Rect::Area() const {
  std::array<double, 2> sz = Size();
  return sz[0] * sz[1];
}

double
Rect::Scale() const {
  auto absMax = glm::max(glm::abs(min), glm::abs(max));
  return glm::max(absMax[0], absMax[1]);
}

std::array<double, 2>
Rect::Center() const {
  std::array<double, 2> acenter;
  acenter[0] = (min[0]+max[0]) * 0.5;
  acenter[1] = (min[1]+max[1]) * 0.5;
  return acenter;
}

bool
Rect::Contains(const std::array<double, 2>& p) const {
  glm::vec2 vp(p[0], p[1]);
  glm::vec2 vmin(min[0], min[1]);
  glm::vec2 vmax(max[0], max[1]);
  return glm::all(glm::greaterThanEqual(vp, vmin)) && glm::all(glm::greaterThanEqual(vmax, vp));
}

bool
Rect::Contains(const Rect& rect) const {
  glm::vec2 rectmin(rect.min[0], rect.min[1]);
  glm::vec2 rectmax(rect.max[0], rect.max[1]);
  glm::vec2 vmin(min[0], min[1]);
  glm::vec2 vmax(max[0], max[1]);
  return glm::all(glm::greaterThanEqual(rectmin, vmin)) &&
         glm::all(glm::greaterThanEqual(vmax, rectmax));
}

bool
Rect::DoesOverlap(const Rect& rect) const {
  return min[0] <= rect.max[0] && min[1] <= rect.max[1] && max[0] >= rect.min[0] &&
         max[1] >= rect.min[1];
}

bool
Rect::IsEmpty() const {
  return max[1] <= min[1] || max[0] <= min[0];
};

bool
Rect::IsFinite() const {
  glm::vec2 vmin(min[0], min[1]);
  glm::vec2 vmax(max[0], max[1]);
  return glm::all(glm::isfinite(vmin)) && glm::all(glm::isfinite(vmax));
}

void
Rect::Union(const std::array<double, 2> p) {
  glm::vec2 vp(p[0], p[1]);
  glm::vec2 vmin(min[0], min[1]);
  glm::vec2 vmax(max[0], max[1]);
  vmin = glm::min(vmin, vp);
  vmax = glm::max(vmax, vp);
  min[0] = vmin.x;
  min[1] = vmin.y;
  max[0] = vmax.x;
  max[1] = vmax.y;
}

Rect
Rect::Union(const Rect& rect) const {
  Rect out;
  glm::vec2 rectmin(rect.min[0], rect.min[1]);
  glm::vec2 rectmax(rect.max[0], rect.max[1]);
  glm::vec2 vmin(min[0], min[1]);
  glm::vec2 vmax(max[0], max[1]);
  glm::vec2 omin = glm::min(vmin, rectmin);
  glm::vec2 omax = glm::max(vmax, rectmax);
  out.min[0] = omin.x;
  out.min[1] = omin.y;
  out.max[0] = omax.x;
  out.max[1] = omax.y;
  return out;
}

Rect operator+(const std::array<double, 2> shift) const {
   Rect out;
     out.min = min + shift;
     out.max = max + shift;
      return out;
    }

Rect& operator+=(const std::array<double, 2> shift) {
     min += shift;
     max += shift;
      return *this;
    }

Rect operator*(const std::array<double, 2> scale) const {
      Rect out;
     out.min = min * scale;
     out.max = max * scale;
      return out;
    }

Rect& operator*=(const std::array<double, 2> scale) {
     min *= scale;
     max *= scale;
      return *this;
    }

Rect Transform(const mat3x2& m) const {
      Rect rect;
     rect.min = m * std::array<double, 3>(min, 1);
     rect.max = m * std::array<double, 3>(max, 1);
      return rect;
    }

}  // namespace manifold
