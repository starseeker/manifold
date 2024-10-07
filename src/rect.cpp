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

std::array<double, 2>
Rect::Size() const {
  glm::vec2 vmin(min[0], min[1]);
  glm::vec2 vmax(max[0], max[1]);
  glm::vec2 vsize = vmax - vmin;
  std::array<double, 2> ret;
  ret[0] = vsize.x;
  ret[1] = vsize.y;
  return ret;
}


}  // namespace manifold
