#include "impl.h"

namespace {
using namespace manifold;

inline int FlipHalfedge(int halfedge) {
  const int tri = halfedge / 3;
  const int vert = 2 - (halfedge - 3 * tri);
  return 3 * tri + vert;
}

struct TransformNormals {
  glm::dmat3 transform;

  glm::dvec3 operator()(glm::dvec3 normal) const {
    normal = glm::normalize(transform * normal);
    if (isnan(normal.x)) normal = glm::dvec3(0.0);
    return normal;
  }
};

struct TransformTangents {
  VecView<glm::dvec4> tangent;
  const int edgeOffset;
  const glm::dmat3 transform;
  const bool invert;
  VecView<const glm::dvec4> oldTangents;
  VecView<const Halfedge> halfedge;

  void operator()(const int edgeOut) {
    const int edgeIn =
        invert ? halfedge[FlipHalfedge(edgeOut)].pairedHalfedge : edgeOut;
    tangent[edgeOut + edgeOffset] =
        glm::dvec4(transform * glm::dvec3(oldTangents[edgeIn]), oldTangents[edgeIn].w);
  }
};

struct FlipTris {
  VecView<Halfedge> halfedge;

  void operator()(const int tri) {
    std::swap(halfedge[3 * tri], halfedge[3 * tri + 2]);

    for (const int i : {0, 1, 2}) {
      std::swap(halfedge[3 * tri + i].startVert, halfedge[3 * tri + i].endVert);
      halfedge[3 * tri + i].pairedHalfedge =
          FlipHalfedge(halfedge[3 * tri + i].pairedHalfedge);
    }
  }
};
}  // namespace
