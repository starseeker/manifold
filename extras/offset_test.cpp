//             O F F S E T _ T E S T . C P P
//
// Published in 2024 by the United States Government.
// This work is in the public domain.

#include <iostream>
#include <set>

#include "linalg.h"
#include "manifold.h"
#include "meshIO.h"

using namespace manifold;

  namespace la = linalg;
  using vec2 = la::vec<double, 2>;
  using vec3 = la::vec<double, 3>;
  using vec4 = la::vec<double, 4>;
  using bvec4 = la::vec<bool, 4>;
  using mat2 = la::mat<double, 2, 2>;
  using mat3x2 = la::mat<double, 3, 2>;
  using mat4x2 = la::mat<double, 4, 2>;
  using mat2x3 = la::mat<double, 2, 3>;
  using mat3 = la::mat<double, 3, 3>;
  using mat4x3 = la::mat<double, 4, 3>;
  using mat3x4 = la::mat<double, 3, 4>;
  using mat4 = la::mat<double, 4, 4>;
  using ivec2 = la::vec<int, 2>;
  using ivec3 = la::vec<int, 3>;
  using ivec4 = la::vec<int, 4>;
  using quat = la::vec<double, 4>;


#define CHECK_INTERMEDIATES

int
main(int argc, const char **argv) {
  if (argc < 2) {
    std::cout << "Specify an input filename.\n";
    return 1;
  }

  const std::string filename(argv[1]);
  MeshGL input = ImportMesh(filename);
  std::cout << input.NumVert() << " vertices, " << input.NumTri() << " triangles\n";

  // Note:  for this test, the input is the source of the elements rather than
  // the input itself, so we don't check for manifoldness of the input. Instead,
  // we need to build up the manifold definition using unioned CSG elements.
  manifold::Manifold c;

  // Collect unordered edges - it will be one cylinder to an edge
  std::set<std::pair<int, int>> edges;
  for (size_t i = 0; i < input.triVerts.size()/3; i++) {
    int eind[3];
    for (int j = 0; j < 3; j++)
      eind[j] = input.triVerts[i*3+j];
    int e11, e12, e21, e22, e31, e32;
    e11 = (eind[0] < eind[1]) ? eind[0] : eind[1];
    e12 = (eind[1] < eind[0]) ? eind[0] : eind[1];
    e21 = (eind[1] < eind[2]) ? eind[1] : eind[2];
    e22 = (eind[2] < eind[1]) ? eind[1] : eind[2];
    e31 = (eind[2] < eind[0]) ? eind[2] : eind[0];
    e32 = (eind[0] < eind[2]) ? eind[2] : eind[0];
    edges.insert(std::make_pair(e11, e12));
    edges.insert(std::make_pair(e21, e22));
    edges.insert(std::make_pair(e31, e32));
  }

  // Add spheres for the vertices
  Manifold sph = Manifold::Sphere(1, 8);
  for (size_t i = 0; i < input.vertProperties.size()/3; i++) {
    Manifold vsph = sph.Translate(glm::vec3(input.vertProperties[3*i+0], input.vertProperties[3*i+1], input.vertProperties[3*i+2]));
    manifold::Manifold left = c;
    manifold::Manifold right(vsph);
    try {
      c = left.Boolean(right, manifold::OpType::Add);
#if defined(CHECK_INTERMEDIATES)
      MeshGL imesh = c.GetMeshGL();
      std::cout << imesh.NumVert() << " vertices, " << imesh.NumTri() << " triangles\n";
#endif
    } catch (const std::exception &e) {
      std::cerr << "Vertices - manifold boolean op failure: " << e.what() << "\n";
      return -1;
    }
  }
  std::cerr << "Processing " << input.NumVert() << " vertices... done.\n";

  std::cerr << "Processing " << edges.size() << " edges... \n";
  std::set<std::pair<int, int>>::iterator e_it;
  int edge_cnt = 0;
  for (e_it = edges.begin(); e_it != edges.end(); ++e_it) {
    vec3 ev1 = vec3(input.vertProperties[3*e_it->first+0], input.vertProperties[3*e_it->first+1], input.vertProperties[3*e_it->first+2]);
    vec3 ev2 = vec3(input.vertProperties[3*e_it->second+0], input.vertProperties[3*e_it->second+1], input.vertProperties[3*e_it->second+2]);
    edge_cnt++;

    vec3 edge = ev2 - ev1;
    double len = la::length(edge);
    if (len < 1)
      continue;
    manifold::Manifold origin_cyl = manifold::Manifold::Cylinder(len, 1, 1, 8);
    glm::vec3 evec(-1*edge.x, -1*edge.y, edge.z);
    manifold::Manifold rotated_cyl = origin_cyl.Transform(manifold::RotateUp(evec));
    manifold::Manifold right = rotated_cyl.Translate(glm::vec3(ev1.x, ev1.y, ev1.z));

    if (!right.NumTri() || right.NumVert() < 8)
      continue;

    // Union
    manifold::Manifold left = c;
    try {
      c = left.Boolean(right, manifold::OpType::Add);
#if defined(CHECK_INTERMEDIATES)
      MeshGL imesh = c.GetMeshGL();
      std::cout << "Edge " << edge_cnt << ": " << imesh.NumVert() << " vertices, " << imesh.NumTri() << " triangles\n";
      if (!imesh.NumTri()) {
	std::cerr << "ev1: " << ev1.x << "," << ev1.y << "," << ev1.z << "\n";
	std::cerr << "ev2: " << ev2.x << "," << ev2.y << "," << ev2.z << "\n";
	ExportMesh(std::string("fail_left.obj"), left.GetMeshGL(), {});
	ExportMesh(std::string("fail_right.obj"), right.GetMeshGL(), {});
	return -1;
      }
#endif
    } catch (const std::exception &e) {
      std::cerr << "Edges - manifold boolean op failure: " << e.what() << "\n";
      std::cerr << "ev1: " << ev1.x << "," << ev1.y << "," << ev1.z << "\n";
      std::cerr << "ev2: " << ev2.x << "," << ev2.y << "," << ev2.z << "\n";
      ExportMesh(std::string("fail_left.glb"), left.GetMeshGL(), {});
      ExportMesh(std::string("fail_right.glb"), right.GetMeshGL(), {});
      return -1;
    }
  }
  std::cerr << "Processing " << edges.size() << " edges... done\n";

  std::cerr << "Processing " << input.NumTri() << " triangles...\n";
  for (size_t i = 0; i < input.triVerts.size()/3; i++) {
    int eind[3];
    for (int j = 0; j < 3; j++)
      eind[j] = input.triVerts[i*3+j];
    vec3 ev1 = vec3(input.vertProperties[3*eind[0]+0], input.vertProperties[3*eind[0]+1], input.vertProperties[3*eind[0]+2]);
    vec3 ev2 = vec3(input.vertProperties[3*eind[1]+0], input.vertProperties[3*eind[1]+1], input.vertProperties[3*eind[1]+2]);
    vec3 ev3 = vec3(input.vertProperties[3*eind[2]+0], input.vertProperties[3*eind[2]+1], input.vertProperties[3*eind[2]+2]);

    // Get the triangle normal
    vec3 a = ev1 - ev3;
    vec3 b = ev2 - ev3;
    vec3 n = la::normalize(la::cross(a, b));

    // Extrude the points above and below the plane of the triangle
    vec3 pnts[6];
    pnts[0] = ev1 + n;
    pnts[1] = ev2 + n;
    pnts[2] = ev3 + n;
    pnts[3] = ev1 - n;
    pnts[4] = ev2 - n;
    pnts[5] = ev3 - n;

    // Construct the points and faces of the new manifold
    double pts[3*6];
    /* 1 */ pts[0] = pnts[4].x; pts[1] = pnts[4].y; pts[2] = pnts[4].z;
    /* 2 */ pts[3] = pnts[3].x; pts[4] = pnts[3].y; pts[5] = pnts[3].z;
    /* 3 */ pts[6] = pnts[0].x; pts[7] = pnts[0].y; pts[8] = pnts[0].z;
    /* 4 */ pts[9] = pnts[1].x; pts[10] = pnts[1].y; pts[11] = pnts[1].z;
    /* 5 */ pts[12] = pnts[5].x; pts[13] = pnts[5].y; pts[14] = pnts[5].z;
    /* 6 */ pts[15] = pnts[2].x; pts[16] = pnts[2].y; pts[17] = pnts[2].z;

    int faces[24];
    faces[ 0] = 0; faces[ 1] = 1; faces[ 2] = 4;  // 1 2 5
    faces[ 3] = 2; faces[ 4] = 3; faces[ 5] = 5;  // 3 4 6
    faces[ 6] = 1; faces[ 7] = 0; faces[ 8] = 3;  // 2 1 4
    faces[ 9] = 3; faces[10] = 2; faces[11] = 1;  // 4 3 2
    faces[12] = 3; faces[13] = 0; faces[14] = 4;  // 4 1 5
    faces[15] = 4; faces[16] = 5; faces[17] = 3;  // 5 6 4
    faces[18] = 5; faces[19] = 4; faces[20] = 1;  // 6 5 2
    faces[21] = 1; faces[22] = 2; faces[23] = 5;  // 2 3 6

    manifold::MeshGL tri_m;
    for (int j = 0; j < 18; j++)
      tri_m.vertProperties.insert(tri_m.vertProperties.end(), pts[j]);
    for (int j = 0; j < 24; j++)
      tri_m.triVerts.insert(tri_m.triVerts.end(), faces[j]);

    manifold::Manifold left = c;
    manifold::Manifold right(tri_m);

    if (!right.NumTri())
      continue;

    try {
      c = left.Boolean(right, manifold::OpType::Add);
#if defined(CHECK_INTERMEDIATES)
      MeshGL imesh = c.GetMeshGL();
      std::cout << "Face " << i << ": " << imesh.NumVert() << " vertices, " << imesh.NumTri() << " triangles\n";
      if (!imesh.NumTri()) {
	std::cerr << "ev1: " << ev1.x << "," << ev1.y << "," << ev1.z << "\n";
	std::cerr << "ev2: " << ev2.x << "," << ev2.y << "," << ev2.z << "\n";
	ExportMesh(std::string("fail_left.obj"), left.GetMeshGL(), {});
	ExportMesh(std::string("fail_right.obj"), right.GetMeshGL(), {});
	return -1;
      }
#endif
    } catch (const std::exception &e) {
      std::cerr << "Faces - manifold boolean op failure: " << e.what() << "\n";
      return -1;
    }
  }
  std::cerr << "Processing " << input.NumTri() << " triangles... done.\n";

  ExportMesh(std::string("out.obj"), c.GetMeshGL(), {});

  return 0;
}


// ex: shiftwidth=2 tabstop=8
