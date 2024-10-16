//             O F F S E T _ T E S T . C P P
//
// Published in 2024 by the United States Government.
// This work is in the public domain.

#include <iostream>
#include <set>

#include "manifold/manifold.h"
#include "manifold/meshIO.h"

using namespace manifold;

#define CHECK_INTERMEDIATES

vec3
orthovec(vec3 vin)
{
  const double in[3] = {vin.x, vin.y, vin.z};
  double out[3] = {0.0};
  int i = 0; int j = 1; int k = 2;
  double f = in[0];
  if (fabs(in[1]) < f) {
    f = fabs(in[1]);
    i = 1; j = 2; k = 0;
  }
  if (fabs(in[2]) < f) {
    i = 2; j = 0; k = 1;
  }
  f = 1 / hypot(in[j], in[k]);
  out[i] = 0.0; out[j] = -in[k] * f; out[k] = in[j] * f;
  return vec3(out[0], out[1], out[2]);
}

#define MAX_CYL_STEPS 100
Manifold
EdgeCylinder(vec3 p1, vec3 p2, double r)
{
  int nsegs = 8;
  vec3 h = p2 - p1;
  vec3 xaxis = r * la::normalize(orthovec(h));
  vec3 yaxis = r * la::normalize(la::cross(xaxis, h));

  // Figure out the step we take for each ring in the cylinder
  double sl = M_PI * r * r / (double)nsegs;
  double el = la::length(h);
  double hl = (el < 2*sl) ? el : -1.0;
  int steps = 1;
  if (hl < 0) {
    steps = (int)(el / sl);
    if (steps > MAX_CYL_STEPS)
      steps = MAX_CYL_STEPS;
    hl = el / (double)steps;
  }
  vec3 hs = hl*la::normalize(h);

  MeshGL64 mgl;

  // Vertices
  for (int i = 0; i <= steps; i++) {
    for (int j = 0; j < nsegs; j++) {
      double alpha = 2 * M_PI * (double)(2*j+1)/(double)(2*nsegs);
      /* vertex geometry */
      vec3 np = p1 + (double)i*hs + cos(alpha)*xaxis + sin(alpha)*yaxis;
      mgl.vertProperties.insert(mgl.vertProperties.end(), np.x);
      mgl.vertProperties.insert(mgl.vertProperties.end(), np.y);
      mgl.vertProperties.insert(mgl.vertProperties.end(), np.z);
    }
  }
  // The two center points of the end caps are the last two points
  mgl.vertProperties.insert(mgl.vertProperties.end(), p1.x);
  mgl.vertProperties.insert(mgl.vertProperties.end(), p1.y);
  mgl.vertProperties.insert(mgl.vertProperties.end(), p1.z);
  mgl.vertProperties.insert(mgl.vertProperties.end(), p2.x);
  mgl.vertProperties.insert(mgl.vertProperties.end(), p2.y);
  mgl.vertProperties.insert(mgl.vertProperties.end(), p2.z);

  // Next, we define the faces.  The two end caps each have one triangle for
  // each segment.  Each step defines 2*nseg triangles.
  // For the steps, we process in quads - each segment gets two triangles
  for (int i = 0; i < steps; i++) {
    for (int j = 0; j < nsegs; j++) {
      int pnts[4];
      pnts[0] = nsegs * i + j;
      pnts[1] = (j < nsegs - 1) ? nsegs * i + j + 1 : nsegs * i;
      pnts[2] = nsegs * (i + 1) + j;
      pnts[3] = (j < nsegs - 1) ? nsegs * (i + 1) + j + 1 : nsegs * (i + 1);
      mgl.triVerts.insert(mgl.triVerts.end(), pnts[0]);
      mgl.triVerts.insert(mgl.triVerts.end(), pnts[2]);
      mgl.triVerts.insert(mgl.triVerts.end(), pnts[1]);
      mgl.triVerts.insert(mgl.triVerts.end(), pnts[2]);
      mgl.triVerts.insert(mgl.triVerts.end(), pnts[3]);
      mgl.triVerts.insert(mgl.triVerts.end(), pnts[1]);
    }
  }

  // Define the end caps.  The first set of triangles uses the base
  // point (stored at verts[steps*nsegs] and the points of the first
  // circle (stored at the beginning of verts)
  for (int j = 0; j < nsegs; j++){
    int pnts[3];
    pnts[0] = (steps+1) * nsegs;
    pnts[1] = j;
    pnts[2] = (j < nsegs - 1) ? j + 1 : 0;
    mgl.triVerts.insert(mgl.triVerts.end(), pnts[0]);
    mgl.triVerts.insert(mgl.triVerts.end(), pnts[1]);
    mgl.triVerts.insert(mgl.triVerts.end(), pnts[2]);
  }
  // The second set of cap triangles uses the second edge point
  // point (stored at verts[steps*nsegs+1] and the points of the last
  // circle (stored at the end of verts = (steps-1) * nsegs)
  for (int j = 0; j < nsegs; j++){
    int pnts[3];
    pnts[0] = (steps+1) * nsegs + 1;
    pnts[1] = steps * nsegs + j;
    pnts[2] = (j < nsegs - 1) ? steps * nsegs + j + 1 : steps * nsegs;
    mgl.triVerts.insert(mgl.triVerts.end(), pnts[0]);
    mgl.triVerts.insert(mgl.triVerts.end(), pnts[2]);
    mgl.triVerts.insert(mgl.triVerts.end(), pnts[1]);
  }

  return Manifold(mgl);
}

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
    Manifold vsph = sph.Translate(vec3(input.vertProperties[3*i+0], input.vertProperties[3*i+1], input.vertProperties[3*i+2]));
    manifold::Manifold left = c;
    manifold::Manifold right(vsph);
    try {
      c = left.Boolean(right, manifold::OpType::Add);
#if defined(CHECK_INTERMEDIATES)
      MeshGL64 imesh = c.GetMeshGL64();
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

    // Make a cylinder for the edge
    Manifold ecyl = EdgeCylinder(ev1, ev2, 1);

    if (!ecyl.NumTri())
      continue;

    // Union
    manifold::Manifold left = c;
    manifold::Manifold right(ecyl);
    try {
      c = left.Boolean(right, manifold::OpType::Add);
#if defined(CHECK_INTERMEDIATES)
      MeshGL64 imesh = c.GetMeshGL64();
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

    manifold::MeshGL64 tri_m;
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
      MeshGL64 imesh = c.GetMeshGL64();
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
