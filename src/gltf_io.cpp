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

// Relatively minimalist logic for reading and writing GLTF files
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_IMPLEMENTATION
#include "tiny_gltf.h"

#include "manifold/meshIO.h"

#include <iostream>

#include "manifold/optional_assert.h"

namespace manifold {

  static std::string gltf_get_ext(const std::string &fname) {
    if (fname.find_last_of(".") == std::string::npos)
      return std::string();
    return fname.substr(fname.find_last_of(".") + 1);
  }

  /**
   * Imports the specified gltf file as a Mesh structure, which can be converted to a
   * Manifold if the mesh is a proper oriented 2-manifold.
   *
   * @param filename Must be a gltf or glb file.
   */
  MeshGL ImportMesh(const std::string& filename) {

    MeshGL mesh_out;
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    loader.SetStoreOriginalJSONForExtrasAndExtensions(true);
    std::string errmsg, warn;
    std::string f_ext = gltf_get_ext(filename);
    if (!f_ext.length())
      return mesh_out;
    bool result = false;
    if (f_ext == std::string("glb")) {
      result = loader.LoadBinaryFromFile(&model, &errmsg, &warn, filename);
    } else {
      result = loader.LoadASCIIFromFile(&model, &errmsg, &warn, filename);
    }
    if (result != 0) {
      DEBUG_ASSERT(result != 0, userErr, "gltf load failure.");
      return mesh_out;
    }

    for (size_t i = 0; i < model.scenes.size(); i++) {
      for (size_t j = 0; j < model.scenes[i].nodes.size(); j++) {
	tinygltf::Node &node = model.nodes[model.scenes[i].nodes[j]];
	if (node.mesh == -1)
	  continue;
	tinygltf::Mesh &mesh = model.meshes[node.mesh];
	// TODO Handle EXT_mesh_manifold
	// TODO - is this data transcription correct??
	for (size_t k = 0; k < mesh.primitives.size(); k++) {
	  tinygltf::Primitive &prim = mesh.primitives[k];
	  // get face data
	  int f_idx = prim.indices;
	  tinygltf::Accessor &a = model.accessors.at(f_idx);
	  tinygltf::BufferView &bv = model.bufferViews[a.bufferView];
	  tinygltf::Buffer &b = model.buffers[bv.buffer];
	  size_t f_cnt = a.count/3;
	  const unsigned short *faces = (const unsigned short *)(b.data.data() + bv.byteOffset + a.byteOffset);
	  // get vertex data
	  std::map<std::string, int>::const_iterator p_it = prim.attributes.find(std::string("POSITION"));
	  int v_idx = p_it->second;
	  a = model.accessors.at(v_idx);
	  bv = model.bufferViews[a.bufferView];
	  b = model.buffers[bv.buffer];
	  size_t v_cnt = a.count;
	  const float *verts = (const float *)(b.data.data() + bv.byteOffset + a.byteOffset);
	  for (size_t l = 0; l < v_cnt; ++l) {
	    mesh_out.vertProperties.insert(mesh_out.vertProperties.end(),{verts[3*l+0], verts[3*l+1], verts[3*l+2]});
	  }
	  for (size_t l = 0; l < f_cnt; ++l) {
	    mesh_out.triVerts.insert(mesh_out.triVerts.end(), {faces[3*l+0], faces[3*l+1], faces[3*l+2]});
	  }
	}
      }
    }

    return mesh_out;
  }

  /**
   * Saves the Mesh to the desired file type, determined from the extension
   * specified. In the case of .glb/.gltf, this will save in version 2.0.
   *
   * This is a very simple export function and is intended primarily as a
   * demonstration. Generally users of this library will need to modify this to
   * write all the important properties for their application and read any custom
   * data structures.
   *
   * @param filename The file extension must be one that Assimp supports for
   * export. GLB & 3MF are recommended.
   * @param mesh The mesh to export, likely from Manifold.GetMeshGL().
   * @param options The options currently only affect an exported GLB's material.
   * Pass {} for defaults.
   */
  void ExportMesh(const std::string& filename, const MeshGL& mesh, const ExportOptions& options) {
    if (mesh.triVerts.size() == 0) {
      std::cout << filename << " was not saved because the input mesh was empty."
	<< std::endl;
      return;
    }
#if 0
    std::string type = GetType(filename);
    const bool isYup = type == "glb2" || type == "gltf2";

    aiScene* scene = CreateScene(options);
    aiMesh* mesh_out = scene->mMeshes[0];

    mesh_out->mNumVertices = mesh.NumVert();
    mesh_out->mVertices = new aiVector3D[mesh_out->mNumVertices];
    if (!options.faceted) {
      bool validChannels = true;
      for (int i : {0, 1, 2}) {
	int c = options.mat.normalChannels[i];
	validChannels &= c >= 3 && c < (int)mesh.numProp;
      }
      DEBUG_ASSERT(
	  validChannels, userErr,
	  "When faceted is false, valid normalChannels must be supplied.");
      mesh_out->mNormals = new aiVector3D[mesh_out->mNumVertices];
    }

    bool validChannels = true;
    bool hasColor = false;
    for (int i : {0, 1, 2, 3}) {
      int c = options.mat.colorChannels[i];
      validChannels &= c < (int)mesh.numProp;
      hasColor |= c >= 0;
    }
    DEBUG_ASSERT(validChannels, userErr, "Invalid colorChannels.");
    if (hasColor) mesh_out->mColors[0] = new aiColor4D[mesh_out->mNumVertices];

    for (size_t i = 0; i < mesh_out->mNumVertices; ++i) {
      vec3 v;
      for (int j : {0, 1, 2}) v[j] = mesh.vertProperties[i * mesh.numProp + j];
      mesh_out->mVertices[i] =
	isYup ? aiVector3D(v.y, v.z, v.x) : aiVector3D(v.x, v.y, v.z);
      if (!options.faceted) {
	vec3 n;
	for (int j : {0, 1, 2})
	  n[j] = mesh.vertProperties[i * mesh.numProp +
	    options.mat.normalChannels[j]];
	mesh_out->mNormals[i] =
	  isYup ? aiVector3D(n.y, n.z, n.x) : aiVector3D(n.x, n.y, n.z);
      }
      if (hasColor) {
	vec4 c;
	for (int j : {0, 1, 2, 3})
	  c[j] = options.mat.colorChannels[j] < 0
	    ? 1
	    : mesh.vertProperties[i * mesh.numProp +
	    options.mat.colorChannels[j]];
	c = la::clamp(c, 0, 1);
	mesh_out->mColors[0][i] = aiColor4D(c.x, c.y, c.z, c.w);
      }
    }

    mesh_out->mNumFaces = mesh.NumTri();
    mesh_out->mFaces = new aiFace[mesh_out->mNumFaces];

    for (size_t i = 0; i < mesh_out->mNumFaces; ++i) {
      aiFace& face = mesh_out->mFaces[i];
      face.mNumIndices = 3;
      face.mIndices = new uint32_t[face.mNumIndices];
      for (int j : {0, 1, 2}) face.mIndices[j] = mesh.triVerts[3 * i + j];
    }

    ExportScene(scene, filename);
#endif
  }
}  // namespace manifold

// Local Variables:
// tab-width: 8
// mode: C++
// c-basic-offset: 2
// indent-tabs-mode: t
// c-file-style: "stroustrup"
// End:
// ex: shiftwidth=2 tabstop=8
