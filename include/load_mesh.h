#ifndef LOAD_MESH_H_
#define LOAD_MESH_H_

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <string>
#include <vector>
#include <iostream>
#include "mesh.h"
// See how to use assimp in https://learnopengl.com/Model-Loading/Model

class Model{
public:
  Model(char* path){
    loadModel(path);
  }
private:
  std::vector<Mesh> meshes;
  std::string directory;

  void loadModel(std::string path);
  void processNode(aiNode *node, const aiScene *scene);
  Mesh processMesh(aiMesh *node, const aiScene *scene);
  std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type , std::string typeName);
};

void Model::processNode(aiNode *node, const aiScene *scene){
  // Process all the node's meshes
  for (size_t i=0;  i < node->mNumMeshes; i++){
    aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
    meshes.push_back(processMesh(meh, scene));
  }
  // Process the children recursively until no children left
  for (size_t i=0;  i < node->mNumChildren; i++){
    processNode(node->mChildren[i], scene);
  }
}

void Model::loadModel(std::string path){
  Assimp::Importer import;
  const aiScene *scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

  // Check errors in the mesh loading
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode){
    std::cerr << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
    return;
  }
  // directory = path.substr(0, path.find_last_of('/'));
  processNode(scene->mRootNode, scene);
}

Mesh Model::processMesh(aiMesh *mesh, const aiScene *scene){
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::vector<Texture> textures;

  for (size_t i=0; i < mesh->mNumVertices; i++){
    // Process vertex with positions normals and texture coordinates
    Vertex vertex;
    // Positions
    vertex.Position = glm::vec3();
    vertex.Position.x = mesh->mVertices[i].x;
    vertex.Position.y = mesh->mVertices[i].y;
    vertex.Position.z = mesh->mVertices[i].z;

    // Normals
    vertex.Normal = glm::vec3();
    vertex.Normal.x = mesh->mNormals[i].x;
    vertex.Normal.y = mesh->mNormals[i].y;
    vertex.Normal.z = mesh->mNormals[i].z;

    // Texture coordinates (if any)
    if (mesh->mTextureCoords[0]){
      vertex.TexCoord = glm::vec2();
      vertex.TexCoord.x = mesh->mTextureCoords[0][i].x;
      vertex.TexCoord.y = mesh->mTextureCoords[0][i].y;
    }
    else vertex.TexCoord = glm::vec2(0.0f);
    vertices.push_back(vertex);
  }
  // Process indices
  for (size_t i=0; i < mesh->mNumFaces; i++){
    aiFace face = mesh->mFaces[i];
    for (size_t j=0; j<face.mNumIndices; j++){
      indices.push_back(face.mIndices[j]);
    }
  }
  // Process materials
  if (mesh->mMaterialIndex != 0){
    aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];

    std::vector<Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
    textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());

    std::vector<Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
    textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
  }

  return Mesh(vertices, indices, textures);
}

std::vector<Texture> Model::loadMaterialTextures(aiMaterial *mat, aiTextureType type , std::string typeName){
    std::vector<Texture> textures;
    for(size_t i = 0; i < mat->GetTextureCount(type); i++)
    {
        aiString str;
        mat->GetTexture(type, i, &str);
        Texture texture;
        // texture.id = TextureFromFile(str.C_Str(), directory);
        // texture.type = typeName;
        // texture.path = str;
        // textures.push_back(texture);
    }
    return textures;
}
#endif // LOAD_MESH_H_
