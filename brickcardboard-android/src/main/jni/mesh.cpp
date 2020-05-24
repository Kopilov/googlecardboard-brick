#include "mesh.h"
#include "stb_image.h"
#include "util.h"
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <android/log.h>

Shader::Shader(const char* vertexPath, const char* fragmentPath) {
    // 1. retrieve the vertex/fragment source code from filePath
    std::string vertexCode;
    std::string fragmentCode;
    std::ifstream vShaderFile;
    std::ifstream fShaderFile;
    // ensure ifstream objects can throw exceptions:
    vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    try {
        // open files
        vShaderFile.open(vertexPath);
        fShaderFile.open(fragmentPath);
        std::stringstream vShaderStream, fShaderStream;
        // read file's buffer contents into streams
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();
        // close file handlers
        vShaderFile.close();
        fShaderFile.close();
        // convert stream into string
        vertexCode   = vShaderStream.str();
        fragmentCode = fShaderStream.str();
    } catch(std::ifstream::failure e) {
        LOGE("ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ");
    }
    const char* vShaderCode = vertexCode.c_str();
    const char* fShaderCode = fragmentCode.c_str();

    // 2. compile shaders
    unsigned int vertex, fragment;
    int success;
    char infoLog[512];

    // vertex Shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, nullptr);
    glCompileShader(vertex);
    // print compile errors if any
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(vertex, 512, nullptr, infoLog);
        LOGE("ERROR::SHADER::VERTEX::COMPILATION_FAILED\n%s", infoLog);
    };

    // similiar for Fragment Shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, nullptr);
    glCompileShader(fragment);
    // print compile errors if any
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if(!success) {
        glGetShaderInfoLog(fragment, 512, nullptr, infoLog);
        LOGE("ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n%s", infoLog);
    };

    // shader Program
    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);
    // print linking errors if any
    glGetProgramiv(ID, GL_LINK_STATUS, &success);
    if(!success) {
        glGetProgramInfoLog(ID, 512, nullptr, infoLog);
        LOGE("ERROR::SHADER::PROGRAM::LINKING_FAILED\n%s", infoLog);
    }

    // delete the shaders as they're linked into our program now and no longer necessery
    glDeleteShader(vertex);
    glDeleteShader(fragment);
}

void Shader::use() {
    glUseProgram(ID);
}

void Shader::setBool(const std::string &name, bool value) const {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
}
void Shader::setInt(const std::string &name, int value) const {
    glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}
void Shader::setFloat(const std::string &name, float value) const {
    glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}
void Shader::setVec3(const std::string &name, float value0, float value1, float value2) const {
    glUniform3f(glGetUniformLocation(ID, name.c_str()), value0, value1, value2);
}
void Shader::setMat4(const std::string &name, const float* value) const {
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, value);
}

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices, std::vector<Texture> textures, Material material) {
    this->vertices = vertices;
    this->indices = indices;
    this->textures = textures;
    this->material = material;

    setupMesh();
}


void Mesh::setupMesh() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                 &indices[0], GL_STATIC_DRAW);

    // vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    // vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
    // vertex texture coords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));

    glBindVertexArray(0);
}

void Mesh::Draw(Shader shader) {
    unsigned int diffuseNr = 1;
    unsigned int specularNr = 1;
    for(unsigned int i = 0; i < textures.size(); i++) {
        glActiveTexture(GL_TEXTURE0 + i); // activate proper texture unit before binding
        // retrieve texture number (the N in diffuse_textureN)
        std::string number;
        std::string name = textures[i].type;
        if(name == "texture_diffuse")
            number = std::to_string(diffuseNr++);
        else if(name == "texture_specular")
            number = std::to_string(specularNr++);

        //shader.setFloat(("material." + name + number).c_str(), i);
        glBindTexture(GL_TEXTURE_2D, textures[i].id);
    }
    glActiveTexture(GL_TEXTURE0);

    shader.setVec3("material.ambient", material.ambient[0], material.ambient[1], material.ambient[2]);
    shader.setVec3("material.diffuse", material.diffuse[0], material.diffuse[1], material.diffuse[2]);
    shader.setVec3("material.specular", material.specular[0], material.specular[1], material.specular[2]);
    shader.setFloat("material.shininess", material.shininess);

    // draw mesh
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Model::Draw(Shader shader) {
    for(unsigned int i = 0; i < meshes.size(); i++) {
        meshes[i].Draw(shader);
    }
}

void Model::loadModel(std::string path) {
    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_ValidateDataStructure);
    printMaterialsMetadata(scene);

    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        LOGE("ERROR::ASSIMP::%s", import.GetErrorString());
        return;
    }
    directory = path.substr(0, path.find_last_of('/'));

    processNode(scene->mRootNode, scene);
}

void Model::processNode(aiNode *node, const aiScene *scene) {
    // process all the node's meshes (if any)
    for(unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
        meshes.push_back(processMesh(mesh, scene));
    }
    // then do the same for each of its children
    for(unsigned int i = 0; i < node->mNumChildren; i++) {
        processNode(node->mChildren[i], scene);
    }
}

Mesh Model::processMesh(aiMesh *mesh, const aiScene *scene) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;

    for(unsigned int i = 0; i < mesh->mNumVertices; i++) {
        Vertex vertex;
        // process vertex positions, normals and texture coordinates
        glm::vec3 vector;
        vector.x = mesh->mVertices[i].x;
        vector.y = mesh->mVertices[i].y;
        vector.z = mesh->mVertices[i].z;
        vertex.Position = vector;
        vector.x = mesh->mNormals[i].x;
        vector.y = mesh->mNormals[i].y;
        vector.z = mesh->mNormals[i].z;
        vertex.Normal = vector;
        vertices.push_back(vertex);

        if (mesh->mTextureCoords[0]) {// does the mesh contain texture coordinates?
            vertex.TexCoords[0] = mesh->mTextureCoords[0][i].x;
            vertex.TexCoords[1] = mesh->mTextureCoords[0][i].y;
        } else {
            vertex.TexCoords[0] = 0;
            vertex.TexCoords[1] = 0;
        }
    }
    // process indices
    for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for(unsigned int j = 0; j < face.mNumIndices; j++) {
            indices.push_back(face.mIndices[j]);
        }
    }
    // process material
    glm::vec3 ambient, diffuse, specular;
    float shininess = 2;
    if(scene->HasMaterials() && mesh->mMaterialIndex < scene->mNumMaterials) {
        aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
        std::vector<Texture> diffuseMaps = loadMaterialTextures(material,
                                            aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
        std::vector<Texture> specularMaps = loadMaterialTextures(material,
                                            aiTextureType_SPECULAR, "texture_specular");
        textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());

        aiColor3D v3;
        material->Get("$clr.ambient", 0, 0, v3);
        ambient.x = v3[0];
        ambient.y = v3[1];
        ambient.z = v3[2];
        material->Get("$clr.diffuse", 0, 0, v3);
        diffuse.x = v3[0];
        diffuse.y = v3[1];
        diffuse.z = v3[2];
        material->Get("$clr.specular", 0, 0, v3);
        specular.x = v3[0];
        specular.y = v3[1];
        specular.z = v3[2];
        ai_real v1;
        material->Get("$mat.shininess", 0, 0, v1);
        shininess = v1;
    } else {
        ambient.x = 1;
        ambient.y = 1;
        ambient.z = 1;
        diffuse.x = 1;
        diffuse.y = 1;
        diffuse.z = 1;
        specular.x = 1;
        specular.y = 1;
        specular.z = 1;
    }
    return Mesh(vertices, indices, textures, Material{ambient, diffuse, specular, shininess});
}

std::vector<Texture> Model::loadMaterialTextures(aiMaterial *mat, aiTextureType type, std::string typeName) {
    std::vector<Texture> textures;
    for(unsigned int i = 0; i < mat->GetTextureCount(type); i++) {
        aiString str;
        mat->GetTexture(type, i, &str);
        bool skip = false;
        for(unsigned int j = 0; j < textures_loaded.size(); j++) {
            if(std::strcmp(textures_loaded[j].path.data(), str.C_Str()) == 0) {
                textures.push_back(textures_loaded[j]);
                skip = true;
                break;
            }
        }
        if(!skip) {   // if texture hasn't been loaded already, load it
            Texture texture;
            texture.id = TextureFromFile(str.C_Str(), directory);
            texture.type = typeName;
            texture.path = str.C_Str();
            textures.push_back(texture);
            textures_loaded.push_back(texture); // add to loaded textures
        }
    }
    return textures;
}

void printMaterialsMetadata(const aiScene *scene) {
    if(!scene->HasMaterials())  {
        LOGD("No materials presented");
    }
    LOGD("materials in scene: %d", scene->mNumMaterials);

    for (unsigned int j = 0; j < scene->mNumMaterials; j++) {
        aiMaterial *material = scene->mMaterials[j];

        aiString name;
        aiReturn r = material->Get(AI_MATKEY_NAME, name);
        if (r == 0) {
            std::cout << "name " << name.C_Str() << "\n";
        } else {
            std::cout << "can not get name of material\n";
        }

        for (unsigned int i = 0; i < material->mNumProperties; i++) {
            aiMaterialProperty* ppp = material->mProperties[i];
            std::cout << "mKey " << ppp->mKey.C_Str() << "\n";
            std::cout << "mData " << ppp->mData << "\n";
            std::cout << "mType " << ppp->mType << "\n";
            std::cout << "mIndex " << ppp->mIndex << "\n";
            std::cout << "mDataLength " << ppp->mDataLength << "\n";

            if (ppp->mType == 4) {
                int v;
                material->Get(ppp->mKey.C_Str(), 0, 0, v);
                std::cout << "v " << v  << "\n" << "\n";
            } else if (ppp->mType == 3) {
                aiString v;
                material->Get(ppp->mKey.C_Str(), 0, 0, v);
                std::cout << "v " << v.C_Str() << "\n" << "\n";
            } else if (ppp->mType == 1) {
                ai_real v1;
                aiColor3D v3;
                aiColor4D v4;
                switch (ppp->mDataLength / sizeof(ai_real)) {
                case 1:
                    material->Get(ppp->mKey.C_Str(), 0, 0, v1);
                    std::cout << "v " << v1 << "\n" << "\n";
                    break;
                case 2:
                    //break??;
                case 3:
                    material->Get(ppp->mKey.C_Str(), 0, 0, v3);
                    std::cout << "v " << v3[0] << " " << v3[1] << " " << v3[2] << "\n" << "\n";
                    break;
                case 4:
                    material->Get(ppp->mKey.C_Str(), 0, 0, v4);
                    std::cout << "v " << v4[0] << " " << v4[1] << " " << v4[2] << " " << v4[3] << "\n" << "\n";
                }
            }
        }
    }
}

unsigned int TextureFromFile(const char *path, const std::string &directory) {
    std::string filename = std::string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        LOGE("Texture failed to load at path: %s", path);
        stbi_image_free(data);
    }

    return textureID;
}
