#ifndef SCENE_H
#define SCENE_H

#include"Material.h"
#include<vector>
#include"Mesh.h"
#include"Ray.h"
#include "acceleration.h"

class Scene
{
public:
    int width, height;
    vec3 eye;      // Position of eye for viewing scene
    quat orient;   // Represents rotation of -Z to view direction
    float ry;
    vec3 ambient;

    Material* currentMat;
    std::vector<Material> materials;
    std::vector<Shape*> shapes;
    std::vector<Shape*> lights;

    MeshData* sphMesh;
    MeshData* boxMesh;
    MeshData* cylMesh;

    AccelerationBvh* acceleration;
    Mode mode{ Phong };
    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
        const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const mat4& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);
    void WriteHdrImage(Color* image, const float pass);
    void AccelerationSetup();
};

vec3 TracePath(const Ray& ray, Scene* acceleration);
Intersection SampleLight(Scene* acceleration);
float PdfLight(const Sphere* sphere, int numLights);
Intersection SampleSphere(vec3 C, float R);
float GeometryFactor(Intersection A, Intersection B);

#endif // !SCENE_H