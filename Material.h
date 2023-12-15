#ifndef MATERIAL_H
#define MATERIAL_H

#include<string>
#include"geom.h"
class Texture
{
public:
    unsigned int id;
    int width, height, depth;
    unsigned char* image;
    Texture(const std::string& path);
};

class Material
{
public:
    vec3 Kd, Ks, Kt;
    float alpha;
    float IOR;
    Texture* tex;

    virtual bool isLight() { return false; }

    Material() : Kd(vec3(1.0, 0.5, 0.0)), Ks(vec3(1, 1, 1)), alpha(1.0), tex(NULL) {}
    Material(const vec3 d, const vec3 s, const float a)
        : Kd(d), Ks(s), alpha(a), tex(NULL) {}
    Material(const vec3 d, const vec3 s, const float a, const vec3 t, const float ior)
        : Kd(d), Ks(s), alpha(a),Kt(t),IOR(ior), tex(NULL) {}
    Material(const Material& o) 
    { Kd = o.Kd;  Ks = o.Ks;  alpha = o.alpha;  tex = o.tex; }
    virtual vec3 EvalRadiance() 
    {
        return Kd;
    };
    void setTexture(const std::string path) { tex = new Texture(path); };
};

class Light : public Material
{
public:

    Light(const vec3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }
    virtual vec3 EvalRadiance() { return Kd; };
    //virtual void apply(const unsigned int program);
};
#endif // !MATERIAL_H