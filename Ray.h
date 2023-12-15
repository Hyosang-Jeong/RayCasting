#ifndef RAY_H
#define RAY_H


#include"geom.h"

class Material;
class MeshData;
class Intersection;
class SimpleBox;
enum Mode
{
	Phong,
	Beckman,
	GGX
};
class Ray
{
public:
	Ray(vec3 Q, vec3 D);
	vec3 Q;
	vec3 D;
	vec3 eval(float t);
};

class Shape 
{
public:
	virtual  bool intersect(Ray, Intersection&) = 0;
	vec3 EvalScattering(float t, vec3 wo, vec3 N, vec3 wi, Mode mode);
	Material* material;
	MeshData* meshdata;
	mat4 modelTR;
	vec3 center;
	vec3 min;
	vec3 max;
	SimpleBox* box;
	float area;
};

class Sphere : public Shape
{
public:
	Sphere(MeshData*  meshData, const vec3 center, const float r, Material* mat);
	float r;
	virtual  bool intersect(Ray, Intersection&) override;
};

class Box : public Shape
{
public:
	Box(MeshData* meshData, const vec3 base, const vec3 diag, Material* mat);
	virtual  bool intersect(Ray, Intersection&) override;
	vec3 base;
	vec3 diag;
};

class Cylinder : public Shape
{
public:
	Cylinder(MeshData* meshData, const vec3 base, const vec3 axis, const float radius, Material* mat);
	vec3 base;
	vec3 axis;
	float radius;
	virtual  bool intersect(Ray, Intersection&) override;
};

class Triangle : public Shape
{
public:
	Triangle(MeshData* meshData,  Material* mat, ivec3 i);
	ivec3 index;
	virtual  bool intersect(Ray, Intersection&) override;
};

class Intersection
{
public:
	float t = 1000.f;
	Shape* object;
	vec3 P;
	vec3 N;
	float distance() const { return t; }  // A function the BVH traversal needs to be supplied.
};

class Slab
{
public:
	vec3 N;
	float d0;
	float d1;
};
class Interval
{
public:
	Interval();

	Interval(float t0_, float t1_, vec3 n1_, vec3 n2_);
	void empty();
	void intersect(const Interval& other);
	bool intersect(const Ray& ray,const Slab& slab);
	float t0, t1;
	vec3 N0, N1;

}; 


vec3 SampleBrdf(Shape* object,vec3 w,vec3 N,float pd,float pr,float alpha, Mode mode);
vec3 SampleLobe(vec3 A, float c, float pi);
float PdfBrdf(Shape* object,vec3 wo, vec3 N, vec3 wi, float pd, float pr, float pt, float alpha,Mode mode);
float D(vec3 m,vec3 N, float alpha, Mode mode);
vec3 F(vec3 Ks,float m);
float G(vec3 v, vec3 m, vec3 N, float alpha, Mode mode);
#endif // !RAY_H