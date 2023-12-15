#include"Scene.h"
#ifdef _WIN32
// Includes for Windows
#include <windows.h>
#include <cstdlib>
#include <limits>
#include <crtdbg.h>
#else
// Includes for Linux
#endif
#include <random>


Scene::Scene()
{
	sphMesh = SphMesh();
	boxMesh = BoxMesh();
	cylMesh = CylMesh();
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh)
{
	for (int i = 0; i < mesh->triangles.size(); i++)
	{
		Shape* tri = new Triangle(mesh,currentMat, mesh->triangles[i]);
		shapes.push_back(tri);
		if (tri->material->isLight())
			lights.push_back(tri);
	}

}

quat Orientation(int i,
	const std::vector<std::string>& strings,
	const std::vector<float>& f)
{
	quat q(1, 0, 0, 0); // Unit quaternion
	while (i < strings.size()) {
		std::string c = strings[i++];
		if (c == "x")
			q *= angleAxis(f[i++] * Radians, Xaxis());
		else if (c == "y")
			q *= angleAxis(f[i++] * Radians, Yaxis());
		else if (c == "z")
			q *= angleAxis(f[i++] * Radians, Zaxis());
		else if (c == "q") {
			q *= quat(f[i + 0], f[i + 1], f[i + 2], f[i + 3]);
			i += 4;
		}
		else if (c == "a") {
			q *= angleAxis(f[i + 0] * Radians, normalize(vec3(f[i + 1], f[i + 2], f[i + 3])));
			i += 4;
		}
	}
	return q;
}

void Scene::Command(const std::vector<std::string>& strings,
	const std::vector<float>& f)
{
	if (strings.size() == 0) return;
	std::string c = strings[0];

	if (c == "screen") {
		// syntax: screen width height
		//realtime->setScreen(int(f[1]),int(f[2]));
		width = int(f[1]);
		height = int(f[2]);
	}

	else if (c == "camera")
	{
		eye = { f[1],f[2],f[3] };
		orient = Orientation(5, strings, f);
		ry = f[4];
	}

	else if (c == "ambient")
	{
		// syntax: ambient r g b
		// Sets the ambient color.  Note: This parameter is temporary.
		ambient = vec3(f[1], f[2], f[3]);
	}

	else if (c == "brdf") {
		// syntax: brdf  r g b   r g b  alpha
		// later:  brdf  r g b   r g b  alpha  r g b ior
		// First rgb is Diffuse reflection, second is specular reflection.
		// third is beer's law transmission followed by index of refraction.
		if(f.size() == 8)
			currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]);
		if(f.size() == 12)
			currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], vec3(f[8], f[9], f[10]), f[11]);
	}

	else if (c == "light") {
		// syntax: light  r g b   
		// The rgb is the emission of the light
		// Creates a Material instance to be picked up by successive shapes
		currentMat = new Light(vec3(f[1], f[2], f[3]));
	}

	else if (c == "sphere") {
		// syntax: sphere x y z   r
		// Creates a Shape instance for a sphere defined by a center and radius
		Shape* sphere = new Sphere(sphMesh, vec3(f[1], f[2], f[3]), f[4], currentMat);
		shapes.push_back(sphere);
		if (sphere->material->isLight())
			lights.push_back(sphere);

	}

	else if (c == "box") {
		// syntax: box bx by bz   dx dy dz
		// Creates a Shape instance for a box defined by a corner point and diagonal vector
		Shape* box = new Box(boxMesh, vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat);
		shapes.push_back(box);
		if (box->material->isLight())
			lights.push_back(box);
	}

	else if (c == "cylinder") {
		// syntax: cylinder bx by bz   ax ay az  r
		// Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
		//realtime->cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
		Shape* cylinder = new Cylinder(cylMesh, vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
		shapes.push_back(cylinder);
		if (cylinder->material->isLight())
			lights.push_back(cylinder);
	}


	else if (c == "mesh") {
		// syntax: mesh   filename   tx ty tz   s   <orientation>
		// Creates many Shape instances (one per triangle) by reading
		// model(s) from filename. All triangles are rotated by a
		// quaternion (qw qx qy qz), uniformly scaled by s, and
		// translated by (tx ty tz) .
		mat4 modelTr = translate(vec3(f[2], f[3], f[4]))
			* scale(vec3(f[5], f[5], f[5]))
			* toMat4(Orientation(6, strings, f));
		ReadAssimpFile(strings[1], modelTr);
	}


	else {
		fprintf(stderr, "\n*********************************************\n");
		fprintf(stderr, "* Unknown command: %s\n", c.c_str());
		fprintf(stderr, "*********************************************\n\n");
	}
}

void Scene::TraceImage(Color* image, const int pass)
{
	float rx = ry * (static_cast<float>(width) / static_cast<float>(height));
	vec3 X = rx * transformVector(orient, Xaxis());
	vec3 Y = ry * transformVector(orient, Yaxis());
	vec3 Z = transformVector(orient, Zaxis());

	for (int i = 0; i < pass; i++)
	{
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop

		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				float x_ = myrandom(RNGen);
				float y_ = myrandom(RNGen);

				float dx = (2.f * (x + x_) / static_cast<float>(width)) - 1.f;
				float dy = (2.f * (y + y_) / static_cast<float>(height)) - 1.f;
				Ray ray{ eye, normalize(dx * X + dy * Y - Z) };
				vec3  color = TracePath(ray, this);
				if(!isnan(color.x)&& !isnan(color.y)&& !isnan(color.z)
					&&!isinf(color.x) && !isinf(color.y) && !isinf(color.z))
					image[y * width + x] += Color(color);
			}

		}
		if (i == 1)
		{
			std::cout << i << std::endl;
			WriteHdrImage(image, i);
		}
		if (i == 8)
		{
			std::cout << i << std::endl;
			WriteHdrImage(image, i);
		}
		if (i== 64)
		{
			std::cout << i << std::endl;
			WriteHdrImage(image, i);
		}
		if (i == 512)
		{
			std::cout << i << std::endl;
			WriteHdrImage(image, i);
		}
		if (i == 4096)
		{
			std::cout << i << std::endl;
			WriteHdrImage(image, i);
		}
	}

	WriteHdrImage(image,pass);

}
#include "rgbe.h"
void Scene::WriteHdrImage(Color* image, const float pass)
{
	// Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
	float* data = new float[width * height * 3];
	float* dp = data;
	for (int y = height - 1; y >= 0; --y)
	{
		for (int x = 0; x < width; ++x)
		{
			Color pixel = (image[y * width + x]) / (pass/2.f);

			*dp++ = pixel[0];
			*dp++ = pixel[1];
			*dp++ = pixel[2];
		}
	}

	// Write image to file in HDR (a.k.a RADIANCE) format
	rgbe_header_info info;
	char errbuf[100] = { 0 };
	std::string outName = "testscene.hdr";
	FILE* fp = fopen(outName.c_str(), "wb");
	info.valid = false;
	int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
	if (r != RGBE_RETURN_SUCCESS)
		printf("error: %s\n", errbuf);

	r = RGBE_WritePixels_RLE(fp, data, width, height, errbuf);
	if (r != RGBE_RETURN_SUCCESS)
		printf("error: %s\n", errbuf);
	fclose(fp);

	delete data;
}

void Scene::AccelerationSetup()
{
	acceleration = new AccelerationBvh(shapes);
}

vec3 TracePath(const Ray& ray, Scene* scene)
{
	auto P = scene->acceleration->intersect(ray);
	scene->mode = GGX;
	vec3 C{ 0,0,0 };
	vec3 W{ 1,1,1 };
	vec3 N = normalize(P.N);
	if (P.object == nullptr)
		return C;
	if (P.object->material->isLight())
		return P.object->material->EvalRadiance();

	const float RussianRoulette = 0.8f;
	vec3 wo = -ray.D;
	vec3 wi{ 0 };
	while (myrandom(RNGen) <= RussianRoulette)
	{
		{
			// Explicit light connection
			Intersection L = SampleLight(scene);


			float s =   length(L.object->material->Kd) + length(L.object->material->Ks) + length(L.object->material->Kt);
			float pd =length(L.object->material->Kd) / s;
			float pr = length(L.object->material->Ks) / s;
			float pt = length(L.object->material->Kt) / s;
			float alpha = L.object->material->alpha;

			float p = PdfLight(reinterpret_cast<Sphere*>(L.object), scene->lights.size()) / GeometryFactor(P, L);
			float q = PdfBrdf(L.object, wo, N, wi, pd, pr, pt, alpha, scene->mode) * RussianRoulette;


			float w_mis = (p * p) / ((p * p) + (q * q));
			wi = normalize(L.P - P.P);
			Ray I = Ray{ P.P, wi };
			auto I_result = scene->acceleration->intersect(I);
			if (p > 0 && I_result.object != nullptr && I_result.P == L.P)
			{
				vec3 f = P.object->EvalScattering(P.t, wo, normalize(N), wi, scene->mode);
				C += W  * w_mis*(f / p) *L.object->material->EvalRadiance();
			}
		}

		{
			//Extend path
			N = normalize(P.N);
			float s = length(P.object->material->Kd) + length(P.object->material->Ks) + length(P.object->material->Kt);
			float pd = length(P.object->material->Kd) / s;
			float pr = length(P.object->material->Ks) / s;
			float pt = length(P.object->material->Kt) / s;
			float alpha = P.object->material->alpha;
			if (scene->mode != Phong)
				alpha = sqrt(1 / ((alpha + 2) / 2));
			wi = normalize(SampleBrdf(P.object,wo, N, pd,pr, alpha, scene->mode)); // Choose a sample direction from P
			Ray r = Ray{ P.P, wi };

			auto Q = scene->acceleration->intersect(r);
			if (Q.object == nullptr)
			{
				break;
			}

			vec3  f = P.object->EvalScattering(P.t,wo, N, wi, scene->mode);
			float  p = PdfBrdf(P.object,wo, N, wi, pd, pr, pt,alpha, scene->mode) * RussianRoulette;
			if (p < 0.00001f)
				break;
			W *= f / p;

			// Implicit light connection
			if (Q.object->material->isLight())
			{
				float q = PdfLight(reinterpret_cast<Sphere*>(Q.object), scene->lights.size()) / GeometryFactor(P, Q);
				float w_mis = (p * p) / ((p * p) + (q * q)) ;
				C += W * w_mis*Q.object->material->EvalRadiance();
				break;
			}
			// Step forward
			P = Q;
			N = normalize(P.N);
			wo =normalize(-wi);

		}

	}

	return C;
}

Intersection SampleLight(Scene* acceleration)
{
	std::random_device Lightdevice;
	std::mt19937_64 RNGenLight(Lightdevice());
	std::uniform_real_distribution<> randomLight(0.0, static_cast<double>(acceleration->lights.size()));
	Sphere* random_light = reinterpret_cast<Sphere*>(acceleration->lights[static_cast<int>(randomLight(Lightdevice))]);
	Intersection result = SampleSphere(random_light->center, random_light->r);
	result.object = random_light;
	return result;
}

float PdfLight(const Sphere* sphere, int numLights)
{
	float area = 4.f * PI * (sphere->r* sphere->r);
	return 1.f / (area * numLights);
}

Intersection SampleSphere(vec3 C, float R)
{
	float p1 = myrandom(RNGen);
	float p2 = myrandom(RNGen);
	float z = (2.f * p1) - 1;
	float r = sqrt(1.f - (z * z));
	float a = 2.f * PI * p2;
	Intersection result;
	result.N = vec3(r * cos(a), r * sin(a), z);
	result.P = C + (R * normalize(result.N));

	return result;
}

float GeometryFactor(Intersection A, Intersection B)
{
	vec3 D = normalize(A.P - B.P);
	float a = dot(normalize(A.N), D);
	float b = dot(normalize(B.N), D);
	float c = dot(D, D) * dot(D, D);
	return abs(a * b / c);
}
