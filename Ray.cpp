#include "Ray.h"
#include"Material.h"
#include"Mesh.h"
#include"acceleration.h"
Sphere::Sphere(MeshData* meshData, const vec3 Center, const float r_, Material* mat)
{
	mat4 m = translate(Center) * scale(vec3(r, r, r));
     center = Center;
	 
	 material = mat;
	 meshdata = meshData;
	 modelTR = m;
	 r = r_;
	 area = 4 * PI * r * r;
	 vec3 rrr = vec3(r);

	 min = center + rrr;
	 max = center - rrr;
	 box = new SimpleBox();
	 box->extend(min);
	 box->extend(max);
}

bool Sphere::intersect(Ray R, Intersection& I)
{
	float a =dot( R.D ,R.D);
	float b =2.f * dot( R.Q - center, R.D);
	float c = dot(R.Q - center, R.Q - center) - (r * r);

	if ((b * b) - (4.f * a * c) < 0.0001f)
		return false;
	if (r <= 0.5)
		r = r;
	float t_minus = (-b - sqrt((b * b) - (4.f * a * c))) / (2.f * a);
	float t_plus    = (-b + sqrt((b * b) - (4.f * a * c))) / (2.f * a);
	if (t_minus < 0.0001f && t_plus < 0.0001f)
		return false;
	if (t_minus > 0.0001f && I.t > t_minus)
		I.t = t_minus;
	else if (t_plus > 0.0001f && I.t > t_plus)
		I.t = t_plus;
	else
		return false;
	I.P = R.eval(I.t);
	I.N = normalize(I.P - center);
	return true;
}

Box::Box(MeshData* meshData, const vec3 base, const vec3 diag, Material* mat): base(base),diag(diag)
{
	mat4 m = translate(base) * scale(vec3(diag[0], diag[1], diag[2]));
	material = mat;
	meshdata = meshData;
	modelTR = m;

	vec3 point1 = base;
	vec3 point2 = base + diag;
	float xMin = std::min(point1.x, point2.x);
	float yMin = std::min(point1.y, point2.y);
	float zMin = std::min(point1.z, point2.z);

	float xMax = std::max(point1.x, point2.x);
	float yMax = std::max(point1.y, point2.y);
	float zMax = std::max(point1.z, point2.z);

	min = vec3(xMin, yMin, zMin);
	max = vec3(xMax, yMax, zMax);
	box = new SimpleBox();
	box->extend(min);
	box->extend(max);
}

bool Box::intersect(Ray R, Intersection& intersection)
{
	float t0 = 0;
	float t1 = std::numeric_limits<float>::infinity();
	Slab s1 = Slab{ vec3(1, 0, 0), -base.x, -base.x - diag.x };
	Slab s2 = Slab{ vec3(0, 1, 0), -base.y, -base.y - diag.y };
	Slab s3 = Slab{ vec3(0, 0, 1), -base.z, -base.z - diag.z };

	Interval interval1;
	Interval interval2;
	Interval interval3;
	bool i1 = interval1.intersect(R, s1);
	bool i2 = interval2.intersect(R, s2);
	bool i3 = interval3.intersect(R, s3);
	if (i1)
	{
		t0 = std::max(t0, interval1.t0);
		t1 = std::min(t1, interval1.t1);
	}
	if (i2)
	{
	
	t0 = std::max(t0, interval2.t0);
	t1 = std::min(t1, interval2.t1);
	}
	if (i3)
	{
		t0 = std::max(t0, interval3.t0);
		t1 = std::min(t1, interval3.t1);
	}

	if (!i1 || !i2 || !i3)
		return false;
	if (t0 > t1)
		return false;
	if (t0 < 0.0001f && t1 < 0.0001f)
	{
		return false;
	}
	else
	{
		if (t0 > 0)
		{
			intersection.t = t0;
			intersection.P = R.eval(intersection.t);

			if (intersection.t - interval1.t0 < 0.0001f && intersection.t - interval1.t0 > -0.0001f)
			{
					intersection.N = vec3(1.f, 0.f, 0.f);
			}
			else if (intersection.t - interval2.t0 < 0.0001f && intersection.t - interval2.t0 > -0.0001f)
			{
				intersection.N = vec3(0.f, 1.f, 0.f);
			}
			else if (intersection.t - interval3.t0 < 0.0001f && intersection.t - interval3.t0 > -0.0001f)
			{
				intersection.N = vec3(0.f, 0.f, 1.f);
			}

			return true;
		}
		else
		{
			intersection.t = t1;
			intersection.P = R.eval(intersection.t);

			if (intersection.t - interval1.t1 == 0)
			{
				intersection.N = -normalize(s1.N);
			}
			else if (intersection.t - interval3.t1 == 0)
			{
				intersection.N = -normalize(s3.N);
			}
			else
				return false;
			return true;
		}
	}

	return false;
}


Cylinder::Cylinder(MeshData* meshData, const vec3 base, const vec3 axis, const float radius, Material* mat):base(base),axis(axis),radius(radius)
{
	vec3 Z(0.0f, 0.0f, 1.0f);
	vec3 C = normalize(axis);
	vec3 B = cross(C, Z);
	if (length(B) < 1e-8)
		B = vec3(0, 1, 0);
	else
		B = normalize(B);
	vec3 A = normalize(cross(B, C));

	mat4 R(A[0], A[1], A[2], 0.0f,
		B[0], B[1], B[2], 0.0f,
		C[0], C[1], C[2], 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f);
	mat4 m = translate(base) * R * scale(vec3(radius, radius, length(axis)));
	vec3 rrr(radius, radius, radius);

	material = mat;
	meshdata = meshData;
	modelTR = m;


	const vec3 point1 = base + rrr;
	const vec3 point2 = base - rrr;
	const vec3 point3 = (base + axis) + rrr;
	const vec3 point4 = (base + axis) - rrr;

	const float xMin =  std::min(std::min(point1.x, point2.x), std::min(point3.x, point4.x));
	const float yMin =  std::min(std::min(point1.y, point2.y), std::min(point3.y, point4.y));
	const float zMin =  std::min(std::min(point1.z, point2.z), std::min(point3.z, point4.z));
	const float xMax =  std::max(std::max(point1.x, point2.x), std::max(point3.x, point4.x));
	const float yMax = std::max(std::max(point1.y, point2.y), std::max(point3.y, point4.y));
	const float zMax =  std::max(std::max(point1.z, point2.z), std::max(point3.z, point4.z));

	min = vec3(xMin, yMin, zMin);
	max = vec3(xMax, yMax, zMax);
	box = new SimpleBox();
	box->extend(min);
	box->extend(max);
	center =base;
}

bool Cylinder::intersect(Ray R, Intersection& I)
{
	Interval interval1;
	Slab slab1 = Slab{ vec3(0,0,1),0, -length(axis) };

		vec3 A = normalize(axis);
		vec3 B = vec3(0, 0, 1);
		if (dot(A, B) == length(A) * length(B)) //PARALLEL
		{
			B = vec3(1, 0, 0);
		}

		B = normalize(cross(B, A));
		vec3 C = cross(A, B);

		mat3  m = mat3(B, C, A);
		mat3 Rotate = glm::transpose(m);

		vec3 Q = Rotate * (R.Q - center);
		vec3 D = Rotate * (R.D);
	bool first = interval1.intersect(Ray(Q,D), slab1);
	if (first)
	{

		const float a = (D.x * D.x) + (D.y * D.y);
		const float b = 2.f * ((D.x * Q.x) + (D.y * Q.y));
		const float c = (Q.x * Q.x) + (Q.y * Q.y) - (radius * radius);
		const float dis = (b * b) - (4.f * a * c);

		if (dis < 0.0001f)
			return false;

		float b0 = (-b - sqrt(dis)) / (2.f * a);
		float b1 = (-b + sqrt(dis)) / (2.f * a);

		const float t0 = std::max(interval1.t0, b0);
		const float t1 = std::min(interval1.t1, b1);

		if (t0 > t1)
			return false;
		if (t1 < 0.0001f)
			return false;

		if (t0 > 0.0001f)
		{
			I.t = t0;
			I.P = R.eval(I.t);
			if (I.t <= interval1.t0 + 0.0001f && I.t >= interval1.t0- 0.0001f)
			{
				if (Q.z < 0.0001f)
				{
					I.N = vec3(0, 0, -1);
					I.N = m * (I.N);
				}
				else
				{
					I.N = vec3(0, 0, 1);
					I.N = m * (I.N);
				}
			}
			else
			{
				vec3 N = { Q.x + (I.t * D.x), Q.y + (I.t * D.y), 0 };
				I.N = m * (N);
			}
		}
		else if (t1 > 0.0001f)
		{
			I.t = t1;
			I.P = R.eval(I.t);
			if (I.t <= interval1.t1 + 0.0001f && I.t >= interval1.t1 - 0.0001f)
			{
				if (Q.z < 0.0001f)
				{
					I.N = vec3(0, 0, -1);
					I.N = m * (I.N);
				}
				else
				{
					I.N = vec3(0, 0, 1);
					I.N = m * (I.N);
				}
			}
			else
			{
				vec3 N = { Q.x + (I.t * D.x), Q.y + (I.t * D.y), 0 };
				I.N = m * (N);
			}
		}
		
		return true;
	}
	return false;

}

Triangle::Triangle(MeshData* meshData, Material* mat, ivec3 i)
{
	meshdata = meshData;
	material = mat;	
	modelTR = Identity();
	index = i;
	vec3 point1 = meshData->vertices[i.x].pnt;
	vec3 point2 = meshData->vertices[i.y].pnt;
	vec3 point3 = meshData->vertices[i.z].pnt;
	float xMin = std::min(std::min(point1.x, point2.x), point3.x);
	float yMin = std::min(std::min(point1.y, point2.y), (point3.y));
	float zMin = std::min(std::min(point1.z, point2.z), point3.z);
	float xMax = std::max(std::max(point1.x, point2.x), point3.x);
	float yMax = std::max(std::max(point1.y, point2.y), point3.y);
	float zMax = std::max(std::max(point1.z, point2.z), point3.z);
	center = (point1 + point2 + point3) / 3.f;
	min = vec3(xMin, yMin, zMin);
	max = vec3(xMax, yMax, zMax);
	box = new SimpleBox();
	box->extend(min);
	box->extend(max);
}



bool Triangle::intersect(Ray R, Intersection& I)
{
	vec3 V0 = meshdata->vertices[index.x].pnt;
	vec3 V1 = meshdata->vertices[index.y].pnt;
	vec3 V2 = meshdata->vertices[index.z].pnt;

	vec3 N0 = meshdata->vertices[index.x].nrm;
	vec3 N1 = meshdata->vertices[index.y].nrm;
	vec3 N2 = meshdata->vertices[index.z].nrm;

	vec3 E1 = V1 - V0;
	vec3 E2 = V2 - V0;
	vec3 p = cross(R.D, E2);
	float d = dot(p, E1);
	if (d == 0.f)
	{
		// Ray is parallel to triangle
		return false;
	}
	vec3 S = R.Q - V0;
	float u = dot(p, S) / d;
	if (u < 0.0001f || u>1- 0.0001f) // Ray intersects plane, but outside E2 edge
		return false;
	vec3 q = cross(S, E1);
	float v = dot(R.D, q) / d;
	if (v < 0.0001f || (u + v) > 1- 0.0001f)// Ray intersects plane, but outside other edges
		return false;
	float t = dot(E2, q) / d;
	if (t < 0.0001f)// Ray's negative half intersects triangle
		return false;

		I.t = t;
		I.P = R.eval(t);
		I.N = (1 - u - v) * (N0)+u * (N1)+v * (N2);

	return true;
}

Ray::Ray(vec3 Q, vec3 D):Q(Q),D(D)
{
}

glm::vec3 Ray::eval(float t)
{
	return Q+ t*D;
}

Interval::Interval()
{
	t0 = 0;
	t1 = std::numeric_limits<float>::infinity();
}

Interval::Interval(float t0_, float t1_, vec3 n1_, vec3 n2_)
{

}

void Interval::empty()
{
	t0 = 0;
	t1 = -1;
}

void Interval::intersect(const Interval& other)
{
}

bool Interval::intersect(const Ray& ray, const Slab& slab)
{
	vec3 N = slab.N;
	vec3 D = ray.D;
	vec3 Q = ray.Q;
	if (dot(N, D) != 0) //Ray intersects both slab planes
	{
		t0 = -(slab.d0 + dot(N, Q))/ dot(N, D);
		t1 = -(slab.d1 + dot(N, Q)) / dot(N, D);
		
		if (t0 > t1)
			std::swap(t0, t1);
		return true;
	}
	else
	{
		float s0 = slab.d0 + dot(N, Q);
		float s1 = slab.d1 + dot(N, Q);

		if (s0 * s1 < 0)
		{
			t0 = 0;
			t1 = std::numeric_limits<float>::infinity();
		}
		else
		{
			t0 = 1;
			t1 = 0;
		}
	}
	return false;

}

vec3 SampleBrdf(vec3 w,vec3 N,float pd,float alpha, Mode mode)
{
	float probability = myrandom(RNGen);
	float a = myrandom(RNGen);
	float b = myrandom(RNGen);
	if (probability < pd)
	{
		return SampleLobe(N,sqrt(a),2.f*PI*b);
	}
	else
	{
		float theta;
		switch (mode)
		{
		case Phong:
			theta = pow(a, 1.f / (alpha + 1));
			break;
		case Beckman:
			theta = cos(atan(    sqrt(-1.0*alpha*alpha*log(1.0-a))      ));
			break;
		case GGX:
			theta = cos(atan((alpha * sqrt(a)) / (sqrt(1 - a))));
			break;
		default:
			break;
		}

		vec3 m = SampleLobe(N, theta, 2.f * PI * b);
		return (2.f * abs(dot(w, m)) * m) - w;
	}
}

vec3 SampleLobe(vec3 A, float c, float pi)
{
	A = normalize(A);
	float s = sqrt((1.f - (c*c)) );

	vec3 K = vec3(s * cos(pi), s * sin(pi), c);
	if (abs(A.z - 1) < 0.001f)
		return K;
	if (abs(A.z + 1) < 0.001f)
		return vec3(K.x, -K.y, -K.z);
	vec3 B = normalize(vec3(-A.y, A.x, 0));
	vec3 C =cross(A,B);
	return (K.x*B) + (K.y*C)+(K.z*A);
}

float PdfBrdf(vec3 wo, vec3 N, vec3 wi,float pd, float pr, float alpha,Mode mode)
{
	float Pd = abs(dot(wi, N)) / PI;
	vec3 m = glm::normalize((wo + wi));
	float Pr = D(m,N,alpha, mode) * abs(dot(m, N)) * (1.f / (4.f * abs(dot(wi, m))));
	return (pd * Pd) + (pr * Pr);
}
float D(vec3 m,vec3 N, float alpha, Mode mode)
{
	
	if (dot(m, N) > 0)
	{
		float result;
		float tantheta = sqrt(1.0 - (dot(m, N)) * (dot(m, N))) / (dot(m, N));
		switch (mode)
		{
		case Phong:
			result = (alpha + 2.f) / (2.f * PI);
			result *= pow(dot(m, N), alpha);
			break;
		case Beckman:
			
			result = (1.0 / (PI * alpha * alpha * pow(dot(N, m), 4))) * exp((-1 * tantheta * tantheta)/(alpha*alpha));
			break;
		case GGX:
			result = (alpha * alpha) / (PI * pow(dot(N, m), 4) * pow(alpha * alpha + tantheta * tantheta, 2));
			break;
		default:
			break;
		}
		return result;
	}
	return 0.0f;
}
vec3 F(vec3 Ks,float d)
{
	return Ks + (1.f - Ks) * powf(1 - abs(d), 5);
}
float G(vec3 v, vec3 m,vec3 N, float alpha, Mode mode)
{
	if (dot(v, m) / dot(v, N) > 0)
	{
		float theta = sqrt(1.f - pow(dot(v, N), 2)) / dot(v, N);
		float a;
		switch (mode)
		{
		case Phong:
			 a = sqrt(alpha / 2.f + 1) / theta;
			if (a < 1.6)
				return ((3.535f * a) + (2.181f * a * a)) / (1.0f + (2.276f * a) + (2.577 * a * a));
			else
				return 1.f;
			break;
		case Beckman:
			 a = 1.0 / (alpha * theta);
			if (a < 1.6)
				return ((3.535f * a) + (2.181f * a * a)) / (1.0f + (2.276f * a) + (2.577 * a * a));
			else
				return 1.f;
			break;
		case GGX:
			return 2 / (1 + sqrt(1 + alpha * alpha * theta * theta));
			break;
		default:
			break;
		}

	}
	return 0.0f;
}
vec3 Shape::EvalScattering(vec3 wo, vec3 N, vec3 wi, Mode mode)
{
	vec3 Ed = material->Kd / PI;
	vec3 m = glm::normalize((wo + wi));
	float alpha = material->alpha;
	if (mode != Phong)
		alpha = sqrt(1 / ((alpha + 2) / 2));
	vec3 Er = (D(m, N, alpha,mode) *G(wi,m,N,alpha, mode) * G(wo, m, N, alpha, mode) * F(material->Ks, dot(wi, m)))/(4.f * abs(dot(wi, N)) * abs(dot(wo, N)));

	return abs(dot(N, wi)) * (Ed + Er);
}
