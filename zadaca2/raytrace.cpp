#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "algorithm"
#include "geometry.h"

using namespace std;
typedef std::vector<Vec3f> Image;

struct Light
{
    Vec3f position;
    float intensity;

    Light(const Vec3f &position, const float &intensity) : position(position), intensity(intensity) {}
};
typedef std::vector<Light> Lights;

struct Material
{
    Vec2f albedo;
    Vec3f diffuse_color;
    float specular_exponent;

    Material(const Vec2f &a, const Vec3f &color, const float &coef) : albedo(a), diffuse_color(color), specular_exponent(coef) {}
    Material() : albedo(Vec2f(1, 0)), diffuse_color(), specular_exponent(1.f) {}
};

struct Object
{
    Material material;
    virtual bool ray_intersect(const Vec3f &p, const Vec3f &d, float &t) const = 0;
    virtual Vec3f normal(const Vec3f &p) const = 0;
};
typedef std::vector<Object *> Objects;

struct Sphere : Object
{
    Vec3f c; 
    float r; 

    Sphere(const Vec3f &c, const float &r, const Material &m) : c(c), r(r)
    {
        Object::material = m;
    }

    bool ray_intersect(const Vec3f &p, const Vec3f &d, float &t) const
    {
        Vec3f v = c - p;

        if (d * v < 0)
        {            
            return false;
        }
        else
        {
            // izracunaj projekciju
            Vec3f pc = p + d * ((d * v) / d.norm());
            if ((c - pc) * (c - pc) > r * r)
            {
                // nema sjeciste
                return false;
            }
            else
            {
                float dist = sqrt(r * r - (c - pc) * (c - pc));

                if (v * v > r * r) // izvor pogleda izvan sfere
                {
                    t = (pc - p).norm() - dist;
                }
                else // izvor pogleda unutar sfere
                {
                    t = (pc - p).norm() + dist;
                }

                return true;
            }
        }
    }

    Vec3f normal(const Vec3f &p) const
    {
        return (p - c).normalize();
    }
};

struct Cuboid : Object
{
    Vec3f c1;
    Vec3f c2;

    Vec3f normal1;
    Vec3f normal2;
    Vec3f normal3;

    Cuboid(const Vec3f &c1, const Vec3f &c2, const Material &m) : c1(c1), c2(c2)
    {
        Object::material = m;
    }

    bool ray_intersect(const Vec3f &p, const Vec3f &d, float &t) const
    {
        float tNear = numeric_limits<float>::min();
        float tFar = numeric_limits<float>::max();

        float minX = min(c1.x, c2.x);
        float minY = min(c1.y, c2.y);
        float minZ = min(c1.z, c2.z);
        float maxX = max(c1.x, c2.x);
        float maxY = max(c1.y, c2.y);
        float maxZ = max(c1.z, c2.z);

        if (d.x == 0)
        {
            if (p.x < minX || p.x > maxX)
            {
                return false;
            }
        }
        else
        {
            float t1 = (minX - p.x) / d.x;
            float t2 = (maxX - p.x) / d.x;

            if (t1 > t2)
            {
                swap(t1, t2);
            }

            tNear = max(tNear, t1);
            tFar = min(tFar, t2);
            if (tNear > tFar || tFar < 0)
                return false;
        }

        t = tNear;

        if (d.y == 0)
        {
            if (p.y < minY || p.y > maxY)
            {
                return false;
            }
        }
        else
        {
            float t1 = (minY - p.y) / d.y;
            float t2 = (maxY - p.y) / d.y;

            if (t1 > t2)
            {
                swap(t1, t2);
            }

            tNear = max(tNear, t1);
            tFar = min(tFar, t2);
            if (tNear > tFar || tFar < 0)
                return false;
        }

        t = tNear;

        if (d.z == 0)
        {
            if (p.z < minZ || p.z > maxZ)
            {
                return false;
            }
        }
        else
        {
            float t1 = (minZ - p.z) / d.z;
            float t2 = (maxZ - p.z) / d.z;

            if (t1 > t2)
            {
                swap(t1, t2);
            }

            tNear = max(tNear, t1);
            tFar = min(tFar, t2);

            if (tNear > tFar || tFar < 0)
                return false;
        }

        t = tNear;

        return true;
    }

    Vec3f normal(const Vec3f &p) const
    {
        Vec3f normal;

        if (abs(p.x - c1.x) < 0.01)
            normal = Vec3f(-1, 0, 0);
        else if (abs(p.x - c2.x) < 0.01)
            normal = Vec3f(1, 0, 0);
        else if (abs(p.y - c1.y) < 0.01)
            normal = Vec3f(0, -1, 0);
        else if (abs(p.y - c2.y) < 0.01)
            normal = Vec3f(0, 1, 0);
        else if (abs(p.z - c1.z) < 0.01)
            normal = Vec3f(0, 0, -1);
        else if (abs(p.z - c2.z) < 0.01)
            normal = Vec3f(0, 0, 1);

        return normal;
    }
};

struct Plane : Object
{
    Vec3f a, b, c, d;

    Plane(const Vec3f &a, const Vec3f &b, const Vec3f &c, const Vec3f &d, const Material &m) : a(a), b(b), c(c), d(d)
    {
        Object::material = m;
    }

    bool inside(const Vec3f &q) const
    {
        Vec3f n = normal(q);

        Vec3f ua = b - a, ub = c - b, uc = d - c, ud = a - d;
        Vec3f va = q - a, vb = q - b, vc = q - c, vd = q - d;

        if ((cross(ua, va) * n > 0) && (cross(ub, vb) * n > 0) && (cross(uc, vc) * n > 0) && (cross(ud, vd) * n > 0))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool ray_intersect(const Vec3f &p, const Vec3f &d, float &t) const
    {
        Vec3f n = normal(p);

        Vec3f vdif = a - p;

        float vdotn = d * n;

        if (fabs(vdotn) < 1.e-4)
            return false;

        float t1 = (vdif * n) / vdotn;

        if (fabs(t) < 0.0001)
            return false;

        Vec3f q = p + d * t;

        if (inside(q))
        {
            t = t1;
            return true;
        }
        else
            return false;
    }

    Vec3f normal(const Vec3f &p) const
    {
        Vec3f result = cross(b - a, c - a);
        result.normalize();
        return result;
    }
};

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const Objects &objs, Vec3f &hit, Material &material, Vec3f &N)
{
    // inicijalno, pretp. da je predmet daleko
    float dist = std::numeric_limits<float>::max();
    float obj_dist = dist;

    for (auto obj : objs)
    {

        if (obj->ray_intersect(orig, dir, obj_dist) && obj_dist < dist)
        {
            dist = obj_dist;
            hit = orig + dir * obj_dist;
            N = obj->normal(hit);
            material = obj->material;
        }
    }

    return dist < 1000;
}

Vec3f reflect(const Vec3f &I, const Vec3f &N)
{
    return I - (N * (2 * (I * N)));
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const Objects &objs, const Lights &lights, const float &depth)
{
    int maxDepth = 12;
    Vec3f hit_point;
    Vec3f hit_normal; 
    Material hit_material;

    if (!scene_intersect(orig, dir, objs, hit_point, hit_material, hit_normal) || depth > maxDepth)
    {
        return Vec3f(0.7, 0.9, 0.7); 
    }
    else
    {
        float diffuse_light_intensity = 0;
        float specular_light_intensity = 0;

        for (auto light : lights)
        {
            Vec3f light_dir = (light.position - hit_point).normalize();
            float light_dist = (light.position - hit_point).norm();

            // sjene
            Vec3f shadow_orig;
            Vec3f shadow_hit_point;
            Vec3f shadow_hit_normal;
            Material shadow_material;

            if (light_dir * hit_normal < 0)
            {
                shadow_orig = hit_point - hit_normal * 0.001;
                hit_normal = -hit_normal;
            }
            else
            {
                shadow_orig = hit_point + hit_normal * 0.001;
            }

            if (scene_intersect(shadow_orig, light_dir, objs, shadow_hit_point, shadow_material, shadow_hit_normal) && (shadow_hit_point - shadow_orig).norm() < light_dist)
            {
                continue;
            }
            diffuse_light_intensity += light.intensity * std::max(0.f, light_dir * hit_normal);

            Vec3f view_dir = (orig - hit_point).normalize();
            Vec3f half_vec = (view_dir + light_dir).normalize();
            specular_light_intensity += light.intensity * powf(std::max(0.f, half_vec * hit_normal), hit_material.specular_exponent);
        }

        Vec3f R = reflect(dir, hit_normal);

        Vec3f hitColor = hit_material.diffuse_color * hit_material.albedo[0] * diffuse_light_intensity 
                         + Vec3f(1, 1, 1) * hit_material.albedo[1] * specular_light_intensity;

        return hitColor = hitColor + cast_ray(hit_point + hit_normal * 0.1, R, objs, lights, depth + 1) * 0.3;
    }
}

// funkcija za crtanje
void render(const Objects &objects, const Lights &lights)
{
    // velicina slike
    const int width = 1024;
    const int height = 768;
    const int fov = 3.14159265358979323846 / 2.0; // pi / 2

    Image buffer(width * height);

    for (size_t j = 0; j < height; ++j)
    {
        for (size_t i = 0; i < width; ++i)
        {
            float x = (2 * (i + 0.5) / (float)width - 1) * tan(fov / 2.) * width / (float)height;
            float y = -(2 * (j + 0.5) / (float)height - 1) * tan(fov / 2.);
            Vec3f dir = Vec3f(x, y, -1).normalize();

            buffer[i + j * width] = cast_ray(Vec3f(0, 0, 0), dir, objects, lights, 0);
        }
    }

    std::ofstream ofs;
    ofs.open("./scene.ppm", std::ofstream::binary);
    ofs << "P6\n"
        << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {           
            unsigned char color = (unsigned char)(255.f * std::max(0.f, std::min(1.f, buffer[i][j])));
            ofs << color;
        }
    }

    ofs.close();
}

int main()
{
	Material red = Material(Vec2f(0.6, 0.3), Vec3f(1, 0, 0), 60);
	Material green = Material(Vec2f(0.6, 0.3), Vec3f(0, 0.5, 0), 60);
	Material blue = Material(Vec2f(0.9, 0.1), Vec3f(0, 0, 1), 10);
	Material gray = Material(Vec2f(0.9, 0.1), Vec3f(0.5, 0.5, 0.5), 10);
	Material purple = Material(Vec2f(0.6, 0.4), Vec3f(0.5, 0, 0.5), 30); 

	Cuboid table(Vec3f(-10, -4, -15), Vec3f(10, -3, -10), purple);
	Cuboid cuboid1(Vec3f(1, -3, -15), Vec3f(3, -1, -13), red);
	Cuboid cuboid2(Vec3f(-4, -3.5, -14), Vec3f(-2, -1.5, -12), gray);
	Cuboid cuboid3(Vec3f(5, -3.5, -14), Vec3f(7, -1.5, -12), blue);

	Sphere sphere1(Vec3f(-1.0, -1.5, -12), 1.5, green);  
	Sphere sphere2(Vec3f(3, -2.5, -11.5), 0.5, blue);

	Objects objs = {&table, &cuboid1, &cuboid2, &cuboid3, &sphere1, &sphere2};

	Light l1 = Light(Vec3f(-20, 25, 20), 1);
	Light l2 = Light(Vec3f(20, 30, 20), 1.5);
	Lights lights = {l1, l2};

	render(objs, lights);

	return 0;
}
