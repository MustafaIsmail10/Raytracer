#ifndef __utils_h__
#define __utils_h__

#include "parser.h"
#include <cmath>
#include <limits>

const double EPSILON = 1e-12;

/* ============================= Vector Operations ============================= */
template <typename X, typename Y>
parser::Vec3f add_vectors(X &v1, Y &v2)
{
    parser::Vec3f result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;
    return result;
}

template <typename X, typename Y>
parser::Vec3f subtract_vectors(X &v1, Y &v2)
{
    parser::Vec3f result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    result.z = v1.z - v2.z;
    return result;
}

template <typename Y>
parser::Vec3f multiply_scalar_with_vector(float s, Y &v1)
{
    parser::Vec3f result;
    result.x = v1.x * s;
    result.y = v1.y * s;
    result.z = v1.z * s;
    return result;
}

template <typename Y, typename X>
parser::Vec3f multiply_vector_with_vector(Y &v1, X &v2)
{
    parser::Vec3f result;
    result.x = v1.x * v2.x;
    result.y = v1.y * v2.y;
    result.z = v1.z * v2.z;
    return result;
}

template <typename Y>
parser::Vec3f divide_vector_by_scalar(float s, Y &v1)
{
    parser::Vec3f result;
    result.x = v1.x / s;
    result.y = v1.y / s;
    result.z = v1.z / s;
    return result;
}

template <typename X, typename Y>
float dot_product(const X &v1, const Y &v2)
{
    float result = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    return result;
}

template <typename X>
float compute_magnitude(const X &v)
{
    float result = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return result;
}

template <typename X>
float compute_determinant(const X &v1, const X &v2, const X &v3)
{
    float result = v1.x * (v2.y * v3.z - v2.z * v3.y) -
                   v1.y * (v2.x * v3.z - v2.z * v3.x) +
                   v1.z * (v2.x * v3.y - v2.y * v3.x);
    return result;
}

template <typename X, typename Y>
parser::Vec3f cross_product(X &v1, Y &v2)
{
    parser::Vec3f result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

template <typename X>
parser::Vec3f compute_unit_vector(X v)
{
    float magnitude = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return divide_vector_by_scalar(magnitude, v);
}

template <typename X, typename Y>
void assing_vector_to_vector(X &v1, const Y &v2)
{
    v1.x = v2.x;
    v1.y = v2.y;
    v1.z = v2.z;
}

/* ============================= Pixel Processing Functions ============================= */
parser::Vec3f compute_color(parser::Ray camera_ray, parser::Scene &scene);
parser::Vec3f compute_image_corner(const parser::Camera &camera, parser::Vec3f u, parser::Vec3f v, parser::Vec3f w);
parser::Vec3f compute_pixel_ray_direction(parser::Vec3f e, parser::Vec3f u, parser::Vec3f v, parser::Vec3f q, int row, int col, float pixel_width, float pixel_height);
parser::Vec3i clamp_color(parser::Vec3f color);

/* ============================= Computing Normals ============================= */
parser::Vec3f compute_triangle_normal(parser::Scene &scene, parser::Face &face);
void compute_all_triangle_normals(parser::Scene &scene);
parser::Vec3f compute_sphere_normal(parser::Scene &scene, parser::Sphere &sphere, parser::Vec3f intersection_point);

/* ============================= Computing Intersections ============================= */
parser::HitRecord intersect_sphere(parser::Sphere &sphere, parser::Ray &ray, parser::Scene &scene);
parser::HitRecord intersect_triangle(parser::Face &face, parser::Ray &ray, parser::Scene &scene, int material_id, bool is_shadow_ray);
parser::HitRecord find_nearest_intersection(parser::Scene &scene, parser::Ray &ray, bool is_shadow_ray);

/* ============================= Computing Shadings ============================= */
parser::Vec3f apply_shading(parser::Scene &scene, parser::Ray &ray, parser::HitRecord &hit_record);
parser::Vec3f compute_diffuse_shading(parser::Material material, parser::Vec3f normal, parser::Vec3f intersection_point, parser::PointLight point_light, parser::Vec3f light_vector, parser::Vec3f light_direction);
parser::Vec3f compute_specular_shading(parser::Material material, parser::Ray &ray, parser::Vec3f normal, parser::Vec3f intersection_point, parser::PointLight point_light, parser::Vec3f light_vector, parser::Vec3f light_direction);
bool is_in_shadow(parser::Scene &scene, parser::HitRecord &hit_record, parser::PointLight &point_light);
#endif
