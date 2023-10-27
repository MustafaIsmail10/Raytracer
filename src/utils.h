#ifndef __utils_h__
#define __utils_h__

#include "parser.h"
#include <cmath>

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
parser::Vec3f cross_product(X &v1, Y &v2)
{
    parser::Vec3f result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

template <typename X, typename Y>
float dot_product(X &v1, Y &v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <typename X>
parser::Vec3f compute_unit_vector(X v)
{
    float magnitude = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return divide_vector_by_scalar(magnitude, v);
}

parser::Vec3f compute_color(parser::Ray camera_ray, parser::Scene &scene);
parser::Vec3f compute_image_corner(const parser::Camera &camera, parser::Vec3f u, parser::Vec3f v, parser::Vec3f w);
parser::Vec3f compute_pixel_ray_direction(parser::Vec3f e, parser::Vec3f u, parser::Vec3f v, parser::Vec3f q, int row, int col, float pixel_width, float pixel_height);
parser::Vec3f compute_diffuse_shading(parser::Material material, parser::Vec3f normal, parser::Vec3f intersection_point, std::vector<parser::PointLight> point_lights);
parser::Vec3f compute_specular_shading(parser::Material material, parser::Vec3f normal, parser::Vec3f intersection_point, parser::Vec3f ray_direction, std::vector<parser::PointLight> point_lights);

#endif
