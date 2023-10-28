#ifndef __utils_h__
#define __utils_h__

#include "parser.h"
#include <cmath>
#include <limits>

const float EPSILON = 1e-6;

/* ============================= Vector Operations ============================= */
parser::Vec3f add_vectors(const parser::Vec3f &v1, const parser::Vec3f &v2);
parser::Vec3f subtract_vectors(const parser::Vec3f &v1, const parser::Vec3f &v2);
parser::Vec3f multiply_scalar_with_vector(float s, const parser::Vec3f &v1);
parser::Vec3f multiply_vector_with_vector(const parser::Vec3f &v1, const parser::Vec3f &v2);
parser::Vec3f divide_vector_by_scalar(float s, const parser::Vec3f &v1);
parser::Vec3f cross_product(const parser::Vec3f &v1, const parser::Vec3f &v2);
parser::Vec3f compute_unit_vector(const parser::Vec3f &v);
float dot_product(const parser::Vec3f &v1, const parser::Vec3f &v2);
float compute_magnitude(const parser::Vec3f &v);
float compute_determinant(const parser::Vec3f &v1, const parser::Vec3f &v2, const parser::Vec3f &v3);

/* ============================= Pixel Processing Functions ============================= */
parser::Vec3i compute_color(parser::Ray camera_ray, parser::Scene &scene);
parser::Vec3f compute_image_corner(const parser::Camera &camera, parser::Vec3f u, parser::Vec3f v, parser::Vec3f w);
parser::Vec3f compute_pixel_ray_direction(parser::Vec3f e, parser::Vec3f u, parser::Vec3f v, parser::Vec3f q, int row, int col, float pixel_width, float pixel_height);

/* ============================= Computing Normals ============================= */
parser::Vec3f compute_triangle_normal(parser::Scene &scene, parser::Face &face);
void compute_all_triangle_normals(parser::Scene &scene);
parser::Vec3f compute_sphere_normal(parser::Scene &scene, parser::Sphere &sphere, parser::Vec3f intersection_point);

/* ============================= Computing Intersections ============================= */
parser::HitRecords intersect_sphere(parser::Sphere &sphere, parser::Ray &ray, parser::Scene &scene);
parser::HitRecords intersect_triangle(parser::Face &face, parser::Ray &ray, parser::Scene &scene, int material_id);
parser::HitRecords find_nearest_intersection(parser::Scene &scene, parser::Ray &ray);

/* ============================= Computing Shadings ============================= */
parser::Vec3f compute_diffuse_shading(parser::Material material, parser::Vec3f normal, parser::Vec3f intersection_point, std::vector<parser::PointLight> point_lights);
parser::Vec3f compute_specular_shading(parser::Material material, parser::Vec3f normal, parser::Vec3f intersection_point, parser::Vec3f ray_direction, std::vector<parser::PointLight> point_lights);

#endif
