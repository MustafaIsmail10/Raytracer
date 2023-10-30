#include "utils.h"
#include <iostream>

/* ============================= Pixel Processing Functions ============================= */
parser::Vec3f compute_color(parser::Ray camera_ray, parser::Scene &scene)
{
    parser::Vec3f result{0.0, 0.0, 0.0};
    if (camera_ray.depth > scene.max_recursion_depth)
    {
        return result;
    }

    parser::HitRecord hit_record = find_nearest_intersection(scene, camera_ray);

    if (hit_record.is_intersected)
    {
        result = apply_shading(scene, camera_ray, hit_record);
        return result;
    }
    else if (camera_ray.depth == 0)
    {
        assing_vector_to_vector(result, scene.background_color);
    }

    // Return (0, 0, 0)
    return result;
}

/*
This function takes a color as input converts it to rounded integer and clamps it to 0-255
*/
parser::Vec3i clamp_color(parser::Vec3f color)
{
    parser::Vec3i clamped_color;
    clamped_color.x = std::min(255, (int)(std::round(color.x)));
    clamped_color.y = std::min(255, (int)(std::round(color.y)));
    clamped_color.z = std::min(255, (int)(std::round(color.z)));
    return clamped_color;
}

/*
This function computes the corner of image plane for a given camera.
The image corner is computed as follows: (image corner) q = m(image plane center) + lu + tv
l and t are the left and top values of the near plane respectively,
v is the up vector and w is the negative of the gaze vector.
u = v x w
*/
parser::Vec3f compute_image_corner(const parser::Camera &camera, parser::Vec3f u, parser::Vec3f v, parser::Vec3f w)
{
    // Compute m
    parser::Vec3f scaled_gaze = multiply_scalar_with_vector(camera.near_distance, camera.gaze);
    parser::Vec3f m = add_vectors(camera.position, scaled_gaze);

    // Compute q
    parser::Vec3f lu = multiply_scalar_with_vector(camera.near_plane.l, u);
    parser::Vec3f tv = multiply_scalar_with_vector(camera.near_plane.t, v);
    parser::Vec3f q = add_vectors(lu, tv);
    q = add_vectors(q, m);

    return q;
}

/*
This function computes the ray for a given pixel in the image. The ray is computed as follows:
r(t) = e + (s - e)d
where e is the camera position, s is the point on the near plane corresponding to the pixel
*/
parser::Vec3f compute_pixel_ray_direction(parser::Vec3f e, parser::Vec3f u, parser::Vec3f v, parser::Vec3f q, int row, int col, float pixel_width, float pixel_height)
{
    // Compute s
    float su = (col + 0.5) * pixel_width;
    float sv = (row + 0.5) * pixel_height;
    parser::Vec3f su_u = multiply_scalar_with_vector(su, u);
    parser::Vec3f sv_v = multiply_scalar_with_vector(sv, v);
    parser::Vec3f s = subtract_vectors(su_u, sv_v);
    s = add_vectors(s, q);
    parser::Vec3f ray_direction = compute_unit_vector(subtract_vectors(s, e));
    return ray_direction;
}

/* ============================= Computing Normals ============================= */

parser::Vec3f compute_triangle_normal(parser::Scene &scene, parser::Face &face)
{
    // Vertext ID of the first VertexData is 1
    parser::Vec3f v0 = scene.vertex_data[face.v0_id - 1];
    parser::Vec3f v1 = scene.vertex_data[face.v1_id - 1];
    parser::Vec3f v2 = scene.vertex_data[face.v2_id - 1];
    parser::Vec3f v1v2 = subtract_vectors(v2, v1);
    parser::Vec3f v1v0 = subtract_vectors(v0, v1);
    return compute_unit_vector(cross_product(v1v2, v1v0));
}

void compute_all_triangle_normals(parser::Scene &scene)
{
    // For each triangle
    for (int i = 0; i < scene.triangles.size(); i++)
    {
        scene.triangles[i].face.normal = compute_triangle_normal(scene, scene.triangles[i].face);
    }

    // For each mesh
    for (int i = 0; i < scene.meshes.size(); i++)
    {
        for (int j = 0; j < scene.meshes[i].faces.size(); j++)
        {
            scene.meshes[i].faces[j].normal = compute_triangle_normal(scene, scene.meshes[i].faces[j]);
        }
    }
}

/* We can't precompute sphere normals because we need intersection point */
parser::Vec3f compute_sphere_normal(parser::Scene &scene, parser::Sphere &sphere, parser::Vec3f intersection_point)
{
    // Vertext ID of the first VertexData is 1
    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1];
    parser::Vec3f normal = subtract_vectors(intersection_point, center);
    normal = divide_vector_by_scalar(sphere.radius, normal);

    return compute_unit_vector(normal);
}

/* ============================= Computing Intersections ============================= */

/* Find point of intersection (or no intersection) for a sphere */
parser::HitRecord intersect_sphere(parser::Sphere &sphere, parser::Ray &ray, parser::Scene &scene)
{
    parser::HitRecord hit_record;
    // Get the center of the sphere
    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id - 1];
    // 1. Compute the discriminant (sqrt part)
    // p1 = (d . (e - c)) ^ 2
    parser::Vec3f e_minuc_c = subtract_vectors(ray.e, center);
    float p1 = pow(dot_product(ray.d, e_minuc_c), 2);
    // p2 = (d .d) ((e - c).(e - c) - R^2)
    float p2 = dot_product(ray.d, ray.d) * (dot_product(e_minuc_c, e_minuc_c) - pow(sphere.radius, 2));
    float discriminant = p1 - p2;

    // 2. If discriminant is negative, no intersection
    if (discriminant < 0)
    {
        hit_record.is_intersected = false;
        return hit_record;
    }

    // 3. Find the two roots (Can do some optimization here?)
    float t1 = (-1 * dot_product(ray.d, e_minuc_c) + sqrt(discriminant)) / dot_product(ray.d, ray.d);
    float t2 = (-1 * dot_product(ray.d, e_minuc_c) - sqrt(discriminant)) / dot_product(ray.d, ray.d);

    // If both t1 and t2 are 0, then no intersection.
    if (std::fabs(t1) < EPSILON && std::fabs(t2) < EPSILON)
    {
        hit_record.is_intersected = false;
        return hit_record;
    }

    // 4. Find the nearest intersection
    float t = std::min(t1, t2);

    // 5. Compute the intersection point
    hit_record.intersection_point = ray.getPointFromTime(t);
    hit_record.normal = compute_sphere_normal(scene, sphere, hit_record.intersection_point);
    hit_record.material_id = sphere.material_id;
    hit_record.is_intersected = true;
    // We can use "t" as a replacement for distance perhaps?
    // hit_record.distance = magnitude(subtract_vectors(e, hit_record.intersection_point));
    hit_record.distance = t;

    return hit_record;
}

/* Find point of intersection (or no intersection) for a triangle(face). Uses Barycentric coordinates */
parser::HitRecord intersect_triangle(parser::Face &face, parser::Ray &ray, parser::Scene &scene, int material_id)
{
    parser::HitRecord hit_record;
    float angle_between_normal_and_ray = dot_product(face.normal, ray.d);
    if (angle_between_normal_and_ray >= 0)
    {
        hit_record.is_intersected = false;
        return hit_record;
    }

    parser::Vec3f a = scene.vertex_data[face.v0_id - 1];
    parser::Vec3f b = scene.vertex_data[face.v1_id - 1];
    parser::Vec3f c = scene.vertex_data[face.v2_id - 1];
    parser::Vec3f e = ray.e;
    parser::Vec3f d = ray.d;

    // We have Ax = B where x = (beta, gamma, t)
    // A = [v1 - v2, v1 - v3, d]
    // B = [v1 - e]
    // 1. Compute the determinant
    float a_determinant = compute_determinant(subtract_vectors(a, b), subtract_vectors(a, c), d);

    // 2. If determinant is 0, no intersection
    if (std::fabs(a_determinant) < EPSILON)
    {
        hit_record.is_intersected = false;
        return hit_record;
    }

    // Using cramers rule
    float beta = compute_determinant(subtract_vectors(a, e), subtract_vectors(a, c), d) / a_determinant;
    float gamma = compute_determinant(subtract_vectors(a, b), subtract_vectors(a, e), d) / a_determinant;
    float t = compute_determinant(subtract_vectors(a, b), subtract_vectors(a, c), subtract_vectors(a, e)) / a_determinant;

    // 3. If all conditions are met, then there is an intersection
    // 0 <= beta, 0 <= gamma, 0 <= beta + gamma <= 1, t > 0
    if ((beta + gamma <= 1) && beta >= 0 && gamma >= 0 && t > EPSILON)
    {
        hit_record.is_intersected = true;
        hit_record.intersection_point = ray.getPointFromTime(t);
        hit_record.normal = face.normal;
        hit_record.material_id = material_id;
        hit_record.distance = t;
        return hit_record;
    }

    hit_record.is_intersected = false;
    return hit_record;
}

/* Find the closest point of intersection of ray with a mesh, triangle or sphere */
parser::HitRecord find_nearest_intersection(parser::Scene &scene, parser::Ray &ray)
{
    float min_distance = std::numeric_limits<float>::max();
    parser::HitRecord min_hit_record = parser::HitRecord{.distance = min_distance, .is_intersected = false};
    // Loop through all meshes, triangles and spheres
    for (parser::Mesh mesh : scene.meshes)
    {
        for (parser::Face face : mesh.faces)
        {
            parser::HitRecord current_hit_record = intersect_triangle(face, ray, scene, mesh.material_id);
            if (current_hit_record.is_intersected && current_hit_record.distance < min_hit_record.distance)
            {
                min_hit_record = current_hit_record;
            }
        }
    }

    for (parser::Triangle triangle : scene.triangles)
    {
        // For each triangle, find the intersection
        parser::HitRecord current_hit_record = intersect_triangle(triangle.face, ray, scene, triangle.material_id);
        if (current_hit_record.is_intersected && current_hit_record.distance < min_hit_record.distance)
        {
            min_hit_record = current_hit_record;
        }
    }

    for (parser::Sphere sphere : scene.spheres)
    {
        // For each sphere, find the intersection
        parser::HitRecord current_hit_record = intersect_sphere(sphere, ray, scene);
        if (current_hit_record.is_intersected && current_hit_record.distance < min_hit_record.distance)
        {
            min_hit_record = current_hit_record;
        }
    }

    return min_hit_record;
}

/* ============================= Computing Shadings ============================= */

parser::Vec3f apply_shading(parser::Scene &scene, parser::Ray &ray, parser::HitRecord &hit_record)
{
    parser::Material material = scene.materials[hit_record.material_id - 1];
    parser::Vec3f final_color = multiply_vector_with_vector(scene.ambient_light, material.ambient);
    if (material.is_mirror)
    {
        parser::Vec3f out_going_ray = multiply_scalar_with_vector(-1, ray.d);
        parser::Vec3f n_cos_theta = multiply_scalar_with_vector(dot_product(hit_record.normal, out_going_ray) * 2, hit_record.normal);
        parser::Vec3f mirror_ray_direction = add_vectors(ray.d, n_cos_theta);
        // parser::Ray mirror_ray;
        // mirror_ray.e = hit_record.intersection_point;
        // mirror_ray.d = mirror_ray_direction;
        // mirror_ray.depth = ray.depth + 1;
        // parser::Vec3f mirror_color = compute_color(mirror_ray, scene);
        // final_color = add_vectors(final_color, multiply_vector_with_vector(material.mirror, mirror_color));
    }

    for (int point_light_num = 0; point_light_num < scene.point_lights.size(); point_light_num++)
    {
        parser::PointLight point_light = scene.point_lights[point_light_num];

        // Check if the object is in shadow or not
        if (is_in_shadow(scene, hit_record, point_light))
        {
            continue;
        }
        parser::Vec3f diffuse_color = compute_diffuse_shading(material, hit_record.normal, hit_record.intersection_point, point_light);
        final_color = add_vectors(final_color, diffuse_color);
    }

    return final_color;
}

bool is_in_shadow(parser::Scene &scene, parser::HitRecord &hit_record, parser::PointLight &point_light)
{
    parser::Vec3f new_intersction_point = multiply_scalar_with_vector(scene.shadow_ray_epsilon, hit_record.normal);
    new_intersction_point = add_vectors(hit_record.intersection_point, new_intersction_point);
    parser::Vec3f light_vector = subtract_vectors(point_light.position, new_intersction_point);
    float min_distance = compute_magnitude(light_vector);
    parser::Ray shadow_ray;
    shadow_ray.e = new_intersction_point;
    shadow_ray.d = compute_unit_vector(light_vector);
    parser::HitRecord shadow_hit_record = find_nearest_intersection(scene, shadow_ray);
    if (shadow_hit_record.is_intersected && shadow_hit_record.distance < min_distance)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
This function computer the diffuse shading a point given the material, normal, point lights.
*/
parser::Vec3f compute_diffuse_shading(parser::Material material, parser::Vec3f normal, parser::Vec3f intersection_point, parser::PointLight point_light)
{
    parser::Vec3f result{0, 0, 0};
    parser::Vec3f light_vector = subtract_vectors(point_light.position, intersection_point);
    parser::Vec3f light_direction = compute_unit_vector(light_vector);
    float cos_angle = dot_product(light_direction, normal);
    if (cos_angle > 0)
    {
        float light_distance_square = dot_product(light_vector, light_vector);
        parser::Vec3f light_indensity = divide_vector_by_scalar(light_distance_square, point_light.intensity);
        parser::Vec3f diffuse = multiply_scalar_with_vector(cos_angle, material.diffuse);
        diffuse = multiply_vector_with_vector(diffuse, light_indensity);
        result = add_vectors(result, diffuse);
    }

    return result;
}

parser::Vec3f compute_specular_shading(parser::Material material, parser::Vec3f normal, parser::Vec3f intersection_point, parser::PointLight point_light)
{
    // w_i -> light_direction
    // intersection_point -> w_0
    parser::Vec3f result{0, 0, 0};
    parser::Vec3f light_vector = subtract_vectors(point_light.position, intersection_point);
    parser::Vec3f light_direction = compute_unit_vector(light_vector);
    parser::Vec3f half_vector = compute_unit_vector(add_vectors(light_direction, intersection_point));
    float cos_angle = dot_product(half_vector, normal);
    cos_angle = pow(cos_angle, material.phong_exponent);
    if (cos_angle > 0)
    {
        float light_distance_square = dot_product(light_vector, light_vector);
        parser::Vec3f intensity = divide_vector_by_scalar(light_distance_square, point_light.intensity);
        parser::Vec3f specular = multiply_scalar_with_vector(cos_angle, material.specular);
        specular = multiply_vector_with_vector(specular, intensity);
        result = add_vectors(result, specular);
    }

    return result;
}
