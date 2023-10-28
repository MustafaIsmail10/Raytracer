#include "utils.h"

/* ============================= Vector Functions ============================= */
parser::Vec3f add_vectors(const parser::Vec3f &v1, const parser::Vec3f &v2)
{
    parser::Vec3f result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;
    return result;
}

parser::Vec3f subtract_vectors(const parser::Vec3f &v1, const parser::Vec3f &v2)
{
    parser::Vec3f result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    result.z = v1.z - v2.z;
    return result;
}

parser::Vec3f multiply_scalar_with_vector(float s, const parser::Vec3f &v1)
{
    parser::Vec3f result;
    result.x = v1.x * s;
    result.y = v1.y * s;
    result.z = v1.z * s;
    return result;
}

parser::Vec3f multiply_vector_with_vector(const parser::Vec3f &v1, const parser::Vec3f &v2)
{
    parser::Vec3f result;
    result.x = v1.x * v2.x;
    result.y = v1.y * v2.y;
    result.z = v1.z * v2.z;
    return result;
}

parser::Vec3f divide_vector_by_scalar(float s, const parser::Vec3f &v1)
{
    parser::Vec3f result;
    result.x = v1.x / s;
    result.y = v1.y / s;
    result.z = v1.z / s;
    return result;
}

parser::Vec3f cross_product(const parser::Vec3f &v1, const parser::Vec3f &v2)
{
    parser::Vec3f result;
    result.x = v1.y * v2.z - v1.z * v2.y;
    result.y = v1.z * v2.x - v1.x * v2.z;
    result.z = v1.x * v2.y - v1.y * v2.x;
    return result;
}

parser::Vec3f compute_unit_vector(const parser::Vec3f &v)
{
    float magnitude = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return divide_vector_by_scalar(magnitude, v);
}

float dot_product(const parser::Vec3f &v1, const parser::Vec3f &v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

float compute_magnitude(const parser::Vec3f &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

float compute_determinant(const parser::Vec3f &v1, const parser::Vec3f &v2, const parser::Vec3f &v3)
{
    return v1.x * (v2.y * v3.z - v2.z * v3.y) -
           v1.y * (v2.x * v3.z - v2.z * v3.x) +
           v1.z * (v2.x * v3.y - v2.y * v3.x);
}

/* ============================= Pixel Processing Functions ============================= */
parser::Vec3i compute_color(parser::Ray camera_ray, parser::Scene &scene)
{
    parser::Vec3i result{0, 0, 0};
    if (camera_ray.depth > scene.max_recursion_depth)
    {
        return result;
    }

    parser::HitRecords hit_record = find_nearest_intersection(scene, camera_ray);
    if (hit_record.is_intersected)
    {
        parser::Vec3f intersection_point = camera_ray.getPointFromTime(hit_record.distance);
        parser::Vec3f normal = hit_record.normal;
        parser::Material material = scene.materials[hit_record.material_id];
        // We should round here
        parser::Vec3f diffuse_shading = compute_diffuse_shading(material, normal, intersection_point, scene.point_lights);
        result.x = std::round(diffuse_shading.x);
        result.y = std::round(diffuse_shading.y);
        result.z = std::round(diffuse_shading.z);
    }
    else if (camera_ray.depth == 0)
    {
        result = scene.background_color;
    }

    // Return (0, 0, 0)
    return result;
}

/*
This function computes the image corner for a given camera. The image corner is computed as follows:
q = m + lu + tv
where e is the camera position, l and t are the left and top values of the near plane respectively,
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
    parser::Vec3f v0v1 = subtract_vectors(v1, v0);
    parser::Vec3f v0v2 = subtract_vectors(v2, v0);
    return compute_unit_vector(cross_product(v0v1, v0v2));
}

void compute_all_triangle_normals(parser::Scene &scene)
{
    // For each triangle
    for (parser::Triangle triangle : scene.triangles)
    {
        triangle.face.normal = compute_triangle_normal(scene, triangle.face);
    }

    // For each mesh
    for (parser::Mesh mesh : scene.meshes)
    {
        for (auto face : mesh.faces)
        {
            face.normal = compute_triangle_normal(scene, face);
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
parser::HitRecords intersect_sphere(parser::Sphere &sphere, parser::Ray &ray, parser::Scene &scene)
{
    parser::HitRecords hit_record;
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
    if (t1 < EPSILON && t2 < EPSILON)
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
parser::HitRecords intersect_triangle(parser::Face &face, parser::Ray &ray, parser::Scene &scene, int material_id)
{
    parser::HitRecords hit_record;
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
    if (a_determinant < EPSILON)
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
    if ((0 <= beta + gamma) && (beta + gamma <= 1) && beta >= 0 && gamma >= 0 && t > 0)
    {
        hit_record.is_intersected = true;
        hit_record.intersection_point = ray.getPointFromTime(t);
        hit_record.normal = compute_triangle_normal(scene, face);
        hit_record.material_id = material_id;
        hit_record.distance = t;
        return hit_record;
    }

    hit_record.is_intersected = false;
    return hit_record;
}

/* Find the closest point of intersection of ray with a mesh, triangle or sphere */
parser::HitRecords find_nearest_intersection(parser::Scene &scene, parser::Ray &ray)
{
    float min_distance = std::numeric_limits<float>::max();
    parser::HitRecords min_hit_record = parser::HitRecords{.distance = min_distance, .is_intersected = false};
    // Loop through all meshes, triangles and spheres
    for (parser::Mesh mesh : scene.meshes)
    {
        for (parser::Face face : mesh.faces)
        {
            parser::HitRecords current_hit_record = intersect_triangle(face, ray, scene, mesh.material_id);
            if (current_hit_record.is_intersected && current_hit_record.distance < min_hit_record.distance)
            {
                min_hit_record = current_hit_record;
            }
        }
    }

    for (parser::Triangle triangle : scene.triangles)
    {
        // For each triangle, find the intersection
        parser::HitRecords current_hit_record = intersect_triangle(triangle.face, ray, scene, triangle.material_id);
        if (current_hit_record.is_intersected && current_hit_record.distance < min_hit_record.distance)
        {
            min_hit_record = current_hit_record;
        }
    }

    for (parser::Sphere sphere : scene.spheres)
    {
        // For each sphere, find the intersection
        parser::HitRecords current_hit_record = intersect_sphere(sphere, ray, scene);
        if (current_hit_record.is_intersected && current_hit_record.distance < min_hit_record.distance)
        {
            min_hit_record = current_hit_record;
        }
    }

    return min_hit_record;
}

/* ============================= Computing Shadings ============================= */

/*
This function computer the diffuse shading a point given the material, normal, point lights.
*/
parser::Vec3f compute_diffuse_shading(parser::Material material, parser::Vec3f normal, parser::Vec3f intersection_point, std::vector<parser::PointLight> point_lights)
{
    parser::Vec3f result{0, 0, 0};
    for (int i = 0; i < point_lights.size(); i++)
    {
        parser::PointLight current_light = point_lights[i];
        parser::Vec3f light_vector = subtract_vectors(current_light.position, intersection_point);
        parser::Vec3f light_direction = compute_unit_vector(light_vector);
        float cos_angle = dot_product(light_direction, normal);
        if (cos_angle > 0)
        {
            float light_distance_square = dot_product(light_vector, light_vector);
            parser::Vec3f light_indensity = divide_vector_by_scalar(light_distance_square, current_light.intensity);
            parser::Vec3f diffuse = multiply_scalar_with_vector(cos_angle, material.diffuse);
            diffuse = multiply_vector_with_vector(diffuse, light_indensity);
            result = add_vectors(result, diffuse);
        }
    }
    return result;
}