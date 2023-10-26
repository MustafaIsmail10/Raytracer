#include "utils.h"
#include "parser.h"
#include <iostream>

int add(int x, int y) { return x + y; }

// void process_image(parser::Camera *camera, unsigned char *image)
// {
//     // For each image row
//     // For each image col
//     // Get the ray
//     parser::Ray ray = compute_pixel_ray_direction(camera, 0, 1);
//     // REST OF ALGO
// }

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
parser::Vec3f compute_pixel_ray_direction(parser::Vec3f e, parser::Vec3f u , parser::Vec3f v , parser::Vec3f q, int row, int col, float pixel_width, float pixel_height)
{
    // Compute s
    float su = (col + 0.5) * pixel_width;
    float sv = (row + 0.5) * pixel_height;
    parser::Vec3f su_u =  multiply_scalar_with_vector(su, u);
    parser::Vec3f sv_v = multiply_scalar_with_vector(sv, v);
    parser::Vec3f s = subtract_vectors(su_u, sv_v);
    s = add_vectors(s, q);
    parser::Vec3f ray_direction = compute_unit_vector(subtract_vectors(s, e));
    return ray_direction;
}

parser::Vec3f compute_triangle_normal(parser::Scene &scene, parser::Triangle &triangle)
{
    // Vertext ID of the first VertexData is 1
    parser::Vec3f v0 = scene.vertex_data[triangle.indices.v0_id - 1];
    parser::Vec3f v1 = scene.vertex_data[triangle.indices.v1_id - 1];
    parser::Vec3f v2 = scene.vertex_data[triangle.indices.v2_id - 1];
    parser::Vec3f v0v1 = subtract_vectors(v1, v0);
    parser::Vec3f v0v2 = subtract_vectors(v2, v0);
    return compute_unit_vector(cross_product(v0v1, v0v2));
}

void compute_all_triangle_normals(parser::Scene &scene)
{
    // For each triangle
    for (int i = 0; i < scene.triangles.size(); i++)
    {
        parser::Triangle triangle = scene.triangles[i];
        parser::Vec3f normal = compute_triangle_normal(scene, triangle);
    }
    // Store the normal in the triangle
}