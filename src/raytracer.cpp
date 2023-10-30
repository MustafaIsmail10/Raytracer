#include "parser.h"
#include "ppm.h"
#include "utils.h"
#include <iostream>

typedef unsigned char RGB[3];

int main(int argc, char *argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    for (int cam_num = 0; cam_num < scene.cameras.size(); cam_num++) // looping through each camera
    {
        parser::Camera current_camera = scene.cameras[cam_num];
        unsigned char *image = new unsigned char[current_camera.image_width *
                                                 current_camera.image_height * 3];
        float pixel_width =
            (current_camera.near_plane.r - current_camera.near_plane.l) / current_camera.image_width;
        float pixel_height =
            (current_camera.near_plane.t - current_camera.near_plane.b) / current_camera.image_height;

        // Compute u, v, w, q
        parser::Vec3f v = current_camera.up;
        parser::Vec3f w = multiply_scalar_with_vector(-1, current_camera.gaze);
        parser::Vec3f u = cross_product(v, w);
        parser::Vec3f q = compute_image_corner(current_camera, u, v, w);
        compute_all_triangle_normals(scene); // compute all triangle normals (for all meshes

        unsigned int i = 0;
        for (int row = 0; row < current_camera.image_height; row++)
        {
            for (int col = 0; col < current_camera.image_width; col++)
            {
                parser::Ray ray;
                ray.d = compute_pixel_ray_direction(current_camera.position, u, v, q, row, col, pixel_width, pixel_height);
                ray.e = current_camera.position;
                ray.depth = 0;
                // Function below should return an RGB color
                // TODO: Add function to clamp the color to 0-255
                parser::Vec3f color = compute_color(ray, scene);
                // std::cout << color.x << " " << color.y << " " << color.z << std::endl;
                parser::Vec3i image_pixel_color = clamp_color(color);
                // std::cout << image_pixel_color.x << " " << image_pixel_color.y << " " << image_pixel_color.z << std::endl;
                image[i++] = image_pixel_color.x;
                image[i++] = image_pixel_color.y;
                image[i++] = image_pixel_color.z;
            }
        }
        write_ppm(current_camera.image_name.c_str(), image, current_camera.image_width, current_camera.image_height);
    }
}
