#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "utils.h"

typedef unsigned char RGB[3];

int main(int argc, char *argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    for (int cam_num = 0; cam_num < scene.cameras.size(); cam_num++)
    {
        parser::Camera current_camera = scene.cameras[cam_num];
        unsigned char *image = new unsigned char[current_camera.image_width * current_camera.image_height * 3];
        float pixel_width = (current_camera.near_plane.y - current_camera.near_plane.x) / current_camera.image_width;
        float pixel_height = (current_camera.near_plane.w - current_camera.near_plane.z) / current_camera.image_height;
        // Redndering the image of the current camera
        parser::Vec3f r;  
        add_vectors(scene.vertex_data[5], scene.vertex_data[6], r);
        std::cout << r.x << " " << r.y << " " << r.z << std::endl;
    }

    const RGB BAR_COLOR[8] =
        {
            {255, 255, 255}, // 100% White
            {255, 255, 0},   // Yellow
            {0, 255, 255},   // Cyan
            {0, 255, 0},     // Green
            {255, 0, 255},   // Magenta
            {255, 0, 0},     // Red
            {0, 0, 255},     // Blue
            {0, 0, 0},       // Black
        };

    int width = 1920, height = 1080;
    int columnWidth = width / 8;

    unsigned char *image = new unsigned char[width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int colIdx = x / columnWidth;
            image[i++] = BAR_COLOR[colIdx][0];
            image[i++] = BAR_COLOR[colIdx][1];
            image[i++] = BAR_COLOR[colIdx][2];
        }
    }

    write_ppm("test.ppm", image, width, height);
}
