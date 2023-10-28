#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>

namespace parser
{
    // Notice that all the structures are as simple as possible
    // so that you are not enforced to adopt any style or design.
    struct Vec3f
    {
        float x, y, z;
    };

    struct Vec3i
    {
        int x, y, z;
    };

    struct Vec4f
    {
        float l, r, b, t;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        bool is_mirror;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
        Vec3f normal;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
    };

    struct Triangle
    {
        int material_id;
        Face face;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
        Vec3f normal;
    };

    struct Scene
    {
        // Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        // Functions
        void loadFromXml(const std::string &filepath);
    };

    struct Ray
    {
        unsigned int depth;
        Vec3f e;
        Vec3f d;
        Vec3f getPointFromTime(float t);
    };

    struct HitRecords
    {
        Vec3f intersection_point;
        Vec3f normal;
        float distance;
        int material_id;
        bool is_intersected;
    };
} // namespace parser

#endif
