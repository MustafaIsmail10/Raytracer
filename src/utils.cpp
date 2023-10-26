#include "utils.h"
#include "parser.h"

int add(int x, int y) { return x + y; }

void process_image(parser::Camera *camera, unsigned char *image) {
  // For each image row
  // For each image col
  // Get the ray
  parser::Ray ray = get_ray(camera, 0, 1);
  // REST OF ALGO
}

parser::Ray get_ray(parser::Camera *camera, int row, int col) {
  float m, q;
  // Compute m
  parser::Vec3f scaled_gaze;
  multiply_scalar_with_vector(camera->near_distance, camera->gaze, scaled_gaze);
  add_vectors(camera->position, scaled_gaze, m);
  // Compute q
  parser::Vec3f u; // Find this with cross product
  cross_product(camera->up, camera->gaze, u);
  parser::Vec3f lu;
  parser::Vec3f tv;
  multiply_scalar_with_vector(camera->near_plane.l, u, lu);
  multiply_scalar_with_vector(camera->near_plane.t, camera->up, tv);
  add_vectors(lu, tv, q);
  add_vectors(q, m, q); // Double check this if the below gives expected result
  // Compute s
  float sv = (row + 0.5) * (camera->near_plane.l - camera->near_plane.r) /
             (camera->image_width);
  float su = (col + 0.5) * (camera->near_plane.t - camera->near_plane.b) /
             (camera->image_height);

  parser::Vec3f sv_v;
  parser::Vec3f su_u;
  multiply_scalar_with_vector(sv, camera->up, sv_v);
  multiply_scalar_with_vector(su, u, su_u);
  parser::Vec3f s;
  add_vectors(sv_v, su_u, s);
  add_vectors(s, q, s); // DOUBLE CHECK THIS AGAIN
  // Compute r(t) = e + (s - e)d
  parser::Vec3f d;
  subtract_vectors(s, camera->position, d);
  parser::Ray ray = parser::Ray{.e = camera->position, .d = d};
  return ray;
}
