#ifndef __utils_h__
#define __utils_h__

#include "parser.h"

template <typename X, typename Y, typename Z>
void add_vectors(X &v1, Y &v2, Z &result) {
  result.x = v1.x + v2.x;
  result.y = v1.y + v2.y;
  result.z = v1.z + v2.z;
}

template <typename X, typename Y, typename Z>
void subtract_vectors(X &v1, Y &v2, Z &result) {
  result.x = v1.x - v2.x;
  result.y = v1.y - v2.y;
  result.z = v1.z - v2.z;
}

template <typename Y, typename Z>
void multiply_scalar_with_vector(float &s, Y &v1, Z &result) {
  result.x = v1.x * s;
  result.y = v1.y * s;
  result.z = v1.z * s;
}

template <typename X, typename Y, typename Z>
void cross_product(X &v1, Y &v2, Z &result) {
  result.x = v1.y * v2.z - v1.z * v2.y;
  result.y = v1.z * v2.x - v1.x * v2.z;
  result.z = v1.x * v2.y - v1.y * v2.x;
}

int add(int x, int y);

void process_image(parser::Camera *, unsigned char *);
parser::Ray get_ray(parser::Camera *, int, int);

#endif
