#ifndef __utils_h__
#define __utils_h__

#include "parser.h"

template <typename X, typename Y, typename Z>
void add_vectors(X& v1, Y& v2, Z& result)
{
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    result.z = v1.z + v2.z;
}


template <typename X, typename Y, typename Z>
void subtract_vectors(X& v1, Y& v2, Z& result)
{
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    result.z = v1.z - v2.z;
}



int add(int x, int y);

#endif
