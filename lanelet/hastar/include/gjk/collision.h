#ifndef TF_COLLISION_H
#define TF_COLLISION_H

#include "epa.h" // gjk, shape, vector2d, iostream, math

// Uses GJK to determine if the shapes intersect
// Uses EPA to determine the penetration vector, if required
bool intersect(const Shape &shape1, const Shape &shape2, Vector2D *penetration = nullptr);

#endif
