#ifndef TF_GJK_H
#define TF_GJK_H

#include "shape.h" // vector2d, iostream, math

struct Simplex {  //定义结构体单纯形
	Vector2D elements[3];
	int index;
};

Vector2D support(const Shape &shape1, const Shape &shape2, const Vector2D &direction);
bool gjk_intersect(const Shape &shape1, const Shape &shape2, Simplex *simplex_to_fill = nullptr);  //intersect相交，nullptr第一次引用声明为空指针

#endif
