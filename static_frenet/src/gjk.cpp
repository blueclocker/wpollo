#include "gjk.h"

bool same_direction(const Vector2D &v1, const Vector2D &v2) {
	return v1.dot(v2) > 0;  //判断二者是否同向
}

void do_simplex_line(Simplex &simplex, Vector2D &direction) {
	Vector2D a = simplex.elements[1]; // a is the newest point in the simplex
	Vector2D b = simplex.elements[0];
	Vector2D ab = b - a;
	Vector2D aO = -a;

	// If ab points in the direction of the origin, then ab is closest to the origin
	if (same_direction(ab, aO)) {
		// The new search direction is perpendicular to ab towards the origin, Simplex := [b, a]
		direction = perpendicular_towards(a, b, Vector2D(0, 0));
	}
	else {
		// a is closest to the origin
		// Simplex := [a]
		direction = aO;
		simplex.elements[0] = a;
		simplex.index = 1;
	}
}

bool do_simplex_triangle(Simplex &simplex, Vector2D &direction) {
	Vector2D a = simplex.elements[2];
	Vector2D b = simplex.elements[1];
	Vector2D c = simplex.elements[0];
	Vector2D ab = b - a;
	Vector2D ac = c - a;
	Vector2D aO = -a;

	Vector2D perp_ac = perpendicular_away(a, c, b);  //垂直于向量ac且远离b的向量
	Vector2D perp_ab = perpendicular_away(a, b, c);

	// If perpendicular-to-ac points to the origin ...
	if (same_direction(perp_ac, aO)) {
		// ... and ac points to the origin, then:
		if (same_direction(ac, aO)) {
			// Simplex := [c, a]
			simplex.elements[0] = c;
			simplex.elements[1] = a;
			simplex.index = 2;
			direction = perp_ac;
		}
		else {
			// Simplex := [a]
			simplex.elements[0] = a;
			simplex.index = 1;
			direction = aO;
		}
	}
	else if (same_direction(perp_ab, aO)) {
		if (same_direction(ab, aO)) {
			// Simplex := [b, a]
			simplex.elements[0] = b;
			simplex.elements[1] = a;
			simplex.index = 2;
			direction = perp_ab;
		}
		else {
			// Simplex := [a]
			simplex.elements[0] = a;
			simplex.index = 1;
			direction = aO;
		}
	}
	else {
		// If neither perp_ac nor perp_ab point towards the origin, it must be enclosed by the simplex
		return true;  //判断出单纯形中包含原点
	}

	return false;
}

// Finds out what part of the simplex is closest to the origin, and modifies it accordingly
// Returns true if an intersection was found
bool do_simplex(Simplex &simplex, Vector2D &direction) {
	if (simplex.index == 2) {
		do_simplex_line(simplex, direction);
		return false;
	}
	else {
		return do_simplex_triangle(simplex, direction);
	}
}

Vector2D support(const Shape &shape1, const Shape &shape2, const Vector2D &direction) {
	return shape1.support(direction) - shape2.support(-direction);
}  //在该方向上最远距离

bool gjk_intersect(const Shape &shape1, const Shape &shape2, Simplex *simplex_to_fill) {
	// Simplex is a list of points that should enclose the origin
	Simplex simplex;
	simplex.index = 0;

	// Pick a random starting direction to get a starting point on the minkowski difference
	Vector2D direction(1, 0);
	Vector2D start = support(shape1, shape2, direction);
	simplex.elements[simplex.index++] = start;  //赋予一个元素

	// Continue searching in the opposite direction
	direction = -start;

	do {
		Vector2D a = support(shape1, shape2, direction);

		// If the dot product is negative, a did not pass the origin, so there is no intersection
		if (a.dot(direction) < 0)
			return false;  //不可能相交
		
		simplex.elements[simplex.index++] = a;  //第一次循环添加第二个元素
	} while (!do_simplex(simplex, direction));

	// If requested, return the resulting simplex as well
	if (simplex_to_fill) {
		simplex_to_fill->index = simplex.index;
		for (int i = 0; i < simplex.index; ++i) {
			simplex_to_fill->elements[i] = simplex.elements[i];
		}
	}
	
	return true;
}
