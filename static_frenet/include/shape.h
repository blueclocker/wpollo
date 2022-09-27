#ifndef TF_SHAPE_H
#define TF_SHAPE_H

#include "vector2d.h" // iostream, math
#include<vector>

class Shape {
public:
	virtual ~Shape() = 0;  //定义虚析构函数
	virtual Vector2D support(const Vector2D &direction) const = 0;  //纯虚函数
};

class Circle : public Shape {
public:
	Vector2D center;
	float radius;

public:
	Circle();
	Circle(const Vector2D &center, float radius);
	Circle(const Circle &other);
	virtual ~Circle();

	Vector2D support(const Vector2D &direction) const override;  //override覆盖
};

class Polygon : public Shape {  //定义类多边形Polygon
public:
	int num_vertices;
	Vector2D *vertices;
	// float velocity_x;
	// float velocity_y;

public:
	Polygon();
	Polygon(int num_vertices, const Vector2D *vertices);
	// Polygon(int num_vertices, const Vector2D *vertices,float velocity_x,float velocity_y);
	Polygon(const Polygon &other);
	// Polygon(const Polygon &&other);
	virtual ~Polygon();

	Vector2D support(const Vector2D &direction) const override;
};


using Polygon_list = std::vector<Polygon>;

#endif
