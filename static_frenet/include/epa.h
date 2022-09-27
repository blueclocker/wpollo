#ifndef TF_EPA_H
#define TF_EPA_H

#include "gjk.h" // shape, vector2d, iostream, math

// Singly linked list
class Polytope {  //定义类多面体
public:
	class Node {
	public:
		Vector2D point;
		Node *next;

		Node();
		Node(const Vector2D &point, Node *next);
	};

	// Result for the closest_edge_to_origin function
	struct Edge {
		Vector2D vertex; // Starting vertex for the line, a for ab  //vertex顶点
		Vector2D penetration; // Vector from Origin to ab, correct length  //penetration穿透
	};

public:
	int num_elements;  //元素
	Node *start_node;

public:
	Polytope(); //构造函数
	Polytope(const Simplex &simplex);
	~Polytope();  //析构函数

	Edge closest_edge_to_origin();
	void add_after(const Vector2D &new_point, const Vector2D &existing_point);
};

Vector2D epa_penetration(const Shape &shape1, const Shape &shape2, const Simplex &start_simplex);

#endif
