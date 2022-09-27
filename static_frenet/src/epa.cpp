#include "epa.h"

Polytope::Node::Node(): point(Vector2D()), next(nullptr) {}  //多面体

Polytope::Node::Node(const Vector2D &point, Node *next): point(point), next(next) {}

Polytope::Polytope(): num_elements(0), start_node(nullptr) {}

Polytope::Polytope(const Simplex &simplex): num_elements(simplex.index) {
	Node *current_start = nullptr;
	for (int i = num_elements - 1; i >= 0; --i) {
		Node *new_node = new Node(simplex.elements[i], current_start);
		current_start = new_node;
	}
	start_node = current_start;
}

Polytope::~Polytope() {
	Node *it = start_node;
	while (it) {
		Node *to_delete = it;
		it = it->next;
		delete to_delete;
	}
}

Polytope::Edge Polytope::closest_edge_to_origin() {
	Edge result;

	float shortest_distance = INT32_MAX;
	Node *it = start_node;
	while (it) {
		// Get the line
		Vector2D a = it->point;
		Vector2D b = (it->next) ? it->next->point : start_node->point;
		// Get the normal away from the origin
		Vector2D ab_normal = perpendicular_away(a, b, Vector2D(0, 0));
		// The projection of a onto the normal is the distance vector from the origin to ab
		Vector2D projected = a.projection_on(ab_normal);
		float distance = projected.length();

		if (distance < shortest_distance) {
			shortest_distance = distance;

			result.vertex = a;
			result.penetration = projected;
		}

		it = it->next;
	}

	return result;
}

void Polytope::add_after(const Vector2D &new_point, const Vector2D &existing_point) {
	Node *it = start_node;
	while (it) {
		if (it->point == existing_point) {
			Node *new_node = new Node(new_point, it->next);
			it->next = new_node;
			break;
		}

		it = it->next;
	}
}



Vector2D epa_penetration(const Shape &shape1, const Shape &shape2, const Simplex &gjk_result) {
	Polytope polytope(gjk_result);

	while (true) {
		Polytope::Edge edge = polytope.closest_edge_to_origin();
		// Origin on the edge?
		if (edge.penetration.is_null()) {
			return edge.penetration;
		}

		// If the support vector is just as long as the penetration, the line is on the edge of the minkowski difference
		Vector2D support_vector = support(shape1, shape2, edge.penetration);
		float support_distance = support_vector.projection_on(edge.penetration).length();
		if (fabsf(support_distance - edge.penetration.length()) < 0.001f) {
			return edge.penetration;
		}

		// Otherwise, add the support vector and continue the loop
		polytope.add_after(support_vector, edge.vertex);
	}
}
