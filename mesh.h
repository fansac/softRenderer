#ifndef MESH_H
#define MESH_H

class Triangle;
class Edge { // th i_th edge of triangle
public:
	Triangle t;
	int32_t i; // in {0, 1, 2}
};

class Vertex {
public:
	vector<Edge> e;

	double getX();

	double getY();

	double getZ();

	void setPosition(Vector3d new_position);

	void setX(double x);

	void setY(double y);

	void setZ(double z);

private:
	Vector3d position;
	Vector3d normal;
	Vector3d color;
};



class Triangle {
public:
	Vertex v[3];
	Edge nbr[3];
};

class TriangleMesh {
public:
	uint32_t n_trianlge;
	uint32_t n_vertex;
	uint32_t n_edge;
	vector<vector<int32_t>> triangle_index;
	vector<int32_t[3]> triangle_neighbour;
	vector<int32_t> adjacent_trangle_of_vertex;
	vector<Vertex> vertices;
	vector<Edge> edges;
	vector<Triangle> trangles;

	TriangleMesh();
};


#endif
