#ifndef RASTERIZER_MESH_H
#define RASTERIZER_MESH_H

#include<iostream>
#include<fstream>
#include <vector>
#include <unordered_map>
#include <Eigen\Dense>
#include "util.hpp"

using namespace std;
using namespace Eigen;


class Vertex {
public:
	uint16_t e_index = UINT16_MAX;

	Vertex();
	Vertex(double x, double y, double z);

	double getX();
	double getY();
	double getZ();
	void setX(double x);
	void setY(double y);
	void setZ(double z);
	void set_position(Vector3d new_position);
	void set_normal(Vector3d n);
	void set_color(Vector3d c);

	Vector3d get_position();
	Vector3d get_normal();
	Vector3d get_color();

private:
	Vector3d position;
	Vector3d normal;
	Vector3d color;
};


class Triangle {
public:
	uint16_t v[3];
	uint16_t e_nbr[3];
	uint16_t n[3];

	Triangle();
};

class Edge { // th i_th edge of triangle
public:
	Edge();
	Edge(uint16_t index, uint16_t t_i);
	uint16_t t_index;
	uint16_t i; // in {0, 1, 2}
};

class TriangleMesh {
public:
	uint16_t n_vertex = 0;
	string obj_name;
	vector<Vertex> vertices;
	vector<Edge> edges;
	vector<Triangle> triangles;
	vector<Vector3d> normals;

	struct pair_hash {
		size_t operator()  (const pair<uint16_t, uint16_t>& p) const
		{
			auto h1 = hash<uint16_t>{}(p.first);
			auto h2 = hash<uint16_t>{}(p.second);
			return static_cast<size_t>(h1) << 16 | h2;
		}
	};
	unordered_map<pair<uint16_t, uint16_t>, uint16_t, pair_hash> e_index_map;


	TriangleMesh();

	void add_vertex(double x, double y, double z);
	void add_normal(double x, double y, double z);
	void add_triangle(vector<uint16_t> v_index, vector<uint16_t> vt, vector<uint16_t> vn);
	vector<uint16_t> find_triangles_index_of_vertex(uint16_t v_i);
};


tuple<vector<uint16_t>, vector<uint16_t>, vector<uint16_t>> read_face_element_line(istringstream& in);


void read_mesh_from_obj_file(TriangleMesh& mesh, const string file_path);

#endif
