//#include <Eigen\Dense>
//#include <vector>
//#include <unordered_map>
//#include <string>
//#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <direct.h>
#include "mesh.h"


using namespace std;
using namespace Eigen;

// class Vertex
Vertex::Vertex() = default;
Vertex::Vertex(double x, double y, double z) : position(x, y, z) {};

double Vertex::getX() { return this->position[0]; }
double Vertex::getY() { return this->position[1]; }
double Vertex::getZ() { return this->position[2]; }
Vector3d Vertex::getPos() { return this->position; }
Vector3d Vertex::getNormal() { return this->normal; }

void Vertex::setX(double x) { this->position[0] = x; }
void Vertex::setY(double y) { this->position[1] = y; }
void Vertex::setZ(double z) { this->position[2] = z; }
void Vertex::setPosition(Vector3d new_position) { this->position = new_position; }
void Vertex::setNormal(Vector3d n) { this->normal = n; }


// class Triangle
Triangle::Triangle() = default;


// class Edge
Edge::Edge() = default;
Edge::Edge(uint16_t index, uint16_t t_i) : t_index(t_i), i(index) {};


// class TriangleMesh
TriangleMesh::TriangleMesh() = default;
	
void TriangleMesh::add_vertex(double x, double y, double z) {
	assert(this->vertices.size() < UINT16_MAX);
	this->vertices.emplace_back(x, y, z);
}

void TriangleMesh::add_normal(double x, double y, double z) { this->normals.emplace_back(x, y, z); }

void TriangleMesh::add_triangle(vector<uint16_t> v_index, vector<uint16_t> vt, vector<uint16_t> vn) { // counterclockwise
	for (unsigned int i = 0; i < 3; ++i) assert(v_index[i] < this->vertices.size());
	this->triangles.emplace_back();
	auto t_iter = this->triangles.end() - 1;

	for (uint16_t i = 0; i < 3; ++i) {
		t_iter->v[i] = v_index[i];
		t_iter->n[i] = vn[i];
		this->edges.emplace_back(i, this->triangles.size() - 1);
		//t_iter->e[i] = this->edges.size() - 1;
		auto v_iter = this->vertices.begin() + v_index[i];
		if (v_iter->e_index == UINT16_MAX) {
			v_iter->e_index = this->edges.size() - 1;
			++ this->n_vertex;
		}
		if (auto e_t_iter = this->e_index_map.find(make_pair(v_index[i], v_index[(i + 1) % 3])) == this->e_index_map.end()) {
			this->e_index_map[make_pair(v_index[(i + 1) % 3], v_index[i])] = this->edges.size() - 1;
		}
		else {
			auto neighbour_index = this->e_index_map[make_pair(v_index[i], v_index[(i + 1) % 3])];
			t_iter->e_nbr[i] = neighbour_index;
			auto edge_neighbour = this->edges[neighbour_index];
			this->triangles[edge_neighbour.t_index].e_nbr[edge_neighbour.i] = this->edges.size() - 1;
		}
	}
}

vector<uint16_t> TriangleMesh::find_triangles_index_of_vertex(uint16_t v_i) { // clockwise
	vector<uint16_t> triangles_index;
	auto e = this->edges[this->vertices[v_i].e_index];
	auto t_i = e.t_index;
	auto i = e.i;
	auto t_start = t_i;
	do {
		triangles_index.push_back(t_i);
		e = this->edges[this->triangles[t_i].e_nbr[i]];
		t_i = e.t_index;
		i = e.i;
		i = (i + 1) % 3; 
	} while (t_i != t_start);
	return triangles_index;
}



tuple<vector<uint16_t>, vector<uint16_t>, vector<uint16_t>> read_face_element_line(istringstream& in) {
	uint16_t a[9] = { 0 };
	vector<string> ss;
	string str;
	while (in){
		str.clear();
		in >> str;
		if (!str.empty()) ss.push_back(str);
	}
	assert(ss.size() == 3);
	for (unsigned int i = 0; i < 3; ++i) {
		unsigned int index = 0;
		for (uint16_t j = 0; j < 3; ++j) {
			uint16_t n = 0;
			while (index < ss[i].size()) {
				char c = ss[i][index++];
				if (c >= '0' && c <= '9') n = n * 10 + (c - '0');
				else break;
			}
			a[i * 3 + j] = n;
		}
	}
	vector<uint16_t> v(3, 0), vt(3, 0), vn(3, 0);
	for (unsigned int i = 0; i < 3; ++i) {
		v[i]  = a[i * 3] - 1;
		vt[i] = a[i * 3 + 1] - 1;
		vn[i] = a[i * 3 + 2] - 1;
	}
	return make_tuple(v, vt, vn);
}


void read_mesh_from_obj_file(TriangleMesh &mesh, const string file_path) {
	ifstream f_in(file_path);
	if (!f_in) {
		cerr << "cannot open file " + file_path + "." << endl;
		exit(1);
	}
	cout << "open file done!" << endl;
	string str_buffer;
	string head;
	while (getline(f_in, str_buffer)) {
		head.clear();
		cout << str_buffer << endl;
		istringstream s_in(str_buffer);
		s_in >> head;
		if (head == "v") {
			double x, y, z;
			s_in >> x >> y >> z;
			mesh.add_vertex(x, y, z);
		}
		if (head == "vn") {
			double x, y, z;
			s_in >> x >> y >> z;
			mesh.add_normal(x, y, z);
		}
		else if (head == "f") {
			auto v_info = read_face_element_line(s_in);
			auto v = get<0>(v_info);
			auto vt = get<1>(v_info);
			auto vn = get<2>(v_info);
			mesh.add_triangle(v, vt, vn);
		}
	}
	f_in.close();
	mesh.e_index_map.clear();
	assert(static_cast<size_t>(mesh.n_vertex) == mesh.vertices.size());
	system("pause");

	cout << "mesh: " << endl;
	unsigned int i = 0;
	for (auto iter = mesh.vertices.begin(); iter != mesh.vertices.end(); ++iter) {
		cout << "vertex " << i++ << endl;
		cout << iter->getPos() << endl;
		cout << iter->getNormal() << endl;
	}
}
