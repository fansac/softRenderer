#include <Eigen\Dense>
#include <vector>
#include <unordered_map>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <direct.h>
// #include "mesh.h"


using namespace std;
using namespace Eigen;


class Triangle;

class Edge { // th i_th edge of triangle
public:
	Edge() = default;
	Edge(uint16_t index, uint16_t t_i) : t_index(t_i), i(index) {};
	uint16_t t_index;
	uint16_t i; // in {0, 1, 2}
};

class Vertex {
public:
	Vertex() = default;
	Vertex(double x, double y, double z) : position(x, y, z) {};

	uint16_t e_index = UINT16_MAX;

	double getX() {
		return this->position[0];
	}

	double getY() {
		return this->position[1];
	}

	double getZ() {
		return this->position[2];
	}

	void setX(double x) {
		this->position[0] = x;
	}

	void setY(double y) {
		this->position[1] = y;
	}

	void setZ(double z) {
		this->position[2] = z;
	}

	void setPosition(Vector3d new_position) {
		this->position = new_position;
	}

	Vector3d getPos() {
		return this->position;
	}

	void setNormal(Vector3d n) {
		this->normal = n;
	}

	Vector3d getNormal() {
		return this->normal;
	}
private:
	Vector3d position;
	Vector3d normal;
	Vector3d color;
};



class Triangle {
public:
	Triangle() = default;

	uint16_t v[3];
	uint16_t e_nbr[3];
	uint16_t n[3];
};

class TriangleMesh {
public:
	TriangleMesh() = default;

	uint16_t n_vertex = 0;
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
	unordered_map<pair<uint16_t, uint16_t>, uint16_t, pair_hash> e_t_map;

	void add_vertex(double x, double y, double z) {
		assert(this->vertices.size() < UINT16_MAX);
		this->vertices.emplace_back(x, y, z);
	}

	void add_normal(double x, double y, double z) {
		this->normals.emplace_back(x, y, z);
	}

	void add_triangle(vector<uint16_t> v_index, vector<uint16_t> vt, vector<uint16_t> vn) { // counterclockwise
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
			if (auto e_t_iter = this->e_t_map.find(make_pair(v_index[i], v_index[(i + 1) % 3])) == this->e_t_map.end()) {
				this->e_t_map[make_pair(v_index[(i + 1) % 3], v_index[i])] = this->edges.size() - 1;
			}
			else {
				auto neighbour_index = this->e_t_map[make_pair(v_index[i], v_index[(i + 1) % 3])];
				t_iter->e_nbr[i] = neighbour_index;
				auto edge_neighbour = this->edges[neighbour_index];
				this->triangles[edge_neighbour.t_index].e_nbr[edge_neighbour.i] = this->edges.size() - 1;
			}
		}
	}

	vector<uint16_t> find_triangles_index_of_vertex(uint16_t v_i) { // clockwise
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
};


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

int main(void) {
	cout << "main start!" << endl;
	char pwd[100];
	auto null_arg = _getcwd(pwd, 100);

	TriangleMesh mesh;
	system("pause");
	// string file_name = "cube.obj"; 
	string file_name = "cube.obj";
	ifstream f_in(file_name);
	if (!f_in) {
		cerr << "cannot open file " + file_name + "." << endl;
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
	assert(static_cast<size_t>(mesh.n_vertex) == mesh.vertices.size());
	system("pause");
	unsigned int i = 0;
	for (auto iter = mesh.vertices.begin(); iter != mesh.vertices.end(); ++iter) {
		cout << "vertex " << i++ << endl;
		cout << iter->getPos() << endl;
		cout << iter->getNormal() << endl;
	}
	cout << "mesh: " << endl;
	auto list = mesh.find_triangles_index_of_vertex(7);
	for (int i = 0; i < list.size(); ++i) {
		cout << list[i] << endl;
	}
	cout << "main end!" << endl;
	system("pause");
	return 0;
}