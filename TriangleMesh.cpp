#include <Eigen\Dense>
#include <vector>
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
	vector<Triangle>::iterator t;
	uint32_t i = 0; // in {0, 1, 2}

};

class Vertex {
public:
	Vertex() = default;
	Vertex(double x, double y, double z) : position(x, y, z), e(nullptr) {};

	vector<Edge>::iterator e;

	double getX() {
		return this->position[0];
	}

	double getY() {
		return this->position[1];
	}

	double getZ() {
		return this->position[2];
	}

	void setPosition(Vector3d new_position) {
		this->position = new_position;
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

	Vector3d getPos() {
		return this->position;
	}

private:
	Vector3d position;
	Vector3d normal;
	Vector3d color;
};



class Triangle {
public:
	Triangle() = default;

	vector<Vertex>::iterator v[3];
	vector<Edge>::iterator e[3];
};

class TriangleMesh {
public:
	TriangleMesh() = default;

	vector<uint32_t[3]> triangle_index;
	vector<uint32_t[3]> triangle_neighbour;
	vector<uint32_t> adjacent_trangle_of_vertex;
	vector<Vertex> vertices;
	vector<Edge> edges;
	vector<Triangle> triangle;

	void add_new_vertex(double x, double y, double z) {
		assert(this->vertices.size() <= UINT32_MAX);
		this->vertices.push_back(Vertex(x, y, z));
	}

	void add_new_triangle(vector<uint32_t> v_index) { // counterclockwise
		uint32_t n_vertices = this->vertices.size();
		for (int i = 0; i < 3; ++i) assert(v_index[i] < n_vertices);
		for (int i = 0; i < 3; ++i) {
			Triangle t = Triangle();
			t.v[i] = this->vertices.begin()+v_index[i];
			
			this->triangle.push_back(Triangle());
		}

	}

	//void get_counterclockwise_order(uint32_t a, uint32_t b, uint32_t c) {
	//	Vector3d a_pos = this->vertices[a].getPos(), b_pos = this->vertices[b].getPos(),
	//		c_pos = this->vertices[c].getPos();
	//	Vector3d cross_vector =  
	//}
};



int main(void) {
	cout << "main start!" << endl;
	char pwd[100];
	_getcwd(pwd, 100);

	TriangleMesh mesh;
	system("pause");
	string file_name = "diamond.obj";
	ifstream f_in(file_name);
	if (!f_in) {
		cerr << "cannot open file " + file_name + "." << endl;
		exit(1);
	}
	cout << "open file done!" << endl;
	string str_buffer;
	string head;
	while (getline(f_in, str_buffer)) {
		istringstream s_in(str_buffer);
		s_in >> head;
		if (head == "v") {
			double x, y, z;
			s_in >> x >> y >> z;
			mesh.add_new_vertex(x, y, z);
			cout << " x " << x << " y " << y << " z " << z << endl;
		}
		else if (head == "f") {
			uint32_t x, y, z;
			s_in >> x >> y >> z;
			mesh.add_new_triangle({x , y, z});
		}
	}
	//cout << "mesh: " << endl << mesh.n_trianlge << endl;
	//for (auto iter = mesh.vertices.begin(); iter != mesh.vertices.end(); ++iter) {
	//	cout << "position: " << endl << iter->getPos() << endl;
	//}
	cout << "main end!" << endl;
	system("pause");
	return 0;
}