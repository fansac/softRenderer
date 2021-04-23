#include "mesh.h"

// class Vertex
mesh::Vertex::Vertex() = default;
mesh::Vertex::Vertex(double x, double y, double z) : position(x, y, z) {};

double mesh::Vertex::getX() { return this->position[0]; }
double mesh::Vertex::getY() { return this->position[1]; }
double mesh::Vertex::getZ() { return this->position[2]; }
Eigen::Vector3d mesh::Vertex::get_position() { return this->position; }
Eigen::Vector3d mesh::Vertex::get_normal() { return this->normal; }
Eigen::Vector3d mesh::Vertex::get_color() { return this->color;  }
Eigen::Vector2d mesh::Vertex::get_tecoord() { return this->texcoord;  }
Eigen::Vector3d mesh::Vertex::get_tangent() { return this->tangent;  }

void mesh::Vertex::setX(double x) { this->position[0] = x; }
void mesh::Vertex::setY(double y) { this->position[1] = y; }
void mesh::Vertex::setZ(double z) { this->position[2] = z; }
void mesh::Vertex::set_position(Eigen::Vector3d new_position) { this->position = new_position; }
void mesh::Vertex::set_normal(Eigen::Vector3d n) { this->normal = n; }
void mesh::Vertex::set_color(Eigen::Vector3d c) { this->color = c;  }
void mesh::Vertex::set_texcoord(Eigen::Vector2d texcorrd) { this->texcoord = texcoord;  }
void mesh::Vertex::set_tangent(Eigen::Vector3d tangent) { this->tangent = tangent;  }
// class Triangle
mesh::Triangle::Triangle() {
	memset(this->v, UINT16_MAX, 3 * sizeof(uint16_t));
	memset(this->e_nbr, UINT16_MAX, 3 * sizeof(uint16_t));
	memset(this->n, UINT16_MAX, 3 * sizeof(uint16_t));
};


// class Edge
mesh::Edge::Edge() : t_index(UINT16_MAX), i(UINT16_MAX) {};
mesh::Edge::Edge(uint16_t index, uint16_t t_i) : t_index(t_i), i(index) {};


// class TriangleMesh
mesh::TriangleMesh::TriangleMesh() = default;

void mesh::TriangleMesh::add_vertex(double x, double y, double z) {
	assert(this->vertices.size() < UINT16_MAX);
	this->vertices.emplace_back(x, y, z);
}

void mesh::TriangleMesh::add_normal(double x, double y, double z) { this->normals.emplace_back(x, y, z); }

void mesh::TriangleMesh::add_texcoord(double u, double v) { this->texcoords.emplace_back(u, v); }

void mesh::TriangleMesh::add_triangle(std::vector<uint16_t> v_index, std::vector<uint16_t> vt, std::vector<uint16_t> vn) {
	// counterclockwise
	for (unsigned int i = 0; i < 3; ++i) assert(v_index[i] < this->vertices.size());
	this->triangles.emplace_back();
	auto t_iter = this->triangles.end() - 1;

	for (uint16_t i = 0; i < 3; ++i) {
		t_iter->v[i] = v_index[i];
		t_iter->n[i] = vn[i];
		t_iter->t[i] = vt[i];
		this->edges.emplace_back(i, this->triangles.size() - 1);

		auto v_iter = this->vertices.begin() + v_index[i];
		if (v_iter->e_index == UINT16_MAX) {
			v_iter->e_index = this->edges.size() - 1;
			++ this->n_vertex;
		}
		if (auto e_t_iter = this->e_index_map.find(std::make_pair(v_index[i], v_index[(i + 1) % 3])) == this->e_index_map.end()) {
			this->e_index_map[std::make_pair(v_index[(i + 1) % 3], v_index[i])] = this->edges.size() - 1;
		}
		else {
			auto neighbour_index = this->e_index_map[std::make_pair(v_index[i], v_index[(i + 1) % 3])];
			t_iter->e_nbr[i] = neighbour_index;
			auto edge_neighbour = this->edges[neighbour_index];
			this->triangles[edge_neighbour.t_index].e_nbr[edge_neighbour.i] = this->edges.size() - 1;
		}
	}
	t_iter->calculate_normal(this->vertices[t_iter->v[0]].get_position(), this->vertices[t_iter->v[1]].get_position(), this->vertices[t_iter->v[2]].get_position());
}

std::vector<uint16_t> mesh::TriangleMesh::find_triangles_index_of_vertex(uint16_t v_i) { // clockwise
	std::vector<uint16_t> triangles_index;
	auto e = this->edges[this->vertices[v_i].e_index];
	auto t_i = e.t_index;
	auto i = e.i;
	auto t_start = t_i;
	do {
		triangles_index.push_back(t_i);
		auto& tri = this->triangles[t_i];
		//if (tri.e_nbr[i] == UINT16_MAX) {
		//	break;
		//}
		e = this->edges[tri.e_nbr[i]];
		t_i = e.t_index;
		i = e.i;
		i = (i + 1) % 3;
	} while (t_i != t_start);
	return triangles_index;
}



std::tuple<std::vector<uint16_t>, std::vector<uint16_t>, std::vector<uint16_t>> mesh::read_face_element_line(std::istringstream& in) {
	uint16_t a[9] = { 0 };
	std::vector<std::string> ss;
	std::string str;
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
	std::vector<uint16_t> v(3, 0), vt(3, 0), vn(3, 0);
	for (unsigned int i = 0; i < 3; ++i) {
		v[i]  = a[i * 3] - 1;
		vt[i] = a[i * 3 + 1] - 1;
		vn[i] = a[i * 3 + 2] - 1;
	}
	return std::make_tuple(v, vt, vn);
}


void mesh::read_mesh_from_obj_file(TriangleMesh &mesh, const std::string file_path) {
	mesh.obj_name = util_rd::find_file_name(file_path);
	std::ifstream f_in(file_path);
	if (!f_in) {
		std::cerr << "cannot open file " + file_path + "." << std::endl;
		exit(1);
	}
	std::cout << "open file done!" << std::endl;
	std::string str_buffer;
	std::string head;
	while (std::getline(f_in, str_buffer)) {
		head.clear();
		std::cout << str_buffer << std::endl;
		std::istringstream s_in(str_buffer);
		s_in >> head;
		if (head == "v") {
			double x, y, z;
			s_in >> x >> y >> z;
			mesh.add_vertex(x , y, z);
		}
		if (head == "vn") {
			double x, y, z;
			s_in >> x >> y >> z;
			mesh.add_normal(x, y, z);
		}
		if (head == "vt") {
			double u, v;;
			s_in >> u >> v;
			mesh.add_texcoord(u, v);
		}
		else if (head == "f") {
			auto v_info = read_face_element_line(s_in);
			auto v = std::get<0>(v_info);
			auto vt = std::get<1>(v_info);
			auto vn = std::get<2>(v_info);
			mesh.add_triangle(v, vt, vn);
		}
	}
	
	f_in.close();
	mesh.e_index_map.clear();
	assert(static_cast<size_t>(mesh.n_vertex) == mesh.vertices.size());

	// cannot be used at current
	//mesh.calculate_average_normal_of_vertices();
	mesh.calculate_AABB_of_scence();
}


void mesh::Triangle::calculate_normal(Eigen::Vector3d point0, Eigen::Vector3d point1, Eigen::Vector3d point2) {
	this->normal = (point1 - point0).cross(point2 - point1).normalized();
}


void mesh::TriangleMesh::calculate_average_normal_of_vertices() {
	for (uint16_t i = 0; i < this->n_vertex; ++i) {
		auto triangles_index_list = this->find_triangles_index_of_vertex(i);
		Eigen::Vector3d average = { 0, 0, 0 };
		for (auto index : triangles_index_list) {
			average += this->triangles[index].normal;
		}
		this->vertices[i].set_normal(average.normalized());
	}
}

void mesh::TriangleMesh::calculate_AABB_of_scence() {
	if (n_vertex > 0) {
		x_max = x_min = vertices[0].getX();
		y_max = y_min = vertices[0].getY();
		z_max = z_min = vertices[0].getZ();
	}
	for (auto iter : vertices) {
		auto pos = iter.get_position();
		util_rd::update_min_max(iter.getX(), x_min, x_max);
		util_rd::update_min_max(iter.getY(), y_min, y_max);
		util_rd::update_min_max(iter.getZ(), z_min, z_max);
	}
}