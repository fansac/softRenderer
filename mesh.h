#ifndef RASTERIZER_MESH_H
#define RASTERIZER_MESH_H

#include<iostream>
#include<fstream>
#include <vector>
#include <unordered_map>
#include <Eigen\Dense>
#include "util.hpp"


namespace mesh {
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
		void set_position(Eigen::Vector3d new_position);
		void set_normal(Eigen::Vector3d n);
		void set_color(Eigen::Vector3d c);

		Eigen::Vector3d get_position();
		Eigen::Vector3d get_normal();
		Eigen::Vector3d get_color();

	private:
		Eigen::Vector3d position;
		Eigen::Vector3d normal;
		Eigen::Vector3d color;
	};


	class Triangle {
	public:
		uint16_t v[3];
		uint16_t e_nbr[3];
		uint16_t n[3];
		Eigen::Vector3d normal;

		Triangle();
		void calculate_normal(Eigen::Vector3d point0, Eigen::Vector3d point1, Eigen::Vector3d point2);
	};

	class Edge { // th i_th edge of triangle
	public:
		uint16_t t_index;
		uint16_t i; // in {0, 1, 2}

		Edge();
		Edge(uint16_t index, uint16_t t_i);
	};

	class TriangleMesh {
	public:
		uint16_t n_vertex = 0;
		std::string obj_name;
		std::vector<Vertex> vertices;
		std::vector<Edge> edges;
		std::vector<Triangle> triangles;
		std::vector<Eigen::Vector3d> normals;
		std::vector<Eigen::Vector3d> colors;

		struct pair_hash {
			size_t operator()  (const std::pair<uint16_t, uint16_t>& p) const
			{
				auto h1 = std::hash<uint16_t>{}(p.first);
				auto h2 = std::hash<uint16_t>{}(p.second);
				return static_cast<size_t>(h1) << 16 | h2;
			}
		};
		std::unordered_map<std::pair<uint16_t, uint16_t>, uint16_t, pair_hash> e_index_map;


		TriangleMesh();

		void add_vertex(double x, double y, double z);
		void add_normal(double x, double y, double z);
		void add_color(double x, double y, double z);
		void add_triangle(std::vector<uint16_t> v_index, std::vector<uint16_t> vt, std::vector<uint16_t> vn);
		std::vector<uint16_t> find_triangles_index_of_vertex(uint16_t v_i);
		void calculate_average_normal_of_vertices();
	};


	std::tuple<std::vector<uint16_t>, std::vector<uint16_t>, std::vector<uint16_t>> read_face_element_line(std::istringstream& in);


	void read_mesh_from_obj_file(TriangleMesh& mesh, const std::string file_path);
}
#endif
