#include "util.hpp"

namespace util_rd {
	bool is_file_exists_ifstream(const std::string name) {
		std::ifstream f(name.c_str());
		return f.good();
	}

	std::string find_file_name(const std::string file_path) {
		size_t start = file_path.find_last_of('/') + 1;
		size_t end = file_path.find_last_of('.');
		return file_path.substr(start, end);
	}
}