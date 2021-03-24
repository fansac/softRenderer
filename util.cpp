#include "util.h"

#include <string>
#include <fstream>

bool isFileExists_ifstream(const std::string name) {
	std::ifstream f(name.c_str());
	return f.good();
}