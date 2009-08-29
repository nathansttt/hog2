#include "StringUtils.h"

std::string int_to_string(int i) {
	std::stringstream s;
	s << i;
	return s.str();
}

std::string double_to_string(double i) {
	std::stringstream s;
	s << i;
	return s.str();
}

std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	std::stringstream ss(s);
	std::string item;
	while(std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}
