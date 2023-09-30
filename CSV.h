#pragma once
#include <string>
#include <vector>

// Contains utility functions for CSV files
class CSV
{
public:
	// Reads a CSV as a vector of string vectors
	static std::vector<std::vector<std::string>> read(std::string path, char delimiter = ',');
};

