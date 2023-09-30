#pragma once
#include <string>
#include <unordered_map>
#include <iostream>
#include <queue>
#include <stack>
#include <unordered_set>
#include <chrono>
#include "City.h"
#include "CSV.h"

class Map
{
public:
	Map(std::string adj, std::string coord);

	// Breadth-first search
	// Accepts hashmap of city names to City structs, a starting city, and an ending city
	// Returns the shortest path from start to end as an ordered vector of cities
	std::vector<std::string> bfs(std::string start, std::string end);

	// Depth-first search
	std::vector<std::string> dfs(std::string start, std::string end);

	// Iterative Deepening Depth-first search
	// This algorithm was created with the help of ChatGPT when asked revise the commented-out IDDFS search in Map.cpp
	std::vector<std::string> iddfs(std::string start, std::string end);

	// Best-first search
	std::vector<std::string> bestfs(std::string start, std::string end);

	// A* search
	std::vector<std::string> astar(std::string start, std::string end);

	bool contains(std::string name);

	City getCity(std::string name);

private:
	std::vector<std::string> ddfs(const std::string current, const std::string end, int depth, std::unordered_map<std::string, std::string> visited);
	std::vector<std::string> getPath(std::unordered_map<std::string, std::string> paths, std::string end);
	std::unordered_map<std::string, City> cities;
};

