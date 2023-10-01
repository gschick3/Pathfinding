#pragma once
#include <string>
#include <unordered_map>
#include <queue>
#include <stack>
#include <unordered_set>
#include "City.h"
#include "CSV.h"

class Map
{
public:
	Map(std::string adj, std::string coord);

	// Breadth-first search
	// Accepts hashmap of city names to City structs, a starting city, and an ending city
	// Returns the shortest path from start to end as an ordered vector of cities
	std::vector<std::string> bfs(std::string start, std::string end) const;

	// Depth-first search
	std::vector<std::string> dfs(std::string start, std::string end) const;

	// Iterative Deepening Depth-first search
	// This algorithm was created with the help of ChatGPT
	std::vector<std::string> iddfs(std::string start, std::string end) const;

	// Best-first search
	std::vector<std::string> bestfs(std::string start, std::string end) const;

	// A* search
	std::vector<std::string> astar(std::string start, std::string end) const;

	bool contains(std::string name) const;

	City getCity(std::string name) const;

private:
	// Define type for minimal priority queue
	template<typename T>
	using min_priority_queue = std::priority_queue<T, std::vector<T>, std::greater<T>>;
	
	std::unordered_map<std::string, City> cities;

	double getHeuristic(std::string currentCityName, std::string endCityName) const;
	void pqRemoveCity(min_priority_queue<CityQueueItem>& q, std::string cityName) const;
	std::vector<std::string> getPath(std::unordered_map<std::string, std::string> paths, std::string end) const;

	// IDDFS helper function
	std::vector<std::string> ddfs(const std::string current, const std::string end, int depth, std::unordered_map<std::string, std::string> visited) const;
};

