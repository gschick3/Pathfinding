#include "Map.h"

Map::Map(std::string adj, std::string coord) {
    // Fill adjacencies of cities
    auto adj_data = CSV::read(adj, ' ');

    for (int i = 0; i < adj_data.size(); i++) {
        City city1, city2;
        std::string city1Name = adj_data[i][0];
        std::string city2Name = adj_data[i][1];
        // Add first element as key and second as value
        if (!cities.count(city1Name))
            cities[city1Name] = city1;
        cities[city1Name].adj.push_back(city2Name);

        // Add second element as key and first as value
        if (!cities.count(city2Name))
            cities[city2Name] = city2;
        cities[city2Name].adj.push_back(city1Name);
    }

    // Fill coordinates of cities
    auto coord_data = CSV::read(coord);

    for (int i = 0; i < coord_data.size(); i++) {
        std::pair<double, double> coord = { std::stod(coord_data[i][1]), std::stod(coord_data[i][2]) };
        cities[coord_data[i][0]].coords = coord;
    }
}

std::vector<std::string> Map::bfs(std::string start, std::string end) const {
    if (!cities.count(start) || !cities.count(end))
        return {};

    std::unordered_map<std::string, std::string> visitedPaths;
    // Queue of not-yet-visited nodes
    std::queue<std::string> unvistitedQueue;

    // Add the start node to the queue and mark as visited
    unvistitedQueue.push(start);
    visitedPaths[start] = std::string();

    // Loop until all paths have been searched
    while (!unvistitedQueue.empty()) {
        // Choose next node to search
        std::string current = unvistitedQueue.front();
        unvistitedQueue.pop();

        if (current == end)
            return getPath(visitedPaths, end);

        // Discover all connected nodes
        for (const std::string& neighbor : cities.at(current).adj) {
            // Mark each node as visited and add to queue
            if (!visitedPaths.count(neighbor)) {
                visitedPaths[neighbor] = current; // Point node towards source
                unvistitedQueue.push(neighbor);
            }
        }
    }

    return {};
}

std::vector<std::string> Map::dfs(std::string start, std::string end) const {
    if (!cities.count(start) || !cities.count(end))
        return {};

    // All paths towards start node; nodes will be added as keys when they are visited
    std::unordered_map<std::string, std::string> visitedPaths;
    // Stack of not-yet-visited nodes
    std::stack<std::string> unvisitedStack;

    // Add the start node to the stack and mark as visited
    unvisitedStack.push(start);
    visitedPaths[start] = std::string();

    // Loop until all paths have been searched
    while (!unvisitedStack.empty()) {
        // Choose next node to search
        std::string current = unvisitedStack.top();
        unvisitedStack.pop();

        if (current == end)
            return getPath(visitedPaths, end);

        // Discover all connected nodes
        for (const std::string& neighbor : cities.at(current).adj) {
            // Mark each node as visited and add to stack
            if (!visitedPaths.count(neighbor)) {
                visitedPaths[neighbor] = current; // Point node towards source
                unvisitedStack.push(neighbor);
            }
        }
    }

    return {};
}

std::vector<std::string> Map::iddfs(std::string start, std::string end) const {
    if (!cities.count(start) || !cities.count(end))
        return {};

    int maxDepth = 0;

    // Loop until path is found or until integer limit is reached
    while (maxDepth < INT_MAX) {
        std::unordered_map<std::string, std::string> visitedPaths;
        visitedPaths[start] = "";

        std::vector<std::string> path = ddfs(start, end, maxDepth, visitedPaths);

        if (!path.empty())
            return path; // Shortest path found

        maxDepth++; // Increase depth limit
    }

    return {}; // No path found
}

std::vector<std::string> Map::ddfs(const std::string current, const std::string end,
    int depth, std::unordered_map<std::string, std::string> paths) const {
    if (current == end)
        return getPath(paths, end);

    if (depth <= 0)
        return {}; // Reached depth limit, no path found at this depth

    // Check out all adjacent unvisited nodes at the current node
    for (const std::string& neighbor : cities.at(current).adj) {
        if (!paths.count(neighbor)) {
            paths[neighbor] = current;
            std::vector<std::string> path = ddfs(neighbor, end, depth - 1, paths);
            if (!path.empty())
                return path; // Shortest path found in one of the child nodes
        }
    }

    return {}; // No path found at this depth
}

std::vector<std::string> Map::bestfs(std::string start, std::string end) const {
    std::unordered_map<std::string, std::string> paths;
    std::unordered_map<std::string, double> homeDistance;
    std::vector<std::string> path;
    min_priority_queue<CityQueueItem> open;

    CityQueueItem startItem(start, cities.at(start), getHeuristic(start, end));
    open.push(startItem);
    paths[start] = std::string();

    while (!open.empty()) {
        std::string current = open.top().name;
        open.pop();

        if (current == end)
            return getPath(paths, end);

        for (const std::string& neighbor : cities.at(current).adj) {
            if (!paths.count(neighbor)) {
                paths[neighbor] = current;
                homeDistance[neighbor] = homeDistance[current] + cities.at(neighbor).distanceTo(cities.at(current));
                open.push(CityQueueItem(neighbor, cities.at(neighbor), homeDistance[neighbor] + getHeuristic(neighbor ,end)));
            }
        }
    }

    return {};
}


std::vector<std::string> Map::astar(std::string start, std::string end) const {
    std::unordered_map<std::string, std::string> openPaths;
    std::unordered_map<std::string, std::string> closedPaths;
    std::unordered_map<std::string, double> distanceFromStart;
    std::vector<std::string> path;
    min_priority_queue<CityQueueItem> openPriority;

    // Add start node to open lists
    openPriority.push(CityQueueItem(start, cities.at(start), getHeuristic(start, end)));
    openPaths[start] = "";
    distanceFromStart[start] = 0;

    while (!openPriority.empty()) {
        // Pop the highest priority element off as the next node to search
        std::string current = openPriority.top().name;
        openPriority.pop();

        // This node has been visited by the shortest path, so it can be closed
        closedPaths[current] = openPaths[current];
        openPaths.erase(current);

        // Check if the node is the end node
        if (current == end)
            return getPath(closedPaths, end);

        for (const std::string& neighbor : cities.at(current).adj) {
            // If the path has already been visited, do nothing
            if (closedPaths.count(neighbor))
                continue;

            // Calculate the path distance from the start node
            double distStart = distanceFromStart[current] + cities.at(neighbor).distanceTo(cities.at(current));

            // If a more optimal path has already been discovered, do nothing
            if (openPaths.count(neighbor) &&  distanceFromStart[neighbor] <= distStart)
                continue;

            // Remove the less optimal path from the open queue
            if (openPaths.count(neighbor))
                pqRemoveCity(openPriority, neighbor);

            // Add the new node to the open lists
            openPaths[neighbor] = current;
            distanceFromStart[neighbor] = distStart;
            openPriority.push(CityQueueItem(neighbor, cities.at(neighbor), distStart + getHeuristic(neighbor, end)));
        }
    }
    return {};
}

double Map::getHeuristic(std::string currentCityName, std::string endCityName) const {
    return cities.at(currentCityName).distanceTo(cities.at(endCityName));
}

void Map::pqRemoveCity(min_priority_queue<CityQueueItem>& q, std::string cityName) const {
    min_priority_queue<CityQueueItem> temp;
    q.swap(temp);

    while (!temp.empty()) {
        if (temp.top().name != cityName)
            q.push(temp.top());
        temp.pop();
    }
}

std::vector<std::string> Map::getPath(std::unordered_map<std::string, std::string> paths, std::string end) const {
    std::string current = end;
    std::vector<std::string> path;

    while (!current.empty()) {
        path.insert(path.begin(), current);
        current = paths[current];
    }

    return path;
}

bool Map::contains(std::string cityName) const {
    return cities.count(cityName) > 0;
}

City Map::getCity(std::string cityName) const {
    if (!cities.count(cityName))
        return City();

    return cities.at(cityName);
}