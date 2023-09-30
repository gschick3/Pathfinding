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

std::vector<std::string> Map::bfs(std::string start, std::string end) {
    if (!cities.count(start) || !cities.count(end))
        return {};

    // All paths towards start node; nodes will be added as keys when they are visited
    std::unordered_map<std::string, std::string> paths;
    // Queue of not-yet-visited nodes
    std::queue<std::string> q;

    // Add the start node to the queue and mark as visited
    q.push(start);
    paths[start] = std::string();

    std::string next;

    // Loop until all paths have been searched
    while (!q.empty()) {
        // Choose next node to search
        next = q.front();
        q.pop();

        if (next == end)
            return getPath(paths, end);

        // Discover all connected nodes
        for (std::string s : cities[next].adj) {
            // Mark each node as visited and add to queue
            if (!paths.count(s)) {
                paths[s] = next; // Point node towards source
                q.push(s);
            }
        }
    }

    return {};
}

std::vector<std::string> Map::dfs(std::string start, std::string end) {
    if (!cities.count(start) || !cities.count(end))
        return {};

    // All paths towards start node; nodes will be added as keys when they are visited
    std::unordered_map<std::string, std::string> paths;
    // Stack of not-yet-visited nodes
    std::stack<std::string> stack;

    // Add the start node to the stack and mark as visited
    stack.push(start);
    paths[start] = std::string();

    std::string next;

    // Loop until all paths have been searched
    while (!stack.empty()) {
        // Choose next node to search
        next = stack.top();
        stack.pop();

        if (next == end)
            return getPath(paths, end);

        // Discover all connected nodes
        for (std::string s : cities[next].adj) {
            // Mark each node as visited and add to stack
            if (!paths.count(s)) {
                paths[s] = next; // Point node towards source
                stack.push(s);
            }
        }
    }

    return {};
}

std::vector<std::string> Map::iddfs(std::string start, std::string end) {
    if (!cities.count(start) || !cities.count(end))
        return {};

    int maxDepth = 0;
    while (true) {
        std::unordered_map<std::string, std::string> paths;
        paths[start] = "";

        std::vector<std::string> path = ddfs(start, end, maxDepth, paths);

        if (!path.empty()) {
            return path; // Shortest path found
        }
        maxDepth++; // Increase depth limit
    }

    return {}; // No path found
}

std::vector<std::string> Map::ddfs(const std::string current, const std::string end,
    int depth, std::unordered_map<std::string, std::string> paths) {
    if (current == end) {
        for (auto pair : paths) {
            std::cout << pair.second << "->" << pair.first << std::endl;
        }
        // Found the destination city, reconstruct and return the path
        std::vector<std::string> result;
        result.push_back(end);
        std::string node = end;
        while (paths.count(paths[node])) {
            node = paths[node];
            result.push_back(node);
        }
        std::reverse(result.begin(), result.end());
        return result;
    }

    if (depth <= 0)
        return {}; // Reached depth limit, no path found at this depth

    // Check out all adjacent unvisited nodes at the current node
    for (const std::string neighbor : cities[current].adj) {
        if (!paths.count(neighbor)) {
            paths[neighbor] = current;
            std::vector<std::string> path = ddfs(neighbor, end, depth - 1, paths);
            if (!path.empty())
                return path; // Shortest path found in one of the child nodes
        }
    }

    paths.erase(current);
    return {}; // No path found at this depth
}

std::vector<std::string> Map::bestfs(std::string start, std::string end) {
    std::unordered_map<std::string, std::string> paths;
    std::unordered_map<std::string, double> homeDistance;
    std::vector<std::string> path;
    CityQueue open(cities, end);

    open.push(start);
    paths[start] = std::string();

    while (!open.empty()) {
        std::string next = open.pop_back().name;

        if (next == end)
            return getPath(paths, end);

        for (std::string s : cities[next].adj) {
            if (!paths.count(s)) {
                paths[s] = next;
                homeDistance[s] = homeDistance[next] + cities[s].distanceTo(cities[next]);
                open.push(s, homeDistance[s]);
            }
        }
    }

    return {};
}

std::vector<std::string> Map::astar(std::string start, std::string end) {
    std::unordered_map<std::string, std::string> openPaths;
    std::unordered_map<std::string, std::string> closed;
    std::unordered_map<std::string, double> homeDistance;
    std::vector<std::string> path;
    CityQueue open(cities, end);

    open.push(start);
    openPaths[start] = "";
    homeDistance[start] = 0;

    while (!open.empty()) {
        std::string next = open.pop_back().name;
        closed[next] = openPaths[next];
        openPaths.erase(next);

        if (next == end)
            return getPath(closed, end);

        for (std::string s : cities[next].adj) {
            if (closed.count(s))
                continue;

            double distStart = homeDistance[next] + cities[s].distanceTo(cities[next]);
            if (!openPaths.count(s)) {
                openPaths[s] = next;
                homeDistance[s] = distStart;
                open.push(s, distStart);
            }
            else if (distStart < homeDistance[s]) {
                openPaths[s] = next;
                homeDistance[s] = distStart;
                open.remove(s);
                open.push(s, distStart);
            }
        }
    }
    return {};
}

City Map::getCity(std::string name) {
    return cities[name];
}

std::vector<std::string> Map::getPath(std::unordered_map<std::string, std::string> paths, std::string end) {
    std::string next = end;
    std::vector<std::string> path;

    while (!next.empty()) {
        path.insert(path.begin(), next);
        next = paths[next];
    }

    return path;
}

bool Map::contains(std::string name) {
    return cities.count(name) > 0;
}

/* My original attempt at the IDDFS
std::vector<std::string> Map::iddfs(std::string start, std::string end) {
    // Shortest path found
    std::vector<std::string> path;

    if (!cities.count(start) || !cities.count(end))
        return path;

    // All paths towards start node; nodes will be added as keys when they are visited
    std::unordered_map<std::string, std::string> paths;
    std::string next;

    int level = 0;
    while (level < INT_MAX) {
        // Stack of not-yet-visited nodes
        std::stack<std::string> stack;
        
        paths.clear();

        // Add the start node to the stack and mark as visited
        stack.push(start);
        paths[start] = std::string();


        // Loop until all paths have been searched
        int currentDepth = -1;
        while (!stack.empty()) {
            currentDepth++;
            // Choose next node to search
            next = stack.top();
            stack.pop();

            // Discover all connected nodes
            for (std::string s : cities[next].adj) {
                // Mark each node as visited and add to stack
                if (!paths.count(s)) {
                    paths[s] = next; // Point node towards source
                    if (currentDepth < level)
                        stack.push(s);
                }
            }
            if (currentDepth == level)
                currentDepth--;

            // Once the end is found, the shortest path has been found
            if (paths.count(end)) break;
        }
        if (paths.count(end)) break;
        level++;
    }

    if (!paths.count(end)) {
        std::cout << "No path found." << std::endl;
        return path;
    }

    // Backtrack through paths to find shortest route
    next = end;
    while (!next.empty()) {
        path.insert(path.begin(), next);
        next = paths[next];
    }

    return path;
}
*/