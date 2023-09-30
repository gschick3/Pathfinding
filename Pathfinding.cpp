// Pathfinding.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <chrono>

#include "Map.h"

double pathDistance(Map map, std::vector<std::string> cities);

int main()
{
    Map map("./data/Adjacencies.txt", "./data/coordinates.csv");

    // User interface
    while (true) {
        int algo;
        std::cout << "Choose an algorithm: "
            << "\n0. quit"
            << "\n1. BFS"
            << "\n2. DFS"
            << "\n3. IDDFS"
            << "\n4. Best-First"
            << "\n5. A*"
            << std::endl;
        std::cin >> algo;

        if (algo == 0) break;

        std::string start, end;
        std::cout << "Choose a start location: ";
        std::cin >> start;
        std::cout << "Choose an end location: ";
        std::cin >> end;

        if (!map.contains(start)) {
            std::cout << "City not found: " << start << std::endl;
            continue;
        }
        if (!map.contains(end)) {
            std::cout << "City not found: " << end << std::endl;
            continue;
        }


        std::vector<std::string> path;
        std::chrono::steady_clock::time_point startTime, endTime;

        switch (algo) {
        case 1:
            startTime = std::chrono::steady_clock::now();
            path = map.bfs(start, end);
            endTime = std::chrono::steady_clock::now();
            break;
        case 2:
            startTime = std::chrono::steady_clock::now();
            path = map.dfs(start, end);
            endTime = std::chrono::steady_clock::now();
            break;
        case 3:
            startTime = std::chrono::steady_clock::now();
            path = map.iddfs(start, end);
            endTime = std::chrono::steady_clock::now();
            break;
        case 4:
            startTime = std::chrono::steady_clock::now();
            path = map.bestfs(start, end);
            endTime = std::chrono::steady_clock::now();
            break;
        case 5:
            startTime = std::chrono::steady_clock::now();
            path = map.astar(start, end);
            endTime = std::chrono::steady_clock::now();
            break;
        default:
            std::cout << "Not an option" << std::endl;
            continue;
        }

        if (path.empty()) {
            std::cout << "No path found." << std::endl;
            continue;
        }

        std::cout << "Found solution in " 
                  << (double) std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000 
                  << "ms" << std::endl;

        for (auto a : path)
            std::cout << "->" << a;
        std::cout << std::endl;
        std::cout << "Distance: " << pathDistance(map, path) << "km" << std::endl;
    }
}

double pathDistance(Map map, std::vector<std::string> cities) {
    double total = 0;
    for (int i = 0; i < cities.size() - 1; i++) {
        total += map.getCity(cities[i]).distanceTo(map.getCity(cities[i + 1]));
    }

    return total;
}