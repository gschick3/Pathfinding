#pragma once

#define _USE_MATH_DEFINES

#include <string>
#include <vector>
#include <unordered_map>
#include <math.h>

struct City {
	std::vector<std::string> adj = {};
	std::pair<double, double> coords = {};

	// Distance to a different city using Haversine formula
	double distanceTo(City c2) {
		double earthRadius = 6371; // km

		// Latitudes in radians
		double rLat1 = coords.first * M_PI / 180.0;
		double rLat2 = c2.coords.first * M_PI / 180.0;

		return acos(sin(rLat1) * sin(rLat2) + cos(rLat1) * cos(rLat2) * cos((c2.coords.second - coords.second) * M_PI / 180.0)) * earthRadius;
	}
};

struct CityQueueItem {
	std::string name;
	City city;
	double distStart;
	double distEnd;
};

class CityQueue {
public:
	CityQueue(std::unordered_map<std::string, City> names, std::string goal) {
		this->names = names;
		this->goal = names[goal];
	}

	void push(std::string city, double distStart) {
		CityQueueItem cityItem = { city, names[city], distStart, names[city].distanceTo(goal)};
		priorityQueue.push_back(cityItem);
		for (int i = priorityQueue.size()-1; i > 0; i--)
			if (priorityQueue[i].distStart + priorityQueue[i].distEnd > priorityQueue[i-1].distStart + priorityQueue[i-1].distEnd) {
				auto temp = priorityQueue[i];
				priorityQueue[i] = priorityQueue[i - 1];
				priorityQueue[i - 1] = temp;
				if (i == 1) break;
			}
	}

	inline void push(std::string city) {
		// This has the same effect as scoring only off of heuristic distance
		push(city, 0);
	}

	CityQueueItem pop_back() {
		auto temp = priorityQueue.back();
		priorityQueue.pop_back();
		return temp;
	}

	void remove(std::string city) {
		// My mind is gone at this point. There are probably better ways of doing this
		int loc = -1;
		int i;
		for (i = 0; i < priorityQueue.size(); i++) {
			if (priorityQueue.at(i).name == city)
				break;
		}
		priorityQueue.erase(priorityQueue.begin() + i);
	}

	size_t size() {
		return priorityQueue.size();
	}

	bool empty() {
		return priorityQueue.empty();
	}

private:
	City goal;
	std::unordered_map<std::string, City> names;
	std::vector<CityQueueItem> priorityQueue;
};