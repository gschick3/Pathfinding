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
	double distanceTo(const City& c2) const {
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
	double score;

	CityQueueItem(std::string _name, City _city, double _score)
		: name(_name), city(_city), score(_score) {}

	// This operator is used by std::greater to order a minimal priority queue
	bool operator >(const CityQueueItem& other) const {
		return score > other.score;
	}
};