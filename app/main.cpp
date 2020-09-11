// main.cpp
//
// ICS 46 Winter 2019
// Project #4: Rock and Roll Stops the Traffic
//
// This is the program's main() function, which is the entry point for your
// console user interface.

#include "InputReader.hpp"
#include "TripReader.hpp"
#include "RoadMapReader.hpp"
#include <iostream>
#include <map>
#include <cmath>
#include <iomanip>

//#include "RoadMapWriter.hpp"

void print_helper(std::vector<double> time) {
	if (time.size() == 3) {
		std::cout << std::round(time[0]) << " hrs " << std::round(time[1]) << " mins " << std::roundf(time[2]) << " secs";
	}
	else if(time.size() == 2)
	{
		std::cout << std::round(time[0]) << " mins " << std::roundf(time[1]) << " secs";
	}
	else {
		std::cout << std::roundf(time[0]) << " secs";	
	}
}

std::vector<double> helper_func(double seconds) {
	double hour = seconds / 3600.00;
	double minute = (((seconds/ 3600) - (int)hour) * 60);
	double sec = (((seconds / 60) - (int)minute) * 60);
	std::vector<double> temp;
	
	if (hour > 1) {
		temp.push_back(hour);
	}
	if (minute > 1) {
		temp.push_back(minute);
	}
	if(hour > 1 || minute > 1) {
		temp.push_back(sec);
	}
	else {
		temp.push_back(seconds);
	}
	
	return temp;
}

int main()
{
	struct vsd {
		int vertex;
		std::string street;
		RoadSegment data;
	};

	InputReader ir = InputReader(std::cin);
	RoadMapReader rmr;
	TripReader tr;
	RoadMap rm = rmr.readRoadMap(ir);
	std::vector<Trip> trips = tr.readTrips(ir);

	if(rm.isStronglyConnected() == false) {
		std::cout << "Disconnected Map" << std::endl;
		return 0;
	}
	//RoadMapWriter rmw;
	//rmw.writeRoadMap(std::cout, rm);
	std::function<double(const RoadSegment&)> shortestPath = [](RoadSegment rs){return rs.miles;};
	std::function<double(const RoadSegment&)> fastestTime = [](RoadSegment rs){return rs.milesPerHour / rs.miles;};

	std::map<int,int> fsp;

	for(auto it = trips.begin(); it != trips.end(); it++) {  

		std::vector<vsd> trip;

		if (it->metric == TripMetric::Distance) {
			fsp = rm.findShortestPaths(it->startVertex, shortestPath);
		}
		else {
			fsp = rm.findShortestPaths(it->startVertex, fastestTime);
		}

		int last_vertex = it->endVertex; 
	
		vsd temp_last = vsd{last_vertex, rm.vertexInfo(last_vertex), rm.edgeInfo(fsp[last_vertex], last_vertex)};
		trip.push_back(temp_last);
	
		double total_distance = temp_last.data.miles;

		while(last_vertex != it->startVertex) {
			last_vertex = fsp[last_vertex];
			if (last_vertex != it->startVertex) {
				temp_last = vsd{last_vertex, rm.vertexInfo(last_vertex), rm.edgeInfo(fsp[last_vertex], last_vertex)};
				trip.push_back(temp_last);
				total_distance += temp_last.data.miles;
			}
			else {
				temp_last = vsd{last_vertex, rm.vertexInfo(last_vertex), {0.0}};
				trip.push_back(temp_last);
			}
		}
		if(it->metric == TripMetric::Distance) {
			std::cout << "Shortest distance from " << rm.vertexInfo(trip.back().vertex) << " to " << rm.vertexInfo(trip.front().vertex) << std::endl;
			std::cout << "\t Begin at " << trip[trip.size() - 1].street << std::endl;
			for (int i = trip.size() - 2; i >= 0; i--) {
				std::cout << "\t Continue to " << trip[i].street << " (" << trip[i].data.miles << " miles)" << std::endl;
			}	
			std::cout << "Total Distance: " << std::setprecision(3) << total_distance << " miles" << std::endl;
		}
		else {
			double total_time = 0;
			std::cout << "Shortest driving time from " << rm.vertexInfo(trip.back().vertex) << " to " << rm.vertexInfo(trip.front().vertex) << std::endl;
			std::cout << "\t Begin at " << trip[trip.size() - 1].street << std::endl;
			for (int i = trip.size() - 2; i >= 0; i--) {
				double seconds = (trip[i].data.miles / trip[i].data.milesPerHour) * 3600;
				std::vector<double> formatted_time = helper_func(seconds);
				std::cout << "\t Continue to " << trip[i].street << " (" << trip[i].data.miles  << " miles & " << trip[i].data.milesPerHour << "mph = ";
				total_time += seconds;
				print_helper(formatted_time);
				std::cout << ")" << std::endl;
			}
			std::vector<double> time = helper_func(total_time);
			std::cout << "Total time: ";
			print_helper(time);
		}
		std::cout<< "\n" << std::endl;
	}
    return 0;
}

