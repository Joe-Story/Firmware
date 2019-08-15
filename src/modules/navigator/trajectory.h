/**
 * Header file for use with trajectory.cpp
 *
 * @author Joseph Story <jdrs3@cam.ac.uk>
 * @author Vasileios Tsoutsouras
 */

#pragma once

#include "mission_block.h"
#include "mission_feasibility_checker.h"
#include "navigator_mode.h"

#include <float.h>
#include <string>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

typedef struct mission_waypoint_struct {
        mission_item_s waypoint;
        int originalIndex;
} mission_waypoint_t;

typedef struct waypoints_struct {
    int id;
    std::string user;
    double lat;
    double lon;
    double alt;
    double speed;
    int deadline;
    int numOfWaypoints;
    double payload;
    int original_index;
} waypoints_t;

class Trajectory
{
    public:
        Trajectory();

        //~Trajectory() override = default;

        void update_trajectory(mission_s mission);

    private:
        //Physical drone properties
        const double mass = 1.38; // kg
        const int num_rotors = 4; // -
        const double rotor_radius = 0.12; // m
        double rotor_area = num_rotors * (pi * pow(rotor_radius, 2)); // m^2
        double max_velocity; // m/s
        double min_velocity; // m/s

        //Battery and electrical properties
        double bat_capacity; // mAh
        const double bat_voltage = 15.2; // V
        const double bat_energy = 81.3; // Wh
        const double bat_mass = 0; // kg
        const double efficiency = 100.0; // %

        const double earth_radius = 6371000; //metres;
        const double pi = 3.1415926;
        const double g = 9.81; //ms^-2
        const double rho = 1.225; //Density of air in kgm^-3

        //Calculates a 2D cost array based on the cost function used
        double** calc_cost(int num_waypoints, std::vector<mission_waypoint_t> array);

        //Sets up all the variables required for the mincost function, and returns the estimated minimum cost route
        std::tuple<double, double, std::vector<mission_waypoint_t>> calc_solution(std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, std::vector<mission_waypoint_t> finalWpsList);

        //Finds the nearest neighbour that hasn't been visited
        std::tuple<int, double, bool> least(int p, int num_waypoints, int completed[], double ** cost_array, double cost);

        //Finds a close to optimal route using the 'Simulateted Annealing' algorithm
        std::tuple <double, double, std::vector<mission_waypoint_t>>
        solution_sa(int position, std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, int visited[], std::vector<mission_waypoint_t> finalWpsList, double ** cost_array);

        //Finds a close to optimal route using the 'Greedy' method
        std::tuple <double, double, std::vector<mission_waypoint_t>> solution_mincost(int position, std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, double cost, int completed[], std::vector<mission_waypoint_t> finalWpsList, double ** cost_array, int n, double energy, double payload_weight);

        //Finds an optimal route using the 'Brute Force' method
        void solution_bruteforce (int level, int maxLevel, int *trajectory, int *visitedNodes, std::vector<mission_waypoint_t> uploadedWpsList, int *numOfTajectories, double ** cost_array);

        //Calculate the distance between two waypoints in spherical polar coordinates (latitude, longitude, and altitude)
        double calc_flight_time(mission_item_s waypoint1, mission_item_s waypoint2, double flight_speed);

        //Calculate an estimate of the maximum drone flight time in minutes
        double calc_max_flight_time(void);

        //Calculate an estimate of the battery percentage used between two waypoints
        double calc_energy_use(mission_item_s waypoint1, mission_item_s waypoint2, double flight_speed, double payload);
};
