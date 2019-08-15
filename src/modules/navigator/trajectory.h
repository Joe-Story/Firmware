/**
 * Header file for use with trajectory.cpp
 *
 * @author Joseph Story <jdrs3@cam.ac.uk>
 * @author Vasileios Tsoutsouras
 */

#pragma once

#include "mission_block.h"
#include "mission_feasibility_checker.h"
//#include "mavlink/mavlink_mission.h"
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

mission_s		_mission {};

const double earth_radius = 6371000; //metres;
const double pi = 3.1415926;
const double g = 9.81; //ms^-2
const double rho = 1.225; //Density of air in kgm^-3

class DRONE{
  public:
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


    //Calculate an estimate of the maximum drone flight time in minutes
    double calc_max_flight_time(){
        double power = sqrt( (2*pow(mass,3)*pow(g,3)) / (rho*rotor_area) );
        double energy = bat_voltage * bat_capacity * pow(10,-3); //Battery energy in Wh
        double max_flight_time = (energy/power) * 60; //Max flight time in minutes
        return max_flight_time;
    }

    //Calculate an estimate of the battery percentage used between two waypoints
    double calc_energy_use(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2, double flight_speed, double payload){
        double lat1_rad = (lat1/180)*pi;
        double lat2_rad = (lat2/180)*pi;
        double lon1_rad = (lon1/180)*pi;
        double lon2_rad = (lon2/180)*pi;

        double x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                        (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));
        double time = (x)/(flight_speed);
        //time += 5; // Approximate time required to deliver and item

        double power = sqrt( (pow(mass+bat_mass+payload,3)*pow(g,3)) / (2*rho*rotor_area) );
        double percent_used = (((power*time) / (bat_energy*3600)) * 100) / (efficiency/100);
        return percent_used;
    }
};

class WAYPOINTS{
  public:
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


    /*** Waypoint trajectory functions ***/

    //Calculate the distance between two waypoints in spherical polar coordinates (latitude, longitude, and altitude)
    double time(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2, double flight_speed)
    {
        double lat1_rad = (lat1/180)*pi;
        double lat2_rad = (lat2/180)*pi;
        double lon1_rad = (lon1/180)*pi;
        double lon2_rad = (lon2/180)*pi;

        double x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                        (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));

        double time = (x)/(flight_speed);
        //time += 5; // Approximate time required to deliver an item
        return time;
    }

};

class TRAJECTORY{
    public:
        //Calculates a 2D cost array based on the cost function used
        double** calc_cost(int num_waypoints, std::vector<mission_waypoint_t> array){
            WAYPOINTS cost_object;
            double ** cost2d;
            double speed = 5.0;
            cost2d = new double * [num_waypoints+2];

            for (int i=0; i < num_waypoints+1; i++){
                cost2d[i] = new double [num_waypoints+2];
            }

            for (int i = 0; i < num_waypoints+1; i++){
                for (int t = 0; t < num_waypoints+1; t++){
                    cost2d[i][t] = cost_object.time(array[i].waypoint.altitude, array[t].waypoint.altitude, array[i].waypoint.lat, array[t].waypoint.lat, array[i].waypoint.lon, array[t].waypoint.lon, speed);
                }
            }

            //Print the 2D Cost array
            std::cout << "2D Cost Array:" << std::endl;
            std::cout << "" << std::endl;
            for (int i = 0; i < num_waypoints+1; i++){
                std::cout << "[";
                for (int t = 0; t < num_waypoints; t++){
                    std::cout << cost2d[i][t] << ",  ";
                }
                std::cout << cost2d[i][num_waypoints];
                std::cout << "]" << std::endl;
            }
            std::cout << "" << std::endl;

            return cost2d;
        }

        //Sets up all the variables required for the mincost function, and returns the estimated minimum cost route
        std::tuple<double, double, std::vector<mission_waypoint_t>> call_mincost(std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, std::vector<mission_waypoint_t> finalWpsList, double ** cost_array){
            double cost = 0;
            int n = 0;
            int completed[num_waypoints+1] = {0};
            double energy = 0;
            //Set the starting payload
            double payload_weight=0;
            for (int i=0; i<num_waypoints+1; i++){
                payload_weight += uploadedWpsList[i].waypoint.payload_weight;
            }

            return TRAJECTORY::mincost(0, uploadedWpsList, num_waypoints, cost, completed, finalWpsList, cost_array, n, energy, payload_weight);

        }


    private:
        //Finds the nearest neighbour that hasn't been visited
        std::tuple<int, double, bool> least(int p, int num_waypoints, int completed[], double ** cost_array, double cost){
            int i,np=0;
            int min=0,kmin;
            bool is_final = true;

            for (i=0;i<num_waypoints+1;i++){
                if((cost_array[p][i]>=0.05)&&(completed[i]==0)){ //Check that this is correct
                    //For first iteration, pick any point
                    if (min == 0){
                        min = cost_array[i][0]+cost_array[p][i];
                        kmin=cost_array[p][i];
                        np=i;
                        is_final = false;
                    }

                    //For other iterations, check to see if there is any better point
                    else if(cost_array[p][i]+cost_array[i][p] < min){
                        min = cost_array[i][0]+cost_array[p][i];
                        kmin=cost_array[p][i];
                        np=i;
                        is_final = false;
                    }
                }
            }
            if(is_final != true){
                cost+=kmin;
            }
            return std::make_tuple(np, cost, is_final);
        }

        //Finds a close to optimal route using the 'Greedy' method
        std::tuple <double, double, std::vector<mission_waypoint_t>> mincost(int position, std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, double cost, int completed[], std::vector<mission_waypoint_t> finalWpsList, double ** cost_array, int n, double energy, double payload_weight){
            int nposition;
            bool is_final = false;
            double speed = 5.0;
            DRONE drone;

            //Set the current position as completed
            completed[position]=1;

            std::cout << position << "--->";

            finalWpsList.at(n) = uploadedWpsList[position];

            std::tie(nposition, cost, is_final) = TRAJECTORY::least(position, num_waypoints, completed, cost_array, cost);
            energy+=drone.calc_energy_use(uploadedWpsList[position].waypoint.altitude, uploadedWpsList[nposition].waypoint.altitude, uploadedWpsList[position].waypoint.lat, uploadedWpsList[nposition].waypoint.lat, uploadedWpsList[position].waypoint.lon, uploadedWpsList[nposition].waypoint.lon, speed, payload_weight);

            //Remove the delivered payload from the total payload weight
            payload_weight -= uploadedWpsList[nposition].waypoint.payload_weight;

            if(is_final == true){
                nposition=0;
                std::cout << nposition;
                cost+=cost_array[position][nposition];
                return std::make_tuple(cost, energy, finalWpsList);
            }

            n++;
            return mincost(nposition, uploadedWpsList, num_waypoints, cost, completed, finalWpsList, cost_array, n, energy, payload_weight);
        }

};
