/**
 * Calculates the optimum drone trajectory
 *
 * @author Joseph Story <jdrs3@cam.ac.uk>
 * @author Vasileios Tsoutsouras
 */

#include "mission.h"
#include "navigator.h"
#include "navigation.h"

#include <string.h>
#include <drivers/drv_hrt.h>
#include <dataman/dataman.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/ecl/geo/geo.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

/*** SETUP FOR COST CALCULATOR CODE ***/
#include <functional>
#include <future>
#include <memory>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <new>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

using namespace std;

#include "trajectory.h"

#define DELIVERY_TIME_SEC 5.0 /* How close to reality is this? */

Trajectory::Trajectory()
{
}

double
Trajectory::calc_flight_time(mission_item_s waypoint1, mission_item_s waypoint2, double flight_speed)
{
    double lat1_rad = (waypoint1.lat/180)*pi;
    double lat2_rad = (waypoint2.lat/180)*pi;
    double lon1_rad = (waypoint1.lon/180)*pi;
    double lon2_rad = (waypoint2.lon/180)*pi;

    double alt1 = waypoint1.altitude;
    double alt2 = waypoint2.altitude;

    double x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                    (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));

    double time = (x/flight_speed) + DELIVERY_TIME_SEC; // Approximate time required to deliver an item

    return time;
}

//Calculate an estimate of the battery percentage used between two waypoints
double
Trajectory::calc_energy_use(mission_item_s waypoint1, mission_item_s waypoint2, double flight_speed, double payload)
{
    double lat1_rad = (waypoint1.lat/180)*pi;
    double lat2_rad = (waypoint2.lat/180)*pi;
    double lon1_rad = (waypoint1.lon/180)*pi;
    double lon2_rad = (waypoint2.lon/180)*pi;

    double alt1 = waypoint1.altitude;
    double alt2 = waypoint2.altitude;

    double x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                    (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));

    double time = (x/flight_speed) + DELIVERY_TIME_SEC; // Approximate time required to deliver an item

    double power;
    /*
    power = pow(mass+bat_mass+payload,3);
    printf("DEBUG0 power = %f\n",power);

    power = (pow(mass+bat_mass+payload,3)*pow(g,3));
    printf("DEBUG1 power = %f\n",power);

    power = (2*1.225*0.568489194);
    printf("DEBUG2 power = %f\n",power);

    power = (pow(mass+bat_mass+payload,3)*pow(g,3)) / (2*rho*rotor_area) ;
    printf("DEBUG3 power = %f\n",power);
    */
    //power = sqrt( (pow(mass+bat_mass+payload,3)*pow(g,3)) / (2*rho*rotor_area) );
    power = sqrt( (pow(mass+bat_mass+payload,3)*pow(g,3)) / (2*1.225*0.568489194) );
    //printf("DEBUG4 power = %f\n",power);

    double percent_used = (((power*time) / (bat_energy*3600)) * 100) / (efficiency/100);

    return percent_used;
}

double
Trajectory::calc_max_flight_time(void)
{
    double power = sqrt( (2*pow(mass,3)*pow(g,3)) / (rho*rotor_area) );
    double energy = bat_voltage * bat_capacity * pow(10,-3); //Battery energy in Wh
    double max_flight_time = (energy/power) * 60; //Max flight time in minutes

    return max_flight_time;
}

//Calculates a 2D cost array based on the cost function used
double**
Trajectory::calc_cost(int num_waypoints, std::vector<mission_waypoint_t> array)
{
    double ** cost2d;
    double speed = 5.0;
    cost2d = new double * [num_waypoints+2];

    for (int i=0; i < num_waypoints+1; i++){
        cost2d[i] = new double [num_waypoints+2];
    }

    for (int i = 0; i < num_waypoints+1; i++){
        for (int t = 0; t < num_waypoints+1; t++){
            cost2d[i][t] = calc_flight_time(array[i].waypoint, array[t].waypoint, speed);
        }
    }

    //Print the 2D Cost array
    std::cout << "2D Cost Array:" << std::endl;

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

//Finds the nearest neighbour that hasn't been visited
std::tuple<int, double, bool>
Trajectory::least(int p, int num_waypoints, int completed[], double ** cost_array, double cost)
{
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

    if (is_final != true){
        cost+=kmin;
    }

    return std::make_tuple(np, cost, is_final);
}

//Finds a close to optimal route using the 'Greedy' method
std::tuple <double, double, std::vector<mission_waypoint_t>>
Trajectory::solution_mincost(int position, std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, double cost, int visited[], std::vector<mission_waypoint_t> finalWpsList, double ** cost_array, int n, double energy, double payload_weight)
{
    int nposition;
    bool is_final = false;
    double speed = 5.0;

    //Set the current position as visited
    visited[position]=1;

    std::cout << position << "--->";

    //finalWpsList.at(n) = uploadedWpsList[position];
    finalWpsList.push_back(uploadedWpsList[position]);

    std::tie(nposition, cost, is_final) = least(position, num_waypoints, visited, cost_array, cost);
    energy += calc_energy_use(uploadedWpsList[position].waypoint, uploadedWpsList[nposition].waypoint, speed, payload_weight);

    //Remove the delivered payload from the total payload weight
    payload_weight -= uploadedWpsList[nposition].waypoint.payload_weight;

    if(is_final == true){
        nposition=0;
        std::cout << nposition;
        cost+=cost_array[position][nposition];
        return std::make_tuple(cost, energy, finalWpsList);
    } else {
        return solution_mincost(nposition, uploadedWpsList, num_waypoints, cost, visited, finalWpsList, cost_array, n++, energy, payload_weight);
    }
}

//Finds an to optimal route using the 'Brute Force' method
void
Trajectory::solution_bruteforce (int level, int maxLevel, int *trajectory, int *visitedNodes, std::vector<mission_waypoint_t> uploadedWpsList, int *numOfTajectories, double ** cost_array) {
	int i, j;

	for (i=0; i<maxLevel; i++) {
		if (visitedNodes[i] == 0) {
			trajectory[level] = i;
			visitedNodes[i] = 1;

			if (level < (maxLevel-1)) {
				Trajectory::solution_bruteforce (level+1, maxLevel, trajectory, visitedNodes, uploadedWpsList, numOfTajectories, cost_array);
			} else if (level == (maxLevel-1)) {
				(*numOfTajectories)++;

                for (j=0; j<maxLevel-1; j++) {
		            printf("%d--->", trajectory[j]);
	            }
                printf("%d ", trajectory[j]);

                /* Return is missing */

                double trajectory_cost = 0;
                double trajectory_payload = uploadedWpsList[0].waypoint.payload_weight;
                for (j=0; j<maxLevel-1; j++) {
                    //printf("\n j: %d trajectory_cost: %f trajectory_payload: %f next: %f \n", j, trajectory_cost, trajectory_payload, uploadedWpsList[j+1].waypoint.payload_weight);
                    trajectory_cost += Trajectory::calc_energy_use(uploadedWpsList[j].waypoint, uploadedWpsList[j+1].waypoint, 5, trajectory_payload);
                    trajectory_payload -= uploadedWpsList[j+1].waypoint.payload_weight;
                }

                printf("with cost %f\n", trajectory_cost);

			} else {
				printf("Level is wrong. Level %d maxLevel %d\n",level,maxLevel);
			}

			trajectory[level] = '\0';
			visitedNodes[i] = 0;
		}
	}
}

//Sets up all the variables required for the mincost function, and returns the estimated minimum cost route
std::tuple<double, double, std::vector<mission_waypoint_t>>
Trajectory::calc_solution(std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, std::vector<mission_waypoint_t> finalWpsList)
{
    double cost = 0;
    int n = 0, numOfTajectories=0;
    int visited[num_waypoints] = {0}, trajectory[num_waypoints] = {0}, visitedNodes[num_waypoints] = {0};
    double energy = 0;
    double **cost_array;

    //Set the starting payload
    double takeoff_weight=0;

    //Create the 2D dynamic and energy cost array
    cost_array = new double * [num_waypoints+1];
    for (int i=0; i < num_waypoints+1; i++){
        cost_array[i] = new double [num_waypoints+1];
    }

    //Update takeoff and land position using PX4 rather than MAVSDK
    /* INSERT CODE HERE */

    //Set the last point as the home position
    /* INSERT CODE HERE */

    //Calculate the 2D cost/time array (using spherical polar coordinates)
    cost_array = calc_cost(num_waypoints, uploadedWpsList);

    //Set the starting payload
    for (int i=0; i<num_waypoints+1; i++){
        takeoff_weight += uploadedWpsList[i].waypoint.payload_weight;
    }

    printf("\nThe takeoff weight is: %f kg\n",(double) takeoff_weight);
    uploadedWpsList[0].waypoint.payload_weight = takeoff_weight;

    /*
    for (i=0; i < num_waypoints; i++) {
		trajectory[i] = 0;
		visitedNodes[i] = 0;
	}
    */

    /* Set takeoff as first point */
    trajectory[0] = 0;
    visitedNodes[0] = 1;
    Trajectory::solution_bruteforce (1, num_waypoints, trajectory, visitedNodes, uploadedWpsList, &numOfTajectories, cost_array);

    printf("\nBrute force evaluated %d routes\n",numOfTajectories);

    return Trajectory::solution_mincost(0, uploadedWpsList, num_waypoints, cost, visited, finalWpsList, cost_array, n, energy, takeoff_weight);

}

void
Trajectory::update_trajectory(mission_s mission)
{
    vector<mission_waypoint_t> uploadedWpsList;
    vector<mission_waypoint_t> finalWpsList;
    mission_waypoint_t oneWaypoint;
    double cost, energy;
    bool write_failed = false;
    int numItems = mission.count;
    int numOfWaypoints;

    printf("Mission Count is: %d\n", numItems);

    if (numItems > 0){

        /* READ MISSION ITEMS */
        numOfWaypoints = 0;
        for (int i = 0; i < numItems; i++){
            struct mission_item_s mission_item {};
            if (!(dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s))) {
                /* error reading, mission is invalid */
                PX4_WARN("READ ERROR");
                //mavlink_log_info(_navigator->get_mavlink_log_pub(), "Error reading offboard mission.");
                return;
            }

            /* check only items with valid lat/lon */
            if (!MissionBlock::item_contains_position(mission_item)) {
                continue;
            } else {
                oneWaypoint.waypoint = mission_item;
                oneWaypoint.originalIndex = i;
                if (oneWaypoint.waypoint.nav_cmd == 16){
                    printf("Waypoint weight is %f deadline is %f\n",oneWaypoint.waypoint.payload_weight,oneWaypoint.waypoint.deadline);
                    uploadedWpsList.push_back(oneWaypoint);
                    //finalWpsList.push_back(oneWaypoint);
                    numOfWaypoints++;
                }
            }
        }

        //Check that waypoints have been found
        if (numOfWaypoints <= 0) {
            PX4_WARN("NO WAYPOINTS FOUND");
            return;
        }

        //Print all of the received information
        /*
        for (int i=0; i < numItems; i++){
            printf("Index: %d, Command: %d, Lat: %d\n", i, uploadedWpsList[i].waypoint.nav_cmd, uploadedWpsList[i].waypoint.lat);
        }
        */

        //Print the number of waypoints received
        printf("Num of Waypoints is: %d\n", numOfWaypoints);

        //Print the received waypoints
        for (int i = 0; i < numOfWaypoints; i++){
            printf("Waypoint %d: Lat: %f Lon: %f nav_cmd: %d\n",
                i, uploadedWpsList[i].waypoint.lat, uploadedWpsList[i].waypoint.lon, uploadedWpsList[i].waypoint.nav_cmd);
        }

        /*** Collate the information needed to create a trajectory ***/

        /*** Attempt to calculate a minimum cost trajectory ***/

        //Calculate the optimal trajectory
        printf("\nCalculating best flight path:\n");

        std::tie (cost, energy, finalWpsList) = calc_solution(uploadedWpsList, numOfWaypoints, finalWpsList);
        cost += 3*numOfWaypoints; /* What is this ? */

        printf("\nMinimum cost is %f seconds\n", (double) cost);
        printf("Battery usage is %f %\n", (double) energy);

        //Print the final trajectory
        for (int i = 0; i < numOfWaypoints; i++){
            printf("Waypoint %d: Lat: %f Lon: %f nav_cmd: %d\n",
                i, finalWpsList[i].waypoint.lat, finalWpsList[i].waypoint.lon, finalWpsList[i].waypoint.nav_cmd);
        }

        //Ensure the flight time is within the maximum possible flight time
        /* INSERT CODE HERE */

        //Clear previous items from dataman
//                    for (int i=0; i < numItems; i++){
//                        dm_lock(DM_KEY_WAYPOINTS_OFFBOARD_0);
//                        dm_lock(DM_KEY_WAYPOINTS_OFFBOARD_1);
//                        dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_0);
//                        dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_1);
//                        dm_unlock(DM_KEY_WAYPOINTS_OFFBOARD_0);
//                        dm_unlock(DM_KEY_WAYPOINTS_OFFBOARD_1);
//                    }

        //Write the optimal trajectory to the first memory locations in dataman
        for (int i=0; i < numOfWaypoints; i++){
            dm_lock(DM_KEY_MISSION_STATE);
            write_failed = dm_write((dm_item_t)mission.dataman_id, i,
                                          DM_PERSIST_POWER_ON_RESET, &finalWpsList[i].waypoint,
                                          sizeof(struct mission_item_s)) != sizeof(struct mission_item_s);
            if (write_failed) {
                PX4_WARN("My Write failed");
                //printf("Tried to write index %d: with original index %d\n", temp_counter, uploadedWpsList[waypointsCnt-1-waypointsIndex].originalIndex);
            }
            dm_unlock(DM_KEY_MISSION_STATE);
        }

        //Update the mission count with the new number of items
        mission.count = numOfWaypoints;
        numItems = mission.count;

        //Print the entire contents of the dataman file system
        printf("Final Trajectory:\n");
        for (int i = 0; i < numItems; i++){
            struct mission_item_s mission_item {};
            if (!(dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s))) {
                /* error reading, mission is invalid */
                PX4_WARN("READ ERROR");
                //mavlink_log_info(_navigator->get_mavlink_log_pub(), "Error reading offboard mission.");
                return;
            }

            oneWaypoint.waypoint = mission_item;
            oneWaypoint.originalIndex = i;
            std::cout << "Index: " << oneWaypoint.originalIndex << ", " << oneWaypoint.waypoint.nav_cmd << ", " << oneWaypoint.waypoint.lat << ", " << oneWaypoint.waypoint.lon << std::endl;
        }
    }
}
