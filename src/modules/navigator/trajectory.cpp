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
//Some of these may not be necessary

//using namespace std::placeholders; //IS THIS NECESSARY?
//using namespace std::chrono; // for seconds(), milliseconds()
//using namespace std::this_thread; // for sleep_for()

using namespace std;

#include "trajectory.h"

//Declare cost function variables and class objects
int mission_iterator = 0;
int numOfWaypoints;
Trajectory _trajectory;

//Declare dynamic cost and energy array
double ** energy_array;

/*** END OF SETUP ***/

Trajectory::Trajectory()
{
}

double
Trajectory::calc_flight_time(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2, double flight_speed)
{
    double lat1_rad = (lat1/180)*pi;
    double lat2_rad = (lat2/180)*pi;
    double lon1_rad = (lon1/180)*pi;
    double lon2_rad = (lon2/180)*pi;

    double x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                    (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));

    double time = (x)/(flight_speed);
    time += 5; // Approximate time required to deliver an item
    return time;
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
            cost2d[i][t] = calc_flight_time(array[i].waypoint.altitude, array[t].waypoint.altitude, array[i].waypoint.lat, array[t].waypoint.lat, array[i].waypoint.lon, array[t].waypoint.lon, speed);
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
std::tuple<double, double, std::vector<mission_waypoint_t>>
Trajectory::call_mincost(std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, std::vector<mission_waypoint_t> finalWpsList, double ** cost_array)
{
    double cost = 0;
    int n = 0;
    int completed[num_waypoints+1] = {0};
    double energy = 0;
    //Set the starting payload
    double payload_weight=0;
    for (int i=0; i<num_waypoints+1; i++){
        payload_weight += uploadedWpsList[i].waypoint.payload_weight;
    }

    return Trajectory::mincost(0, uploadedWpsList, num_waypoints, cost, completed, finalWpsList, cost_array, n, energy, payload_weight);

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
    if(is_final != true){
        cost+=kmin;
    }
    return std::make_tuple(np, cost, is_final);
}

//Finds a close to optimal route using the 'Greedy' method
std::tuple <double, double, std::vector<mission_waypoint_t>>
Trajectory::mincost(int position, std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, double cost, int completed[], std::vector<mission_waypoint_t> finalWpsList, double ** cost_array, int n, double energy, double payload_weight)
{
    int nposition;
    bool is_final = false;
    double speed = 5.0;

    //Set the current position as completed
    completed[position]=1;

    std::cout << position << "--->";

    finalWpsList.at(n) = uploadedWpsList[position];

    std::tie(nposition, cost, is_final) = least(position, num_waypoints, completed, cost_array, cost);
    energy += calc_energy_use(uploadedWpsList[position].waypoint.altitude, uploadedWpsList[nposition].waypoint.altitude, uploadedWpsList[position].waypoint.lat, uploadedWpsList[nposition].waypoint.lat, uploadedWpsList[position].waypoint.lon, uploadedWpsList[nposition].waypoint.lon, speed, payload_weight);

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

//Calculate an estimate of the battery percentage used between two waypoints
double
Trajectory::calc_energy_use(double alt1, double alt2, double lat1, double lat2, double lon1, double lon2, double flight_speed, double payload)
{
    double lat1_rad = (lat1/180)*pi;
    double lat2_rad = (lat2/180)*pi;
    double lon1_rad = (lon1/180)*pi;
    double lon2_rad = (lon2/180)*pi;

    double x = sqrt(pow(earth_radius+alt1, 2)+pow(earth_radius+alt2, 2)-2*(earth_radius+alt1)*
                    (earth_radius+alt2)*(sin(lat1_rad)*sin(lat2_rad)+cos(lat1_rad)*cos(lat2_rad)*cos(lon1_rad-lon2_rad)));
    double time = (x)/(flight_speed);
    time += 5; // Approximate time required to deliver and item

    double power = sqrt( (pow(mass+bat_mass+payload,3)*pow(g,3)) / (2*rho*rotor_area) );
    double percent_used = (((power*time) / (bat_energy*3600)) * 100) / (efficiency/100);

    return percent_used;
}

void
Trajectory::update_trajectory(mission_s mission)
{
    vector<mission_waypoint_t> uploadedWpsList;
    vector<mission_waypoint_t> finalWpsList;
    mission_waypoint_t oneWaypoint;
    double** local_cost_array;
    double cost, energy;
    bool write_failed = false;
    int numItems = mission.count;

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
                    finalWpsList.push_back(oneWaypoint);
                    numOfWaypoints++;
                }
            }
        }

        //Check that waypoints have been found
        if (numOfWaypoints > 0){
            //Subtract 1 from the total as one item is the home position
            numOfWaypoints--;
        }
        else {
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
        for (int i = 0; i < numOfWaypoints+1; i++){
            printf("Waypoint %d: Lat: %f Lon: %f nav_cmd: %d\n",
                i, uploadedWpsList[i].waypoint.lat, uploadedWpsList[i].waypoint.lon, uploadedWpsList[i].waypoint.nav_cmd);
        }

        /*** Collate the information needed to create a trajectory ***/

        //Create the 2D dynamic cost array
        local_cost_array = new double * [numOfWaypoints+2];
        for (int i=0; i < numOfWaypoints+1; i++){
            local_cost_array[i] = new double [numOfWaypoints+2];
        }

        //Create the 2D dynamic energy array
        energy_array = new double * [numOfWaypoints+2];
        for (int i=0; i < numOfWaypoints+1; i++){
            energy_array[i] = new double [numOfWaypoints+2];
        }

        //Update takeoff and land position using PX4 rather than MAVSDK
        /* INSERT CODE HERE */

        //Set the last point as the home position
        /* INSERT CODE HERE */

        //Set the starting payload
        double takeoff_weight=0;
        for (int i=0; i<numOfWaypoints+1; i++){
            takeoff_weight += uploadedWpsList[i].waypoint.payload_weight;
        }

        printf("\nThe takeoff weight is: %d kg\n",takeoff_weight);

        /*** Attempt to calculate a minimum cost trajectory ***/

        //Calculate the 2D cost/time array (using spherical polar coordinates)
        local_cost_array = calc_cost(numOfWaypoints, uploadedWpsList);

        //Calculate the optimal trajectory
        printf("Calculating best flight path:\n");

        std::tie (cost, energy, finalWpsList) = call_mincost(uploadedWpsList, numOfWaypoints, finalWpsList, local_cost_array);
        cost += 3*numOfWaypoints; /* What is this ? */

        printf("\nMinimum cost is %f seconds\n", (double) cost);
        printf("Battery usage is %f %\n", (double) energy);

        //Print the final trajectory
        for (int i = 0; i < numOfWaypoints+1; i++){
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
        for (int i=0; i < numOfWaypoints+1; i++){
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
        mission.count = numOfWaypoints+1;
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
