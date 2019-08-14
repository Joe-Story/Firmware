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

#include "trajectory.h"
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

//Declare cost function variables and class objects
int mission_iterator = 0;
double cost = 0;
int n=0;
int numOfWaypoints;
DRONE drone;
TRAJECTORY trajectory;

//Declare dynamic cost and energy array
double ** cost_array;
double ** energy_array;

/*** END OF SETUP ***/



void update_trajectory(mission_s mission = _mission){
    /***  INSERT MISSION PLANNING CODE HERE  ***/

    //Create vectors to contain the PX4 waypoint structure
    vector<mission_waypoint_t> uploadedWpsList;
    vector<mission_waypoint_t> finalWpsList;
    mission_waypoint_t oneWaypoint;
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
        std::cout.precision(8);
        for (int i=0; i < numItems; i++){
            std::cout << "Index: " << i << ", Command: " << uploadedWpsList[i].waypoint.nav_cmd << ", Lat: " << uploadedWpsList[i].waypoint.lat << std::endl;
        }

        //Print the number of waypoints received
        std::cout << "" << std::endl;
        printf("Num of Waypoints is: %d\n", numOfWaypoints);

        //Print the received waypoints
        for (int i = 0; i < numOfWaypoints+1; i++){
            printf("Waypoint %d: Lat: %f Lon: %f nav_cmd: %d\n",
                i, uploadedWpsList[i].waypoint.lat, uploadedWpsList[i].waypoint.lon, uploadedWpsList[i].waypoint.nav_cmd);
        }

        bool write_failed = false;



        /*** Collate the information needed to create a trajectory ***/

        //Get Drone Information
        /* INSERT CODE HERE */

        //Create the 2D dynamic cost array
        cost_array = new double * [numOfWaypoints+2];
        for (int i=0; i < numOfWaypoints+1; i++){
            cost_array[i] = new double [numOfWaypoints+2];
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
        std::cout << "The takeoff weight is: " << takeoff_weight << "kg" << std::endl;

        /*** Attempt to calculate a minimum cost trajectory ***/

        //Calculate the 2D cost/time array (using spherical polar coordinates)
        std::cout << "This program minimises the number of missed deadlines" << std::endl;
        std::cout << "" << std::endl;
        std::cout.precision(3);
        cost_array = trajectory.calc_cost(numOfWaypoints, uploadedWpsList);
        std::cout.precision(8);
        double energy;

        //Calculate the optimal trajectory
        std::cout << "Calculating best flight path:" << std::endl;
        std::cout << "" << std::endl;
        std::tie (cost, energy, finalWpsList) = trajectory.call_mincost(uploadedWpsList, numOfWaypoints, finalWpsList, cost_array);
        cost += 3*numOfWaypoints;
        //ADD TAKEOFF ENERGY?
        std::cout << "\n\nMinimum cost is " << cost << " seconds" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "Battery usage is " << energy << "%" << std::endl;
        std::cout << "" << std::endl;

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
        std::cout << "" << std::endl;
        std::cout << "Final Trajectory:" << std::endl;
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

        /***  END INSERTED CODE  ***/
}
