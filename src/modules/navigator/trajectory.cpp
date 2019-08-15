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
#include <px4_module_params.h>

using namespace std;

#include "trajectory.h"

double delivery_time = 5.0; /* How close to reality is this? */

Trajectory::Trajectory()
{
}

//int update_mission(dm_item_t dataman_id, uint16_t count, int32_t seq){
//    mission_s mission{};

//    mission.timestamp = hrt_absolute_time();
//    mission.dataman_id = dataman_id;
//    mission.count = count;
//    mission.current_seq = seq;

//    int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);
//    if (dm_lock_ret != 0) {
//            PX4_ERR("DM_KEY_MISSION_STATE lock failed");
//    }

//    int res = dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));

//    /* unlock MISSION_STATE item */
//    if (dm_lock_ret == 0) {
//            dm_unlock(DM_KEY_MISSION_STATE);
//    }
//    if (res == sizeof(mission_s)) {
//        /* update active mission state */
//        //_dataman_id = dataman_id;
//        //_count[MAV_MISSION_TYPE_MISSION] = count;
//        //_current_seq = seq;
//        //_my_dataman_id = _dataman_id;
//        orb_advert_t _offboard_mission_pub = nullptr;

//        orb_publish(ORB_ID(mission), _offboard_mission_pub, &mission);

//        /* mission state saved successfully, publish offboard_mission topic */
////        if (_offboard_mission_pub == nullptr) {
////                _offboard_mission_pub = orb_advertise(ORB_ID(mission), &mission);

////        } else {
////                orb_publish(ORB_ID(mission), _offboard_mission_pub, &mission);
////        }
//        return PX4_OK;
//    }
//    else {
//        PX4_ERR("WPM: can't save mission state");
//        return PX4_ERROR;
//    }
//    return 0;
//}

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

    double time = (x/flight_speed); // Approximate time required to deliver an item

    /* What is this ?
    if (time >= 2.9 && time <= 3.1){
        return 3;
    }
    */

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

    double time = (x/flight_speed) + delivery_time; // Approximate time required to deliver an item

    double power;
    //Object
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
    power = sqrt( (pow(mass+bat_mass+payload,3)*pow(g,3)) / (2*1.225*0.568489194) ); //Calculate the rotor area?
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

    for (int i = 0; i < num_waypoints; i++){
        std::cout << "[";
        for (int t = 0; t < num_waypoints-1; t++){
            std::cout << cost2d[i][t] << ",  ";
        }
        std::cout << cost2d[i][num_waypoints-1];
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

    for (i=0;i<num_waypoints;i++){
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

trajectory_cost_t
Trajectory::calculateTrajectoryCost(std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, int trajectoryMatrix[], double speedMatrix[]) {
    trajectory_cost_t curTrajCost = {0};
    int j;
    double sumOfMissedDelaysValues = 0;
    double arrival_at_j = 0;
    double trajectory_payload = uploadedWpsList[0].waypoint.payload_weight;

    //printf("Init curTrajCost.requiredEnergy is %f\n",curTrajCost.requiredEnergy);

    for (j=0; j<num_waypoints-1; j++) {
        curTrajCost.requiredEnergy += Trajectory::calc_energy_use(uploadedWpsList[trajectoryMatrix[j]].waypoint, uploadedWpsList[trajectoryMatrix[j+1]].waypoint, speedMatrix[j], trajectory_payload);
        //printf("j = %d curTrajCost.requiredEnergy is %f speedMatrix is %f\n",j,curTrajCost.requiredEnergy,speedMatrix[j]);
        /* Time to fly from j to j+1 */
        arrival_at_j += Trajectory::calc_flight_time(uploadedWpsList[trajectoryMatrix[j]].waypoint, uploadedWpsList[trajectoryMatrix[j+1]].waypoint, speedMatrix[j]);

        if (arrival_at_j > uploadedWpsList[trajectoryMatrix[j+1]].waypoint.deadline) {
            curTrajCost.missedDeadlines++;
            sumOfMissedDelaysValues += arrival_at_j - uploadedWpsList[trajectoryMatrix[j+1]].waypoint.deadline;
        }

        trajectory_payload -= uploadedWpsList[trajectoryMatrix[j+1]].waypoint.payload_weight;
    }

    /* Add energy cost of returning to takeoff -- there is not deadline cost yet*/
    curTrajCost.requiredEnergy += Trajectory::calc_energy_use(uploadedWpsList[trajectoryMatrix[j]].waypoint, uploadedWpsList[num_waypoints].waypoint, speedMatrix[j], trajectory_payload);
    curTrajCost.avgDelay = sumOfMissedDelaysValues / curTrajCost.missedDeadlines;
    // double trajectory_energy = 0;
    // double trajectory_cost = 0;
    // double trajectory_payload = uploadedWpsList[0].waypoint.payload_weight;

    // for (j=0; j<maxLevel-1; j++) {
    //     trajectory_energy += Trajectory::calc_energy_use(uploadedWpsList[trajectory[j]].waypoint, uploadedWpsList[trajectory[j+1]].waypoint, departureSpeed, trajectory_payload);
    //     trajectory_cost += Trajectory::calc_flight_time(uploadedWpsList[trajectory[j]].waypoint, uploadedWpsList[trajectory[j+1]].waypoint, departureSpeed);
    //     trajectory_payload -= uploadedWpsList[trajectory[j+1]].waypoint.payload_weight;
    // }

    // trajectory_energy += Trajectory::calc_energy_use(uploadedWpsList[trajectory[j]].waypoint, uploadedWpsList[maxLevel].waypoint, departureSpeed, trajectory_payload);
    // trajectory_cost += Trajectory::calc_flight_time(uploadedWpsList[trajectory[j]].waypoint, uploadedWpsList[maxLevel].waypoint, departureSpeed);


    return curTrajCost;
}

//Finds a close to optimal route using the 'Simulated Annealing' algorithm
std::tuple <double, double>
Trajectory::solution_sa(std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, int *solutionTrajMatrix)
{
    int curIter, iterationsMax = 100;
    bool terminate = false;
    double cost=0, minEnergy=0, newEnergy=0;
    int random_index, tmpTrajPoint;
    int tmpTrajMatrix[num_waypoints] = {0};
    //double tmpSpeedMatrix[num_waypoints] = {5.0};
    double accProbability, probThreshold=0.5, Temperature=0.5;
    std::vector<mission_waypoint_t> finalWpsList;

    /* Initialise structures */
    std::srand ( unsigned ( std::time(0) ) );
    std::vector<int> tempVector;

    // set some values:
    for (int i=1; i<=num_waypoints; i++) {
        tempVector.push_back(i); // Initialize with the waypoints index
    }

    // using built-in random generator to generate initial point
    std::random_shuffle (tempVector.begin(), tempVector.end());

    tmpTrajMatrix[0] = 0;
    solutionTrajMatrix[0] = 0;
    for (int i=1; i<=num_waypoints; i++) {
        tmpTrajMatrix[i] = tempVector[i];
        solutionTrajMatrix[i] = tempVector[i];
    }

    //std::tie (cost, minEnergy) = Trajectory::calculateTrajectoryCost(uploadedWpsList, num_waypoints, solutionTrajMatrix, tmpSpeedMatrix);

    while (terminate == false) {

        for (curIter=0; curIter < iterationsMax; curIter++) {
            /* Generate new solution */
            random_index = std::rand();
            while ((random_index == 0) || (random_index == num_waypoints)){
                random_index = std::rand();
            }

            /* Swap points */
            tmpTrajPoint = tmpTrajMatrix[random_index];
            tmpTrajMatrix[random_index] = tmpTrajMatrix[random_index+1];
            tmpTrajMatrix[random_index+1] = tmpTrajPoint;

            /* Calculate feasibilit and cost */
            //std::tie (cost, newEnergy) = Trajectory::calculateTrajectoryCost(uploadedWpsList, num_waypoints, tmpTrajMatrix, tmpSpeedMatrix);

            /* Check improvement and acceptance */
            if (newEnergy < minEnergy) {
                minEnergy = newEnergy;

                for (int i=1; i<=num_waypoints; i++) {
                    solutionTrajMatrix[i] = tmpTrajMatrix[i];
                }
            //} else if (newEnergy == minEnergy) {
            } else {
                /* if random [ 0, 1 ] ≥ accProbability then accept -- Don't like that */
                /* Metropolis rule */
                accProbability = exp(-(newEnergy-minEnergy/Temperature));

                if (accProbability > probThreshold) {
                    minEnergy = newEnergy;

                    for (int i=1; i<=num_waypoints; i++) {
                        solutionTrajMatrix[i] = tmpTrajMatrix[i];
                    }
                }
            }
        }

        /* Evaluate termination condition */
        terminate = true;

        /* Adjust temperature */
    }

    /*
    for (int i=0; i<num_waypoints; i++) {
        finalWpsList.push_back(uploadedWpsList[solutionTrajMatrix[i]]); // Initialize with the waypoints index
    }
    */

    return std::make_tuple(cost, minEnergy);
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

//Finds an to optimal route using the 'Brute Force' method -- maxLevel is equal to num_waypoints.
void
Trajectory::solution_bruteforce (int level, int maxLevel, int *trajectory, int *visitedNodes, std::vector<mission_waypoint_t> uploadedWpsList, int *numOfTrajectories, double *departureSpeedMatrix, trajectory_cost_t *solutionTrajCost, int *solutionTrajMatrix) {
	int i, j;
    trajectory_cost_t curTrajCost;

	for (i=0; i<maxLevel; i++) {
		if (visitedNodes[i] == 0) {
			trajectory[level] = i;
			visitedNodes[i] = 1;

			if (level < (maxLevel-1)) {
                    Trajectory::solution_bruteforce (level+1, maxLevel, trajectory, visitedNodes, uploadedWpsList, numOfTrajectories, departureSpeedMatrix, solutionTrajCost, solutionTrajMatrix);
            } else if (level == (maxLevel-1)) {
                (*numOfTrajectories)++; //What is this?

                for (j=0; j<maxLevel-1; j++) {
                    printf("%d--->", trajectory[j]);
                }
                printf("%d--->0 ", trajectory[j]);

                curTrajCost = Trajectory::calculateTrajectoryCost(uploadedWpsList, maxLevel, trajectory, departureSpeedMatrix);

                printf("with energy %f and missed deadlines %d\n", curTrajCost.requiredEnergy, curTrajCost.missedDeadlines);

                if (curTrajCost.requiredEnergy < solutionTrajCost->requiredEnergy) {
                    *solutionTrajCost = curTrajCost;

                    for (j=0; j<maxLevel; j++) {
                        solutionTrajMatrix[j] = trajectory[j];
                    }
                }

			} else {
				printf("Level is wrong. Level %d maxLevel %d\n",level,maxLevel);
			}

			trajectory[level] = 0;
            visitedNodes[i] = 0;
		}
	}
}

//Sets up all the variables required for the mincost function, and returns the estimated minimum cost route
std::tuple<double, double, std::vector<mission_waypoint_t>>
Trajectory::calc_solution(std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints)
{
    int i, numOfTrajectories=0, trajectory[num_waypoints] = {0}, visitedNodes[num_waypoints] = {0}, solutionTrajMatrix[num_waypoints] = {0};
    std::vector<mission_waypoint_t> finalWpsList;
    double departureSpeedMatrix[num_waypoints] = {5.0}, minEnergy = INFINITY, minCost = INFINITY;
    trajectory_cost_t solutionTrajCost = {.requiredEnergy = INFINITY, .missedDeadlines = num_waypoints, .avgDelay = 0.0};

    //Update takeoff and land position using PX4 rather than MAVSDK
    /* INSERT CODE HERE */

    //Set the last point as the home position
    /* INSERT CODE HERE */

    //Set the starting payload
    double takeoff_weight=0;
    for (i=0; i<num_waypoints; i++){
        departureSpeedMatrix[i] = 5.0;
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

    Trajectory::solution_bruteforce (1, num_waypoints, trajectory, visitedNodes, uploadedWpsList, &numOfTrajectories, departureSpeedMatrix, &solutionTrajCost, solutionTrajMatrix);

    printf("\nBrute force evaluated %d routes\n",numOfTrajectories);
    printf("Chosen one is:\n");

    for (i=0; i<num_waypoints; i++) {
        printf("%d--->", solutionTrajMatrix[i]);
        finalWpsList.push_back(uploadedWpsList[solutionTrajMatrix[i]]); // Initialize with the waypoints index
        finalWpsList[i].departureSpeed = departureSpeedMatrix[i];
    }
    printf("0\n");
    
	printf("Energy consumption: %f, number of missed deadlines: %d, Avg Delay %f\n\n",
		solutionTrajCost.requiredEnergy, solutionTrajCost.missedDeadlines, solutionTrajCost.avgDelay);

	finalWpsList.push_back(uploadedWpsList[i]); // Initialize with the waypoints index
    finalWpsList[i].departureSpeed = 0;

    return std::make_tuple(minCost, minEnergy, finalWpsList);

    //int n = 0, visited[num_waypoints] = {0};
    //double cost = 0, energy = 0;
    // double **cost_array;
    // //Calculate the 2D cost/time array (using spherical polar coordinates)
    // cost_array = calc_cost(num_waypoints, uploadedWpsList);
    //return Trajectory::solution_mincost(0, uploadedWpsList, num_waypoints, cost, visited, finalWpsList, cost_array, n, energy, takeoff_weight);
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

    //Find the maximum horizontal speed
    float _param_xy_vel_cruise{0.0f};
    float _param_xy_vel_max{0.0f};
    param_t _handle_param_xy_vel_cruise = param_find("MPC_XY_CRUISE");
    param_t _handle_param_xy_vel_max = param_find("MPC_XY_VEL_MAX");
    param_get(_handle_param_xy_vel_cruise, &_param_xy_vel_cruise);
    param_get(_handle_param_xy_vel_max, &_param_xy_vel_max);
    double max_speed;
    max_speed = std::min(_param_xy_vel_cruise, _param_xy_vel_max);
    printf("Max speed is %f\n", max_speed);

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

        printf("\nCalculating best flight path:\n");

        /* Adding return to takeoff */
        uploadedWpsList.push_back(uploadedWpsList[0]);
        uploadedWpsList[numOfWaypoints].waypoint.payload_weight = 0;

        /* Calculate the optimal trajectory */
        std::tie (cost, energy, finalWpsList) = calc_solution(uploadedWpsList, numOfWaypoints);
        numOfWaypoints++;

        //printf("\nMinimum cost is %f seconds\n", cost);
        //printf("Battery usage is %f %\n", (double) energy);

        //Print the final trajectory
        for (int i = 0; i < numOfWaypoints; i++){
            printf("Waypoint %d: Lat: %f Lon: %f nav_cmd: %d\n",
                i, finalWpsList[i].waypoint.lat, finalWpsList[i].waypoint.lon, finalWpsList[i].waypoint.nav_cmd);
        }

        //Ensure the flight time is within the maximum possible flight time
        /* INSERT CODE HERE */

        //Clear previous items from dataman
        //update_mission((dm_item_t)mission.dataman_id, mission.count, mission.current_seq);

        //dm_lock(DM_KEY_WAYPOINTS_OFFBOARD_0);
        //dm_lock(DM_KEY_WAYPOINTS_OFFBOARD_1);
        //dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_0);
        //dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_1);
        //dm_clear(DM_KEY_WAYPOINTS_OFFBOARD_1);
        //dm_unlock(DM_KEY_WAYPOINTS_OFFBOARD_0);
        //dm_unlock(DM_KEY_WAYPOINTS_OFFBOARD_1);

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
        printf("\nFinal Trajectory:\n");
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
