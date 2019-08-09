/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file mission.h
 *
 * Navigator mode to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#pragma once

#include "mission_block.h"
#include "mission_feasibility_checker.h"
#include "navigator_mode.h"

#include <float.h>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <px4_module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/uORB.h>



/*** TRAJ CALC SETUP ***/

#include <string>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

typedef struct mission_waypoint_struct {
        mission_item_s waypoint;
        int originalIndex;
} mission_waypoint_t;


const double earth_radius = 6371000; //metres;
const double pi = 3.1415926;
const double g = 9.81; //ms^-2
const double rho = 1.225; //Density of air in kgm^-3

class DRONE{
  public:
    //Physical drone properties
    double mass;
    int num_rotors;
    double rotor_radius;
    double max_velocity;
    double min_velocity;

    //Battery and electrical properties
    double bat_capacity;
    double bat_voltage;
    double efficiency;

    //Calculate an estimate of the maximum drone flight time in minutes
    double calc_max_flight_time(){
        double rotor_area = num_rotors * (pi * pow(rotor_radius, 2));
        double power = sqrt( (2*pow(mass,3)*pow(g,3)) / (rho*rotor_area) );
        double energy = bat_voltage * bat_capacity * pow(10,-3); //Battery energy in Wh
        double max_flight_time = (energy/power) * 60; //Max flight time in minutes
        return max_flight_time;
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
                for (int t = 0; t < num_waypoints+1; t++){
                    std::cout << cost2d[i][t] << ",  ";
                }
                std::cout << "]" << std::endl;
            }
            std::cout << "" << std::endl;

            return cost2d;
        }

        //Sets up all the variables required for the mincost function, and returns the estimated minimum cost route
        std::tuple<double, std::vector<mission_waypoint_t>> call_mincost(std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, std::vector<mission_waypoint_t> finalWpsList, double ** cost_array){
            double cost = 0;
            int n = 0;
            int completed[num_waypoints+1] = {0};

            return TRAJECTORY::mincost(0, uploadedWpsList, num_waypoints, cost, completed, finalWpsList, cost_array, n);

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
            return {np, cost, is_final};
        }

        //Finds a close to optimal route using the 'Greedy' method
        std::tuple <double, std::vector<mission_waypoint_t>> mincost(int position, std::vector<mission_waypoint_t> uploadedWpsList, int num_waypoints, double cost, int completed[], std::vector<mission_waypoint_t> finalWpsList, double ** cost_array, int n){
            int nposition;
            bool is_final = false;

            completed[position]=1;

            std::cout << position << "--->";

            finalWpsList.at(n) = uploadedWpsList[position];
            n++;

            //nposition = least(position, num_waypoints);
            std::tie(nposition, cost, is_final) = TRAJECTORY::least(position, num_waypoints, completed, cost_array, cost);

            if(is_final == true){
                nposition=0;
                std::cout << nposition;
                cost+=cost_array[position][nposition];
                return {cost, finalWpsList};
            }

            return mincost(nposition, uploadedWpsList, num_waypoints, cost, completed, finalWpsList, cost_array, n);
        }

};

/*** REGULAR CODE ***/




class Navigator;

class Mission : public MissionBlock, public ModuleParams
{
public:
	Mission(Navigator *navigator);
	~Mission() override = default;

	void on_inactive() override;
	void on_inactivation() override;
	void on_activation() override;
	void on_active() override;

	enum mission_altitude_mode {
		MISSION_ALTMODE_ZOH = 0,
		MISSION_ALTMODE_FOH = 1
	};

	bool set_current_mission_index(uint16_t index);

	bool land_start();
	bool landing();

	uint16_t get_land_start_index() const { return _land_start_index; }
	bool get_land_start_available() const { return _land_start_available; }
	bool get_mission_finished() const { return _mission_type == MISSION_TYPE_NONE; }
	bool get_mission_changed() const { return _mission_changed ; }
	bool get_mission_waypoints_changed() const { return _mission_waypoints_changed ; }

	void set_closest_item_as_current();

	/**
	 * Set a new mission mode and handle the switching between the different modes
	 *
	 * For a list of the different modes refer to mission_result.msg
	 */
	void set_execution_mode(const uint8_t mode);
private:

	/**
	 * Update mission topic
	 */
	void update_mission();

	/**
	 * Move on to next mission item or switch to loiter
	 */
	void advance_mission();

	/**
	 * Set new mission items
	 */
	void set_mission_items();

	/**
	 * Returns true if we need to do a takeoff at the current state
	 */
	bool do_need_vertical_takeoff();

	/**
	 * Returns true if we need to move to waypoint location before starting descent
	 */
	bool do_need_move_to_land();

	/**
	 * Returns true if we need to move to waypoint location after vtol takeoff
	 */
	bool do_need_move_to_takeoff();

	/**
	 * Copies position from setpoint if valid, otherwise copies current position
	 */
	void copy_position_if_valid(struct mission_item_s *mission_item, struct position_setpoint_s *setpoint);

	/**
	 * Create mission item to align towards next waypoint
	 */
	void set_align_mission_item(struct mission_item_s *mission_item, struct mission_item_s *mission_item_next);

	/**
	 * Calculate takeoff height for mission item considering ground clearance
	 */
	float calculate_takeoff_altitude(struct mission_item_s *mission_item);

	/**
	 * Updates the heading of the vehicle. Rotary wings only.
	 */
	void heading_sp_update();

	/**
	 * Updates the altitude sp to follow a foh
	 */
	void altitude_sp_foh_update();

	/**
	 * Update the cruising speed setpoint.
	 */
	void cruising_speed_sp_update();

	/**
	 * Abort landing
	 */
	void do_abort_landing();

	/**
	 * Read the current and the next mission item. The next mission item read is the
	 * next mission item that contains a position.
	 *
	 * @return true if current mission item available
	 */
	bool prepare_mission_items(mission_item_s *mission_item,
				   mission_item_s *next_position_mission_item, bool *has_next_position_item);

	/**
	 * Read current (offset == 0) or a specific (offset > 0) mission item
	 * from the dataman and watch out for DO_JUMPS
	 *
	 * @return true if successful
	 */
	bool read_mission_item(int offset, struct mission_item_s *mission_item);

	/**
	 * Save current mission state to dataman
	 */
	void save_mission_state();

	/**
	 * Inform about a changed mission item after a DO_JUMP
	 */
	void report_do_jump_mission_changed(int index, int do_jumps_remaining);

	/**
	 * Set a mission item as reached
	 */
	void set_mission_item_reached();

	/**
	 * Set the current mission item
	 */
	void set_current_mission_item();

	/**
	 * Check whether a mission is ready to go
	 */
	void check_mission_valid(bool force);

	/**
	 * Reset mission
	 */
	void reset_mission(struct mission_s &mission);

	/**
	 * Returns true if we need to reset the mission
	 */
	bool need_to_reset_mission(bool active);

	/**
	 * Project current location with heading to far away location and fill setpoint.
	 */
	void generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw);

	/**
	 * Find and store the index of the landing sequence (DO_LAND_START)
	 */
	bool find_mission_land_start();

	/**
	 * Return the index of the closest mission item to the current global position.
	 */
	int32_t index_closest_mission_item() const;

	bool position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MIS_DIST_1WP>) _param_mis_dist_1wp,
		(ParamFloat<px4::params::MIS_DIST_WPS>) _param_mis_dist_wps,
		(ParamInt<px4::params::MIS_ALTMODE>) _param_mis_altmode,
		(ParamInt<px4::params::MIS_MNT_YAW_CTL>) _param_mis_mnt_yaw_ctl
	)

	uORB::Subscription	_mission_sub{ORB_ID(mission)};		/**< mission subscription */
	mission_s		_mission {};

	int32_t _current_mission_index{-1};

	// track location of planned mission landing
	bool	_land_start_available{false};
	uint16_t _land_start_index{UINT16_MAX};		/**< index of DO_LAND_START, INVALID_DO_LAND_START if no planned landing */

	bool _need_takeoff{true};					/**< if true, then takeoff must be performed before going to the first waypoint (if needed) */

	enum {
		MISSION_TYPE_NONE,
		MISSION_TYPE_MISSION
	} _mission_type{MISSION_TYPE_NONE};

	bool _inited{false};
	bool _home_inited{false};
	bool _need_mission_reset{false};
	bool _mission_waypoints_changed{false};
	bool _mission_changed{false}; /** < true if the mission changed since the mission mode was active */

	float _min_current_sp_distance_xy{FLT_MAX}; /**< minimum distance which was achieved to the current waypoint  */

	float _distance_current_previous{0.0f}; /**< distance from previous to current sp in pos_sp_triplet,
					    only use if current and previous are valid */

	enum work_item_type {
		WORK_ITEM_TYPE_DEFAULT,		/**< default mission item */
		WORK_ITEM_TYPE_TAKEOFF,		/**< takeoff before moving to waypoint */
		WORK_ITEM_TYPE_MOVE_TO_LAND,	/**< move to land waypoint before descent */
		WORK_ITEM_TYPE_ALIGN,		/**< align for next waypoint */
		WORK_ITEM_TYPE_CMD_BEFORE_MOVE,
		WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF,
		WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION,
		WORK_ITEM_TYPE_PRECISION_LAND
	} _work_item_type{WORK_ITEM_TYPE_DEFAULT};	/**< current type of work to do (sub mission item) */

	uint8_t _mission_execution_mode{mission_result_s::MISSION_EXECUTION_MODE_NORMAL};	/**< the current mode of how the mission is executed,look at mission_result.msg for the definition */
	bool _execution_mode_changed{false};
};
