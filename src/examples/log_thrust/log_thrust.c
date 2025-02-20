/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>

void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;

    // Stroing start time
    clock_t start_time = clock();

    // looping till required time is not acheived
    while (clock() < start_time + milli_seconds)
        ;
}

__EXPORT int log_thrust_main(int argc, char *argv[]);

int log_thrust_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");

    /* subscribe to sensor_combined topic */
    int actuator_sub_fd = orb_subscribe(ORB_ID(actuator_outputs));
    int armed_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
    /* limit the update rate to 5 Hz */
    orb_set_interval(actuator_sub_fd, 200);
    orb_set_interval(armed_sub_fd, 200);

    /* advertise actuator topic */
    struct actuator_outputs_s act;
    struct actuator_armed_s arm;
    memset(&act, 0, sizeof(act));
    memset(&arm, 0, sizeof(arm));
    //orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_outputs), &act);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = actuator_sub_fd, .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
    };

    int error_counter = 0;

    FILE *fptr;
    fptr = fopen("/home/joestory/src/Firmware/src/examples/log_thrust/logged_data.txt", "w");
    if(fptr == NULL){
        printf("Error! File not opened ");
        printf("");
        return(1);
    }

    fprintf(fptr, "%s", "Body Thrust: ");

    //Check to see if the drone is armed
    orb_copy(ORB_ID(actuator_armed), armed_sub_fd, &arm);

    while (arm.armed == false){
        PX4_INFO("You are not armed!");
        delay(1000);
        orb_copy(ORB_ID(actuator_armed), armed_sub_fd, &arm);
    }


    while (arm.armed == true) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);
        orb_copy(ORB_ID(actuator_armed), armed_sub_fd, &arm);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {

            if (fds[0].revents & POLLIN) {
                struct actuator_outputs_s raw_act;
                orb_copy(ORB_ID(actuator_outputs), actuator_sub_fd, &raw_act);
                PX4_INFO("Thrust:\t%8.4f,\t%8.4f,\t%8.4f,\t%8.4f",
                         (double)raw_act.output[0],
                         (double)raw_act.output[1],
                         (double)raw_act.output[2],
                         (double)raw_act.output[3]);

                fprintf(fptr, "%f , %f , %f , %f ,\n", (double)raw_act.output[0], (double)raw_act.output[1], (double)raw_act.output[2], (double)raw_act.output[3]);

//                act.control[1] = raw_act.control[1];
//                act.control[2] = raw_act.control[2];
//                act.control[3] = raw_act.control[3];

                //orb_publish(ORB_ID(actuator_outputs), act_pub, &act);

            }

            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */
        }
    }

    PX4_INFO("exiting");

    fclose(fptr);

    return 0;
}
