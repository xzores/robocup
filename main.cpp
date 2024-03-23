
/*
 #***************************************************************************
 #*   Copyright (C) 2023 by DTU
 #*   jcan@dtu.dk
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */

// System libraries
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <cmath>

//
// include local files for data values and functions
#include "uservice.h"
#include "cmixer.h"
#include "sgpiod.h"
#include "bplan20.h"
#include "bplan21.h"
#include "bplan40.h"
#include "bplan100.h"
#include "bplan101.h"
#include "medge.h"
#include "cedge.h"
#include "bplan_test_move.h"
#include "furbs_control.h"

#include "simu.h"
#include "sedge.h"
#include "sdist.h"


#define mes_dist(x) filter_dist = 0; for (int i = 0; i < sampels; i++) { filter_dist += dist.dist[0]/sampels; usleep(filter_dist_wait); printf("filter_dist %i : %f\n", x, filter_dist); }

int main (int argc, char **argv)
{

	// prepare all modules and start data flow
	// but also handle command-line options
	service.setup(argc, argv);
	imu.setup();
	furbs.setup();
	sedge.setup();
	mixer.setup();
	cedge.setup();
	
	int sampels = 10;
	float filter_dist = 0;
	float target_dist = 0.1;
	float float_mes_dist = 0;

	int filter_dist_wait = 21*0000;

	if (not service.theEnd) { 

		gpio.setPin(16, 1);
		{
			auto p = furbs.vel;
			furbs.go_for(3.73, left_line_mode, 0, 0.03, p);
			furbs.go_for(0.50, no_line_mode, 0.03, 0.03, p);
			p.max_vel -= 0.1; //slow down a bit
			furbs.go_for(1.95, left_line_mode, 0.03, 0, p);
			p.max_vel += 0.1; //regain speed
			
			mes_dist(1);
			
			//Wait till the thing comes by
			while (filter_dist > 0.3) {
				mes_dist(2);
			}

			//Do a distance meassture
			//how far are we away
			mes_dist(3);
			float_mes_dist = filter_dist;
			
			//Go closer
			furbs.go_for(float_mes_dist-target_dist, left_line_mode, 0, 0, p);

			//Now we wait for the thing to go by again
			while (filter_dist > target_dist + 0.1) {
				mes_dist(4);
				float_mes_dist = filter_dist;
			}
			//Once the dist becomes far we go fast
			while (filter_dist < 0.5) {
				mes_dist(5);
			}
			//usleep(1*1000*1000);
			p.max_acc += 0.2;
			p.max_vel += 0.2;
			furbs.go_for(0.5, left_line_mode, 0, 0, p);
			p.max_acc -= 0.2;
			p.max_vel -= 0.2;
			
			furbs.go_for(1.3, left_line_mode, 0, 0, p);
			p.max_acc -= 0.2;
			p.max_vel -= 0.2;
			furbs.go_for(1, right_line_mode, 0, 0, p);
			p.max_acc += 0.2;
			p.max_vel += 0.2;
			furbs.go_for(4, right_line_mode, 0, 0, p);
		}
		gpio.setPin(16, 0);
	
	}

	// close all logfiles etc.
	service.terminate();
	return service.theEnd;
}

/*
[service]
use_robot_hardware = true

[edge]
rate_ms = 8
highpower = true
lograw = true
printraw = false
calibwhite = 0 0 1 0 1 0 0 0
calibblack = 25 28 31 34 35 33 28 29
whitethreshold = 500
sensorwidth = 0.14
log = true
lognorm = true
print = false
kp = 85.0
lead = 0.2 1.2
taui = 0.00001
logcedge = true
logctrl = false
printctrl = false
maxturnrate = 5.0


[dist]
rate_ms = 20
ir13cm = 47870 70000
ir50cm = 15911 20000
uscalib = 0.00126953125
log = true
print = false
sensor1 = sharp
sensor2 = sharp

[postion]
max_acc = 0.6
max_vel = 0.35
time_interval = 0.05
dist_margin = 0.03
min_vel = 0.05
heading_vel = 0.05
heading_threshold = 50
heading_buildup_remove = 0.5

[heading]
kp = 65.0
lead = 0.5 1.0
taui = 0.000001
maxturnrate = 3.0
log = true
print = false


[motor]
kp = 8.0
lead = 0.0 1.0
taui = 0.05
maxmotv = 10.0
log = true
print_m1 = false
print_m2 = false

[encoder]
rate_ms = 8
log = true
print = false
encoder_reversed = true

[pose]
gear = 19.0
wheeldiameter = 0.146
enctickperrev = 64
wheelbase = 0.28
log = true
print = false

[camera]
device = 0
width = 1280
height = 720
fps = 25
matrix = 1000 0 640 0 1000 360 0 0 1
distortion = -0.415 0.2244 -6.875e-5 0.001279 -0.073412
imagepath = img
imagename = %d
log = true
print = false
enabled = true
pos = 0.11 0 0.23
cam_tilt = 0.01
*/