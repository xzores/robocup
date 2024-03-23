#include "furbs_control.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <cmath>

#include "simu.h"
#include "sedge.h"
#include "uservice.h"
#include "cmixer.h"

// create value
Furbs furbs;

void Furbs::setup () {
	
	if (not ini.has("postion")) { // no data yet, so generate some default values

		ini["postion"]["max_acc"] = "1.0";
		ini["postion"]["max_vel"] = "0.5";
		ini["postion"]["time_interval"] = "0.05";
		ini["postion"]["dist_margin"] = "0.05";
		ini["postion"]["min_vel"] = "0.03";

		// get values from ini-file
		ini["postion"]["heading_vel"] = "0.01";
		ini["postion"]["heading_threshold"] = "20";
		ini["postion"]["heading_threshold"] = "0.5";
	}

	vel.max_acc 		= strtof(ini["postion"]["max_acc"].c_str(), nullptr);
	vel.max_vel 		= strtof(ini["postion"]["max_vel"].c_str(), nullptr);
	vel.time_interval = strtof(ini["postion"]["time_interval"].c_str(), nullptr);
	vel.dist_margin 	= strtof(ini["postion"]["dist_margin"].c_str(), nullptr);
	vel.min_vel 		= strtof(ini["postion"]["min_vel"].c_str(), nullptr);

	vel.heading_vel 			= strtof(ini["postion"]["heading_vel"].c_str(), nullptr);
	
	heading_threshold 		= strtof(ini["postion"]["heading_threshold"].c_str(), nullptr);
	heading_buildup_remove 	= strtof(ini["postion"]["heading_buildup_remove"].c_str(), nullptr);
}

void Furbs::terminate () {
	//TODO
}

void Furbs::go_for (float meters, Linemode lm, float start_off, float end_off, Furbs_vel_params p) {
	
	bool backwards = false;
	if (meters < 0) {
		backwards = true;
	}
	meters = abs(meters);

	float cur_vel = 0;
	float target_vel = p.max_vel;
	float start[2] = {pose.x, pose.y};
	float start_dist = pose.dist;
	float dist = 0;
	
	float h = pose.h;

	float t = (meters / dist);
	float cur_off = (t * end_off) + (t - 1) * start_off;

	if (lm == left_line_mode) {
		mixer.setEdgeMode(true, p.left_line_offset + cur_off);
	}
	else if (lm == right_line_mode) {
		mixer.setEdgeMode(false, p.right_line_offset + cur_off);
	}
	else {
		mixer.setDesiredHeading(h);
	}

	while (true) {

		///////////////////////// Distance Calculation /////////////////////////
		//TODO make it intergrating instead of abseluote
		//dist = sqrt((start[0] - pose.x)*(start[0] - pose.x) + (start[1] - pose.y)*(start[1] - pose.y));
		dist = abs(pose.dist - start_dist);

		// Calculate the stopping distance
		float stopping_distance = cur_vel * cur_vel / (2 * p.max_acc);

		//The distance it will take to reach 0 m/s. A dist_margin is added so it can slow down beforehand.
		if ((meters - dist - p.dist_margin) <= stopping_distance) {
			target_vel = 0;
		}
		
		if (cur_vel < target_vel) {
			cur_vel += p.max_acc * p.time_interval;
		}
		else if (cur_vel > target_vel) {
			cur_vel -= p.max_acc * p.time_interval;
		}
		
		cur_vel = fmax(p.min_vel, cur_vel);

		if (backwards) {
			mixer.setVelocity(-cur_vel);
		}
		else {
			mixer.setVelocity(cur_vel);
		}
		///////////////////////// Time and ending /////////////////////////
		float time_interval_usec = p.time_interval * 1000.0f * 1000.0f;
		usleep((useconds_t)time_interval_usec); //ms before updating velocity and heading
		printf("dist, cur_vel, target_vel,  %f, %f, %f\n", dist, cur_vel, target_vel);
		//printf("edge raw : %i, %i, %i, %i, %i, %i, %i, %i \n", sedge.edgeRaw[0], sedge.edgeRaw[1], sedge.edgeRaw[2], sedge.edgeRaw[3], sedge.edgeRaw[4], sedge.edgeRaw[5], sedge.edgeRaw[6], sedge.edgeRaw[7]);

		if (dist >= meters) {
			mixer.setVelocity(0);
			printf("Final dist, cur_vel, target_vel,  %f, %f, %f\n", dist, cur_vel, target_vel);
			break;
		}
	}
}

void Furbs::turn (float t, Furbs_vel_params p) {


	float start_heading = pose.h;
	float heading = start_heading;
	float target = start_heading + t * 3.14f / 180.0f;

	mixer.setDesiredHeading(start_heading);

	while (true) {

		heading = abs(pose.h - start_heading);
		
		if (heading < target) {
			heading += p.heading_vel * p.time_interval;
		}
		else if (heading > target) {
			heading -= p.heading_vel * p.time_interval;
		}
		
		mixer.setDesiredHeading(heading);

		///////////////////////// Time and ending /////////////////////////
		float time_interval_usec = p.time_interval * 1000.0f * 1000.0f;
		usleep((useconds_t)time_interval_usec); //ms before updating velocity and heading
		printf("start_heading, heading,  %f, %f\n", start_heading, heading);
		//printf("edge raw : %i, %i, %i, %i, %i, %i, %i, %i \n", sedge.edgeRaw[0], sedge.edgeRaw[1], sedge.edgeRaw[2], sedge.edgeRaw[3], sedge.edgeRaw[4], sedge.edgeRaw[5], sedge.edgeRaw[6], sedge.edgeRaw[7]);

		if (abs(pose.h - target) < 0.001) {
			mixer.setDesiredHeading(heading);
			break;
		}
	}
}

/*
void Furbs::go_for_line (float meters, bool follow_line, Furbs_vel_params p) {
	
	float cur_vel = 0;
	float target_vel = p.max_vel;
	float start[2] = {pose.x, pose.y};
	float dist = 0;
	float heading = mixer.desiredHeading;
	float heading_buildup = 0.0f;
	
	int init_left_sum_int = sedge.edgeRaw[0] + sedge.edgeRaw[1] + sedge.edgeRaw[2] + sedge.edgeRaw[3];
	int init_right_sum_int = sedge.edgeRaw[4] + sedge.edgeRaw[5] + sedge.edgeRaw[6] + sedge.edgeRaw[7];
	
	while (true) {

		///////////////////////// Distance Calculation /////////////////////////
		//TODO make it intergrating instead of abseluote
		dist = sqrt((start[0] - pose.x)*(start[0] - pose.x) + (start[1] - pose.y)*(start[1] - pose.y));

		// Calculate the stopping distance
		float stopping_distance = cur_vel * cur_vel / (2 * p.max_acc);

		//The distance it will take to reach 0 m/s. A dist_margin is added so it can slow down beforehand.
		if ((meters - dist - p.dist_margin) <= stopping_distance) {
			target_vel = 0;
		}

		if (cur_vel < target_vel) {
			cur_vel += p.max_acc * p.time_interval;
		}
		else if (cur_vel > target_vel) {
			cur_vel -= p.max_acc * p.time_interval;
		}
		
		cur_vel = fmax(p.min_vel, cur_vel);

		mixer.setVelocity(cur_vel);
		
		///////////////////////// Heading calculation /////////////////////////

		int left_sum_int = sedge.edgeRaw[0] + sedge.edgeRaw[1] + sedge.edgeRaw[2] + sedge.edgeRaw[3] - init_left_sum_int;
		int right_sum_int = sedge.edgeRaw[4] + sedge.edgeRaw[5] + sedge.edgeRaw[6] + sedge.edgeRaw[7] - init_right_sum_int;
		
		float left_sum = (float)left_sum_int;
		float right_sum = (float)right_sum_int;

		if (left_sum - right_sum > heading_threshold) {
			heading += heading_vel * p.time_interval;
			heading_buildup += heading_vel * p.time_interval;
		}
		if (right_sum - left_sum > heading_threshold) {
			heading -= heading_vel * p.time_interval;
			heading_buildup += heading_vel * p.time_interval;
		}

		heading -= heading_buildup_remove * heading_buildup * p.time_interval;
		heading_buildup -= heading_buildup_remove * heading_buildup * p.time_interval;

		mixer.setDesiredHeading(heading);

		///////////////////////// Time and ending /////////////////////////
		float time_interval_usec = p.time_interval * 1000.0f * 1000.0f;
		usleep((useconds_t)time_interval_usec); //ms before updating velocity and heading
		printf("dist, cur_vel, target_vel,  %f, %f, %f\n", dist, cur_vel, target_vel);
		printf("left_sum, right_sum, heading,  %f, %f, %f\n", left_sum, right_sum, heading);
		printf("edge raw : %i, %i, %i, %i, %i, %i, %i, %i \n", sedge.edgeRaw[0], sedge.edgeRaw[1], sedge.edgeRaw[2], sedge.edgeRaw[3], sedge.edgeRaw[4], sedge.edgeRaw[5], sedge.edgeRaw[6], sedge.edgeRaw[7]);

		if (dist >= meters) {
			mixer.setVelocity(0);
			usleep(1000*1000);
			printf("Final dist, cur_vel, target_vel,  %f, %f, %f\n", dist, cur_vel, target_vel);
			break;
		}
	}
}
*/

void Furbs::go_to (float x, float y, Furbs_vel_params p) {
	
	float cur_vel = 0;
	float target_vel = p.max_vel;
	float start[2] = {pose.x, pose.y};
	float start_dist = pose.dist;
	float dist = 0;
	float heading = pose.h;
	
	float total_dist = sqrt((x - start[0])*(x - start[0]) + (y - start[1])*(y - start[1]));
	
	while (true) {

		///////////////////////// Distance Calculation /////////////////////////
		//TODO make it intergrating instead of abseluote
		dist = pose.dist - start_dist;

		// Calculate the stopping distance
		float stopping_distance = cur_vel * cur_vel / (2 * p.max_acc);

		//The distance it will take to reach 0 m/s. A dist_margin is added so it can slow down beforehand.
		if ((total_dist - dist - p.dist_margin) <= stopping_distance) {
			target_vel = 0;
		}

		if (cur_vel < target_vel) {
			cur_vel += p.max_acc * p.time_interval;
		}
		else if (cur_vel > target_vel) {
			cur_vel -= p.max_acc * p.time_interval;
		}
		
		cur_vel = fmax(p.min_vel, cur_vel);
		
		mixer.setVelocity(cur_vel);
		
		///////////////////////// Heading Calculation /////////////////////////
		//float t = dist / total_dist;
		//float target_heading = (heading * t) + (start_heading * (1-t)); //linear interpolation
		//mixer.setDesiredHeading(target_heading);
		//float dir[2] = {sin(pose.h), cos(pose.h)};

		float target_heading = atan2(y - pose.y, x - pose.x);

		if (heading < target_heading) {
			heading += p.heading_vel * p.time_interval;
		}
		if (heading > target_heading) {
			heading -= p.heading_vel * p.time_interval;
		}

		mixer.setDesiredHeading(heading);	

		///////////////////////// Time and ending /////////////////////////
		float time_interval_usec = p.time_interval * 1000.0f * 1000.0f;
		usleep((useconds_t)time_interval_usec); //ms before updating velocity and heading
		//printf("dist, cur_vel, target_vel,  %f, %f, %f\n", dist, cur_vel, target_vel);
		//printf("heading, target_heading, pose.x, pose.y,  %f, %f, %f, %f\n", heading, target_heading, pose.x, pose.y);

		if (dist >= total_dist) {
			mixer.setVelocity(0);
			//printf("Final dist, cur_vel, target_vel,  %f, %f, %f\n", dist, cur_vel, target_vel);
			break;
		}
	}
}


