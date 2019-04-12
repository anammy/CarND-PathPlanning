#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  //Car's starting lane (middle)
  int car_lane = 1;

  //Initial Reference velocity
  double ref_vel = 0;

  h.onMessage([&car_lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

	  int path_size = previous_path_x.size();
  
	  if (path_size > 0) {
              car_s = end_path_s;
          }

	  //Prediction

	  //Flags
	  int flag_SlowCarAhead = 0;
	  int flag_CarToLeft = 0;
	  int flag_CarToRight = 0;
	  //Surrounding vehicles' s coordinate in frenet coordinates. Initialized to a very large number.
	  double right_veh_s = 100000;
          double left_veh_s = 100000;
	  double ahead_veh_s = 100000;
	  
	  double slowcar_speed = 0; //Speed of slow car ahead
	  double PROX_THRES = 30; //Threshold for distance to other cars
	  for (int i=0; i < sensor_fusion.size(); i++) {
		int veh_id = sensor_fusion[i][0];
		double veh_x = sensor_fusion[i][1];
		double veh_y = sensor_fusion[i][2];
		double veh_vx = sensor_fusion[i][3];
		double veh_vy = sensor_fusion[i][4];
		double veh_s = sensor_fusion[i][5];
		double veh_d = sensor_fusion[i][6];
		//Total vehicle speed
		double veh_v = sqrt(pow(veh_vx,2) + pow(veh_vy,2));

		//Check which lane the other vehicles are in
		int veh_lane = -1;
		if (veh_d > 0 && veh_d < 4){
			veh_lane = 0;
		} else if (veh_d > 4 && veh_d < 8){
			veh_lane = 1;
		} else if (veh_d > 8 && veh_d < 12){
			veh_lane = 2;
		}

		if (veh_lane == -1){
			continue;
		}

		//Project the other car's s-position ahead in time to previous path end
		veh_s = veh_s + veh_v*((double)0.02*path_size);

		//Check if the other car is ahead of us, to the left, or to the right
		if ((veh_lane == car_lane) && (veh_s > car_s) && ((veh_s - car_s) < PROX_THRES) && (veh_v < car_speed)){
			flag_SlowCarAhead = 1;
			slowcar_speed = veh_v;
			ahead_veh_s = veh_s; 
		}

		if ((veh_lane > car_lane) && (abs(veh_s - car_s) < PROX_THRES)){
                        flag_CarToRight = 1;
		} else if ((veh_lane > car_lane) && (veh_s > car_s) && (abs(veh_s - car_s) > (PROX_THRES-10))) {
			right_veh_s = veh_s;
		}

                if ((veh_lane < car_lane) && (abs(veh_s - car_s) < PROX_THRES)){
                        flag_CarToLeft = 1;
                } else if ((veh_lane < car_lane) && (veh_s > car_s) && (abs(veh_s - car_s) < (PROX_THRES-10))) {
			left_veh_s = veh_s;
		}
		                                               	 
	  }

	  //Behaviour Planning
	  double MAX_SPEED = 49.5; //Max. speed in mph
	  double MAX_ACC = 0.224; //Max. acceleration - equivalent to 5m/s increase in speed
	  double car_accel = 0; //Car's acceleration
	  double FORW_PROG = 10; //Forward progress threshold for lane change. Change lanes if the lane changes allows for significant forward progress (i.e. > FORW_PROG) than staying in current lane

	  if (flag_SlowCarAhead == 1){
		//Tie breaker if both lanes are available for lane change from the middle lane
		if ((flag_CarToLeft == 0) && (flag_CarToRight == 0) && (car_lane == 1)) {
			if ((left_veh_s > right_veh_s) && ((left_veh_s - ahead_veh_s) > FORW_PROG)) {
				//if left lane allows more forward progress, lane change left
				car_lane--;
			} else if ((left_veh_s <= right_veh_s) && ((right_veh_s - ahead_veh_s) > FORW_PROG)) {
				//if right lane allows more forward progress, lane change right
				car_lane++;
			}

		} else {
			if ((flag_CarToLeft == 0) && (car_lane > 0) && ((left_veh_s - ahead_veh_s) > FORW_PROG)){
				//Lane change to the left
				car_lane--;
			} else if ((flag_CarToRight == 0) && (car_lane < 2)  && ((right_veh_s - ahead_veh_s) > FORW_PROG)){
				//Lane change to the right
				car_lane++;
			} else {
				//If unsafe to make a lane change, slow down and match speed of vehicle ahead
				if (ref_vel > slowcar_speed) {
					car_accel = (slowcar_speed - ref_vel);
					if (abs(car_accel) > MAX_ACC){
						car_accel = -MAX_ACC;
					}
				} else {
					car_accel = 0;
				}
			}
		}
	  } else {
		if (car_lane != 1) { //Return to center lane when safe to change lanes
			if ((car_lane == 0 && flag_CarToRight == 0) || (car_lane == 2 && flag_CarToLeft == 0)) {
				car_lane = 1;
			}
		}

		//Accelerate if driving lower than max speed and there are no cars ahead
		if (ref_vel < MAX_SPEED){
			car_accel = MAX_ACC;
		}
	  }

	  //Trajectory Generation

	  /*
	  //Trial code from class
	  double pos_x;
	  double pos_y;
	  double angle;

	  for (int i = 0; i < path_size; ++i) {
  		next_x_vals.push_back(previous_path_x[i]);
  		next_y_vals.push_back(previous_path_y[i]);
	  }

	  if (path_size == 0) {
  		pos_x = car_x;
  		pos_y = car_y;
  		angle = deg2rad(car_yaw);
	  } else {
  		pos_x = previous_path_x[path_size-1];
  		pos_y = previous_path_y[path_size-1];

  		double pos_x2 = previous_path_x[path_size-2];
  		double pos_y2 = previous_path_y[path_size-2];
  		angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
	  }
	
	  double dist_inc = 0.5;
	  for (int i = 0; i < 50-path_size; ++i) {    
 		next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
  		next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
  		pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
  		pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
	  }

	  double dist_inc = 0.5;
	  for (int i = 0; i < 50; ++i) {
		double next_s = car_s + dist_inc*(i+1);
		double next_d = (double)car_lane*4 - 2;
		vector<double> next_xypts = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		next_x_vals.push_back(next_xypts[0]);
		next_y_vals.push_back(next_xypts[1]);
	  }

	 */

	  //FROM UDACITY Q&A SESSION:
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Use previous points to ensure smooth trajectory generation
          if ( path_size < 2 ) {
		//If there are less than two points in the previous path list, project the car pose back one step
          	double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
          } else {
                // Use the last two points
                ref_x = previous_path_x[path_size - 1];
                ref_y = previous_path_y[path_size - 1];

                double ref_x_prev = previous_path_x[path_size - 2];
                double ref_y_prev = previous_path_y[path_size - 2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
          }

            // Setting up target points in the future.
          vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*car_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

            //Transform global coordinates to local car coordinates.
          for ( int i = 0; i < ptsx.size(); i++ ) {
          	double shift_x = ptsx[i] - ref_x;
          	double shift_y = ptsy[i] - ref_y;

           	ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
          	ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create the spline.
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Use last two points from previous path to ensure smooth transition
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for ( int i = 0; i < path_size; i++ ) {
          	next_x_vals.push_back(previous_path_x[i]);
            	next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate x, y trajectory points using spline
	  // Calculate y point using spline at x = 30 m ahead 
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

	  //Accelerate or decelerate the car (ref_vel)
          for( int i = 1; i < 50 - path_size; i++ ) {
          	ref_vel += car_accel;
          	if ( ref_vel > MAX_SPEED ) {
          		ref_vel = MAX_SPEED; //Stay under the speed limit
          	} else if ( ref_vel < MAX_ACC ) {
          		ref_vel = MAX_ACC;  //Don't stop on the highway
          }
          double N = target_dist/(0.02*ref_vel/2.24);
          double x_point = x_add_on + target_x/N;
          double y_point = s(x_point);

          x_add_on = x_point;

          double x_ref = x_point;
          double y_ref = y_point;

          x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
          y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

          x_point += ref_x;
          y_point += ref_y;

          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
