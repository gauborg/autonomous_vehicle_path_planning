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
using std::endl;


int main()
{
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
  while (getline(in_map_, line))
  {
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

  // start in lane 1;
  int ego_lane = 1;

  // set a target velocity
  double ego_vel = 0.0; // mph


  h.onMessage([&ego_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

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

          // get the previous path size
          // we do this to have a smooth transition between waypoints
          int prev_size = previous_path_x.size();

          // here we will calculate the distance of other cars in our lane
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          
          // ---------------- Track other vehicles and adjust behavior ----------------- //
          
          /*

          Here, we track speeds of different vehicles and calculate their
          future s co-ordinates based on current speed so as to make lane
          change decisions.
          We decide to make lane changes only when the vehicle behind is
          going slower and when it is sufficiently behind the ego vehicle.

          To debug this part of the code, I have written status messages to
          be displayed to the output screen, whenever the ego vehicle detects
          a slow moving vehicle and needs to change lane.

          */
          
          // ----------------------- TRACKING OTHER VEHICLES ------------------------- //
          
          // Track slow moving car in ego vehicle's path
          bool car_in_lane = false;

          // parameters to check cars in left and right lanes
          bool left_lane_car = false;
          bool right_lane_car = false;

          // iterate through all the other cars on the track and get their all
          // properties such as d co-ordinate (lane), s co-ordinate and speed

          // vector<double> costs;

          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            // d co-ordinate of the i'th car
            float d = sensor_fusion[i][6];

            // default car lane in case the vehicle is not moving in the direction of the ego vehicle
            int car_lane = -1;

            // compare the d coordinate of the other vehicle to check its lane
            if (d > 0 && d < 4)
            {
              // leftmost lane
              car_lane = 0;
            }
            else if (d > 4 && d < 8)
            {
              // second from the leftmost lane
              car_lane = 1;
            }
            else if (d > 8 && d < 12)
            {
              // third from the leftmost lane
              car_lane = 2;
            }

            // applies for incoming traffic, doesn't concern our lane change decision making
            if (d < 0)
            {
              continue;
            }
            else
            {
              // get the velocities of the i car in x and y directions
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];

              // get the speed value using the distance formula
              double check_speed = sqrt(vx*vx + vy*vy);
              // check the car's s value
              double check_car_s = sensor_fusion[i][5];

              // if using previous points, can project s value
              check_car_s += ((double)prev_size*0.02*check_speed);

              /*
              We will try using some cost functions here to determine the best lane
              by looking at the locations of other vehicles ahead of ego vehicle.
              This can help us in situations where we have multiple vehicles ahead of us
              and we have to decide the best possible lane for overtaking.
              */

              /*

              if ((check_car_s > car_s) && (car_lane == ego_lane))
              {
                

              }
              
              */

              // check for cars in the vicinity of ego vehicle for which we may need to
              // predict future behavior planning
              if (car_lane == ego_lane)
              {
                // a car is detected in our lane
                car_in_lane |= (check_car_s > car_s) && (check_car_s - car_s < 30);
                if(car_in_lane)
                {
                  std::cout<<"Slow moving vehicle in current lane!"<<std::endl;
                }
              }
              else if (car_lane - ego_lane == 1)
              {
                // car in right lane
                right_lane_car |= (car_s - 30 < check_car_s) && (car_s + 30 > check_car_s);
              }
              else if (car_lane - ego_lane == -1)
              {
                // car in left lane
                left_lane_car |= (car_s - 30 < check_car_s) && (car_s + 30 > check_car_s);
              }
            }   // end of for loop
          }


          //* ------------------------ END OF VEHICLE TRACKING --------------------------- *//
          
          //* ------------------------- LANE CHANGE EXECUTION ---------------------------- *//

          // DECIDE BEHAVIOR BASED ON PREDICTIONS //
          double speed_diff = 0.0;
          
          // maximum permissible acceleration in m/sec2
          const double max_acc = 0.224;

          // define maximum allowed velocity in mph
          const double max_vel = 49.5;

          // if a slow vehicle is detected in ego vehicle lane, make the lane change
          // by checking both left and right lanes for safety
          if (car_in_lane)
          {
            // check if there is no car on left and ego vehicle lane is not in the leftmost lane
            if (ego_lane > 0 && left_lane_car == false)
            {
              // make a left lane change
              ego_lane--;
              std::cout<<"Change to left lane "<<ego_lane<<"..."<<std::endl;
            }
            // check if there is no car on right and tgat 
            else if (ego_lane != 2 && right_lane_car == false)
            {
              // make a right lane change
              ego_lane++;
              std::cout<<"Change to right lane "<<ego_lane<<"..."<<std::endl;
            }
            else
            {
              speed_diff -= max_acc;
              std::cout<<"Overtaking not possible because of vehicles in surrounding lanes ..."<<std::endl;
            }
          }
          // if no car is in the ego vehicle's path, and the ego vehicle is not in the center lane
          // move back to center lane
          else
          {
            // check that we are not on the center lane
            if (ego_lane != 1)
            {
              // if there is no car in either lanes in proximity
              if ((ego_lane == 0 && !right_lane_car) || (ego_lane == 2 && !left_lane_car))
              {
                // move back to center lane
                ego_lane = 1;
                std::cout<<"Moving back to center lane ..."<<std::endl;
              }
            }
            // accelerate if no obstacle in front and speed is less than the speed limit
            if (ego_vel < max_vel)
            {
                // accelerate faster
                ego_vel += 1.1 * max_acc;
            }
          }

          //* -------------------------- END OF VEHICLE TRACKING ---------------------------- *//

          // ------------------------------ START OF PART 1 --------------------------------- //

          json msgJson;
          
          // PART 1: Keep the car below 50 mph and keep the car in its lane
          // PART 1: Keep the Jerk and acceleration smooth
          // create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
          // later I will interpolate these points with a spline and add more points

          vector<double> ptsx;
          vector<double> ptsy;

          // reference x, y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);


          // if previous size is almost empty, use the car as starting reference
          if (prev_size < 2)
          {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }

          // use the previous path's end point as starting reference
          else
          {
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }
          
          // in Frenet, we will add 30m evenly spaced points ahead of starting reference
          // we add three more waypoints in between the two waypoints
          vector<double> next_wp0 = getXY(car_s+30,(2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // std::cout<<"ptsx size = "<<ptsx.size()<<endl;
          // std::cout<<"ptsy size = "<<ptsy.size()<<endl;

          // shift to car's co-ordinates
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

            // std::cout<<"x = "<<ptsx[i]<<endl;
            // std::cout<<"y = "<<ptsy[i]<<endl;

          }

          // create a spline
          tk::spline s;

          // set (x, y) points to the spline
          s.set_points(ptsx,ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // TODO: define a path made up of (x,y) points that the car will visit

          // start with all of the previous path points from last time
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }


          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          // get the corresponding y point for target x value from spline
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          // add rest of the path planner after filling it with previous points
          for (int i = 1; i < 50-prev_size; i++)
          { 
              // default, accelerate the ego vehicle
              ego_vel += speed_diff;
              
              // if the velocity increases beyond max permissible speed,
              // set speed to max speed to avoid further acceleration
              if (ego_vel > max_vel)
              {
                ego_vel = max_vel;
              }
              else if (ego_vel < max_acc)
              {
                ego_vel = max_acc;
              }

              // define the number of points
              // divide by 2.2369 to convert mph to m/sec
              double N = target_dist/(0.02*ego_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back to normal after rotating the system earlier
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            
          }

          // ---------------------------------- END OF PART 1 -------------------------------------- //          

          /*
          // basic run check to see how car runs around the track
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double dist_inc = 0.4;
          for (int i = 0; i < 50; i++)
          {
            double next_s = car_s+(i+1)*dist_inc;
            double next_d = 6;
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          */

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