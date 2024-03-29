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

// Checks whether other lane is free, so lane changing can be safe
bool isLaneFree(const int laneNumber, const double myCarS, const double mySpeed, const vector<vector<double>>& sensorFusionData)
{
  for (size_t i = 0; i < sensorFusionData.size(); ++i) {
    // Determine lane if of the other car
    const int laneOfOtherCar = static_cast<int>(sensorFusionData[i][6]) / 4;
    // If the other car is in the investigated lane
    if (laneOfOtherCar == laneNumber) {
      const double otherCarS = sensorFusionData[i][5];
      // If the other car is too close to our car
      if ((myCarS <= otherCarS && otherCarS <= myCarS + 20) || (myCarS - 10 < otherCarS && otherCarS < myCarS)) {
        return false;
      }
      const double otherVelocityX = sensorFusionData[i][3];
      const double otherVelocityY = sensorFusionData[i][4];
      const double otherVelocity = sqrt(otherVelocityX * otherVelocityX + otherVelocityY * otherVelocityY);
      // If the other car is not too close and behind us and its speed is greater than our speed
      if (myCarS - 20 < otherCarS && otherCarS < myCarS - 10 && mySpeed < otherVelocity) {
        return false;
      }
    }
  }
  // If no investigated condition was true, we can change the lane safely
  return true;
}

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
  
  int lane = 1;
  
  double ref_vel = 0.0; // in mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel]
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          bool too_close = false;
          
          // Iterate through each car detected by sensors
          for (size_t i = 0; i < sensor_fusion.size(); ++i) {
            // Determine the lane of the current other car
            int laneOfOtherCar = static_cast<int>(sensor_fusion[i][6]) / 4;
            // If the other car is in the same lane we are
            if (laneOfOtherCar == lane) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];
              
              check_car_s += static_cast<double>(prev_size) * 0.02 * check_speed;
              
              // If the other car is in front of our car and too close to our car
              if (check_car_s > car_s && check_car_s - car_s < 30) {
                // Brake! Our new velocity is calculated regarding the speed of the other car
                ref_vel -= (ref_vel - check_speed) / 30.0;
                too_close = true;
                // If right lane is free change lane. If the left lane is free change to there.
                if ((lane == 1 && isLaneFree(2, j[1]["s"], car_speed, sensor_fusion)) || (lane == 0 && isLaneFree(1, j[1]["s"], car_speed, sensor_fusion))) {
                  ++lane;
                } else if ((lane == 1 && isLaneFree(0, j[1]["s"], car_speed, sensor_fusion)) || (lane == 2 && isLaneFree(1, j[1]["s"], car_speed, sensor_fusion))) {
                  --lane;
                }
              }
            }
          }
          
          // If there is not an other car in our lane in front of us and too close to us, and our speed could be higher, accelerate!
          if (!too_close && ref_vel < 49.5) {
            ref_vel += 0.224;
          }
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // If we currently in a lane changing process, the trajectory should be smoother.
          const int firstIncrease = (lane == static_cast<int>(car_d) / 4) ? 30 : 45;
          vector<double> next_wp0 = getXY(car_s + firstIncrease, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (size_t i = 0; i < ptsx.size(); ++i) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
            ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
          }
          
          tk::spline s;
          
          s.set_points(ptsx, ptsy);
          
          for (size_t i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          
          double x_add_on = 0;
          
          for (size_t i = 1; i <= 50 - previous_path_x.size(); ++i) {
            double n = target_dist / (0.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / n;
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
          
          /**
           * END TODO
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