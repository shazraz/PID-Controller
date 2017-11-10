#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  PID steer_controller;
  PID speed_controller;
  // TODO: Initialize the pid variable.
  double Kp = atof(argv[1]);
  double Ki = atof(argv[2]);
  double Kd = atof(argv[3]);
  double target_speed = atof(argv[4]);

  double Kp_speed = 0.2;
  double Ki_speed = 0.0;
  double Kd_speed = 0.1;

  steer_controller.Init(Kp, Ki, Kd);
  speed_controller.Init(Kp_speed, Ki_speed, Kd_speed);

  h.onMessage([&steer_controller, &speed_controller, &target_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          
          double steer_value;
          double throttle_value;
          double actual_speed;
          double cte_speed;

          //Bounds for throttle values
  		  double max_throttle = 1.0;
		  double min_throttle = -1.0;
		  //Bounds for steering values
		  double max_steer = 1.0;
		  double min_steer = -1.0;
		  //maximum amount to reduce speed by
          double speed_reduction = 45.0; 
          
          //Update the error and get the new steer value
          steer_controller.UpdateError(cte);
          steer_value = steer_controller.TotalError();

          //Cap the steer valuee
          if (steer_value > max_steer){
          	steer_value = max_steer;
          }
          else if (steer_value < min_steer){
          	steer_value = min_steer;
          }

          //Set the actual speed based on the magnitude of steering angle
          actual_speed = target_speed - fabs(steer_value)*speed_reduction;

          //Calculate the CTE for speed
          cte_speed = speed - actual_speed;
          //Update the error and get the new throttle value
          speed_controller.UpdateError(cte_speed);
          throttle_value = speed_controller.TotalError();

		  //Cap the throttle value
          if (throttle_value > max_throttle){
          	throttle_value = max_throttle;
          }
          else if (throttle_value < min_throttle){
          	throttle_value = min_throttle;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Actual target speed: " << actual_speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;//0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
