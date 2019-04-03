#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <chrono>
#include <fstream>

// for convenience
using nlohmann::json;
using std::string;
using Clock = std::chrono::high_resolution_clock;
using std::chrono::time_point;
using std::chrono::duration;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  // Read gain and setup parameters from csv file
  string line;
  double p_term = 0.0;
  double i_term = 0.0;
  double d_term = 0.0;
  double max_i_err = 0.0;
  bool twiddle_active = false;
  std::ifstream param_file("pid_params.txt");  
  if(param_file.is_open()){
    int line_count = 0;
    while(getline(param_file, line)){
      switch(line_count){
        case 0:
          p_term = std::stod(line);
          break;
        case 1:
          i_term = std::stod(line);
          break;
        case 2:
          d_term = std::stod(line);
          break;
        case 3:
          max_i_err = std::stod(line);
          break;
        case 4:
          twiddle_active = std::stoi(line);
          break;
      }
      line_count += 1;
    }
    param_file.close();
  }
  
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  //pid.Init(0.15, 0, 3.0);
  //pid.Init(0.15, 0.01, 0.15); // This passes
  //pid.Init(0.10, 0.05, 0.25);
  pid.Init(p_term, i_term, d_term, max_i_err, twiddle_active);
  time_point<Clock> previous_message_time = Clock::now();
  int message_counter = 0;
  
  h.onMessage([&pid,&previous_message_time](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          //message_counter += 1;
          
          time_point<Clock> current_message_time = Clock::now();
          double time_between_messages = duration<double, std::milli>(current_message_time - previous_message_time).count();
          previous_message_time = current_message_time;

          pid.UpdateError(cte, time_between_messages);
          //steer_value = angle + pid.TotalError();
          steer_value = pid.TotalError();
          if (steer_value > 1.0){
            steer_value = 1.0;
          }else if(steer_value < -1.0){
            steer_value = -1.0;
          }
          
          // DEBUG
          std::cout << "Time between messages (ms): " << time_between_messages << "; Angle: " << angle << "; PID TotalError: " << pid.TotalError() << std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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