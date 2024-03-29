#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

  double Kp = 0.4;
  double Kd = 10;
  double Ki = 0.0002;

  double Kp_s = 2.0; //1.0;
  double Kd_s = 13.8514; //5.0;
  double Ki_s = 0.00891601; //0.005;

  double desired_speed = 25.0;

  //Control flag of tune params by Twidlle online.
  bool tune_params = false;
  // Use during tuning - number of steps per trial
  double evaluation_steps;
  // Tolerance to stop twiddle
  double tolerance = 0.2;
  int num_param = 3;

  PID pid_speed;
  TWIDDLE twid;

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
  uWS::Hub h;

  // Define PID object
  PID pid;
  

  if(!tune_params){
    pid.Init(Kp, Kd, Ki, evaluation_steps);
    pid_speed.Init(Kp_s, Kd_s, Ki_s, evaluation_steps);
  }
  else{
    evaluation_steps = 2000;
    twid.Init(tolerance, num_param);
    vector<double> params = twid.get_Parameters();
    pid.Init(Kp, Kd, Ki, evaluation_steps);
    //pid.Init(params[0], params[1], params[2], evaluation_steps);
    //pid_speed.Init(Kp_s, Kd_s, Ki_s, evaluation_steps);
    pid_speed.Init(params[0], params[1], params[2], evaluation_steps);
  }
  
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

          double speed_diff = speed - desired_speed;
          double throttle;
          double error;

          if(tune_params && pid.step_counter == 0){
            // Implementing Twiddle for tuning PID parameters
            // Send error to optimizer
            error = (pid.TotalError() / evaluation_steps); // error when tuning position controller
            //error = 0.7*(pid.TotalError() / evaluation_steps) + 0.3*(pid_speed.TotalError() / evaluation_steps); // error when tuning speed controller
            twid.UpdateError(error);

            // Get next parameter set
            vector<double> params = twid.get_Parameters();

            if(twid.is_Over()){
              cout << "---------------------------------------------------" << endl;
              cout << "---------------------------------------------------" << endl;
              cout << "Twiddle terminated!" << endl;
              cout << "Found parameters: \t Kp = " << params[0] << "\t Kd = " << params[1] << "\t Ki = " << params[2] << endl;
              cout << "---------------------------------------------------" << endl;
              cout << "---------------------------------------------------" << endl;
              exit(0);
            }
            else{
              // Restart environment with new PID
              pid.Init(Kp, Kd, Ki, evaluation_steps);
              //pid.Init(params[0], params[1], params[2], evaluation_steps);
              // pid_speed.Init(Kp_s, Kd_s, Ki_s, evaluation_steps);
              pid_speed.Init(params[0], params[1], params[2], evaluation_steps);

              std::string msg2 = "42[\"reset\",{}]";
              ws.send(msg2.data(), msg2.length(), uWS::OpCode::TEXT);

              cout << "Error: " << error << endl;
              cout << "Restarting: \t Kp = " << params[0] << "\t Kd = " << params[1] << "\t Ki = " << params[2] << endl;
              cout << "Restarting: \t delta Kp = " << twid.d_params[0] << "\t delta Kd = " << twid.d_params[1] << "\t delta Ki = " << twid.d_params[2] << endl;
            }
          }
          else{
            pid.UpdateError(cte);
            pid_speed.UpdateError(speed_diff);

            steer_value = pid.get_ControlValue();
            throttle = pid_speed.get_ControlValue();

            steer_value = max(-1.0,steer_value);
            steer_value = min(1.0,steer_value);

            throttle = max(-1.0,throttle);
            throttle = min(1.0,throttle);

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }          
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