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

class sTwiddle {
public:
  double p[3];
  double dp[3];
  int param;
  int phase;
  sTwiddle() {
    for (int i=0; i<3; i++) {
      p[i] = 0.0;
      dp[i] = 0.1;
    }
    param = 0;
    phase = 0;
  }
};

int main()
{
  uWS::Hub h;

  PID pid = PID();
  PID pid_throttle = PID();

  bool do_twiddle = false;
  sTwiddle twiddle;
  double best_err = 1.0e6;
  int t = 0;

  // if (do_twiddle) {
  //   pid.Init(twiddle.p[0], twiddle.p[1], twiddle.p[2]);
  // } else {
  //   pid.Init(0.2, 0.001, 1.0);  // these starting values are good enough to drive around the track
  // }

  // these starting values are good enough to drive around the track
  twiddle.p[0] = 0.11;  // 0.15;
  twiddle.p[1] = 0.008;  // 0.006;
  twiddle.p[2] = 1.5;  // 2.0;
  pid.Init(twiddle.p[0], twiddle.p[1], twiddle.p[2]);

  pid_throttle.Init(0.2, 0.0, 1.0);

  // For twiddle:
  //   Run one parameter config until CTE is excessive or 500 steps of simulation
  //   Reset simulator
  //   Switch parameter config
  //   Repeat until sum(twiddle_dp) < 0.2
  //   Log the parameters
  //   Exit

  h.onMessage([&pid, &pid_throttle, do_twiddle, &twiddle, &best_err, &t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
          auto s = hasData(std::string(data).substr(0, length));
          if (s != "") {
            auto j = json::parse(s);
            std::string event = j[0].get<std::string>();
            
            //std::cout << "got json '" << j << "'" << std::endl;
            
            if (event == "telemetry") {
              // j[1] is the data JSON object
              double cte = std::stod(j[1]["cte"].get<std::string>());
              double speed = std::stod(j[1]["speed"].get<std::string>());
              double angle = std::stod(j[1]["steering_angle"].get<std::string>());
              double steer_value;
          
              /*
               * TODO: Calcuate steering value here, remember the steering value is
               * [-1, 1].
               * NOTE: Feel free to play around with the throttle and speed. Maybe use
               * another PID controller to control the speed!
               */

              pid.UpdateError(cte);
              steer_value = pid.getControlResponse();

              // Speed regulation; minimum speed setting 10MPH so the car doesn't stall
              double target_speed = 45.0;
              double cte_int = abs(pid.getCTE_Int());
              target_speed -= (1.4 * cte_int);
              target_speed = (target_speed < 10.0) ? 10.0 : target_speed;

              pid_throttle.UpdateError(speed - target_speed);
              double throttle_value = pid_throttle.getControlResponse();
              throttle_value = 0.5 + (0.5 * throttle_value);  // range 0.0 - 1.0

              std::cout << "target " << target_speed << " throttle " << throttle_value << std::endl;
              
              //double extra_throttle = 0.3 * pow(1.0 - abs(steer_value), 2);
              
              // DEBUG
              if (false) {
                std::cout << "speed " << speed << " ";
                std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
              }

              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;  // 0.3 + extra_throttle;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              if (false) {
                std::cout << msg << std::endl;
              }
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              t++;
              if (do_twiddle) {
                if (abs(cte) > 2.0 || t == 500) {

                  std::cout << "CTE " << cte << " t " << t << " phase " << twiddle.phase << std::endl;
                  
                  if (twiddle.phase == 0) {
                    if (pid.TotalError() / (double) t < best_err) {
                      best_err = pid.TotalError() / (double) t;
                    }
                    // try p+dp
                    twiddle.p[twiddle.param] += twiddle.dp[twiddle.param];
                    twiddle.phase = 1;
                  } else {
                    if (pid.TotalError() / (double) t < best_err) {
                      std::cout << "success!" << std::endl;
                      best_err = pid.TotalError() / (double) t;
                      twiddle.dp[twiddle.param] *= 1.1;
                      
                      std::cout << "adjust next param" << std::endl;
                      twiddle.param = (twiddle.param + 1) % 3;
                      twiddle.p[twiddle.param] += twiddle.dp[twiddle.param];
                      twiddle.phase = 1;
                    } else {
                      if (twiddle.phase == 1) {
                        // try p-dp
                        std::cout << "reverse coarse!" << std::endl;
                        twiddle.p[twiddle.param] -= (2.0 * twiddle.dp[twiddle.param]);
                        twiddle.phase = 2;
                      } else {
                        // reset to previous p
                        std::cout << "revert" << std::endl;
                        twiddle.p[twiddle.param] += twiddle.dp[twiddle.param];
                        twiddle.dp[twiddle.param] *= 0.9;

                        std::cout << "adjust next param" << std::endl;
                        twiddle.param = (twiddle.param + 1) % 3;
                        twiddle.p[twiddle.param] += twiddle.dp[twiddle.param];
                        twiddle.phase = 1;
                      }
                    }
                  }
                  std::cout << "Reset the simulator (best_err = " << best_err << ")" << std::endl;
                  std::cout << "Twiddle: " << twiddle.p[0] << ", " << twiddle.p[1] << ", " << twiddle.p[2] << std::endl;
                  // reset the PID for next round
                  pid.Init(twiddle.p[0], twiddle.p[1], twiddle.p[2]);
                  t = 0;
                  // reset the simulator for next round
                  std::string msg = "42[\"reset\",{}]";
                  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
              } else {
                if (t % 50 == 0) {
                  std::cout << "Error " << pid.TotalError() / (double) t << std::endl;
                }
              }
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
