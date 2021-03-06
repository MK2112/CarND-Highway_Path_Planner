#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

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
    // The max s value before wrapping around the track back to 0 ('length' of one lap)
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

    int lane = 1; // 0 | 1 | 2 lanes, start is lane 1 per default
    double ref_vel = 0.0; // mph to drive at max
    const int point_space = 30;

    h.onMessage([&lane, &ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                        &map_waypoints_dx, &map_waypoints_dy]
                        (uWS::WebSocket <uWS::SERVER> ws, char *data, size_t length,
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

                    // Previous path data given to the Planner (historical)
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    // of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    /**
                     * DONE: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */

                    int prev_size = previous_path_x.size();

                    // Making sure the car remains s-wise where the previous path ended
                    if (prev_size > 0) {
                        car_s = end_path_s;
                    }

                    bool too_close = false;      // The car in front (same lane) is too close
                    bool left_lane_free = true;  // Lane space to the left is free
                    bool right_lane_free = true; // Lane space to the right is free

                    double check_speed;

                    // Check environment for potential collision
                    // Also check if other lanes are empty
                    for (int i = 0; i < sensor_fusion.size(); ++i) {
                        float d = sensor_fusion[i][6];
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        check_speed = sqrt(std::pow(vx, 2) + std::pow(vy, 2));
                        double check_car_s = sensor_fusion[i][5];

                        // If using previous points can project s value out
                        check_car_s += ((double) prev_size * .02 * check_speed);

                        // Check for car being right ahead in current lane
                        if ((d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
                            && ((check_car_s > car_s) && ((check_car_s - car_s) < point_space))) {
                            too_close = true;
                        }

                        // Check for car(s) to the left
                        if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2)
                            && (((check_car_s > car_s) && ((check_car_s - car_s) < point_space)) ||
                                ((check_car_s < car_s) && ((check_car_s - car_s) > -point_space)))) {
                            left_lane_free = false;
                        }

                        // Check for car(s) to the right
                        if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2)
                            && (((check_car_s > car_s) && ((check_car_s - car_s) < point_space)) ||
                                ((check_car_s < car_s) && ((check_car_s - car_s) > -point_space)))) {
                            right_lane_free = false;
                        }
                    }

                    // There is a car ahead -> We need to act -> Determine what to do
                    if (too_close) {
                        // Check if proposed left lane change is possible (there is a lane) and execute
                        if (left_lane_free && lane > 0) {
                            lane -= 1;
                            // If not make sure proposed right lane change is possible (there is a lane) and execute
                        } else if (right_lane_free && lane < 2) {
                            lane += 1;
                            // If not reduce speed and stay in lane
                        } else if (ref_vel > check_speed - 5) {
                            ref_vel -= 0.24;
                        }
                        // There is nothing ahead, we can accelerate (slowly) to v_max
                    } else if (ref_vel < 49.5) {
                        ref_vel += 0.24;
                    }

                    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
                    // Later we will interpolate these waypoints with a spline and fill it in with more waypoints
                    // that control our speed
                    vector<double> points_x;
                    vector<double> points_y;

                    // Reference x,y, yaw states
                    // Either we will reference the starting point as where the car is
                    // or at the previous path's end point
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    // If previous size is almost empty, use the car as starting reference
                    if (prev_size < 2) {
                        // Use two points that make the path tangent to the car
                        // (for that, go backwards in time based on its angle)
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        points_x.push_back(prev_car_x);
                        points_x.push_back(car_x);

                        points_y.push_back(prev_car_y);
                        points_y.push_back(car_y);
                    } else {
                        // Use previous path's end point as starting reference
                        // Re-define reference state as previous path's end point
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        // Use two points that make the path tangent to the previous path's end point
                        points_x.push_back(ref_x_prev);
                        points_y.push_back(ref_y_prev);
                        points_x.push_back(ref_x);
                        points_y.push_back(ref_y);
                    }

                    // In Frenet: Add evenly 30m (point_space) spaced points ahead of the starting reference
                    vector<double> next_wp0 = getXY(car_s + point_space, (2 + 4 * lane),
                                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 2 * point_space, (2 + 4 * lane),
                                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 3 * point_space, (2 + 4 * lane),
                                                    map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    points_x.push_back(next_wp0[0]);
                    points_y.push_back(next_wp0[1]);

                    points_x.push_back(next_wp1[0]);
                    points_y.push_back(next_wp1[1]);

                    points_x.push_back(next_wp2[0]);
                    points_y.push_back(next_wp2[1]);

                    for (int i = 0; i < points_x.size(); ++i) {
                        // Shift car reference angle to 0 degrees
                        double shift_x = points_x[i] - ref_x;
                        double shift_y = points_y[i] - ref_y;

                        points_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        points_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }

                    // Create a spline
                    tk::spline s;

                    // Set spline points to (x,y)
                    s.set_points(points_x, points_y);

                    // Define the actual (x,y) points we will use for the planner
                    // Start with all (that remains) of the previous path points from last time
                    for (int i = 0; i < previous_path_x.size(); ++i) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // Calculate how to 'break up' spline points so that adding points
                    // makes us travel at desired reference velocity
                    double target_x = (double) point_space;
                    double target_y = s(target_x);
                    double target_dist = sqrt(std::pow(target_x, 2) + std::pow(target_y, 2));

                    double x_add_on = 0;

                    // Fill up the rest of our path planner after filling it with previous points,
                    // here we will always total the output to 50 points
                    for (int i = 1; i <= 50 - previous_path_x.size(); ++i) {
                        double N = (target_dist / (.02 * ref_vel / 2.24)); // 2.24 -> mph to m/s
                        double x_point = x_add_on + target_x / N;
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        // Rotate back to normal after rotating earlier
                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket <uWS::SERVER> ws, int code,
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