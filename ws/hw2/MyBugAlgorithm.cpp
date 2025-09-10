#include "MyBugAlgorithm.h"
#include <iostream>
#include "utils.h"
#include <cmath>

// Implement your methods in the `.cpp` file, for example:


#define BUG 1
//#define BUG 2





amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    double epsilon = 1e-3;
    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d curr_point = problem.q_init;


    int count2 = 0;
    int count = 0;
    Eigen::Vector2d direction = problem.q_goal - problem.q_init;
    double theta = std::atan2(direction.y(), direction.x());
    bool wall_follow = false;
    path.waypoints.push_back(problem.q_init);
    while((curr_point - problem.q_goal).norm() > 2 * epsilon){
        count2 ++;
        if(count2 > 800000){
            std::cout << "hit path count. curr_point: " << curr_point << std::endl;
            std::cout << "dist to goal: " << (problem.q_goal - curr_point).norm() << std::endl;
            //try to move forward a little...
            if(!utils::sensor(epsilon/10, theta, problem.obstacles, curr_point)){
                curr_point = curr_point + epsilon * Eigen::Vector2d(std::cos(theta), std::sin(theta));
                std::cout << "inch" << std::endl;
            }
            else{
                std::cout<< "coudln't move a little..." << std::endl;
            }
            // return path;
        }
        if(!utils::sensor(epsilon, theta, problem.obstacles, curr_point)){
            // path.waypoints.push_back(curr_point);
            curr_point = curr_point + epsilon * Eigen::Vector2d(std::cos(theta), std::sin(theta));
        }
        else{
            path.waypoints.push_back(curr_point);
            //enter wall follow mode. go around the obstacle, until we get close to the orig point and have a straight line path. Once that happens, we can append our new part of the path...
            
            if(BUG == 1){

            
                Eigen::Vector2d start_point = curr_point;
                Eigen::Vector2d closest_point = curr_point;
                double dist_to_goal = (curr_point - problem.q_goal).norm();
                bool started_moving = false;
                int num_path_points = path.waypoints.size();
                int move_count = 0;
                while(!started_moving || (curr_point - start_point).norm() > epsilon){
                    count++;
                    if(count > 599990){
                        std::cout << "hit wall count " << theta << std::endl;
                    }
                    if(count > 600000){
                        
                        return path;
                    }
                    //go around the obstacle
                    
                    //if the front sensor sees the obstacle or TODO: pi/8 sensor sees the obstacle turn left
                    if(utils::sensor(epsilon, theta, problem.obstacles, curr_point)){
                        while(utils::sensor(epsilon, theta,     problem.obstacles, curr_point)){
                        theta = theta + epsilon;
                        }
                        path.waypoints.push_back(curr_point);
                        curr_point = curr_point + epsilon/5 * Eigen::Vector2d(std::cos(theta), std::sin(theta)); 
                        double dist = (curr_point - problem.q_goal).norm();
                        if(dist < dist_to_goal){
                            dist_to_goal = dist;
                            closest_point = curr_point;
                        }
                    } 
                    //if the pi/4 doesn't see the obstacle, turn right
                    else if(!utils::sensor(epsilon * 1.43, theta-3.1415/4, problem.obstacles, curr_point)){
                        theta = theta - epsilon/2;
                    }
                    else if(!utils::sensor(epsilon, theta-3.1415/2, problem.obstacles, curr_point)){
                        theta = theta - 3.1415/2;
                        path.waypoints.push_back(curr_point);
                        curr_point = curr_point + epsilon/5 * Eigen::Vector2d(std::cos(theta), std::sin(theta)); 
                        double dist = (curr_point - problem.q_goal).norm();
                        if(dist < dist_to_goal){
                            dist_to_goal = dist;
                            closest_point = curr_point;
                        }
                        theta = theta + 3.1415/2 - epsilon;
                    }
                    else{
                        //update
                        path.waypoints.push_back(curr_point);
                        curr_point = curr_point + epsilon * Eigen::Vector2d(std::cos(theta), std::sin(theta));
                        //check if we have moved enough, if so let us get out of while loop
                        if(move_count < 5 && !started_moving){
                            move_count ++;
                            started_moving = true;
                        }
                        else if(move_count < 5){
                            move_count ++;
                        }
                        
                        //check dist to goal
                        double dist = (curr_point - problem.q_goal).norm();
                        if(dist < dist_to_goal){
                            dist_to_goal = dist;
                            closest_point = curr_point;
                        }
                    }
                }

                //if the 3pi/8 sensor doesn't see the obstacle, turn right
                //if the pi/2 sensor doesn't see the obstacle, 
            
                //go back to the closest point.
                while((curr_point - problem.q_goal).norm() != dist_to_goal){
                    curr_point = path.waypoints[num_path_points];
                    path.waypoints.push_back(curr_point);
                    num_path_points++;

                }
                // std::cout << "Closest point: " << curr_point<< std::endl;
                //reset theta
                direction = problem.q_goal - curr_point;
                theta = std::atan2(direction.y(), direction.x());
                // return path;
            }
        }
    }
    path.waypoints.push_back(problem.q_goal);
    









    // // path.waypoints.push_back(problem.q_goal);
    // if(true){
    // //while(curr_point != problem.q_goal) {
    //     //First try and travel in a straight line to the goal
    //     Eigen::Vector2d next_point = utils::raytrace(problem.obstacles, curr_point, problem.q_goal);
    //     //if we are in at the goal, go there and return
    //     if(next_point == problem.q_goal){
    //         path.waypoints.push_back(problem.q_goal);
    //         return path;
    //     }
    //     //otherwise, we hit an obstacle. go around the obstacle.
    //     else {
    //         //which obstacle are we circumnav?
    //         int issue_obs = utils::findIssueObs(problem.obstacles, curr_point, next_point);
    //         //we now have the obstacle we need to go around. Get the first point
    //         next_point = utils::intersect(curr_point, next_point, problem.obstacles[issue_obs]);
    //         next_point = utils::interp(curr_point, next_point, epsilon);

            
            
    //         std::vector<Eigen::Vector2d> next_points;
    //         // std::cout << "first point " << next_point << std::endl;
    //         next_points.push_back(next_point);
    //         utils::circumnavObs(next_points, problem.obstacles, issue_obs, next_point, epsilon, true, 0);
    //         //
    //         // next_points = utils::offsetObstaclePath(problem.obstacles[issue_obs], epsilon, next_point);
    //         //first circumnavigage the obstacle

    //         //Then, find the closest point
    //         //Then go to the closest point.

    //         for(auto p : next_points) {
    //             path.waypoints.push_back(p);
    //             // std::cout << p << std::endl;
    //         }
    //         // if(path.waypoints[path.waypoints.size() - 1] == next_point) {
    //         //     std::cout << "SUCCESS FINDING PATH" << std::endl;
    //         // }
    //         // else{
    //         // }
    //         //path.waypoints.push_back(curr_point);
    //         //now go aroudn the obstacle...

    //     }
    // }
    
    return path;
}
