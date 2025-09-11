#include "utils.h"
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <queue>
#include <unordered_set>
#include <iostream>

namespace utils{
    
    bool sensor(double epsilon, double theta, std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d point) {
        // for(int i = 0; i < 50; i+=5){
            double new_theta = theta;
            //get the point in the direction of theta
            Eigen::Vector2d sensor_hit = point + epsilon * Eigen::Vector2d(std::cos(new_theta), std::sin(new_theta));
            //check if this is inside of an obstacle
            for(auto& obstacle : obstacles){
                if(raycastObstacleDetection(obstacle, sensor_hit)){
                    return true; //hit an obstacle
                }
            }
        // }
        return false;
    }












    void circumnavObs(std::vector<Eigen::Vector2d>& path, const std::vector<amp::Obstacle2D>& obstacles, int curr_obs, Eigen::Vector2d start, double epsilon, bool first_step, int call_count) {
        call_count ++;

        std::vector<Eigen::Vector2d> new_path = offsetObstaclePath(obstacles[curr_obs], epsilon, path[path.size()-1]);
        // std::cout << "obstacle index: " << curr_obs << std::endl;
        
        // // new_path.erase(new_path.begin());
        while((new_path[0] - path[path.size()-1]).norm() < epsilon) {
            new_path.erase(new_path.begin());
        }
        if(call_count > 500){
            // path.push_back(new_path[0]);
            // for(auto p : new_path){
            //     path.push_back(p);
            // }

            std::cout << "BAD PATH" << std::endl;
            return;
        }       
        for(int i = 0; i < new_path.size(); i++) {
            // std::cout << new_path[i] << std::endl;
            // std::cout << "current point: " << path[path.size() - 1] << std::endl;
            // std::cout << "next point try: " << new_path[i] << std::endl;
            Eigen::Vector2d np = raytrace(obstacles, path[path.size() - 1], new_path[i], epsilon);
            if (new_path[i] != np) {
                //determine the next obstacle;
                int issue_obs = findIssueObs(obstacles, path[path.size()-1], np); 
                //find the point where the two lines intersect, and go epsilon closer to the spot we want...
                np = intersect(path[path.size() - 1], np, obstacles[issue_obs]);
                // path.push_back(np);
                // return;
                path.push_back(interp(path[path.size() - 1], np, epsilon));
                // std::cout << "bad path check" << std::endl;
                // std::cout << "issue obs: " << issue_obs << std::endl;
                circumnavObs(path, obstacles, issue_obs, start, epsilon, first_step, call_count);
                return;
            }
            //check if start point is within acceptable dist of the current path.
            
            if(i > 1){
                first_step = false;
            }
            if((closestPointLine(path[path.size()-1], new_path[i], start) - start).norm() < epsilon && !first_step){
                // std::cout << "good path check with points path: " << new_path[(i-1+new_path.size()) % new_path.size()] << " and " << start << std::endl;
                path.push_back(start);
                std::cout << "CASE 2 MET" << std::endl;
                return;
            }
            
            path.push_back(new_path[i]);
        }
        std::cout << "CASE 1" << std::endl;
        return;

    }

    int findIssueObs(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d start, Eigen::Vector2d end) {
        int issue_obs = 0;
        double dist = 0;
        for(int i = 0; i < obstacles.size(); i++) {
            if(raycastObstacleDetection(obstacles[i], end)) {
                if (dist == 0) {
                    issue_obs = i; 
                    dist = (intersect(start, end, obstacles[i]) - start).norm();
                }
                else{
                    double new_dist = (intersect(start, end, obstacles[i]) - start).norm();
                    if(new_dist < dist){
                        issue_obs = i;
                        dist = new_dist;
                    }
                }
            }
        }
        return issue_obs;
    }

    bool checkIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2){
        //TODO: write this function
        Eigen::Vector2d r = p2 - p1;
            Eigen::Vector2d s = q2 - q1;
            Eigen::Vector2d qp = q1 - p1;
            double rxs = r.x() * s.y() - r.y() * s.x();
            double qpxr = qp.x() * r.y() - qp.y() * r.x();
            if(std::abs(rxs) < 1e-10){
                return false;
            }
            double t = (qp.x() * s.y() - qp.y() * s.x()) / rxs;
            double u = qpxr / rxs;
            if(t >= 0 && t<=1 && u>=0 && u<=1){
                return true;
            }
            return false;

    }

    Eigen::Vector2d intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const amp::Obstacle2D& obstacle) {
        Eigen::Vector2d intersec;
        double dist = 0;
        for(int i = 0; i < obstacle.verticesCCW().size(); i++) {
            Eigen::Vector2d q1 = obstacle.verticesCCW()[i];
            Eigen::Vector2d q2 = obstacle.verticesCCW()[(i+1)%obstacle.verticesCCW().size()];

            Eigen::Vector2d r = p2 - p1;
            Eigen::Vector2d s = q2 - q1;
            Eigen::Vector2d qp = q1 - p1;
            double rxs = r.x() * s.y() - r.y() * s.x();
            double qpxr = qp.x() * r.y() - qp.y() * r.x();
            if(std::abs(rxs) < 1e-10){
                continue;
            }
            double t = (qp.x() * s.y() - qp.y() * s.x()) / rxs;
            double u = qpxr / rxs;
            if(t >= 0 && t<=1 && u>=0 && u<=1){
                if(dist == 0){
                    intersec = p1+t*r;
                    dist = (p1 - intersec).norm();
                }
                else if ((p1 - (p1+t*r)).norm() < dist) {
                    intersec = p1+t*r;
                    dist = (p1 - intersec).norm();
                }
            }

        }
        //bad
        if(dist == 0) {
           std::cout << "BAD INPUT NO INTERSECT" << std::endl; 
        }
        return intersec;
    }

    //return the raytrace hitpoint from q_init to q_goal. If no hitpoint return q_goal.
    Eigen::Vector2d raytrace(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d q_init, Eigen::Vector2d q_goal, double epsilon) {
        //linear interpolation with stepsize epsilon from q_init to q_goal.
        std::vector<Eigen::Vector2d> interp;
        Eigen::Vector2d direction = q_goal - q_init;
        double distance = direction.norm();
        if(distance == 0) {
            return q_goal;
        }

        direction = direction.normalized();
        int numSteps = static_cast<int>(std::ceil(distance / epsilon)) - 1; //-1 so that we don't do the last point, for that we will use q_goal as the last poitn might be past q_goal.

        for(int i = 0; i < numSteps; i++) {
            double stepDist = i * epsilon;
            interp.push_back(q_init + direction * stepDist);
        }

        //now we want to raymarch through the points and do collision detection for each of those points.
        for(int i = 0; i < numSteps; i++) {
            //for each obstacle check if we are inside it
            for(const auto& obstacle : obstacles) {
                if(raycastObstacleDetection(obstacle, interp[i]))
                    return interp[i];
            }
        }
        return q_goal;

    }

    Eigen::Vector2d interp(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double dist) {
        Eigen::Vector2d direction = end - start;
        direction = direction.normalized();
        return end - direction * dist;
    }

    bool isPointOnSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return false;
        // Vector from a to b and a to p
        Eigen::Vector2d ab = b - a;
        Eigen::Vector2d ap = p - a;

        // Check if cross product is zero => colinear
        double cross = ab.x() * ap.y() - ab.y() * ap.x();
        if (std::abs(cross) > 1e-10) return false; // Not colinear

        // Check if dot product is within segment range
        double dot = ap.dot(ab);
        if (dot < 0 || dot > ab.squaredNorm()) return false; // Outside segment

        return true; // Point is on the segment
    }

    bool raycastObstacleDetection(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& point) {
        bool inside = false;
        int numV = obstacle.verticesCCW().size();
        for(int i = 0, j = numV-1; i < numV; j = i++){
            const Eigen::Vector2d& pi = obstacle.verticesCCW()[i];
            const Eigen::Vector2d& pj = obstacle.verticesCCW()[j];
            if (isPointOnSegment(point, pi, pj)) {
                return true; // On the boundary is considered "inside"
            }
            //now check for intersects when casting to the right
            //if the line is horizontal, no intersect so stay same
            if(pi.y() == pj.y()) {
                continue;
            }
            //otherwise check if our poitns y is between these two
            bool yBetween = (point.y() > std::min(pi.y(), pj.y())) && (point.y() <= std::max(pi.y(), pj.y()));
            if (yBetween) {
                //get x coordinate of intersect
                double t = (point.y() - pi.y()) / (pj.y() - pi.y());  
                double intersectX = pi.x() + t * (pj.x() - pi.x());
                if (intersectX > point.x()){
                    //flip inside
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    std::vector<Eigen::Vector2d> offsetObstaclePath(const amp::Obstacle2D& obstacle, double epsilon, const Eigen::Vector2d& start) {
        std::vector<Eigen::Vector2d> offset;
        
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        int n = vertices.size();
        for(int i = n-1; i >= 0; i--) {
            Eigen::Vector2d pi = vertices[i];
            Eigen::Vector2d next = vertices[(i - 1 + n) % n];
            Eigen::Vector2d prev = vertices[(i + 1 + n) % n];

            Eigen::Vector2d edge1 = (pi - prev).normalized();
            Eigen::Vector2d edge2 = (pi - next).normalized();

            Eigen::Vector2d norm1(-edge1.y(), edge1.x());
            Eigen::Vector2d norm2(edge2.y(), -edge2.x());

            Eigen::Vector2d bisect = (norm1 + norm2).normalized();

            double angle = std::acos(std::clamp(edge1.dot(edge2), -1.0, 1.0));
            double offsetFactor = 1.0 / std::sin(angle / 2.0);

            Eigen::Vector2d offset_point = pi + epsilon * offsetFactor * bisect;
            offset.push_back(offset_point);
        }

        //find closest segment on polygon
        int index = 0;
        double min_dist = std::numeric_limits<double>::max();
        Eigen::Vector2d closest_point;

        for (int i = 0; i < offset.size(); i++) {
            int j = (i + 1) % offset.size();
            Eigen::Vector2d cp = closestPointLine(offset[i], offset[j], start);
            double dist = (cp - start).norm();
            if (dist < min_dist){
                min_dist = dist;
                index = i;
                closest_point = cp;
            }
        }

        std::vector<Eigen::Vector2d> reordered;
        reordered.push_back(start);
        reordered.push_back(closest_point);
        //we need to get the reordered set st. we don't go backwards to the previous point...

        for (size_t i = 0; i < offset.size(); ++i) {
            size_t idx = (index + 1 + i) % offset.size();
            reordered.push_back(offset[idx]);
        }

        reordered.push_back(start);
        // reordered.erase(reordered.begin());
        return reordered;
        

        
    }



    Eigen::Vector2d closestPointLine(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& p) {
        Eigen::Vector2d ab = b - a;
        if(ab.norm() < 1e-8) {
            return a;
        }
        double t = (p - a).dot(ab) / ab.squaredNorm();
        t = std::clamp(t, 0.0, 1.0);
        return a + t * ab;
    }

    void recurCircumnavObstacle(const std::vector<amp::Obstacle2D>& obstacles, double epsilon, const Eigen::Vector2d& start, const Eigen::Vector2d& curr_point, int curr_obstacle_index, std::vector<Eigen::Vector2d>& path) {
        //generate path around the obstacle
        std::vector<Eigen::Vector2d> new_path = offsetObstaclePath(obstacles[curr_obstacle_index], epsilon, curr_point);

        //raytrace around the obstacle. If we encounter a collision, check which obstacle.
        //recursively call for that obstacle. also check if we get within epsilon of the start point, in which case return.
        for(int i = 0; i < new_path.size(); i++) {
            Eigen::Vector2d cp = new_path[i];
            Eigen::Vector2d ep = new_path[(i + 1) % new_path.size()];
            Eigen::Vector2d np = raytrace(obstacles, cp, ep, epsilon);
            int issue_obs = -1;
            if(np != ep) { 
                for(int j = 0; j < obstacles.size(); j++) {
                    if(raycastObstacleDetection(obstacles[j], np)) {
                        issue_obs = i;
                    }
                }
            }
            //check if a point between cp and np is the start
            Eigen::Vector2d closest_point = closestPointLine(cp, np, start);
            if((start - closest_point).norm() < epsilon) {
                //we are done, we can add start to our path and return
                path.push_back(start);
                return;
            }
            //else, if we hit an obstacle, then call this recursively for that obstacle.
            if(issue_obs != -1 && issue_obs != curr_obstacle_index) {
                np = interp(cp, np, epsilon);
                //TODO: fix recur
                // std::cout << "got here " << (issue_obs != curr_obstacle_index) << std::endl;
                // return;
                recurCircumnavObstacle(obstacles, epsilon, start, np, issue_obs, path);
            }
            //else continue and push the np to the path
            path.push_back(np);
        }



    }

    std::vector<Eigen::Vector2d> circumnavObstacle(const std::vector<amp::Obstacle2D>& obstacles, double epsilon, const Eigen::Vector2d& start, int curr_obstacle_index){
        
        std::vector<Eigen::Vector2d> path;
        // path.push_back(start);
        std::vector<Eigen::Vector2d> new_path = offsetObstaclePath(obstacles[curr_obstacle_index], epsilon, start);
        Eigen::Vector2d np = interp(new_path[0], start, epsilon*2);
        // path.push_back(np);
        while((start - np).norm() < epsilon) {
            new_path.erase(new_path.begin());
            np = interp(new_path[0], start, epsilon*2);
        }
        path.push_back(new_path[0]);


        recurCircumnavObstacle(obstacles, epsilon, start, np, curr_obstacle_index, path);
        // for( auto & p : new_path) {
            // path.push_back(p);
        // }
        return path;
        
    }
        
    // std::vector<Eigen::Vector2d> offsetMergedObstaclePath(const std::vector<amp::Obstacle2D>& obstacles, double epsilon, const Eigen::Vector2d& start, int start_obstacle_index, std::vector<int> obs_ignore) {
    //     //start by going around the first obstacle
    //     std::vector<Eigen::Vector2d> orig_path = offsetObstaclePath(obstacles[start_obstacle_index], epsilon, start);
    //     std::vector,Eigen::Vector2d> final_path = orig_path.copy();

    //     //raycast points, check if we hit an obstacle.
    //     for(int i = 0; i < orig_path.size(); i++) {
    //         Eigen::Vector2d sp = orig_path[i];
    //         Eigen::Vector2d ep = orig_path[(i + 1 % orig_path.size())];
    //         Eigen::Vector2d np = raytrace(obstacles, sp, ep, epsilon);
    //         if (np != ep) {
    //             //we need to handle this, hit an obstacle
    //             int issue_obs = -1;
    //             for(int j = 0; j < obstacles.size(); j++) {
    //                 if(raycastObstacleDetection(obstacles[j], np) && std::find(obs_ignore.begin(), obs_ignore.end(), j) == obs_ignore.end()) {
    //                     issue_obs = i;
    //                 }
    //             }
    //             if(issue_obs != -1) {

    //             }
    //             np = interp(sp, np, epsilon);

    //         }
            
    //     }

    //     //if we do, find a path around that obstacle.
        
    //     //follow that until we re-intersect with our original obstacle, and return the new path. then continue.
    //     std::unordered_set<int> visited;
    //     std::queue<std::pair<int, Eigen::Vector2d>> to_visit;

    //     to_visit.push_back({start_idx, start});
    //     visited.push_back(start_idx);

    //     while(!to_visit.empty()) {
            

    //     }
    // }
};