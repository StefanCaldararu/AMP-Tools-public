#include "AMPCore.h"
#include <Eigen/Core>
namespace utils {
    bool sensor(double epsilon, double theta, std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d point);
    Eigen::Vector2d raytrace(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d q_init, Eigen::Vector2d q_goal, double epsilon = 1e-1);

    bool raycastObstacleDetection(const amp::Obstacle2D& obstacle, const Eigen::Vector2d& point);
    Eigen::Vector2d interp(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double dist);
    Eigen::Vector2d intersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const amp::Obstacle2D& obstacle);

    std::vector<Eigen::Vector2d> offsetObstaclePath(const amp::Obstacle2D& obstacle, double epsilon, const Eigen::Vector2d& start);

    Eigen::Vector2d closestPointLine(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& p);

    void recurCircumnavObstacle(const std::vector<amp::Obstacle2D>& obstacles, double epsilon, const Eigen::Vector2d& start, const Eigen::Vector2d& curr_point, int curr_obstacle_index, std::vector<Eigen::Vector2d>& path);
    
    std::vector<Eigen::Vector2d> circumnavObstacle(const std::vector<amp::Obstacle2D>& obstacles, double epsilon, const Eigen::Vector2d& start, int curr_obstacle_index);
    
    void circumnavObs(std::vector<Eigen::Vector2d>& path, const std::vector<amp::Obstacle2D>& obstacles, int curr_obs, Eigen::Vector2d start, double epsilon, bool first_step, int call_count);
    
    int findIssueObs(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d start, Eigen::Vector2d end);
    // std::vector<Eigen::Vector2d> offsetMergedObstaclePath(const std::vector<amp::Obstacle2D>& obstacles, double epsilon, const Eigen::Vector2d& start, int start_obstacle_index, std::vector<int> obs_ignore);
};
