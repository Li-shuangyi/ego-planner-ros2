#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    rclcpp::Node::SharedPtr node;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr failed_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr a_star_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr guide_vector_pub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intermediate_pt0_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intermediate_pt1_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad0_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad1_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad_smoo_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad_dist_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad_feas_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad_swarm_pub;

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(rclcpp::Node::SharedPtr &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayMultiOptimalPathList(vector<vector<Eigen::Vector3d>> optimal_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    
    void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);
    void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);
    // void displayNewArrow(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &guide_vector_pub, ego_planner::PolyTrajOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif
