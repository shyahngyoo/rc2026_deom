// Test-only native equivalent of FAST-LIO's synchronized C++ input publishers.
// It reads an ASCII PCD, expresses its points at a known stationary sensor pose,
// and publishes only under /test_real_gicp. No motion or accepted pose is emitted.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <sstream>
#include <array>
#include <cstring>
#include <cmath>
#include <vector>

class SyntheticInputs : public rclcpp::Node {
public:
  SyntheticInputs() : Node("synthetic_gicp_inputs") {
    const auto path=declare_parameter<std::string>("map_path", "");
    std::ifstream file(path);
    if(!file) throw std::runtime_error("test PCD not found");
    std::string line; bool data=false;
    std::vector<std::array<float,3>> points;
    while(std::getline(file,line)) {
      if(!data) {if(line=="DATA ascii") data=true;continue;}
      std::istringstream values(line);float x,y,z;
      if(values>>x>>y>>z) {
        if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) throw std::runtime_error("invalid test PCD");
        points.push_back({x-1.0f,y,z}); // map-to-body truth is +1 m on X.
      }
    }
    if(points.size()<100) throw std::runtime_error("insufficient test PCD points");
    cloud_.height=1;cloud_.width=points.size();cloud_.point_step=12;
    cloud_.row_step=cloud_.width*cloud_.point_step;cloud_.is_dense=true;
    cloud_.is_bigendian=false;cloud_.header.frame_id="livox_frame";
    for(size_t i=0;i<3;++i) {
      sensor_msgs::msg::PointField field;
      field.name=std::string(1,"xyz"[i]);field.offset=i*4;
      field.datatype=sensor_msgs::msg::PointField::FLOAT32;field.count=1;
      cloud_.fields.push_back(field);
    }
    cloud_.data.resize(cloud_.row_step);
    for(size_t i=0;i<points.size();++i) std::memcpy(cloud_.data.data()+i*12,points[i].data(),12);
    odom_.header.frame_id="odom";odom_.child_frame_id="livox_frame";
    odom_.pose.pose.position.x=0.3;odom_.pose.pose.orientation.w=1.0;
    cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/test_real_gicp/cloud",10);
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/test_real_gicp/odom",10);
    timer_=create_wall_timer(std::chrono::milliseconds(80),[this] {
      const auto stamp=now();
      if(stamp.nanoseconds()<=0) return;
      cloud_.header.stamp=stamp;odom_.header.stamp=stamp;
      cloud_pub_->publish(cloud_);odom_pub_->publish(odom_);
    });
  }
private:
  sensor_msgs::msg::PointCloud2 cloud_;
  nav_msgs::msg::Odometry odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<SyntheticInputs>());}
  catch(const std::exception &e) {RCLCPP_FATAL(rclcpp::get_logger("synthetic_gicp_inputs"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
