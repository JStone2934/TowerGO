#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class ScanProcessor : public rclcpp::Node {
  using Cloud = sensor_msgs::msg::PointCloud2;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<Cloud>::SharedPtr front_,back_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  Cloud::ConstSharedPtr pending_front_,pending_back_;
  std::string source_; double tolerance_; size_t dropped_=0;
  double latest_front_=-1.,latest_back_=-1.;
  void diagnostic(const std::string& message, uint8_t level=1) {
    diagnostic_msgs::msg::DiagnosticArray m; m.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s; s.name="towergo/scan_processor"; s.level=level;
    s.message=message+"; dropped="+std::to_string(dropped_); m.status.push_back(s); diagnostics_->publish(m);
  }
  void receive(Cloud::ConstSharedPtr m,bool front) {
    auto t=rclcpp::Time(m->header.stamp).seconds(); auto current=now().seconds();
    if (current-t>.5 || t-current>.05) { ++dropped_; diagnostic("Stale/future cloud"); return; }
    auto &last=front?latest_front_:latest_back_;
    if(t<last) {pending_front_.reset();pending_back_.reset();latest_front_=latest_back_=-1.;++dropped_;diagnostic("Clock/order reset; buffers cleared");}
    last=t;
    if(source_=="front") { if(front) publish(m,nullptr); return; }
    auto &pending=front?pending_front_:pending_back_;
    if(pending) ++dropped_;
    pending=m;
    if(!pending_front_ || !pending_back_) return;
    double delta=(rclcpp::Time(pending_front_->header.stamp)-rclcpp::Time(pending_back_->header.stamp)).seconds();
    if(std::abs(delta)>tolerance_) {
      if(delta<0) pending_front_.reset(); else pending_back_.reset();
      ++dropped_; diagnostic("Cloud pair exceeds synchronization tolerance"); return;
    }
    publish(pending_front_,pending_back_); pending_front_.reset();pending_back_.reset();
  }
  void publish(Cloud::ConstSharedPtr front,Cloud::ConstSharedPtr back) {
    sensor_msgs::msg::LaserScan scan;
    scan.header=front->header; scan.header.frame_id=front->header.frame_id;
    scan.angle_min=-M_PI; scan.angle_increment=2*M_PI/720.;
    scan.angle_max=scan.angle_min+719*scan.angle_increment;
    scan.range_min=.15; scan.range_max=12.; scan.scan_time=.1; scan.time_increment=0.;
    scan.ranges.assign(720,std::numeric_limits<float>::infinity());
    std::vector<Cloud::ConstSharedPtr> clouds{front};if(back)clouds.push_back(back);
    try {
      auto ref=buffer_->lookupTransform(scan.header.frame_id,"base_link",rclcpp::Time(front->header.stamp));
      const auto &q=ref.transform.rotation; const auto &v=ref.transform.translation;
      tf2::Transform to_reference(tf2::Quaternion(q.x,q.y,q.z,q.w),tf2::Vector3(v.x,v.y,v.z));
      for(const auto &input:clouds) {
        // Each cloud is transformed at its own timestamp. No latest-TF fallback.
        auto tf=buffer_->lookupTransform("base_link",input->header.frame_id,rclcpp::Time(input->header.stamp));
        Cloud transformed; tf2::doTransform(*input,transformed,tf);
        sensor_msgs::PointCloud2ConstIterator<float> x(transformed,"x"),y(transformed,"y"),z(transformed,"z");
        for(;x!=x.end();++x,++y,++z) {
          if(!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z)||*z<.15||*z>1.5)continue;
          if(std::abs(*x)<=.4 && std::abs(*y)<=.35)continue;
          auto point=to_reference*tf2::Vector3(*x,*y,*z);
          auto range=static_cast<float>(std::hypot(point.x(),point.y())); if(range<scan.range_min||range>scan.range_max)continue;
          double angle=std::atan2(point.y(),point.x());
          int i=static_cast<int>(std::lround((angle-scan.angle_min)/scan.angle_increment));
          i=((i%720)+720)%720;
          // Preserve the front sensor's true ray origin. Rear data may add a
          // closer obstacle on an observed front ray, never extend its free space.
          if(input==back && !std::isfinite(scan.ranges[i]))continue;
          scan.ranges[i]=std::min(scan.ranges[i],range);
        }
      }
      scan_->publish(scan);diagnostic("Scan ready",0);
    } catch(const std::exception &e) {++dropped_;diagnostic(std::string("Missing transform/fields: ")+e.what(),2);}
  }
public:
  ScanProcessor():Node("scan_processor") {
    source_=declare_parameter("scan_source",std::string("front"));
    tolerance_=declare_parameter("sync_tolerance",.02);
    if(source_!="front" && source_!="dual")throw std::invalid_argument("scan_source must be front or dual");
    if(tolerance_<0 || tolerance_>.1)throw std::invalid_argument("invalid sync_tolerance");
    buffer_=std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_);
    scan_=create_publisher<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS());
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    front_=create_subscription<Cloud>("/livox/lidar_front",rclcpp::SensorDataQoS(),[this](Cloud::ConstSharedPtr m){receive(m,true);});
    back_=create_subscription<Cloud>("/livox/lidar_back",rclcpp::SensorDataQoS(),[this](Cloud::ConstSharedPtr m){receive(m,false);});
  }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<ScanProcessor>());rclcpp::shutdown();}
