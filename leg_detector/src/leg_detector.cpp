#include <rclcpp/rclcpp.hpp>
#include <leg_detector/LegDetectorConfig.hpp>
#include <leg_detector/laser_processor.hpp>
#include <leg_detector/calc_leg_features.hpp>

#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>

#include <people_msgs/msg/position_measurement.hpp>
#include <people_msgs/msg/position_measurement_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <tf2/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.hpp>

#include <people_tracking_filter/tracker_kalman.hpp>
#include <people_tracking_filter/state_pos_vel.hpp>
#include <people_tracking_filter/rgb.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <list>
#include <set>
#include <string>
#include <vector>

using namespace laser_processor;
using namespace rclcpp;
using namespace tf2;
using namespace estimation;
using namespace BFL;
using namespace MatrixWrapper;

static double no_observation_timeout_s = 0.5;
static double max_second_leg_age_s = 2.0;
static double max_track_jump_m = 1.0;
static double max_meas_jump_m = 0.75;  // 1.0
static double leg_pair_separation_m = 1.0;
static std::string fixed_frame = "odom_combined";

static double kal_p = 4, kal_q = .002, kal_r = 10;
static bool use_filter = true;

class SavedFeature
{
public:
  static int nextid;
  tf2_ros::Buffer& tf_buffer_;

  BFL::StatePosVel sys_sigma_;
  TrackerKalman filter_;

  std::string id_;
  std::string object_id;
  rclcpp::Time time_;
  rclcpp::Time meas_time_;

  double reliability, p;

  geometry_msgs::msg::PointStamped position_;
  SavedFeature* other;
  float dist_to_person_;

  // one leg tracker
  SavedFeature(geometry_msgs::msg::PointStamped loc, tf2_ros::Buffer& tf_buffer)
    : tf_buffer_(tf_buffer),
      sys_sigma_(Vector3(0.05, 0.05, 0.05), Vector3(1.0, 1.0, 1.0)),
      filter_("tracker_name", sys_sigma_),
      reliability(-1.), p(4)
  {
    char id[100];
    snprintf(id, 100, "legtrack%d", nextid++);
    id_ = std::string(id);

    object_id = "";
    time_ = loc.header.stamp;
    meas_time_ = loc.header.stamp;
    other = nullptr;

    try
    {
      tf2::doTransform(loc, loc, tf_buffer_);
    }
    catch (...)
    {
      RCLCPP_WARN(rclcpp::get_logger("LegDetector"), "TF exception spot 6.");
    }
    geometry_msgs::msg::TransformStamped pose;
    pose.header.stamp = loc.header.stamp;
    pose.header.frame_id = loc.header.frame_id;
    pose.child_frame_id = id_;
    pose.transform.translation = loc.point;
    pose.transform.rotation.w = 1.0;
    tf_buffer_.setTransform(pose);

    StatePosVel prior_sigma(Vector3(0.1, 0.1, 0.1), Vector3(0.0000001, 0.0000001, 0.0000001));
    filter_.initialize(loc, prior_sigma, time_.seconds());

    StatePosVel est;
    filter_.getEstimate(est);

    updatePosition();
  }

  void propagate(rclcpp::Time time)
  {
    time_ = time;
    filter_.updatePrediction(time.seconds());
    updatePosition();
  }

  void update(geometry_msgs::msg::PointStamped loc, double probability)
  {
    geometry_msgs::msg::TransformStamped pose;
    pose.header.stamp = loc.header.stamp;
    pose.header.frame_id = loc.header.frame_id;
    pose.child_frame_id = id_;
    pose.transform.translation = loc.point;
    pose.transform.rotation.w = 1.0;
    tf_buffer_.setTransform(pose);

    meas_time_ = loc.header.stamp;
    time_ = meas_time_;

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = 0.0025;
    cov(1, 1) = 0.0025;
    cov(2, 2) = 0.0025;

    filter_.updateCorrection(loc, cov);

    updatePosition();

    if (reliability < 0 || !use_filter)
    {
      reliability = probability;
      p = kal_p;
    }
    else
    {
      p += kal_q;
      double k = p / (p + kal_r);
      reliability += k * (probability - reliability);
      p *= (1 - k);
    }
  }

  double getLifetime()
  {
    return filter_.getLifetime();
  }

  double getReliability()
  {
    return reliability;
  }

private:
  void updatePosition()
  {
    StatePosVel est;
    filter_.getEstimate(est);

    position_.point.x = est.pos_[0];
    position_.point.y = est.pos_[1];
    position_.point.z = est.pos_[2];
    position_.header.stamp = time_;
    position_.header.frame_id = fixed_frame;
    double nreliability = std::min(1.0, std::max(0.1, est.vel_.length() / 0.5));
  }
};

int SavedFeature::nextid = 0;

class MatchedFeature
{
public:
  SampleSet* candidate_;
  SavedFeature* closest_;
  float distance_;
  double probability_;

  MatchedFeature(SampleSet* candidate, SavedFeature* closest, float distance, double probability)
    : candidate_(candidate)
    , closest_(closest)
    , distance_(distance)
    , probability_(probability)
  {}

  inline bool operator< (const MatchedFeature& b) const
  {
    return (distance_ <  b.distance_);
  }
};

class LegDetector : public rclcpp::Node
{
public:
  explicit LegDetector(const rclcpp::NodeOptions& options)
    : Node("leg_detector", options),
      mask_count_(0),
      feat_count_(0),
      next_p_id_(0),
      people_sub_(this, "people_tracker_filter"),
      laser_sub_(this, "scan")
  {
    forest = cv::ml::RTrees::create();

    std::string feature_file = "/path/to/forest.xml"; // Update this with actual path
    forest = cv::ml::StatModel::load<cv::ml::RTrees>(feature_file);
    feat_count_ = forest->getVarCount();
    RCLCPP_INFO(this->get_logger(), "Loaded forest with %d features: %s", feat_count_, feature_file.c_str());

    this->declare_parameter("use_seeds", true);
    this->get_parameter("use_seeds", use_seeds_);

    // advertise topics
    leg_measurements_pub_ = this->create_publisher<people_msgs::msg::PositionMeasurementArray>("leg_tracker_measurements", 10);
    people_measurements_pub_ = this->create_publisher<people_msgs::msg::PositionMeasurementArray>("people_tracker_measurements", 10);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    if (use_seeds_)
    {
      people_sub_.register_callback(std::bind(&LegDetector::peopleCallback, this, std::placeholders::_1));
    }

    laser_sub_.register_callback(std::bind(&LegDetector::laserCallback, this, std::placeholders::_1));

    // Não achei nada sobre esse dynamic no ros2: https://github.com/tier4/AutowareArchitectureProposal.proj/blob/main/docs/developer_guide/knowhow/PortingToROS2.md#dynamic_reconfigure
  }

  ~LegDetector() {}

private:

  double distance(std::list<SavedFeature*>::iterator it1,  std::list<SavedFeature*>::iterator it2)
  {

      geometry_msgs::msg::PointStamped one = (*it1)->position_;
      geometry_msgs::msg::PointStamped two = (*it2)->position_;
      
      double dx = one.point.x - two.point.x;
      double dy = one.point.y - two.point.y;
      double dz = one.point.z - two.point.z;

      return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  void peopleCallback(const people_msgs::msg::PositionMeasurementArray::SharedPtr msg)
  {
    // Handle people callback
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Handle laser callback
  }

  rclcpp::Publisher<people_msgs::msg::PositionMeasurementArray>::SharedPtr people_measurements_pub_;
  rclcpp::Publisher<people_msgs::msg::PositionMeasurementArray>::SharedPtr leg_measurements_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_pub_;

  message_filters::Subscriber<people_msgs::msg::PositionMeasurement> people_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cv::Ptr<cv::ml::RTrees> forest;
  int mask_count_;
  int feat_count_;
  bool use_seeds_;
  int next_p_id_;
  double leg_reliability_limit_;
  int min_points_per_group;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegDetector>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
