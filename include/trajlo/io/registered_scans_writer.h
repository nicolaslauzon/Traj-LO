/**
LIO Bench output patch (MIT, same terms as the rest of Traj-LO).

Writes Traj-LO's own deskewed points to a ROS 1 bag, one sensor_msgs/PointCloud2
per input LiDAR scan on /registered_scans: fields x, y, z, intensity (float32),
frame_id "world", stamped with the scan's first-point time. Output only: nothing
here feeds back into the odometry.

Traj-LO optimizes time segments, not scans, so points arrive per segment; each
point goes back to the scan it came from by its timestamp, and a scan is written
once points of a later scan arrive (points come in time order).
*/

#ifndef TRAJLO_REGISTERED_SCANS_WRITER_H
#define TRAJLO_REGISTERED_SCANS_WRITER_H

#include <algorithm>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>

namespace traj {
class RegisteredScansWriter {
 public:
  using Ptr = std::shared_ptr<RegisteredScansWriter>;

  explicit RegisteredScansWriter(const std::string &path) {
    bag_.open(path, rosbag::bagmode::Write);
  }
  ~RegisteredScansWriter() { Close(); }

  // A scan entering the odometry: its first point time (s) and stamp (ns).
  void AddScan(double first_ts, int64_t stamp_ns) {
    starts_.push_back(first_ts);
    stamps_.push_back(stamp_ns);
  }

  // One deskewed point in the world frame, with its raw time (s) and intensity.
  void AddPoint(double ts, const Eigen::Vector3f &p_w, float intensity) {
    if (starts_.empty() || !p_w.allFinite()) return;
    long k = std::upper_bound(starts_.begin(), starts_.end(), ts) -
             starts_.begin() - 1;
    k = std::max(k, 0L);
    if (k != current_) {
      Flush();
      current_ = k;
    }
    buffer_.insert(buffer_.end(), {p_w.x(), p_w.y(), p_w.z(), intensity});
  }

  void Close() {
    if (closed_) return;
    Flush();
    bag_.close();
    closed_ = true;
    std::cout << "Registered Scans: " << written_ << " of " << starts_.size()
              << " scans written" << std::endl;
  }

 private:
  void Flush() {
    if (buffer_.empty()) return;
    sensor_msgs::PointCloud2 msg;
    msg.header.seq = written_;
    msg.header.stamp.fromNSec(stamps_[current_]);
    msg.header.frame_id = "world";
    const char *names[] = {"x", "y", "z", "intensity"};
    for (uint32_t i = 0; i < 4; ++i) {
      sensor_msgs::PointField f;
      f.name = names[i];
      f.offset = 4 * i;
      f.datatype = sensor_msgs::PointField::FLOAT32;
      f.count = 1;
      msg.fields.push_back(f);
    }
    msg.height = 1;
    msg.width = buffer_.size() / 4;
    msg.is_bigendian = false;
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
    msg.data.resize(buffer_.size() * sizeof(float));
    std::memcpy(msg.data.data(), buffer_.data(), msg.data.size());
    bag_.write("/registered_scans", msg.header.stamp, msg);
    buffer_.clear();
    written_++;
  }

  rosbag::Bag bag_;
  std::vector<double> starts_;
  std::vector<int64_t> stamps_;
  std::vector<float> buffer_;  // x, y, z, intensity of the current scan
  long current_ = -1;
  uint32_t written_ = 0;
  bool closed_ = false;
};
}  // namespace traj

#endif  // TRAJLO_REGISTERED_SCANS_WRITER_H
