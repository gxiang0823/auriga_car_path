/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include "cartesian_planner/discretized_trajectory.h"
#include "cartesian_planner/math/math_utils.h"

#include <algorithm>

namespace cartesian_planner {

//初始化，将中心线信息放入data_中
DiscretizedTrajectory::DiscretizedTrajectory(const DiscretizedTrajectory &rhs, size_t begin, size_t end) {
  if (end < 0) {
    end = rhs.data_.size();
  }
  data_.resize(end - begin);
  std::copy_n(std::next(rhs.data_.begin(), begin), data_.size(), data_.begin());
}

DiscretizedTrajectory::DataType::const_iterator
//根据s坐标获取第一个比其大的TrajectoryPoint的迭代器
DiscretizedTrajectory::QueryLowerBoundStationPoint(double station) const {
  //对比station和data_的第一个和最后一个s的值
  if (station >= data_.back().s) {
    return data_.end() - 1;
  } else if (station < data_.front().s) {
    return data_.begin();
  }
  //查找到第一个比s坐标大的迭代器
  return std::lower_bound(
    data_.begin(), data_.end(), station,
    [](const TrajectoryPoint &t, double station) {
      return t.s < station;
    });
}

//已知对应的s信息，根据前一个p0轨迹点与后一个p1轨迹点信息，获得该s坐标点TrajectoryPoint信息，利用插值法
TrajectoryPoint LinearInterpolateTrajectory(const TrajectoryPoint &p0, const TrajectoryPoint &p1, double s) {
  double s0 = p0.s;
  double s1 = p1.s;
  if (std::abs(s1 - s0) < math::kMathEpsilon) {
    return p0;
  }

  TrajectoryPoint pt;
  double weight = (s - s0) / (s1 - s0);
  pt.s = s;
  pt.x = (1 - weight) * p0.x + weight * p1.x;
  pt.y = (1 - weight) * p0.y + weight * p1.y;
  pt.theta = math::slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  pt.kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
  pt.left_bound = (1 - weight) * p0.left_bound + weight * p1.left_bound;
  pt.right_bound = (1 - weight) * p0.right_bound + weight * p1.right_bound;

  return pt;
}

//根据s坐标获得TrajectoryPoint的信息
TrajectoryPoint DiscretizedTrajectory::EvaluateStation(double station) const {
  auto iter = QueryLowerBoundStationPoint(station);

  if (iter == data_.begin()) {
    iter = std::next(iter);
  }

  auto prev = std::prev(iter, 1);

  return LinearInterpolateTrajectory(*prev, *iter, station);
}

DiscretizedTrajectory::DataType::const_iterator
DiscretizedTrajectory::QueryNearestPoint(const Vec2d &point, double *out_distance) const {
  auto nearest_iter = data_.begin();
  double nearest_distance = std::numeric_limits<double>::max();

  for (auto iter = data_.begin(); iter != data_.end(); iter++) {
    double dx = iter->x - point.x(), dy = iter->y - point.y();
    double distance = dx * dx + dy * dy;
    if (distance < nearest_distance) {
      nearest_iter = iter;
      nearest_distance = distance;
    }
  }

  if (out_distance != nullptr) {
    *out_distance = sqrt(nearest_distance);
  }
  return nearest_iter;
}

Vec2d DiscretizedTrajectory::GetProjection(const Vec2d &xy) const {
  long point_idx = std::distance(data_.begin(), QueryNearestPoint(xy));
  auto project_point = data_[point_idx];
  int index_start = std::max(0l, point_idx - 1);
  int index_end = std::min(data_.size() - 1, (ulong) point_idx + 1);

  if (index_start < index_end) {
    double v0x = xy.x() - data_[index_start].x;
    double v0y = xy.y() - data_[index_start].y;

    double v1x = data_[index_end].x - data_[index_start].x;
    double v1y = data_[index_end].y - data_[index_start].y;

    // ROS_INFO("1:%f,2:%f",data_[index_end] , index_end);
    // std::cerr << data_.size() << std::endl;

    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    // project_point = LinearInterpolateTrajectory(data_[index_start], data_[index_end], data_[index_start].s + delta_s);
  }

  double nr_x = xy.x() - project_point.x, nr_y = xy.y() - project_point.y;
  double lateral = copysign(hypot(nr_x, nr_y), nr_y * cos(project_point.theta) - nr_x * sin(project_point.theta));
  return {project_point.s, lateral};
  // return {0, 0};
}

// 根据获得sl坐标点获得xy信息
Vec2d DiscretizedTrajectory::GetCartesian(double station, double lateral) const {
  auto ref = EvaluateStation(station);
  return {ref.x - lateral * sin(ref.theta), ref.y + lateral * cos(ref.theta)};
}

}
