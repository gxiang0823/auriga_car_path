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

#pragma once

#include "trajectory_nlp.h"

#include "math/aabox2d.h"
#include "math/polygon2d.h"

#include "discretized_trajectory.h"
#include "cartesian_planner_config.h"

namespace cartesian_planner {

using math::Polygon2d;
using math::AABox2d;
using math::Box2d;


class TrajectoryOptimizer {
public:
  TrajectoryOptimizer(const CartesianPlannerConfig &config, const Env &env);

  bool OptimizeIteratively(const DiscretizedTrajectory &coarse, const Constraints &constraints, States &result);

private:
  CartesianPlannerConfig config_;
  Env env_;
  VehicleParam vehicle_;
  TrajectoryNLP nlp_;

  void CalculateInitialGuess(States &states) const;

  bool FormulateCorridorConstraints(States &states, Constraints &constraints);

  bool GenerateBox(double time, double &x, double &y, double radius, AABox2d &result) const;

  // //判断盒子是否在time时刻与动静态障碍物以及道路边界碰撞
  // inline bool CheckCollision(double time, double x, double y, const AABox2d &bound) const {
  //   Box2d box(bound);
  //   box.Shift({x, y});
  //   //初始的box移动到xy的位置

  //   return env_->CheckCollision(time, box);//判断盒子是否在time时刻与动静态障碍物以及道路边界碰撞
  // }

  inline bool CheckCollision(double time, double x, double y, const AABox2d &bound) const {
    int height = env_->occupancy_grid_.info.height;
    int width = env_->occupancy_grid_.info.width;
    double resolution = env_->occupancy_grid_.info.resolution;    

    int rectStartX = floor((width / 2) + (bound.min_x() + x) / resolution);
    int rectStartY = floor((height / 2) - (bound.max_y() + y) / resolution);
    int rectEndX = ceil((width / 2) + (bound.max_x() + x) / resolution);
    int rectEndY = ceil((height / 2) - (bound.min_y() + y) / resolution);
    // std::cout << height << ", " << width << ","<<  resolution<< ","<< x << ","<< y <<std::endl;
    // std::cout << rectStartX << ", " << rectStartY << ","<<  rectStartY<< ","<<  rectStartY<<std::endl;

    // 遍历矩形框内的每个栅格
    for (int y = rectStartY; y <= rectEndY; ++y) {
        for (int x = rectStartX; x <= rectEndX; ++x) {
            // 计算一维数组中的索引
            int index = (height - y) * width + x - 1;
            // std::cout << sizeof(env_->occupancy_grid_.data)/sizeof(env_->occupancy_grid_.data[0]) - 1 << ", " << index << ","<<  rectStartY<<std::endl;
            // std::cout << height << ", " << width << ","<< resolution << ","<< rectStartX << ","<< rectStartY<< ","<< rectEndX<< ","<< rectEndY  << ","<<((width / 2) + bound.min_x() / resolution)<<std::endl;
            // if (index > sizeof(env_->occupancy_grid_.data)/sizeof(env_->occupancy_grid_.data[0]) - 1) {
            //   return true;
            // }
            // 检查该栅格是否有障碍物
            if (env_->occupancy_grid_.data[index] != 0) {
                return true; // 如果存在障碍物，返回 true
            }
        }
    }
    // 如果矩形框内没有任何栅格有障碍物，则返回 false
    return false;
  }

};

}