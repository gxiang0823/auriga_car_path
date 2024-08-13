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

#include "cartesian_planner/trajectory_optimizer.h"

#include <bitset>

#include "cartesian_planner/math/math_utils.h"
#include "cartesian_planner/visualization/plot.h"

namespace cartesian_planner {

TrajectoryOptimizer::TrajectoryOptimizer(const CartesianPlannerConfig &config, const Env &env)
  : config_(config), env_(env), nlp_(config) {
  vehicle_ = config_.vehicle;
}

//根据初解、约束迭代求解最优化问题得到最终结果
bool TrajectoryOptimizer::OptimizeIteratively(const DiscretizedTrajectory &coarse, const Constraints &constraints,
                                              States &result) {
  States guess;
  for (auto &pt: coarse.data()) {
    guess.x.push_back(pt.x);
    guess.y.push_back(pt.y);
    guess.theta.push_back(pt.theta);
  }
  CalculateInitialGuess(guess);//根据粗解中的xytheta信息计算：v、phi、a、omega、jerk信息

  int iter = 0;
  double w_penalty = config_.opti_w_penalty0;//处罚权重初始为1e5，后面每次迭代会*10

  Constraints iterative_constraints = constraints;

  //开始迭代求解最优化问题
  while (iter < config_.opti_iter_max) {
    //根据states信息，生成前后圆心的可行驶区域constraints，放置于：front_bound、rear_bound
    FormulateCorridorConstraints(guess, iterative_constraints);

    // std::cout << "FormulateCorridorConstraints" << std::endl;

    double cur_infeasibility = nlp_.SolveIteratively(w_penalty, iterative_constraints, guess, coarse, guess);
    visualization::Plot(guess.x, guess.y, 0.05, visualization::Color::Red, iter, "Intermediate Trajectory");
    visualization::Trigger();

    // ROS_INFO("iter = %d, cur_infeasibility = %f, w_penalty = %f", iter, cur_infeasibility, w_penalty);

    if (cur_infeasibility < config_.opti_varepsilon_tol) {
      result = guess;
      return true;
    } else {
      w_penalty *= config_.opti_alpha;
      iter++;
    }
  }

  return false;
}

//函数只使用了一次，为了通过粗解中的xytheta信息计算：v、phi、a、omega、jerk信息
void TrajectoryOptimizer::CalculateInitialGuess(States &states) const {
  
  //补充粗解中的速度与横摆角信息
  states.v.resize(config_.nfe, 0.0);
  states.phi.resize(config_.nfe, 0.0);
  double hi = config_.tf / (config_.nfe - 1);//每个步长中的时间长度为hi
  for (size_t i = 1; i < states.x.size(); i++) {
    //利用此点和上一时刻点的欧氏距离计算速度
    double velocity = hypot(states.y[i] - states.y[i - 1], states.x[i] - states.x[i - 1]) / hi;
    states.v[i] = std::min(vehicle_.max_velocity, velocity);
    //计算横摆角
    states.phi[i] = std::min(vehicle_.phi_max, std::max(-vehicle_.phi_max, atan(
      (states.theta[i] - states.theta[i - 1]) * vehicle_.wheel_base / (states.v[i] * hi))));
  }

  //补充粗解中的加速度与横摆角速度信息
  states.a.resize(config_.nfe, 0.0);
  states.omega.resize(config_.nfe, 0.0);
  for (size_t i = 1; i < states.x.size(); i++) {
    states.a[i] = std::min(vehicle_.max_acceleration,
                           std::max(vehicle_.min_acceleration, (states.v[i] - states.v[i - 1]) / hi));
    states.omega[i] = std::min(vehicle_.omega_max,
                               std::max(-vehicle_.omega_max, (states.phi[i] - states.phi[i - 1]) / hi));
  }

  //补充粗解中的加加速度信息信息
  states.jerk.resize(config_.nfe, 0.0);
  for (size_t i = 1; i < states.x.size(); i++) {
    states.jerk[i] = std::min(vehicle_.jerk_max, std::max(-vehicle_.jerk_max, (states.a[i] - states.a[i - 1]) / hi));
  }
}

//根据states信息，生成前后圆心的可行驶区域constraints，放置于：front_bound、rear_bound
bool TrajectoryOptimizer::FormulateCorridorConstraints(States &states, Constraints &constraints) {
  constraints.front_bound.resize(config_.nfe);
  constraints.rear_bound.resize(config_.nfe);
  states.xf.resize(config_.nfe);
  states.yf.resize(config_.nfe);
  states.xr.resize(config_.nfe);
  states.yr.resize(config_.nfe);

  double hi = config_.tf / (config_.nfe - 1);

  for (size_t i = 0; i < config_.nfe; i++) {
    double time = hi * i;
    //两个中心点分别设置为整车长的0.75与0.25位置，计算这两点的xy坐标，分别用r和f标识
    std::tie(states.xf[i], states.yf[i], states.xr[i], states.yr[i]) = vehicle_.GetDiscPositions(states.x[i],
                                                                                                 states.y[i],
                                                                                                 states.theta[i]);
    
    // std::cout << states.x[i] << ", " << states.y[i] << ", " << states.theta[i] << std::endl;
    // std::cout << states.xf[i] << ", " << states.yf[i] << ", " << states.xr[i] << ", " << states.yr[i] << std::endl;
    
    //生成前圆心的可行驶区域
    math::AABox2d box;
    if (!GenerateBox(time, states.xf[i], states.yf[i], vehicle_.radius, box)) {
      return false;
    }
    constraints.front_bound[i] = {box.min_x(), box.max_x(), box.min_y(), box.max_y()};

    visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.01, visualization::Color::Yellow, i, "Front Corridor");

    //生成后圆心的可行驶区域
    if (!GenerateBox(time, states.xr[i], states.yr[i], vehicle_.radius, box)) {
      return false;
    }
    constraints.rear_bound[i] = {box.min_x(), box.max_x(), box.min_y(), box.max_y()};

    visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.01, visualization::Color::White, i, "Rear Corridor");
  }

  // visualization::Trigger();

  return true;
}

//生成可行驶区域的形状
bool TrajectoryOptimizer::GenerateBox(double time, double &x, double &y, double radius, AABox2d &result) const {
  // double ri = radius;
  double ri = 0.001;
  AABox2d bound({-ri, -ri}, {ri, ri});//创造一个原点为中心的，边长为2radius的正方形

  //判断盒子是否在time时刻与动静态障碍物以及道路边界碰撞，若碰撞则四个方向扩展寻找新的中心点
  if (CheckCollision(time, x, y, bound)) {
    // initial condition not satisfied, involute to find feasible box：碰撞则需要重新寻找位置

    // std::cout << "xyold: " << x << ", " << y << std::endl;
    int inc = 4;//迭代次数，用4是因为想要第一轮就有操作
    double real_x, real_y;

    do {
      int iter = inc / 4;
      uint8_t edge = inc % 4;

      real_x = x;
      real_y = y;
      if (edge == 0) {
        real_x = x - iter * 0.05;
      } else if (edge == 1) {
        real_x = x + iter * 0.05;
      } else if (edge == 2) {
        real_y = y - iter * 0.05;
      } else if (edge == 3) {
        real_y = y + iter * 0.05;
      }

      inc++;
    } while (CheckCollision(time, real_x, real_y, bound) && inc < config_.corridor_max_iter);

    // std::cout << "inc: " << inc << std::endl;
    
    if (inc > config_.corridor_max_iter) {
      return false;//1000次寻找后还未找到结果，则失败
    }

    x = real_x;
    y = real_y;

    //std::cout << "xynew: " << x << ", " << y << std::endl;  
  }

  //通过四个方向扩张来获得可行驶区域
  int inc = 4;
  std::bitset<4> blocked;//判断是否各个方向是否可以扩张，能扩为0，不能扩为1
  double incremental[4] = {0.0};//四个方向扩张的大小，step的整数倍
  double step = radius * 0.1;//每次扩张一个方向只增加一个step

  do {
    int iter = inc / 4;//表明迭代次数
    uint8_t edge = inc % 4;//表明扩张方向
    inc++;

    if (blocked[edge]) continue;//表明这个方向已经无法扩张了

    incremental[edge] = iter * step;

    AABox2d test({-ri - incremental[0], -ri - incremental[2]},
                 {ri + incremental[1], ri + incremental[3]});

    //如果碰撞则该方向以后不扩张且曾经扩张的内容退回
    if (CheckCollision(time, x, y, test) || incremental[edge] >= config_.corridor_incremental_limit) {
      incremental[edge] -= step;
      blocked[edge] = true;
    }

  } while (!blocked.all() && inc < config_.corridor_max_iter);

  if (inc > config_.corridor_max_iter) {
    return false;
  }

  //可行驶区域的矩形形状
  result = {{x - incremental[0], y - incremental[2]},
            {x + incremental[1], y + incremental[3]}};
  return true;
}
}