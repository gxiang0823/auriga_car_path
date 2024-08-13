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

#include "cartesian_planner/dp_planner.h"

#include <bitset>
#include <utility>
#include <sstream>

#include "cartesian_planner/math/math_utils.h"
#include "cartesian_planner/math/polygon2d.h"
#include "cartesian_planner/visualization/plot.h"

#include <cmath>

#include <iostream>

namespace cartesian_planner {

constexpr double kMathEpsilon = 1e-3;

DpPlanner::DpPlanner(const CartesianPlannerConfig &config, const Env &env)
  : env_(env), config_(config), nseg_(config.nfe / NT), unit_time_(config.tf / NT) {
  //: env_(env), config_(config), nseg_(config.nfe / NT), unit_time_(3) {
  time_ = math::LinSpaced<NT>(unit_time_, config.tf);//整个时域内的时间
  station_ = math::LinSpaced<NS>(0, unit_time_ * config_.vehicle.max_velocity);//和父亲之间的s逐步增加，从0到步长*最高速度
  lateral_ = math::LinSpaced<NL - 1>(0, 1);//是一个百分比，从0到1
  safe_margin_ = config_.vehicle.width / 2 * 1.5;//safety 
}

// double DpPlanner::GetCollisionCost(StateIndex parent_ind, StateIndex cur_ind) {
//   double parent_s = state_.start_s, grandparent_s = state_.start_s;
//   double last_l = state_.start_l, last_s = state_.start_s;
//   if (parent_ind.t >= 0) {
//     auto &cell = state_space_[parent_ind.t][parent_ind.s][parent_ind.l];
//     parent_s = cell.current_s;
//     if (parent_ind.t > 0) {
//       auto &parent_cell = state_space_[parent_ind.t - 1][cell.parent_s_ind][cell.parent_l_ind];
//       grandparent_s = parent_cell.current_s;
//     }
//     //获得离父亲点最近的一个点的信息
//     auto prev_path = InterpolateLinearly(grandparent_s, cell.parent_l_ind, parent_ind.s, parent_ind.l);
//     last_l = prev_path.back().y();
//     last_s = prev_path.back().x();
//   }
//   auto path = InterpolateLinearly(parent_s, parent_ind.l, cur_ind.s, cur_ind.l);//获得从父亲到此刻点的nseg_个点的sl坐标
//   // 逐个检查，从父节点到当前节点之间，nseg_个点是否碰撞
//   for (int i = 0; i < path.size(); i++) {
//     auto &pt = path[i];
//     double dl = pt.y() - last_l;
//     double ds = std::max(pt.x() - last_s, kMathEpsilon);
//     last_l = pt.y();
//     last_s = pt.x();
//     auto cart = env_->reference().GetCartesian(pt.x(), pt.y());//根据获得sl坐标点获得xy信息
//     auto ref = env_->reference().EvaluateStation(pt.x());//根据此时s坐标获得对应的参考线上点信息
//     double lb = std::min(0.0, -ref.right_bound + safe_margin_);
//     double ub = std::max(0.0, ref.left_bound - safe_margin_);
//     //与道路碰撞检测方案：如果l坐标超出上下限则直接返回最大值
//     if (pt.y() < lb - kMathEpsilon || pt.y() > ub + kMathEpsilon) {
//       return config_.dp_w_obstacle;
//     }
//     double heading = ref.theta + atan((dl / ds) / (1 - ref.kappa * pt.y()));
//     math::Pose pose(cart.x(), cart.y(), heading);
//     double parent_time = parent_ind.t < 0 ? 0.0 : time_[parent_ind.t];
//     double time = parent_time + i * (unit_time_ / nseg_);
//     //与行人碰撞检测方案：计算相关信息，若碰撞直接返回结果
//     if(env_->CheckOptimizationCollision(time, pose)) {
//       return config_.dp_w_obstacle;
//     }
//   }
//   return 0.0;  //若不碰撞，则直接返回0
// }

double DpPlanner::GetCollisionCost(StateIndex parent_ind, StateIndex cur_ind) {
  double parent_s = state_.start_s, grandparent_s = state_.start_s;
  double last_l = state_.start_l, last_s = state_.start_s;
  if (parent_ind.t >= 0) {
    auto &cell = state_space_[parent_ind.t][parent_ind.s][parent_ind.l];
    parent_s = cell.current_s;

    if (parent_ind.t > 0) {
      auto &parent_cell = state_space_[parent_ind.t - 1][cell.parent_s_ind][cell.parent_l_ind];
      grandparent_s = parent_cell.current_s;
    }
    //获得离父亲点最近的一个点的信息
    auto prev_path = InterpolateLinearly(grandparent_s, cell.parent_l_ind, parent_ind.s, parent_ind.l);
    last_l = prev_path.back().y();
    last_s = prev_path.back().x();
  }

  auto path = InterpolateLinearly(parent_s, parent_ind.l, cur_ind.s, cur_ind.l);//获得从父亲到此刻点的nseg_个点的sl坐标

  // 逐个检查，从父节点到当前节点之间，nseg_个点是否碰撞
  for (int i = 0; i < path.size(); i++) {
    auto &pt = path[i];

    auto cart = env_->reference().GetCartesian(pt.x(), pt.y());//根据获得sl坐标点获得xy信息
    
    //与栅格地图碰撞检测方案：对应格子是否为0
    if(cart.y() >= env_->occupancy_grid_.info.height * env_->occupancy_grid_.info.resolution / 2 ||
    cart.y() <= -1.0*env_->occupancy_grid_.info.height * env_->occupancy_grid_.info.resolution / 2 ||
    cart.x() >= env_->occupancy_grid_.info.width * env_->occupancy_grid_.info.resolution / 2 ||
    cart.x() <= -1.0*env_->occupancy_grid_.info.width * env_->occupancy_grid_.info.resolution / 2 ) {
      // std::cout << "xy:" <<  cart.x() << "," << cart.y() << std::endl;
      return config_.dp_w_obstacle;
    }
    
    int row = ceil(env_->occupancy_grid_.info.height / 2 - cart.y() / env_->occupancy_grid_.info.resolution);
    int col = ceil(cart.x() / env_->occupancy_grid_.info.resolution + env_->occupancy_grid_.info.width / 2);

    // std::cout << "col+row:" << row << "," << col << std::endl;

    if (env_->occupancy_grid_.data[(env_->occupancy_grid_.info.height - row) * env_->occupancy_grid_.info.width + col - 1] != 0) {
      // std::cout << "return config_.dp_w_obstacle;" << row << "," << col << std::endl;
        // if(pt.y() < 0.1 && pt.y() > -0.1) {
        //   std::cout << row << "," << col <<" = "<<pt.x()<<" "<<pt.y()<<"="<<cart.x() <<" "<<cart.y()<<std::endl;
        // }
      return config_.dp_w_obstacle;
    }

    // std::cout << "xy:" <<  cart.x() << "," << cart.y() << std::endl;

  }
  return 0.0;  //若不碰撞，则直接返回0
}

// 根据父亲的代价代价获得自己的代价：获取状态间的总成本和路径距离
std::pair<double, double> DpPlanner::GetCost(StateIndex parent_ind, StateIndex cur_ind) {
  // 根据父亲的代价代价获得自己的代价
  //初始化父亲和祖父信息
  double parent_s = state_.start_s, grandparent_s = state_.start_s;
  double parent_l = state_.start_l, grandparent_l = state_.start_l;

  if (parent_ind.t >= 0) {  // 如果存在父节点
    auto &cell = state_space_[parent_ind.t][parent_ind.s][parent_ind.l];  // 根据索引获取父节点的状态
    int grandparent_s_ind = cell.parent_s_ind;  // 获取祖父状态的s索引
    int grandparent_l_ind = cell.parent_l_ind;  // 获取祖父状态的l索引
    parent_s = cell.current_s;  // 更新父状态的s值
    parent_l = GetLateralOffset(parent_s, parent_ind.l);  // 获得父状态的l值

    if (parent_ind.t >= 1) {  // 如果父节点不是初始状态，将父节点t-1得到祖父节点的信息
      grandparent_s = state_space_[parent_ind.t - 1][grandparent_s_ind][grandparent_l_ind].current_s;  // 获取祖父状态的s值
      grandparent_l = GetLateralOffset(grandparent_s, grandparent_l_ind);
    }
  }

  double cur_s = parent_s + station_[cur_ind.s];
  double cur_l = GetLateralOffset(cur_s, cur_ind.l);

  double ds1 = cur_s - parent_s;
  double dl1 = cur_l - parent_l;

  double ds0 = parent_s - grandparent_s;
  double dl0 = parent_l - grandparent_l;

  //获得碰撞代价，若碰撞则返回碰撞代价值
  double cost_obstacle = GetCollisionCost(parent_ind, cur_ind);
  if (cost_obstacle >= config_.dp_w_obstacle) {
    return std::make_pair(cur_s, config_.dp_w_obstacle);
  }

  double cost_lateral = fabs(cur_l);
  double cost_lateral_change = fabs(parent_l - cur_l) / (station_[cur_ind.s] + kMathEpsilon);
  double cost_lateral_change_t = fabs(dl1 - dl0) / unit_time_;
  double cost_longitudinal_velocity = fabs(ds1 / unit_time_ - config_.dp_nominal_velocity);
  double cost_longitudinal_velocity_change = fabs((ds1 - ds0) / unit_time_);
  double cost_target_point = (cur_s - endState_.start_s) * (cur_s - endState_.start_s) + (cur_l - endState_.start_l) * (cur_l - endState_.start_l);

  double delta_cost = (
    config_.dp_w_lateral * cost_lateral +
    config_.dp_w_lateral_change * cost_lateral_change +
    config_.dp_w_lateral_velocity_change * cost_lateral_change_t +
    config_.dp_w_longitudinal_velocity_bias * cost_longitudinal_velocity +
    config_.dp_w_longitudinal_velocity_change * cost_longitudinal_velocity_change +
    config_.dp_w_target_point * cost_target_point);

  if(cur_l < 3 && cur_l > -3) {
    auto cart = env_->reference().GetCartesian(cur_s, cur_l);
    // std::cout << delta_cost << "," << cost_target_point <<" = "<<cur_s<<" "<<cur_l<<" = "<<cart.x() <<" "<<cart.y()<<std::endl;
  }

  // ROS_INFO("delta_cost = %f", delta_cost);
  // ROS_INFO("cost_target_point = %f", cost_target_point);
 
  return std::make_pair(cur_s, delta_cost);
}

bool DpPlanner::Plan(double start_x, double start_y, double start_theta, double end_x, double end_y, double end_theta, DiscretizedTrajectory &result) {
  auto sl = env_->reference().GetProjection({start_x, start_y});
  state_.start_s = sl.x();
  state_.start_l = sl.y();
  state_.start_theta = start_theta;

  auto endsl = env_->reference().GetProjection({end_x, end_y});
  endState_.start_s = endsl.x();
  endState_.start_l = endsl.y();
  endState_.start_theta = end_theta;

  // std::cout << "endState_ Point:" << endState_.start_s << "," << endState_.start_l << "," << endState_.start_theta <<std::endl;

  // reset state space：初始化原始代价，撒点数量s方向ns个，l方向nl个，t方向nt个
  for (int i = 0; i < NT; i++) {
    for (int j = 0; j < NS; j++) {
      for (int k = 0; k < NL; k++) {
        state_space_[i][j][k] = StateCell();
      }
    }
  }

  // evaluate first layer：t=0层的初始化
  for (int i = 0; i < NS; i++) {
    for (int j = 0; j < NL; j++) {
      auto tup = GetCost(StateIndex(-1, -1, -1), StateIndex(0, i, j));
      state_space_[0][i][j].current_s = tup.first;
      state_space_[0][i][j].cost = tup.second;
    }
  }

  // dynamic programming
  for (int i = 0; i < NT - 1; i++) {
    for (int j = 0; j < NS; j++) {
      for (int k = 0; k < NL; k++) {
        StateIndex parent_ind(i, j, k);//第i层jk的点信息，作为父节点

        for (int m = 0; m < NS; m++) {
          for (int n = 0; n < NL; n++) {
            StateIndex current_ind(i + 1, m, n);//第i+1层mn的点信息，作为当前节点
            auto tup = GetCost(parent_ind, current_ind);//利用父节点获得当前节点的cost
            double delta_cost = tup.second;
            double cur_s = tup.first;

            double cur_cost = state_space_[i][j][k].cost + delta_cost;//根据父节点cost获得当前节点的cost
            //若此时计算出的cost小于信息表内存储的cost，则换父
            if (cur_cost < state_space_[i + 1][m][n].cost) {
              state_space_[i + 1][m][n] = StateCell(cur_cost, cur_s, j, k);
            }
          }
        }
      }
    }
  }

  // find the least cost in final layer:找到最后一层代价最小的点作为终点
  // 一种可能性：min_cost如果已经大于碰撞值，则向前回溯到最近不碰撞的地方，输出轨迹
  double min_cost = Inf;
  int min_s_ind = 0, min_l_ind = 0;
  for (int i = 0; i < NS; i++) {
    for (int j = 0; j < NL; j++) {
      double cost = state_space_[NT - 1][i][j].cost;
      // std::cout << "NT - 1层：" << i << ", " << 5 << ", " << cost << std::endl;
      if (cost < min_cost) {
        min_s_ind = i;
        min_l_ind = j;
        min_cost = cost;
      }
    }
  }

  std::vector<std::pair<StateIndex, StateCell>> waypoints(NT);

  // trace back layers to find optimum trajectory：从最后一层开始向前逐层寻找最小点
  for (int i = NT - 1; i >= 0; i--) {
    auto &cell = state_space_[i][min_s_ind][min_l_ind];
    waypoints[i] = std::make_pair(StateIndex(i, min_s_ind, min_l_ind), cell);//waypoint是一个StateIndex，一个StateCell 
    min_s_ind = cell.parent_s_ind;
    min_l_ind = cell.parent_l_ind;
  }

  // //打印出相关信息
  // ROS_INFO("s[0] = %f", state_.start_s);
  // int t_ind = 1;
  // for(auto &wp: waypoints) {
  //   ROS_INFO("s[%d] = %f, %d", t_ind++, wp.second.current_s, wp.second.parent_l_ind);
  // } 

  // interpolation：将结果细化为多个点
  Trajectory data;
  data.resize(config_.nfe);
  double last_l = state_.start_l, last_s = state_.start_s;

  for (int i = 0; i < NT; i++) {
    double parent_s = i > 0 ? waypoints[i - 1].second.current_s : state_.start_s;//若i>0，则为上一个点的s，否则就是开始点的s
    //从父节点到此节点，划分为 nseg_ 个小段，返回序列多个点的xy坐标（应该是sl坐标）
    auto segment = InterpolateLinearly(parent_s, waypoints[i].second.parent_l_ind, waypoints[i].first.s,
                                       waypoints[i].first.l);

    for (int j = 0; j < nseg_; j++) {
      auto dl = segment[j].y() - last_l;
      auto ds = std::max(segment[j].x() - last_s, kMathEpsilon);
      last_l = segment[j].y();
      last_s = segment[j].x();

      auto xy = env_->reference().GetCartesian(segment[j].x(), segment[j].y());
      auto tp = env_->reference().EvaluateStation(segment[j].x());
      int n = i * nseg_ + j;
      data[n].s = segment[j].x();
      data[n].x = xy.x();
      data[n].y = xy.y();
      data[n].theta = tp.theta + atan((dl / ds) / (1 - tp.kappa * segment[j].y()));
    }
  }
  data[0].theta = state_.start_theta;
  data[config_.nfe - 1].theta = endState_.start_theta;

  result = DiscretizedTrajectory(data);

  return min_cost < config_.dp_w_obstacle;//通过最后一层是否大于碰撞权重来判断是否碰撞（若终点设置较小则会导致误返回false）
// return true;
}

//从父节点到此节点，划分为 nseg_ 个小段，返回序列多个点的xy坐标（应该是sl坐标）
std::vector<Vec2d> DpPlanner::InterpolateLinearly(double parent_s, int parent_l_ind, int cur_s_ind, int cur_l_ind) {
  std::vector<Vec2d> result(nseg_);

  double p_l = state_.start_l;
  double p_s = state_.start_s;
  if (parent_l_ind >= 0) {
    p_s = parent_s;
    p_l = GetLateralOffset(p_s, parent_l_ind);
  }
  double cur_s = p_s + station_[cur_s_ind];
  double cur_l = GetLateralOffset(cur_s, cur_l_ind);

  double s_step = station_[cur_s_ind] / nseg_;//计算 从父节点到这个节点的s长度 内每个小时间步长下的s步长值
  double l_step = (cur_l - p_l) / nseg_;//计算 从父节点到这个节点的l长度 内每个小时间步长下的l步长值

  for (int i = 0; i < nseg_; i++) {
    result[i].set_x(p_s + i * s_step);
    result[i].set_y(p_l + i * l_step);
  }

  return result;
}

}
