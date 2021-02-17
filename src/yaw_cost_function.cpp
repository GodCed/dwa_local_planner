/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Universite de Sherbrooke
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Cedric Godin
 *********************************************************************/

#include <dwa_local_planner/yaw_cost_function.h>
#include <angles/angles.h>
#include <tf2/utils.h>

using namespace base_local_planner;
using namespace geometry_msgs;
using namespace tf2;
using namespace angles;
using namespace std;

/**
 * @brief Set the current robot pose
 * 
 * @param current_pose the current robot pose
 */
void YawCostFunction::setCurrentPose(const geometry_msgs::PoseStamped current_pose)
{
    current_pose_ = current_pose;
}

/**
 * @brief Set the goal poses against which the cost function will score given trajectories
 * 
 * @param goal_poses goal poses containing orientation to follow
 */
void YawCostFunction::setGoalPoses(const vector<PoseStamped> goal_poses)
{
    goal_poses_ = goal_poses;
}

/**
 * @brief Prepare this cost function to score a trajectory
 * 
 * In this case we do nothing because no prior work is needed before scoring.
 * 
 * @return true we succeeded (everytime)
 */
bool YawCostFunction::prepare()
{
    return true;
}

/**
 * @brief Scores the given trajectory according to its respect of the goal poses orientation
 * 
 * @param traj the trajectory to score
 * @return double the trajectory cost
 */
double YawCostFunction::scoreTrajectory(Trajectory &traj)
{
    // Cost is zero if trajectory is empty
    if (traj.getPointsSize() == 0)
    {
        return 0;
    }

    // Goal theta
    PoseStamped closest;
    // We cancel if we cannot find closest pose
    if (closestToCurrent(closest) < 0.0)
    {
        return -1.0;
    }
    double gth = getYaw(closest.pose.orientation);

    // Trajectory endpoint
    double endx, endy, endth;
    traj.getEndpoint(endx, endy, endth);

    // Trajectory velocity
    double vth = fabs(traj.thetav_);
    double vtrans = sqrt(traj.xv_ * traj.xv_ + traj.yv_ * traj.yv_);

    // Cost according to if the trajectory steers in the correct direction
    double delta_theta = fabs(shortest_angular_distance(endth, gth));
    double thcost = orientation_scale_ * delta_theta;

    // Cost according to the translation velocity
    double vcost = delta_theta < M_PI_4 ? velocity_scale_ * max(0.0, max_lin_vel_ - vtrans) : 1000.0;

    return thcost + vcost;
}

/**
 * @brief Get the pose from goal_poses_ closest to the current_pose_
 * 
 * @param closest Closest pose
 * @return double Distance between current and closest, -1.0 if no closest found
 */
double YawCostFunction::closestToCurrent(PoseStamped &closest) const
{
    double closest_distance = numeric_limits<double>::max();

    // We have no goal poses so we abort lookup
    if (goal_poses_.size() <= 0)
    {
        return -1.0;
    }

    // Traverse goal_poses_ vector to find closest pose
    for (auto goal_pose : goal_poses_)
    {
        double dx = goal_pose.pose.position.x - current_pose_.pose.position.x;
        double dy = goal_pose.pose.position.y - current_pose_.pose.position.y;
        double distance = sqrt(dx * dx + dy * dy);

        if (distance < closest_distance)
        {
            closest_distance = distance;
            closest = goal_pose;
        }
    }

    return closest_distance;
}
