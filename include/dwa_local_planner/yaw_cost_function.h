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

#ifndef YAW_COST_FUNCTION_H
#define YAW_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace base_local_planner
{
    /**
    * This class provides a cost function that aim at following the orientation contained in the global plan.
    */
    class YawCostFunction : public base_local_planner::TrajectoryCostFunction
    {
    public:
        void setCurrentPose(const geometry_msgs::PoseStamped current_pose);
        void setGoalPoses(const std::vector<geometry_msgs::PoseStamped> goal_poses);

        double scoreTrajectory(Trajectory &traj) override;
        bool prepare() override;

        inline void setOrientationScale(double scale)
        {
            orientation_scale_ = scale;
        }

        inline void setVelocityScale(double scale)
        {
            velocity_scale_ = scale;
        }

        inline void setRobotParameters(double max_lin_vel, double max_lin_acc)
        {
            max_lin_vel_ = max_lin_vel;
            max_lin_acc_ = max_lin_acc;
        }

        inline void setCurrentLinVel(double current_lin_vel)
        {
            current_lin_vel_ = current_lin_vel;
        }

    private:
        double orientation_scale_;
        double velocity_scale_;

        double max_lin_vel_;
        double max_lin_acc_;
        double current_lin_vel_;

        geometry_msgs::PoseStamped current_pose_;
        std::vector<geometry_msgs::PoseStamped> goal_poses_;

        double closestToCurrent(geometry_msgs::PoseStamped &closest) const;
    };

} /* namespace base_local_planner */
#endif /* YAW_COST_FUNCTION_H */
