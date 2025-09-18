
#include <ftc_local_planner/ftc_planner.h>

#include <algorithm>
#include <limits>
#include <cmath>

#include <pluginlib/class_list_macros.h>
#include "mbf_msgs/ExePathAction.h"
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlanner, mbf_costmap_core::CostmapController)

#define RET_SUCCESS 0
#define RET_COLLISION 104
#define RET_BLOCKED 109

namespace ftc_local_planner
{

    FTCPlanner::FTCPlanner()
    {
    }

    void FTCPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);

        progress_server = private_nh.advertiseService(
            "planner_get_progress", &FTCPlanner::getProgress, this);

        global_point_pub = private_nh.advertise<geometry_msgs::PoseStamped>("global_point", 1);
        global_plan_pub = private_nh.advertise<nav_msgs::Path>("global_plan", 1, true);
        obstacle_marker_pub = private_nh.advertise<visualization_msgs::Marker>("costmap_marker", 10);
        corridor_centerline_pub_ = private_nh.advertise<nav_msgs::Path>("corridor_centerline", 1);
        best_trajectory_pub_ = private_nh.advertise<nav_msgs::Path>("best_corridor_trajectory", 1);
        corridor_boundary_pub_ = private_nh.advertise<visualization_msgs::Marker>("corridor_bounds", 1);

        costmap = costmap_ros;
        costmap_map_ = costmap->getCostmap();
        tf_buffer = tf;

        // Parameter for dynamic reconfigure
        reconfig_server = new dynamic_reconfigure::Server<FTCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<FTCPlannerConfig>::CallbackType cb = boost::bind(&FTCPlanner::reconfigureCB, this,
                                                                                     _1, _2);
        reconfig_server->setCallback(cb);

        current_state = PRE_ROTATE;

        // PID Debugging topic
        if (config.debug_pid)
        {
            pubPid = private_nh.advertise<ftc_local_planner::PID>("debug_pid", 1, true);
        }

        // Recovery behavior initialization
        failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));

        ROS_INFO("FTCLocalPlannerROS: Version 2 Init.");
    }

    void FTCPlanner::reconfigureCB(FTCPlannerConfig &c, uint32_t level)
    {
        if (c.restore_defaults)
        {
            reconfig_server->getConfigDefault(c);
            c.restore_defaults = false;
        }
        config = c;

        // just to be sure
        current_movement_speed = config.speed_slow;

        // set recovery behavior
        failure_detector_.setBufferLength(std::round(config.oscillation_recovery_min_duration * 10));
    }

    bool FTCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        current_state = PRE_ROTATE;
        state_entered_time = ros::Time::now();
        is_crashed = false;

        global_plan = plan;
        current_index = 0;
        current_progress = 0.0;

        last_time = ros::Time::now();
        current_movement_speed = config.speed_slow;

        lat_error = 0.0;
        lon_error = 0.0;
        angle_error = 0.0;
        i_lon_error = 0.0;
        i_lat_error = 0.0;
        i_angle_error = 0.0;

        nav_msgs::Path path;

        if (global_plan.size() > 2)
        {
            // duplicate last point
            global_plan.push_back(global_plan.back());
            // give second from last point last oriantation as the point before that
            global_plan[global_plan.size() - 2].pose.orientation = global_plan[global_plan.size() - 3].pose.orientation;
            path.header = plan.front().header;
            path.poses = plan;
        }
        else
        {
            ROS_WARN_STREAM("FTCLocalPlannerROS: Global plan was too short. Need a minimum of 3 poses - Cancelling.");
            current_state = FINISHED;
            state_entered_time = ros::Time::now();
        }
        global_plan_pub.publish(path);
        last_best_path_world_.clear();

        ROS_INFO_STREAM("FTCLocalPlannerROS: Got new global plan with " << plan.size() << " points.");

        return true;
    }

    FTCPlanner::~FTCPlanner()
    {
        if (reconfig_server != nullptr)
        {
            delete reconfig_server;
            reconfig_server = nullptr;
        }
    }

    double FTCPlanner::distanceLookahead()
    {
        if (global_plan.size() < 2)
        {
            return 0;
        }
        Eigen::Quaternion<double> current_rot(current_control_point.linear());
        double lookahead_distance = 0.0;
        Eigen::Affine3d last_straight_point = current_control_point;
        Eigen::Affine3d current_point;
        for (uint32_t i = current_index + 1; i < global_plan.size(); i++)
        {
            tf2::fromMsg(global_plan[i].pose, current_point);

            // check, if direction is the same. if so, we add the distance
            Eigen::Quaternion<double> rot2(current_point.linear());

            if (lookahead_distance > config.speed_fast_threshold ||
                abs(rot2.angularDistance(current_rot)) > config.speed_fast_threshold_angle * (M_PI / 180.0))
            {
                break;
            }

            lookahead_distance += (current_point.translation() - last_straight_point.translation()).norm();
            last_straight_point = current_point;

        }

        return lookahead_distance;
    }

    uint32_t FTCPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                                 const geometry_msgs::TwistStamped &velocity,
                                                 geometry_msgs::TwistStamped &cmd_vel, std::string &message)
    {

        ros::Time now = ros::Time::now();
        double dt = now.toSec() - last_time.toSec();
        last_time = now;

        if (is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_COLLISION;
        }

        if (current_state == FINISHED)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_SUCCESS;
        }

        // We're not crashed and not finished.
        // First, we update the control point if needed. This is needed since we need the local_control_point to calculate the next state.
        update_control_point(dt);
        // Then, update the planner state.
        auto new_planner_state = update_planner_state();
        if (new_planner_state != current_state)
        {
            ROS_INFO_STREAM("FTCLocalPlannerROS: Switching to state " << new_planner_state);
            state_entered_time = ros::Time::now();
            current_state = new_planner_state;
        }

        if (checkCollision(config.obstacle_lookahead))
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            is_crashed = true;
            return RET_BLOCKED;
        }

        // Finally, we calculate the velocity commands.
        calculate_velocity_commands(dt, cmd_vel);

        if (is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return RET_COLLISION;
        }

        return RET_SUCCESS;
    }


    bool FTCPlanner::isGoalReached(double dist_tolerance, double angle_tolerance)
    {
        return current_state == FINISHED && !is_crashed;
    }

    bool FTCPlanner::cancel()
    {
        ROS_WARN_STREAM("FTCLocalPlannerROS: FTC planner was cancelled.");
        current_state = FINISHED;
        state_entered_time = ros::Time::now();
        return true;
    }

    FTCPlanner::PlannerState FTCPlanner::update_planner_state()
    {
        switch (current_state)
        {
        case PRE_ROTATE:
        {
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: Error reaching goal. config.goal_timeout (" << config.goal_timeout << ") reached - Timeout in PRE_ROTATE phase.");
                is_crashed = true;
                return FINISHED;
            }
            if (abs(angle_error) * (180.0 / M_PI) < config.max_goal_angle_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: PRE_ROTATE finished. Starting following");
                return FOLLOWING;
            }
        }
        break;
        case FOLLOWING:
        {
            double distance = local_control_point.translation().norm();
            // check for crash
            if (distance > config.max_follow_distance)
            {
                ROS_ERROR_STREAM("FTCLocalPlannerROS: Robot is far away from global plan. distance (" << distance << ") > config.max_follow_distance (" << config.max_follow_distance << ") It probably has crashed.");
                is_crashed = true;
                return FINISHED;
            }

            // check if we're done following
            if (current_index == global_plan.size() - 2)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: switching planner to position mode");
                return WAITING_FOR_GOAL_APPROACH;
            }
        }
        break;
        case WAITING_FOR_GOAL_APPROACH:
        {
            double distance = local_control_point.translation().norm();
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_WARN_STREAM("FTCLocalPlannerROS: Could not reach goal position. config.goal_timeout (" << config.goal_timeout << ") reached - Attempting final rotation anyways.");
                return POST_ROTATE;
            }
            if (distance < config.max_goal_distance_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: Reached goal position.");
                return POST_ROTATE;
            }
        }
        break;
        case POST_ROTATE:
        {
            if (time_in_current_state() > config.goal_timeout)
            {
                ROS_WARN_STREAM("FTCLocalPlannerROS: Could not reach goal rotation. config.goal_timeout (" << config.goal_timeout << ") reached");
                return FINISHED;
            }
            if (abs(angle_error) * (180.0 / M_PI) < config.max_goal_angle_error)
            {
                ROS_INFO_STREAM("FTCLocalPlannerROS: POST_ROTATE finished.");
                return FINISHED;
            }
        }
        break;
        case FINISHED:
        {
            // Nothing to do here
        }
        break;
        }

        return current_state;
    }

    void FTCPlanner::update_control_point(double dt)
    {

        switch (current_state)
        {
        case PRE_ROTATE:
            tf2::fromMsg(global_plan[0].pose, current_control_point);
            break;
        case FOLLOWING:
        {
            // Normal planner operation
            double straight_dist = distanceLookahead();
            double speed;
            if (straight_dist >= config.speed_fast_threshold)
            {
                speed = config.speed_fast;
            }
            else
            {
                speed = config.speed_slow;
            }

            if (speed > current_movement_speed)
            {
                // accelerate
                current_movement_speed += dt * config.acceleration;
                if (current_movement_speed > speed)
                    current_movement_speed = speed;
            }
            else if (speed < current_movement_speed)
            {
                // decelerate
                current_movement_speed -= dt * config.acceleration;
                if (current_movement_speed < speed)
                    current_movement_speed = speed;
            }

            double distance_to_move = dt * current_movement_speed;
            double angle_to_move = dt * config.speed_angular * (M_PI / 180.0);

            Eigen::Affine3d nextPose, currentPose;
            while (angle_to_move > 0 && distance_to_move > 0 && current_index < global_plan.size() - 2)
            {

                tf2::fromMsg(global_plan[current_index].pose, currentPose);
                tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);

                double pose_distance = (nextPose.translation() - currentPose.translation()).norm();

                Eigen::Quaternion<double> current_rot(currentPose.linear());
                Eigen::Quaternion<double> next_rot(nextPose.linear());

                double pose_distance_angular = current_rot.angularDistance(next_rot);

                if (pose_distance <= 0.0)
                {
                    ROS_WARN_STREAM("FTCLocalPlannerROS: Skipping duplicate point in global plan.");
                    current_index++;
                    continue;
                }

                double remaining_distance_to_next_pose = pose_distance * (1.0 - current_progress);
                double remaining_angular_distance_to_next_pose = pose_distance_angular * (1.0 - current_progress);

                if (remaining_distance_to_next_pose < distance_to_move &&
                    remaining_angular_distance_to_next_pose < angle_to_move)
                {
                    // we need to move further than the remaining distance_to_move. Skip to the next point and decrease distance_to_move.
                    current_progress = 0.0;
                    current_index++;
                    distance_to_move -= remaining_distance_to_next_pose;
                    angle_to_move -= remaining_angular_distance_to_next_pose;
                }
                else
                {
                    // we cannot reach the next point yet, so we update the percentage
                    double current_progress_distance =
                        (pose_distance * current_progress + distance_to_move) / pose_distance;
                    double current_progress_angle =
                        (pose_distance_angular * current_progress + angle_to_move) / pose_distance_angular;
                    current_progress = fmin(current_progress_angle, current_progress_distance);
                    if (current_progress > 1.0)
                    {
                        ROS_WARN_STREAM("FTCLocalPlannerROS: FTC PLANNER: Progress > 1.0");
                        //                    current_progress = 1.0;
                    }
                    distance_to_move = 0;
                    angle_to_move = 0;
                }
            }

            tf2::fromMsg(global_plan[current_index].pose, currentPose);
            tf2::fromMsg(global_plan[current_index + 1].pose, nextPose);
            // interpolate between points
            Eigen::Quaternion<double> rot1(currentPose.linear());
            Eigen::Quaternion<double> rot2(nextPose.linear());

            Eigen::Vector3d trans1 = currentPose.translation();
            Eigen::Vector3d trans2 = nextPose.translation();

            Eigen::Affine3d result;
            result.translation() = (1.0 - current_progress) * trans1 + current_progress * trans2;
            result.linear() = rot1.slerp(current_progress, rot2).toRotationMatrix();

            if (config.enable_corridor_planning)
            {
                Eigen::Affine3d corridor_target;
                if (planCorridorTarget(result, corridor_target))
                {
                    current_control_point = corridor_target;
                }
                else
                {
                    current_control_point = result;
                }
            }
            else
            {
                current_control_point = result;
            }
        }
        break;
        case POST_ROTATE:
            tf2::fromMsg(global_plan[global_plan.size() - 1].pose, current_control_point);
            break;
        case WAITING_FOR_GOAL_APPROACH:
            break;
        case FINISHED:
            break;
        }

        {
            geometry_msgs::PoseStamped viz;
            viz.header = global_plan[current_index].header;
            viz.pose = tf2::toMsg(current_control_point);
            global_point_pub.publish(viz);
        }
        auto map_to_base = tf_buffer->lookupTransform("base_link", "map", ros::Time(), ros::Duration(1.0));
        tf2::doTransform(current_control_point, local_control_point, map_to_base);

        lat_error = local_control_point.translation().y();
        lon_error = local_control_point.translation().x();
        angle_error = local_control_point.rotation().eulerAngles(0, 1, 2).z();
    }

    bool FTCPlanner::planCorridorTarget(const Eigen::Affine3d &fallback, Eigen::Affine3d &corridor_target)
    {
        std::string frame_id;
        std::vector<Eigen::Vector2d> centerline;
        std::vector<double> headings;
        std::vector<double> cumulative_distance;
        std::vector<size_t> indices;
        if (!extractLocalCenterline(frame_id, centerline, headings, cumulative_distance, indices))
        {
            return false;
        }

        std::vector<std::pair<double, double>> bounds(centerline.size(), std::make_pair(0.0, 0.0));
        computeCorridorBounds(centerline, headings, bounds);

        std::vector<Eigen::Vector2d> best_path;
        if (!generateBestTrajectory(centerline, headings, cumulative_distance, bounds, best_path))
        {
            return false;
        }

        if (best_path.empty())
        {
            return false;
        }

        double lookahead_distance = std::max(0.0, config.corridor_tracking_lookahead);
        size_t tracking_index = 0;
        double accumulated = 0.0;
        for (size_t i = 1; i < best_path.size(); ++i)
        {
            accumulated += (best_path[i] - best_path[i - 1]).norm();
            if (accumulated >= lookahead_distance)
            {
                tracking_index = i;
                break;
            }
        }
        if (tracking_index >= best_path.size())
        {
            tracking_index = best_path.size() - 1;
        }

        Eigen::Vector2d heading_vec(1.0, 0.0);
        if (tracking_index + 1 < best_path.size())
        {
            heading_vec = best_path[tracking_index + 1] - best_path[tracking_index];
        }
        else if (tracking_index > 0)
        {
            heading_vec = best_path[tracking_index] - best_path[tracking_index - 1];
        }
        else if (!headings.empty())
        {
            heading_vec = Eigen::Vector2d(std::cos(headings.front()), std::sin(headings.front()));
        }

        if (heading_vec.squaredNorm() < 1e-9)
        {
            double fallback_yaw = std::atan2(fallback.linear()(1, 0), fallback.linear()(0, 0));
            heading_vec = Eigen::Vector2d(std::cos(fallback_yaw), std::sin(fallback_yaw));
        }

        double yaw = std::atan2(heading_vec.y(), heading_vec.x());

        corridor_target = Eigen::Affine3d::Identity();
        corridor_target.translation() = Eigen::Vector3d(best_path[tracking_index].x(), best_path[tracking_index].y(),
                                                       fallback.translation().z());
        corridor_target.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        publishCorridorDebug(frame_id, centerline, headings, bounds, best_path);
        last_best_path_world_ = best_path;

        return true;
    }

    bool FTCPlanner::extractLocalCenterline(std::string &frame_id, std::vector<Eigen::Vector2d> &centerline,
                                            std::vector<double> &headings, std::vector<double> &cumulative_distance,
                                            std::vector<size_t> &indices)
    {
        if (global_plan.size() < 2 || current_index >= global_plan.size())
        {
            return false;
        }

        frame_id = global_plan[current_index].header.frame_id;
        centerline.clear();
        headings.clear();
        cumulative_distance.clear();
        indices.clear();

        double max_length = std::max(0.1, config.corridor_length);
        double accumulated_distance = 0.0;

        auto poseToVector = [](const geometry_msgs::PoseStamped &pose) {
            return Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y);
        };

        Eigen::Vector2d last_point = poseToVector(global_plan[current_index]);
        centerline.push_back(last_point);
        cumulative_distance.push_back(0.0);
        indices.push_back(current_index);

        size_t idx = current_index;
        while (idx + 1 < global_plan.size() && accumulated_distance < max_length)
        {
            Eigen::Vector2d current_point = poseToVector(global_plan[idx]);
            Eigen::Vector2d next_point = poseToVector(global_plan[idx + 1]);

            double dist = (next_point - current_point).norm();
            if (dist < 1e-6)
            {
                idx++;
                continue;
            }

            accumulated_distance += dist;
            centerline.push_back(next_point);
            cumulative_distance.push_back(accumulated_distance);
            indices.push_back(idx + 1);
            last_point = next_point;
            idx++;
        }

        if (centerline.size() < 2)
        {
            return false;
        }

        headings.resize(centerline.size(), 0.0);
        for (size_t i = 0; i < centerline.size(); ++i)
        {
            Eigen::Vector2d direction(0.0, 0.0);
            if (i + 1 < centerline.size())
            {
                direction = centerline[i + 1] - centerline[i];
            }
            else if (i > 0)
            {
                direction = centerline[i] - centerline[i - 1];
            }

            if (direction.norm() > 1e-6)
            {
                headings[i] = std::atan2(direction.y(), direction.x());
            }
            else
            {
                double yaw = tf2::getYaw(global_plan[indices[i]].pose.orientation);
                headings[i] = yaw;
            }
        }

        return true;
    }

    void FTCPlanner::computeCorridorBounds(const std::vector<Eigen::Vector2d> &centerline,
                                           const std::vector<double> &headings,
                                           std::vector<std::pair<double, double>> &bounds)
    {
        if (centerline.empty())
        {
            return;
        }

        double max_width = std::max(0.0, config.corridor_max_half_width);
        double resolution = std::max(0.01, config.corridor_lateral_resolution);
        double lethal_threshold = config.corridor_lethal_cost;

        for (size_t i = 0; i < centerline.size(); ++i)
        {
            Eigen::Vector2d normal(-std::sin(headings[i]), std::cos(headings[i]));
            double positive_limit = 0.0;
            double negative_limit = 0.0;

            for (double offset = resolution; offset <= max_width + 1e-6; offset += resolution)
            {
                bool lethal = false;
                Eigen::Vector2d test_point = centerline[i] + normal * offset;
                double cost = costAt(test_point, lethal) * 255.0;
                if (lethal || cost >= lethal_threshold)
                {
                    break;
                }
                positive_limit = offset;
            }

            for (double offset = resolution; offset <= max_width + 1e-6; offset += resolution)
            {
                bool lethal = false;
                Eigen::Vector2d test_point = centerline[i] - normal * offset;
                double cost = costAt(test_point, lethal) * 255.0;
                if (lethal || cost >= lethal_threshold)
                {
                    break;
                }
                negative_limit = offset;
            }

            bounds[i].first = -negative_limit;
            bounds[i].second = positive_limit;
        }
    }

    bool FTCPlanner::generateBestTrajectory(const std::vector<Eigen::Vector2d> &centerline,
                                            const std::vector<double> &headings,
                                            const std::vector<double> &cumulative_distance,
                                            const std::vector<std::pair<double, double>> &bounds,
                                            std::vector<Eigen::Vector2d> &best_path)
    {
        best_path.clear();

        if (centerline.empty())
        {
            return false;
        }

        size_t N = centerline.size();
        int lateral_samples = std::max(0, config.corridor_lateral_samples);
        double max_offset = std::max(0.0, config.corridor_max_half_width);

        std::vector<double> offset_samples;
        offset_samples.push_back(0.0);
        if (lateral_samples > 0 && max_offset > 0.0)
        {
            double step = max_offset / static_cast<double>(lateral_samples);
            for (int i = 1; i <= lateral_samples; ++i)
            {
                double value = step * static_cast<double>(i);
                offset_samples.push_back(value);
                offset_samples.push_back(-value);
            }
        }
        std::sort(offset_samples.begin(), offset_samples.end());

        size_t K = offset_samples.size();
        const double INF = std::numeric_limits<double>::infinity();

        std::vector<std::vector<double>> dp(N, std::vector<double>(K, INF));
        std::vector<std::vector<int>> parent(N, std::vector<int>(K, -1));
        std::vector<std::vector<bool>> valid(N, std::vector<bool>(K, false));
        std::vector<std::vector<Eigen::Vector2d>> points(N, std::vector<Eigen::Vector2d>(K, Eigen::Vector2d::Zero()));
        std::vector<std::vector<double>> point_costs(N, std::vector<double>(K, INF));

        double collision_weight = std::max(0.0, config.corridor_collision_weight);
        double offset_weight = std::max(0.0, config.corridor_offset_weight);
        double smoothness_weight = std::max(0.0, config.corridor_smoothness_weight);
        double progress_weight = std::max(0.0, config.corridor_progress_weight);

        for (size_t i = 0; i < N; ++i)
        {
            Eigen::Vector2d normal(-std::sin(headings[i]), std::cos(headings[i]));
            for (size_t j = 0; j < K; ++j)
            {
                double offset = offset_samples[j];
                if (offset < bounds[i].first - 1e-6 || offset > bounds[i].second + 1e-6)
                {
                    continue;
                }
                Eigen::Vector2d candidate = centerline[i] + normal * offset;
                bool lethal = false;
                double collision_cost = costAt(candidate, lethal);
                if (lethal)
                {
                    continue;
                }
                points[i][j] = candidate;
                valid[i][j] = true;
                double point_cost = collision_weight * collision_cost + offset_weight * offset * offset;
                if (i > 0)
                {
                    point_cost -= progress_weight * (cumulative_distance[i] - cumulative_distance[i - 1]);
                }
                else
                {
                    point_cost -= progress_weight * cumulative_distance[i];
                }
                point_costs[i][j] = point_cost;
                if (i == 0)
                {
                    dp[i][j] = point_cost;
                }
            }
        }

        for (size_t i = 1; i < N; ++i)
        {
            for (size_t j = 0; j < K; ++j)
            {
                if (!valid[i][j])
                {
                    continue;
                }

                double point_cost = point_costs[i][j];
                if (!std::isfinite(point_cost))
                {
                    continue;
                }

                double offset = offset_samples[j];
                for (size_t pj = 0; pj < K; ++pj)
                {
                    if (!valid[i - 1][pj])
                    {
                        continue;
                    }

                    if (dp[i - 1][pj] >= INF)
                    {
                        continue;
                    }

                    bool lethal = false;
                    double seg_cost = segmentCollisionCost(points[i - 1][pj], points[i][j], lethal);
                    if (lethal)
                    {
                        continue;
                    }

                    double smooth_cost = smoothness_weight * std::pow(offset - offset_samples[pj], 2);
                    double total = dp[i - 1][pj] + point_cost + collision_weight * seg_cost + smooth_cost;
                    if (total < dp[i][j])
                    {
                        dp[i][j] = total;
                        parent[i][j] = static_cast<int>(pj);
                    }
                }
            }
        }

        double best_cost = INF;
        int best_index = -1;
        size_t best_stage = N - 1;
        for (size_t j = 0; j < K; ++j)
        {
            if (!valid[N - 1][j])
            {
                continue;
            }
            if (dp[N - 1][j] < best_cost)
            {
                best_cost = dp[N - 1][j];
                best_index = static_cast<int>(j);
                best_stage = N - 1;
            }
        }

        if (best_index < 0)
        {
            for (size_t i = N; i-- > 0;)
            {
                for (size_t j = 0; j < K; ++j)
                {
                    if (!valid[i][j])
                    {
                        continue;
                    }
                    if (dp[i][j] < best_cost)
                    {
                        best_cost = dp[i][j];
                        best_index = static_cast<int>(j);
                        best_stage = i;
                    }
                }
                if (best_index >= 0)
                {
                    break;
                }
            }
        }

        if (best_index < 0)
        {
            return false;
        }

        std::vector<Eigen::Vector2d> reversed_path;
        size_t stage = best_stage;
        int idx = best_index;
        while (idx >= 0 && stage < N)
        {
            reversed_path.push_back(points[stage][idx]);
            idx = parent[stage][idx];
            if (stage == 0)
            {
                break;
            }
            stage--;
        }

        if (reversed_path.empty())
        {
            return false;
        }

        best_path.assign(reversed_path.rbegin(), reversed_path.rend());
        return true;
    }

    double FTCPlanner::costAt(const Eigen::Vector2d &point, bool &lethal)
    {
        lethal = false;
        unsigned int mx = 0, my = 0;
        if (!costmap_map_->worldToMap(point.x(), point.y(), mx, my))
        {
            lethal = true;
            return 1.0;
        }

        unsigned char cost = costmap_map_->getCost(mx, my);
        if (cost >= config.corridor_lethal_cost)
        {
            lethal = true;
        }

        return static_cast<double>(cost) / 255.0;
    }

    double FTCPlanner::segmentCollisionCost(const Eigen::Vector2d &from, const Eigen::Vector2d &to, bool &lethal)
    {
        lethal = false;
        Eigen::Vector2d delta = to - from;
        double length = delta.norm();
        double step = std::max(0.01, config.corridor_sampling_step);
        int steps = std::max(1, static_cast<int>(std::ceil(length / step)));

        double total_cost = 0.0;
        for (int i = 1; i <= steps; ++i)
        {
            double ratio = static_cast<double>(i) / static_cast<double>(steps);
            Eigen::Vector2d sample = from + delta * ratio;
            bool sample_lethal = false;
            double c = costAt(sample, sample_lethal);
            if (sample_lethal)
            {
                lethal = true;
                return std::numeric_limits<double>::infinity();
            }
            total_cost += c;
        }

        return total_cost / static_cast<double>(steps);
    }

    void FTCPlanner::publishCorridorDebug(const std::string &frame_id, const std::vector<Eigen::Vector2d> &centerline,
                                          const std::vector<double> &headings,
                                          const std::vector<std::pair<double, double>> &bounds,
                                          const std::vector<Eigen::Vector2d> &best_path)
    {
        if (!config.corridor_publish_debug)
        {
            return;
        }

        ros::Time stamp = ros::Time::now();

        if (corridor_centerline_pub_.getNumSubscribers() > 0)
        {
            nav_msgs::Path centerline_msg;
            centerline_msg.header.frame_id = frame_id;
            centerline_msg.header.stamp = stamp;
            centerline_msg.poses.reserve(centerline.size());
            for (const auto &pt : centerline)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = centerline_msg.header;
                pose.pose.position.x = pt.x();
                pose.pose.position.y = pt.y();
                pose.pose.orientation.w = 1.0;
                centerline_msg.poses.push_back(pose);
            }
            corridor_centerline_pub_.publish(centerline_msg);
        }

        if (best_trajectory_pub_.getNumSubscribers() > 0 && !best_path.empty())
        {
            nav_msgs::Path best_msg;
            best_msg.header.frame_id = frame_id;
            best_msg.header.stamp = stamp;
            best_msg.poses.reserve(best_path.size());
            for (size_t i = 0; i < best_path.size(); ++i)
            {
                geometry_msgs::PoseStamped pose;
                pose.header = best_msg.header;
                pose.pose.position.x = best_path[i].x();
                pose.pose.position.y = best_path[i].y();
                double yaw = 0.0;
                if (i + 1 < best_path.size())
                {
                    Eigen::Vector2d dir = best_path[i + 1] - best_path[i];
                    yaw = std::atan2(dir.y(), dir.x());
                }
                else if (!best_path.empty() && i > 0)
                {
                    Eigen::Vector2d dir = best_path[i] - best_path[i - 1];
                    yaw = std::atan2(dir.y(), dir.x());
                }
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yaw);
                pose.pose.orientation = tf2::toMsg(q);
                best_msg.poses.push_back(pose);
            }
            best_trajectory_pub_.publish(best_msg);
        }

        if (corridor_boundary_pub_.getNumSubscribers() > 0 && !centerline.empty())
        {
            visualization_msgs::Marker bounds_marker;
            bounds_marker.header.frame_id = frame_id;
            bounds_marker.header.stamp = stamp;
            bounds_marker.ns = "corridor_bounds";
            bounds_marker.id = 0;
            bounds_marker.type = visualization_msgs::Marker::LINE_LIST;
            bounds_marker.action = visualization_msgs::Marker::ADD;
            bounds_marker.scale.x = 0.05;
            bounds_marker.color.r = 0.2f;
            bounds_marker.color.g = 0.7f;
            bounds_marker.color.b = 1.0f;
            bounds_marker.color.a = 0.8f;

            for (size_t i = 0; i < centerline.size(); ++i)
            {
                Eigen::Vector2d normal(-std::sin(headings[i]), std::cos(headings[i]));
                geometry_msgs::Point left, right;
                Eigen::Vector2d left_vec = centerline[i] + normal * bounds[i].second;
                Eigen::Vector2d right_vec = centerline[i] + normal * bounds[i].first;
                left.x = left_vec.x();
                left.y = left_vec.y();
                left.z = 0.0;
                right.x = right_vec.x();
                right.y = right_vec.y();
                right.z = 0.0;
                bounds_marker.points.push_back(left);
                bounds_marker.points.push_back(right);
            }

            corridor_boundary_pub_.publish(bounds_marker);
        }
    }

    void FTCPlanner::calculate_velocity_commands(double dt, geometry_msgs::TwistStamped &cmd_vel)
    {
        // check, if we're completely done
        if (current_state == FINISHED || is_crashed)
        {
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = 0;
            return;
        }

        i_lon_error += lon_error * dt;
        i_lat_error += lat_error * dt;
        i_angle_error += angle_error * dt;

        if (i_lon_error > config.ki_lon_max)
        {
            i_lon_error = config.ki_lon_max;
        }
        else if (i_lon_error < -config.ki_lon_max)
        {
            i_lon_error = -config.ki_lon_max;
        }
        if (i_lat_error > config.ki_lat_max)
        {
            i_lat_error = config.ki_lat_max;
        }
        else if (i_lat_error < -config.ki_lat_max)
        {
            i_lat_error = -config.ki_lat_max;
        }
        if (i_angle_error > config.ki_ang_max)
        {
            i_angle_error = config.ki_ang_max;
        }
        else if (i_angle_error < -config.ki_ang_max)
        {
            i_angle_error = -config.ki_ang_max;
        }

        double d_lat = (lat_error - last_lat_error) / dt;
        double d_lon = (lon_error - last_lon_error) / dt;
        double d_angle = (angle_error - last_angle_error) / dt;

        last_lat_error = lat_error;
        last_lon_error = lon_error;
        last_angle_error = angle_error;

        // allow linear movement only if in following state

        if (current_state == FOLLOWING)
        {
            double lin_speed = lon_error * config.kp_lon + i_lon_error * config.ki_lon + d_lon * config.kd_lon;
            if (lin_speed < 0 && config.forward_only)
            {
                lin_speed = 0;
            }
            else
            {
                if (lin_speed > config.max_cmd_vel_speed)
                {
                    lin_speed = config.max_cmd_vel_speed;
                }
                else if (lin_speed < -config.max_cmd_vel_speed)
                {
                    lin_speed = -config.max_cmd_vel_speed;
                }

                if (lin_speed < 0)
                {
                    lat_error *= -1.0;
                }
            }
            cmd_vel.twist.linear.x = lin_speed;
        }
        else
        {
            cmd_vel.twist.linear.x = 0.0;
        }

        if (current_state == FOLLOWING)
        {

            double ang_speed = angle_error * config.kp_ang + i_angle_error * config.ki_ang + d_angle * config.kd_ang +
                               lat_error * config.kp_lat + i_lat_error * config.ki_lat + d_lat * config.kd_lat;

            if (ang_speed > config.max_cmd_vel_ang)
            {
                ang_speed = config.max_cmd_vel_ang;
            }
            else if (ang_speed < -config.max_cmd_vel_ang)
            {
                ang_speed = -config.max_cmd_vel_ang;
            }

            cmd_vel.twist.angular.z = ang_speed;
        }
        else
        {
            double ang_speed = angle_error * config.kp_ang + i_angle_error * config.ki_ang + d_angle * config.kd_ang;
            if (ang_speed > config.max_cmd_vel_ang)
            {
                ang_speed = config.max_cmd_vel_ang;
            }
            else if (ang_speed < -config.max_cmd_vel_ang)
            {
                ang_speed = -config.max_cmd_vel_ang;
            }

            cmd_vel.twist.angular.z = ang_speed;

            // check if robot oscillates
            bool is_oscillating = checkOscillation(cmd_vel);
            if (is_oscillating)
            {
                ang_speed = config.max_cmd_vel_ang;
                cmd_vel.twist.angular.z = ang_speed;
            }
        }

        if (config.debug_pid)
        {
            ftc_local_planner::PID debugPidMsg;
            debugPidMsg.kp_lon_set = lon_error;

            // proportional
            debugPidMsg.kp_lat_set = lat_error * config.kp_lat;
            debugPidMsg.kp_lon_set = lon_error * config.kp_lon;
            debugPidMsg.kp_ang_set = angle_error * config.kp_ang;

            // integral
            debugPidMsg.ki_lat_set = i_lat_error * config.ki_lat;
            debugPidMsg.ki_lon_set = i_lon_error * config.ki_lon;
            debugPidMsg.ki_ang_set = i_angle_error * config.ki_ang;

            // diff
            debugPidMsg.kd_lat_set = d_lat * config.kd_lat;
            debugPidMsg.kd_lon_set = d_lon * config.kd_lon;
            debugPidMsg.kd_ang_set = d_angle * config.kd_ang;

            // errors
            debugPidMsg.lon_err = lon_error;
            debugPidMsg.lat_err = lat_error;
            debugPidMsg.ang_err = angle_error;

            // speeds
            debugPidMsg.ang_speed = cmd_vel.twist.angular.z;
            debugPidMsg.lin_speed = cmd_vel.twist.linear.x;

            pubPid.publish(debugPidMsg);
        }
    }

    bool FTCPlanner::getProgress(PlannerGetProgressRequest &req, PlannerGetProgressResponse &res)
    {
        res.index = current_index;
        return true;
    }

    bool FTCPlanner::checkCollision(int max_points)
    {
        unsigned int x;
        unsigned int y;

        std::vector<geometry_msgs::Point> footprint;
        visualization_msgs::Marker obstacle_marker;

        if (!config.check_obstacles)
        {
            return false;
        }
        // maximal costs
        unsigned char previous_cost = 255;
        // ensure look ahead not out of plan
        if (global_plan.size() < max_points)
        {
            max_points = global_plan.size();
        }

        // calculate cost of footprint at robots actual pose
        if (config.obstacle_footprint)
        {
        costmap->getOrientedFootprint(footprint);
        for (int i = 0; i < footprint.size(); i++)
        {
            // check cost of each point of footprint
            if (costmap_map_->worldToMap(footprint[i].x, footprint[i].y, x, y))
            {
                unsigned char costs = costmap_map_->getCost(x, y);
                if (costs >= costmap_2d::LETHAL_OBSTACLE)
                {
                    ROS_WARN("FTCLocalPlannerROS: Possible collision of footprint at actual pose. Stop local planner.");
                    return true;
                }
            }
        }
        }

        for (int i = 0; i < max_points; i++)
        {
            geometry_msgs::PoseStamped x_pose;
            int index = current_index + i;
            if (index > global_plan.size())
            {
                index = global_plan.size();
            }
            x_pose = global_plan[index];

            if (costmap_map_->worldToMap(x_pose.pose.position.x, x_pose.pose.position.y, x, y))
            {
                unsigned char costs = costmap_map_->getCost(x, y);
                if (config.debug_obstacle)
                {
                    debugObstacle(obstacle_marker, x, y, costs, max_points);
                }
                // Near at obstacel
                if (costs > 0)
                {
                    // Possible collision
                    if (costs > 127 && costs > previous_cost)
                    {
                        ROS_WARN("FTCLocalPlannerROS: Possible collision. Stop local planner.");
                        return true;
                    }
                }
                previous_cost = costs;
            }
        }
        return false;
    }

    bool FTCPlanner::checkOscillation(geometry_msgs::TwistStamped &cmd_vel)
    {
        bool oscillating = false;
        // detect and resolve oscillations
        if (config.oscillation_recovery)
        {
            // oscillating = true;
            double max_vel_theta = config.max_cmd_vel_ang;
            double max_vel_current = config.max_cmd_vel_speed;

            failure_detector_.update(cmd_vel, config.max_cmd_vel_speed, config.max_cmd_vel_speed, max_vel_theta,
                                     config.oscillation_v_eps, config.oscillation_omega_eps);

            oscillating = failure_detector_.isOscillating();

            if (oscillating) // we are currently oscillating
            {
                if (!oscillation_detected_) // do we already know that robot oscillates?
                {
                    time_last_oscillation_ = ros::Time::now(); // save time when oscillation was detected
                    oscillation_detected_ = true;
                }
                // calculate duration of actual oscillation
                bool oscillation_duration_timeout = !((ros::Time::now() - time_last_oscillation_).toSec() < config.oscillation_recovery_min_duration); // check how long we oscillate
                if (oscillation_duration_timeout)
                {
                    if (!oscillation_warning_) // ensure to send warning just once instead of spamming around
                    {
                        ROS_WARN("FTCLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
                        oscillation_warning_ = true;
                    }
                    return true;
                }
                return false; // oscillating but timeout not reached
            }
            else
            {
                // not oscillating
                time_last_oscillation_ = ros::Time::now(); // save time when oscillation was detected
                oscillation_detected_ = false;
                oscillation_warning_ = false;
                return false;
            }
        }
        return false; // no check for oscillation
    }

    void FTCPlanner::debugObstacle(visualization_msgs::Marker &obstacle_points, double x, double y, unsigned char cost, int maxIDs)
    {
        if (obstacle_points.points.empty())
        {
            obstacle_points.header.frame_id = costmap->getGlobalFrameID();
            obstacle_points.header.stamp = ros::Time::now();
            obstacle_points.action = visualization_msgs::Marker::ADD;
            obstacle_points.pose.orientation.w = 1.0;
            obstacle_points.type = visualization_msgs::Marker::POINTS;
            obstacle_points.scale.x = 0.2;
            obstacle_points.scale.y = 0.2;
        }
        obstacle_points.id = obstacle_points.points.size() + 1;

        if (cost < 127)
        {
            obstacle_points.color.g = 1.0f;
        }

        if (cost >= 127 && cost < 255)
        {
            obstacle_points.color.r = 1.0f;
        }
        obstacle_points.color.a = 1.0;
        geometry_msgs::Point p;
        costmap_map_->mapToWorld(x, y, p.x, p.y);
        p.z = 0;

        obstacle_points.points.push_back(p);
        if (obstacle_points.points.size() >= maxIDs || cost > 0)
        {
            obstacle_marker_pub.publish(obstacle_points);
            obstacle_points.points.clear();
        }
    }
}
