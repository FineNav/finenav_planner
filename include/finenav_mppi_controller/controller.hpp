// Copyright (c) 2026.

#ifndef FINENAV_MPPI_CONTROLLER_CONTROLLER_HPP_
#define FINENAV_MPPI_CONTROLLER_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "finenav_core/plugin_planner.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"
#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"
#include "finenav_mppi_controller/world_view_def.hpp"

namespace finenav::viz {
class FineNavVisualizer;
}  // namespace finenav::viz

namespace nav2_mppi_controller {

class MPPIController {
public:
    using IWorldView = nav2_mppi_controller::IWorldView;
    using Context = finenav::core::PlanningContext<MPPIController>;
    using ConfigType = finenav_mppi_controller::Params;

    static constexpr finenav::core::IOProfile InputProfile{.pose = true,
                                                              .vel = false,
                                                              .accel = false,
                                                              .time = false};
    static constexpr finenav::core::IOProfile OutputProfile{.pose = true,
                                                               .vel = true,
                                                               .accel = false,
                                                               .time = false};

    MPPIController();
    ~MPPIController();

    MPPIController(const MPPIController&) = delete;
    MPPIController& operator=(const MPPIController&) = delete;

    void setLogger(const rclcpp::Logger& logger);
    void setVisualizer(const finenav::viz::FineNavVisualizer& visualizer);
    void configure(const ConfigType& params);
    std::optional<finenav::core::Trajectory<OutputProfile>> plan(
        const Context& ctx, const IWorldView& world_view);
    void setPlan(const nav_msgs::msg::Path& path);
    void setSpeedLimit(const double& speed_limit, const bool& percentage);
    void reset();

private:
    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
    mppi::TrajectoryVisualizer visualizer_;
    bool visualizer_enabled_{false};
    bool configured_{false};
    mppi::VisualizerConfig visualizer_config_;
    std::atomic<std::uint64_t> visualization_generation_{0};
    mppi::Optimizer optimizer_;
    std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics_;
};

}  // namespace nav2_mppi_controller

#endif  // FINENAV_MPPI_CONTROLLER_CONTROLLER_HPP_
