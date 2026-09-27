#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::test {

class Gm6020TestController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Gm6020TestController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , mode_(parse_mode(get_parameter("mode").as_string())) // 获取控制模式
        , deadband_(get_parameter("deadband").as_double())     // 设置摇杆死区
        , max_velocity_(get_parameter("max_velocity").as_double())
        , max_velocity_target_rate_(get_parameter("max_velocity_target_rate").as_double())
        , max_torque_rate_(get_parameter("max_torque_rate").as_double())
        , velocity_startup_torque_(get_parameter("velocity_startup_torque").as_double())
        , velocity_startup_duration_(get_parameter("velocity_startup_duration_s").as_double())
        , velocity_feedforward_torque_(get_parameter("velocity_feedforward_torque").as_double())
        , velocity_pid_(pid::make_pid_calculator(*this, "velocity_"))
        , angle_pid_(pid::make_pid_calculator(*this, "angle_")) {

        velocity_pid_output_min_ = velocity_pid_.output_min;
        velocity_pid_output_max_ = velocity_pid_.output_max;

        register_input(get_parameter("remote_input").as_string(), remote_input_);
        register_input(get_parameter("enabled").as_string(), enabled_);
        register_input(get_parameter("angle").as_string(), angle_);
        register_input(get_parameter("velocity").as_string(), velocity_);
        register_input("/predefined/update_rate", update_rate_);

        register_output(get_parameter("control_angle").as_string(), control_angle_, 0.0);
        register_output(get_parameter("control_velocity").as_string(), control_velocity_, 0.0);
        register_output(get_parameter("control_torque").as_string(), control_torque_, 0.0);
    }

    void update() override {
        if (!*enabled_) {
            disable();
            return;
        }

        const double remote = apply_deadband(std::clamp(*remote_input_, -1.0, 1.0));
        const double dt = update_dt();
        double target_velocity;

        if (mode_ == Mode::kVelocity) {
            target_angle_ = *angle_;
            target_initialized_ = true;
            target_velocity = remote * max_velocity_;
        } else {
            if (!target_initialized_) {
                angle_origin_ = *angle_;
                target_angle_ = angle_origin_;
                target_initialized_ = true;
                angle_pid_.reset();
            }

            // Stick position selects an angle relative to the position at enable:
            // center = origin, each end = its configured angular limit.
            target_angle_ = angle_origin_ + remote * std::numbers::pi;
            target_velocity = angle_pid_.update(target_angle_ - *angle_, dt);
        }

        target_velocity = std::clamp(target_velocity, -max_velocity_, max_velocity_);
        const double torque = update_velocity_loop(target_velocity, dt);
        *control_angle_ = target_angle_;
        *control_velocity_ = control_velocity_state_;
        *control_torque_ = torque;
    }

private:
    enum class Mode { kVelocity, kAngle };
    enum class StartupState { kReady, kActive, kDone };
    using Clock = std::chrono::steady_clock;
    static constexpr double kStoppedSpeed = 0.15;
    static constexpr double kStartupMinTargetSpeed = 1.0;
    static constexpr double kStartupMinDuration = 0.02;

    double update_velocity_loop(double target_velocity, double dt) {
        const bool was_starting = startup_state_ == StartupState::kActive;
        const int direction = (target_velocity > 0.0) - (target_velocity < 0.0);
        reset_on_direction_change(direction);

        if (direction == 0) {
            velocity_pid_.reset();
            if (std::abs(*velocity_) <= kStoppedSpeed) {
                control_velocity_state_ = 0.0;
                control_torque_state_ = 0.0;
                return 0.0;
            }
        }

        //让control_velocity_state_随控制周期平缓增加到目标速度，避免扭矩突变
        const double target_step = max_velocity_target_rate_ * dt;
        control_velocity_state_ +=
            std::clamp(target_velocity - control_velocity_state_, -target_step, target_step);//每个控制周期内只增加一点目标速度，
              //施加启动转矩                                                                                          //避免因为控制频率过高造成速度环的过度震荡
        if (apply_startup_torque(target_velocity, direction))
            return control_torque_state_;

            //施加前馈力矩克服低速时的电磁阻力
        double feedforward_torque = direction * velocity_feedforward_torque_;
        if (mode_ == Mode::kAngle && std::abs(target_velocity) < 0.25)
            feedforward_torque = 0.0;

        velocity_pid_.output_min =
            (mode_ == Mode::kVelocity && direction > 0 ? 0.0 : velocity_pid_output_min_)
            - feedforward_torque;
        velocity_pid_.output_max =
            (mode_ == Mode::kVelocity && direction < 0 ? 0.0 : velocity_pid_output_max_)
            - feedforward_torque;

        const double torque =
            feedforward_torque + velocity_pid_.update(control_velocity_state_ - *velocity_, dt);
        return torque;
    }
//前后速度方向变化的时候重置pid和相关状态
    void reset_on_direction_change(int direction) {
        if (direction != velocity_startup_direction_) {
            velocity_pid_.reset();
            startup_state_ = StartupState::kReady;
            velocity_startup_direction_ = direction;
            control_velocity_state_ = 0.0;
            control_torque_state_ = 0.0;
        }
    }
//判断是否需要施加启动转矩，当目标速度大于1，电机实际速度小于0.15时施加一段时间
    bool apply_startup_torque(double target_velocity, int direction) {
        if (velocity_startup_duration_ <= 0.0)
            return false;

        const auto now = Clock::now();
        const bool stalled = direction != 0 && std::abs(target_velocity) >= kStartupMinTargetSpeed
                          && std::abs(*velocity_) <= kStoppedSpeed;
        if (startup_state_ == StartupState::kReady && stalled) {
            startup_state_ = StartupState::kActive;
            velocity_startup_started_at_ = now;
            velocity_pid_.reset();
        }
        if (startup_state_ != StartupState::kActive)
            return false;

        const auto elapsed = now - velocity_startup_started_at_;
        const bool reached_target = *velocity_ * direction >= std::abs(target_velocity);
        if (std::abs(target_velocity) >= kStartupMinTargetSpeed
            && elapsed < std::chrono::duration<double>{velocity_startup_duration_}
            && (elapsed < std::chrono::duration<double>{kStartupMinDuration} || !reached_target)) {
            control_torque_state_ = direction * velocity_startup_torque_;
            return true;
        }
        startup_state_ = StartupState::kDone;
        return false;
    }
//将读取到的参数转换为控制模式枚举
    static Mode parse_mode(const std::string& mode) {
        if (mode == "velocity")
            return Mode::kVelocity;
        if (mode == "angle")
            return Mode::kAngle;
        throw std::invalid_argument{"mode must be 'velocity' or 'angle'"};
    }
//对摇杆-0.05到+0.05的区域进行处理，避免摇杆中间的映射过于灵敏
    double apply_deadband(double input) const {
        if (std::abs(input) <= deadband_)
            return 0.0;
        return std::copysign((std::abs(input) - deadband_) / (1.0 - deadband_), input);
    }

    double update_dt() const {
        if (update_rate_.ready() && *update_rate_ > 0.0)
            return 1.0 / *update_rate_;
        return 0.001;
    }

    void disable() {
        velocity_pid_.reset();
        angle_pid_.reset();
        startup_state_ = StartupState::kReady;
        velocity_startup_direction_ = 0;
        target_initialized_ = false;
        control_velocity_state_ = 0.0;
        control_torque_state_ = 0.0;
        *control_angle_ = angle_.ready() ? *angle_ : 0.0;
        *control_velocity_ = 0.0;
        *control_torque_ = 0.0;
    }

    Mode mode_;
    double deadband_;
    double max_velocity_;
    double max_velocity_target_rate_;
    double max_torque_rate_;
    double velocity_startup_torque_;
    double velocity_startup_duration_;
    double velocity_feedforward_torque_;

    pid::PidCalculator velocity_pid_;
    pid::PidCalculator angle_pid_;
    double velocity_pid_output_min_{0.0};
    double velocity_pid_output_max_{0.0};

    InputInterface<double> remote_input_;
    InputInterface<bool> enabled_;
    InputInterface<double> angle_;
    InputInterface<double> velocity_;
    InputInterface<double> update_rate_;

    OutputInterface<double> control_angle_;
    OutputInterface<double> control_velocity_;
    OutputInterface<double> control_torque_;

    bool target_initialized_{false};
    double angle_origin_{0.0};
    double target_angle_{0.0};
    double control_velocity_state_{0.0};
    double control_torque_state_{0.0};
    StartupState startup_state_{StartupState::kReady};
    int velocity_startup_direction_{0};
    Clock::time_point velocity_startup_started_at_{};
};

} // namespace rmcs_core::controller::test

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::Gm6020TestController, rmcs_executor::Component)
