#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <librmcs/board/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "filter/low_pass_filter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {

class RemoteControlMotor
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    RemoteControlMotor()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<MotorCommand>(get_component_name() + "_command", *this))
        , motor_(*this, *command_component_, "/test_motor/raw")
        , velocity_filter_(
              get_parameter("velocity_filter_cutoff_hz").as_double(),
              get_parameter("update_rate_hz").as_double()) {

        const auto motor_id = static_cast<std::uint8_t>(get_parameter("motor_id").as_int());
        if (motor_id < 1 || motor_id > 7)
            throw std::invalid_argument{"GM6020 motor_id must be in [1, 7]"};
        const auto encoder_zero_point =
            static_cast<int>(get_parameter("encoder_zero_point").as_int());

        auto motor_config = device::DjiMotor::Config{device::DjiMotor::Type::kGM6020, motor_id}
                                .set_encoder_zero_point(encoder_zero_point)
                                .enable_multi_turn_angle();
        if (get_parameter("motor_reversed").as_bool())
            motor_config.set_reversed();
        motor_.configure(motor_config);

        register_output("/test_motor/angle", motor_angle_, 0.0);
        register_output("/test_motor/velocity", motor_velocity_, 0.0);
        register_output("/test_motor/torque", motor_torque_, 0.0);
        register_output("/test_motor/remote_input", remote_input_, 0.0);
        register_output("/test_motor/enabled", motor_enabled_, false);

        // This test component has no VT13 fallback, so DR16 itself must time out.
        dr16_.set_timeout_enabled(true);

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
    }

    void update() override {
        dr16_.update_status();
        motor_.update_status();

        // The right switch is a deliberate enable switch. DOWN and UNKNOWN are safe states.
        const auto right_switch = dr16_.switch_right();
        const bool remote_enabled = dr16_.valid() && right_switch != rmcs_msgs::Switch::DOWN
                                 && right_switch != rmcs_msgs::Switch::UNKNOWN;
        command_enabled_ = remote_enabled;

        *motor_angle_ = motor_.angle();
        *motor_velocity_ = velocity_filter_.update(motor_.velocity());
        *motor_torque_ = motor_.torque();
        *remote_input_ = command_enabled_ ? dr16_.joystick_left().y() : 0.0;
        *motor_enabled_ = command_enabled_;
    }

private:
    class MotorCommand : public rmcs_executor::Component {
    public:
        explicit MotorCommand(RemoteControlMotor& hardware)
            : hardware_(hardware) {}

        void update() override { hardware_.command_update(); }

    private:
        RemoteControlMotor& hardware_;
    };

    void command_update() {
        // Keep transmitting a valid frame. Any safety failure forces its command to zero.
        const double control_torque = command_enabled_ ? motor_.control_torque() : 0.0;
        auto packet = device::CanPacket8{std::uint64_t{0}};
        packet.data[(motor_.id() - 1) % 4] = motor_.generate_command(control_torque).data;

        board_->start_transmit().can_transmit(
            Spec::kCans.kCan1, {
                                   .can_id = motor_.send_id(),
                                   .can_data = packet.as_bytes(),
                               });
    }

protected:
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8)
            [[unlikely]]
            return;

        if (can == Spec::kCans.kCan1)
            motor_.match_then_store_status(data.can_id, data.can_data);
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

private:
    // Members initialize in declaration order; command_component_ must precede motor_.
    std::shared_ptr<MotorCommand> command_component_;
    device::DjiMotor motor_;
    device::Dr16 dr16_;
    filter::LowPassFilter<1> velocity_filter_;

    std::unique_ptr<librmcs::board::CBoard> board_;
    bool command_enabled_{false};

    OutputInterface<double> motor_angle_;
    OutputInterface<double> motor_velocity_;
    OutputInterface<double> motor_torque_;
    OutputInterface<double> remote_input_;
    OutputInterface<bool> motor_enabled_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::RemoteControlMotor, rmcs_executor::Component)
