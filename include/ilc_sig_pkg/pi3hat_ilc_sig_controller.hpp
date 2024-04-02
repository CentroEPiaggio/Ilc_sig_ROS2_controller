#ifndef PI3HAT_ILC_SIG_CONTROLLER_HPP
#define PI3HAT_ILC_SIG_CONTROLLER_HPP
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <fstream>

#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "pi3hat_moteus_int_msgs/msg/joints_command.hpp"
#include "realtime_tools/realtime_buffer.h"

#include "rosbag2_cpp/writer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define JNT_NUM 8
#define STATES_NUM 10          // Position and Velocity of base and contacts
#define BASE_STATES 6
#define FEET_STATES 4
#define FRC_NUM 4              // Contact forces number
#define VEL_NUM 4              // Feet velocities number
// #define LINK_LENGTH 0.16
// #define L1 0.0
// #define L2 0.16
// #define L3 0.16
// #define BASE_SH_LEN 0.1946


namespace pi3hat_ilc_sig_controller
{
    using CmdMsgs = pi3hat_moteus_int_msgs::msg::JointsCommand;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        enum Controller_State {HOMING,ACTIVE,ENDED};
        using CmdType = sensor_msgs::msg::JointState;

    class Pi3Hat_Ilc_Sig_Controller : public controller_interface::ControllerInterface
    {
        public:
            Pi3Hat_Ilc_Sig_Controller();

            ~Pi3Hat_Ilc_Sig_Controller();
            
            CallbackReturn on_init() override;
            
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override; 

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period
            ) override;

        //     bool get_reference();

                bool read_file();  

                double duration_to_s(rclcpp::Duration d);

                void execute_homing(int ind, std::vector<double>& jnt_pos, std::vector<double>& jnt_vel, std::vector<double>& jnt_tor);
                
                void reference_extraction(int ind, std::vector<double>& jnt_pos, std::vector<double>& jnt_vel, std::vector<double>& jnt_tor);

                void ILC_FF_torque_update(int ind, std::vector<double>& take_off_pos_, std::vector<double>& take_off_vel_);
                
                void integrative_term_torque_update(std::vector<double>& jnt_err, std::vector<double>& jnt_tor);
                
                void integrative_err_update(std::vector<double>& take_off_pos_, std::vector<double>& jnt_pos, double dt);
                
        private:
            rclcpp::Subscription<CmdMsgs>::SharedPtr cmd_sub_;
            std::string logger_name_;
            std::map<std::string,double> position_cmd_, velocity_cmd_,effort_cmd_;
            std::map<std::string,double> position_stt_, velocity_stt_,effort_stt_;
            
            realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsgs>> rt_buffer_;
            
            std::shared_ptr<CmdMsgs> joints_rcvd_msg_;
            bool default_init_pos_;

                Controller_State state_;
                rclcpp::Publisher<CmdType>::SharedPtr pub_;
                double period_; // [milliseconds]
                std::ifstream fin_;
                std::ifstream fhom_;
                int N_, N_homing_, N_rest_, N_homing_rest_;
                int Task_rep_;
                double T_task_, T_homing_, T_rest_, T_homing_rest_;
                double ILC_kp_, ILC_kd_;
                double Ki;
                std::vector<double> q_homing_vect_;  // Homing states+controls vector
                std::vector<double> q_opt_vect_;  // Optimization states+controls vector
                std::vector<double> time_reference;  // Vector in which are stored timing references
        

                int index_, index_rep_;
                std::string file_name_, file_homing_;
                sensor_msgs::msg::JointState act_out_;
                sensor_msgs::msg::JointState joint_ref_;
                sensor_msgs::msg::JointState joint_mis_;
                double time_, time_hom_;

                std::vector<double> land_pos_, take_off_pos_, take_off_vel_, take_off_eff_;
                std::vector<double> jnt_err_;     // Vettore in cui conservo errore di posizione ai giunti

                std::unique_ptr<rosbag2_cpp::Writer> writer_ref_;    // writes references inside the bag
                std::unique_ptr<rosbag2_cpp::Writer> writer_mis_;    // writes measures inside the bag

                std::vector<std::string> joint_;
    };
};


#endif