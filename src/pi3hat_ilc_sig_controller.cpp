#include "pluginlib/class_list_macros.hpp"
#include "ilc_sig_pkg/pi3hat_ilc_sig_controller.hpp"
#include "pi3hat_hw_interface/motor_manager.hpp"
#include <cstdint>

namespace pi3hat_ilc_sig_controller
{       
    using hardware_interface::LoanedStateInterface;
    Pi3Hat_Ilc_Sig_Controller::Pi3Hat_Ilc_Sig_Controller():
    cmd_sub_(nullptr),
    logger_name_("Pi3Hat_Ilc_Sig_Controller"),
    rt_buffer_(nullptr),
    joints_rcvd_msg_(nullptr),
    default_init_pos_(false)
    
    {}

    Pi3Hat_Ilc_Sig_Controller::~Pi3Hat_Ilc_Sig_Controller()
    {
        // rt_buffer_.~RealtimeBuffer();
    }

    CallbackReturn Pi3Hat_Ilc_Sig_Controller::on_init()
    {
        try
        {
                auto_declare<std::vector<std::string>>("joints",std::vector<std::string>());
                auto_declare<double>("period",0);
                auto_declare<std::string>("solo12_urdf","");
                auto_declare<std::string>("file_name","");
                auto_declare<std::string>("topic_name","");
                auto_declare<double>("T_init", 3.0);
                auto_declare<double>("T_task", 1.0);
                auto_declare<double>("T_rest", 1.0);
                auto_declare<double>("ILC_kp_", 0.05);
                auto_declare<double>("ILC_kd_", 0.01);
                auto_declare<double>("Ki", 0.01);
                auto_declare<int>("N",100);
                auto_declare<int>("N_init",300);
                auto_declare<int>("Task_rep", 4);
        }
         catch(const std::exception & e)
        {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Exception thrown during declaration of joints name with message: %s", e.what());
                return CallbackReturn::ERROR;
        }
        try
        {
                auto_declare<std::vector<double>>("init_pos",std::vector<double>());
        }
        catch(const std::exception & e)
        {
            RCLCPP_WARN(rclcpp::get_logger(logger_name_),"Exception thrown during declaretion of init position with message: %s it gets default values", e.what());    
        }
        
        default_init_pos_ = true;
        joints_rcvd_msg_ = std::make_shared<CmdMsgs>();
        RCLCPP_INFO(get_node()->get_logger(),"initialize succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Ilc_Sig_Controller::on_configure(const rclcpp_lifecycle::State &)
    {
        std::vector<double> init_positions;
        // size_t sz;
        std::string topic_name;
        
        // get the controlled joints name
        joint_ = get_node()->get_parameter("joints").as_string_array();
        
        if(joint_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'joints' parameter is empty");
            return CallbackReturn::ERROR;
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'joints' size is %d", (int)joint_.size());
        }

        period_ = get_node() -> get_parameter("period").as_double();
        file_name_ = get_node()->get_parameter("file_name").as_string();
        file_homing_ = get_node()->get_parameter("file_homing").as_string();
        topic_name = get_node()->get_parameter("topic_name").as_string();
        // N_ = get_node() -> get_parameter("N").as_int();
        // N_homing_ = get_node() -> get_parameter("N_homing").as_int();
        Task_rep_ = get_node() -> get_parameter("Task_rep").as_int();
        T_task_ = get_node()->get_parameter("T_task").as_double();
        T_homing_ = get_node()->get_parameter("T_init").as_double();
        T_rest_ = get_node()->get_parameter("T_rest").as_double();
        ILC_kp_ = get_node()->get_parameter("ILC_kp").as_double();
        ILC_kd_ = get_node()->get_parameter("ILC_kd").as_double();
        Ki = get_node()->get_parameter("KI").as_double();
        
        if(file_name_.empty())
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "File Name parameter must be setted"
            );
            return CallbackReturn::ERROR;
        }
        if(topic_name.empty())
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Topic Name parameter must be setted"
            );
            return CallbackReturn::ERROR;
        }
        
        // init the initial position if its needed
        if(!default_init_pos_)
        {
            init_positions = get_node()->get_parameter("init_pos").as_double_array();
            if(init_positions.empty())
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'init_pos' parameter is empty");
                return CallbackReturn::ERROR;
            }
            if(init_positions.size() != joint_.size())
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'init_pos' and 'joints' can not have different dimension");
                return CallbackReturn::ERROR;
            }
        }
        else
            init_positions.resize(joint_.size(),0.0);
        
        // // fill the map structure 
        // sz = joint.size();
        // for(size_t i = 0; i < sz; i++)
        // {
        //     position_cmd_.emplace(std::make_pair(joint[i],init_positions[i]));
        //     velocity_cmd_.emplace(std::make_pair(joint[i],0.0));
        //     effort_cmd_.emplace(std::make_pair(joint[i],0.0));

        // }
        // for(size_t i = 0; i < sz; i++)
        // {
        //     position_stt_.emplace(std::make_pair(joint[i],init_positions[i]));
        //     velocity_stt_.emplace(std::make_pair(joint[i],0.0));
        //     effort_stt_.emplace(std::make_pair(joint[i],0.0));
        // }

        // build the subscriber
        
        // cmd_sub_ = get_node()->create_subscription<CmdMsgs>(
        //     "~/command",
        //     5,
        //     [this](const CmdMsgs::SharedPtr msg)
        //     {
        //         // rt_buffer_.writeFromNonRT(msg);
        //         joints_rcvd_msg_->set__name(msg->name);
        //         joints_rcvd_msg_->set__position(msg->position);
        //         joints_rcvd_msg_->set__velocity(msg->velocity);
        //         joints_rcvd_msg_->set__effort(msg->effort);
        //         joints_rcvd_msg_->set__kp_scale(msg->kp_scale);
        //         joints_rcvd_msg_->set__kd_scale(msg->kd_scale);
        //     }
        // );
        // RCLCPP_INFO(get_node()->get_logger(),"configure succesfully");
        // return CallbackReturn::SUCCESS;

        // Assuming to resampling always at 1kHz
        N_ = (int) (T_task_/period_);
        N_homing_ = (int) (T_homing_/period_);
        N_rest_ = (int) (T_rest_/period_);
        
        if(N_ <= 0  || period_ <= 0) 
        {
             RCLCPP_ERROR(
                get_node()->get_logger(),
                "The number of sample and the period must be a positive integer"
            );
            return CallbackReturn::ERROR;
        }


        act_out_.position.resize(JNT_NUM);
        act_out_.velocity.resize(JNT_NUM);
        act_out_.effort.resize(JNT_NUM);

        joint_ref_.position.resize(JNT_NUM);
        joint_ref_.velocity.resize(JNT_NUM);
        joint_ref_.effort.resize(JNT_NUM);

        joint_mis_.position.resize(JNT_NUM);
        joint_mis_.velocity.resize(JNT_NUM);
        joint_mis_.effort.resize(JNT_NUM);

        take_off_pos_.resize(JNT_NUM);
        take_off_vel_.resize(JNT_NUM);
        take_off_eff_.resize(JNT_NUM);

        jnt_err_.resize(JNT_NUM, 0.0);   //integrative error 

        // RCLCPP_ERROR(
        //         get_node()->get_logger(),
        //         "%d", JNT_NUM*2*(int)(T_task_/period_)
        //     );
        //opt_per = T_task_ / N_;
        

        pub_ = get_node() -> create_publisher<sensor_msgs::msg::JointState>(
            topic_name,
            rclcpp::QoS(10)
        );

        // // Define the writer variables
        // writer_ref_ = std::make_unique<rosbag2_cpp::Writer>();
        // writer_mis_ = std::make_unique<rosbag2_cpp::Writer>();
        // // Open the bag to start writing
        // writer_ref_ -> open("rosbags/joint_ref_bag");
        // writer_mis_ -> open("rosbags/joint_mis_bag");
        // // Create the synthetic topics to be written
        // writer_ref_ -> create_topic({"planner/joint_ref_bag", "sensor_msgs/msg/JointState", rmw_get_serialization_format(), ""});
        // writer_mis_ -> create_topic({"planner/joint_mis_bag", "sensor_msgs/msg/JointState", rmw_get_serialization_format(), ""});

        index_ = 0;
        index_rep_ = 0;
        time_ = 0.0;
        time_hom_ = 0.0;
        

        q_homing_vect_.resize(JNT_NUM*3*(int)(T_homing_/period_), 0.1);
        q_opt_vect_.resize(JNT_NUM*3*(int)(T_task_/period_), 0.1);
        
        if(!this->read_file())
          return CallbackReturn::ERROR;

        state_ = Controller_State::HOMING;

        RCLCPP_INFO(get_node()->get_logger(),
       "CONFIGURE COMPLETE");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Ilc_Sig_Controller::on_activate(const rclcpp_lifecycle::State &)
    {
        rt_buffer_.reset();
        std::vector<std::reference_wrapper<LoanedStateInterface>> ordered_interfaces;
        RCLCPP_INFO(get_node()->get_logger(),"activate succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Ilc_Sig_Controller::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Ilc_Sig_Controller::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }
        
        /* Questa funzione legge il file csv, quindi il vettore di ottimizzazione, cioè stati e controlli.  */
        bool Pi3Hat_Ilc_Sig_Controller::read_file()      
        {
                int i = 0;
                int i_hom = 0;
                std::string rr;
                std::string rr_hom;


                fhom_.open(file_homing_);

                if(!fhom_.is_open())
                {
                RCLCPP_ERROR(
                        get_node()->get_logger(),"error in homing file opening"
                        );
                return false;
                }
                while(!fhom_.eof())
                {
                fhom_>>rr_hom;
                // Record all the optimization elements inside a vector
                q_homing_vect_[i_hom] = static_cast<double>(std::stod(rr_hom));
                // RCLCPP_INFO(get_node()->get_logger(),"readed homing data %f",q_homing_vect_[i_hom]);
                i_hom += 1;
                }


                fin_.open(file_name_);

                if(!fin_.is_open())
                {
                RCLCPP_ERROR(
                        get_node()->get_logger(),"error in file opening"
                        );
                return false;
                }
                while(!fin_.eof())
                {
                fin_>>rr;
                // Record all the optimization elements inside a vector
                q_opt_vect_[i] = static_cast<double>(std::stod(rr));
                //RCLCPP_INFO(get_node()->get_logger(),"readed data %f",q_opt_vect_[i]);
                i += 1;
                }

                return true;  
        };  

        double Pi3Hat_Ilc_Sig_Controller::duration_to_s(rclcpp::Duration d)
        {
                long  ns = d.nanoseconds();
                double tot_s = (double)ns/1000000000.0;
                return tot_s;
        }

        void Pi3Hat_Ilc_Sig_Controller::execute_homing(int ind, std::vector<double>& jnt_pos, std::vector<double>& jnt_vel, std::vector<double>& jnt_tor)
        {
        if (ind < N_homing_){
                for (int i=0; i<JNT_NUM; i++){
                jnt_pos[i] = q_homing_vect_[3*JNT_NUM*ind + i];
                // RCLCPP_INFO(get_node()->get_logger(),"readed homing pos data %f",q_homing_vect_[3*JNT_NUM*ind + i]);
                jnt_vel[i] = q_homing_vect_[3*JNT_NUM*ind + i + JNT_NUM];
                jnt_tor[i] = q_homing_vect_[3*JNT_NUM*ind + i + 2*JNT_NUM];
                }
        }
        else{
                RCLCPP_INFO(get_node()->get_logger(),"Homing done, setting controller state ACTIVE.");
                state_ = Controller_State::ACTIVE;
        }
        };

        void Pi3Hat_Ilc_Sig_Controller::reference_extraction(int ind, std::vector<double>& jnt_pos, std::vector<double>& jnt_vel, std::vector<double>& jnt_tor){
                for (int i=0; i<JNT_NUM; i++){
                jnt_pos[i] = q_opt_vect_[3*JNT_NUM*ind + i];
                jnt_vel[i] = q_opt_vect_[3*JNT_NUM*ind + i + JNT_NUM];
                jnt_tor[i] = q_opt_vect_[3*JNT_NUM*ind + i + 2*JNT_NUM];
                }
        };

        void Pi3Hat_Ilc_Sig_Controller::ILC_FF_torque_update(int ind, std::vector<double>& take_off_pos_, std::vector<double>& take_off_vel_){
                for (int i=0; i<JNT_NUM; i++){
                q_opt_vect_[3*JNT_NUM*ind + i + 2*JNT_NUM] +=  ILC_kp_ * (q_opt_vect_[3*JNT_NUM*ind + i] - take_off_pos_[i]) + ILC_kd_ * (q_opt_vect_[3*JNT_NUM*ind + i + JNT_NUM] - take_off_vel_[i]);
                // RCLCPP_INFO(get_node()->get_logger(),"FF torque computed %f ", q_opt_vect_[3*JNT_NUM*ind + i + 2*JNT_NUM]);
                }
        };

        void Pi3Hat_Ilc_Sig_Controller::integrative_term_torque_update(std::vector<double>& jnt_err, std::vector<double>& jnt_tor){
                for (int i=0; i<JNT_NUM; i++){
                jnt_tor[i] +=  Ki * jnt_err[i];
                // RCLCPP_INFO(get_node()->get_logger(),"FF torque computed %f ", q_opt_vect_[3*JNT_NUM*ind + i + 2*JNT_NUM]);
                }
        };

        void Pi3Hat_Ilc_Sig_Controller::integrative_err_update(std::vector<double>& take_off_pos_, std::vector<double>& jnt_pos, double dt){
                for (int i=0; i<JNT_NUM; i++){
                jnt_err_[i] +=  (jnt_pos[i] - take_off_pos_[i])*dt;
                // RCLCPP_INFO(get_node()->get_logger(),"FF torque computed %f ", q_opt_vect_[3*JNT_NUM*ind + i + 2*JNT_NUM]);
                }
        };

    controller_interface::return_type Pi3Hat_Ilc_Sig_Controller::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
 
      // return controller_interface::return_type::OK;
      std::vector<double> zeros(JNT_NUM,0.0);

      std::vector<double> jnt_pos(JNT_NUM), jnt_vel(JNT_NUM), jnt_tor(JNT_NUM);
      double d_s = duration_to_s(period);
      // RCLCPP_INFO(get_node()->get_logger(),"the period is %f", d_s);
      time_ += d_s;
      // RCLCPP_INFO(get_node()->get_logger(),"time has value %f",time_);


      if (time_ < T_homing_+ Task_rep_ * (T_task_ + T_rest_)){

        // Extract measure after previus iteration
        for(uint j = 0;j<JNT_NUM;j++)
        { 
          // RCLCPP_INFO(get_node()->get_logger(),"j value is %d",j);
          take_off_pos_[j] = state_interfaces_[3*j].get_value();            //Tira fuori le posizioni dei giunti
          // RCLCPP_INFO(get_node()->get_logger(),"Pos is %f", take_off_pos_[j]);
          take_off_vel_[j] = state_interfaces_[3*j+1].get_value();          //Tira fuori le velocità dei giunti
          // RCLCPP_INFO(get_node()->get_logger(),"Vel is %f", take_off_vel_[j]);
          take_off_eff_[j] = state_interfaces_[3*j+2].get_value();          //Tira fuori le coppie dei giunti
          // RCLCPP_INFO(get_node()->get_logger(),"Eff is %f", take_off_eff_[j]);
        }

        

        switch (state_)
          {
          case Controller_State::HOMING:
              if(index_ < N_homing_){
                execute_homing(index_, jnt_pos, jnt_vel, jnt_tor);
                integrative_err_update(take_off_pos_, jnt_pos, d_s);
                integrative_term_torque_update(jnt_err_, jnt_tor);
              }
              else{
                index_ = 0;
                RCLCPP_INFO(get_node()->get_logger(),"Homing done, setting controller state ACTIVE. Time is %f", time_);
                state_ = Controller_State::ACTIVE;
                reference_extraction(index_, jnt_pos, jnt_vel, jnt_tor);           
              }
              break;
          case Controller_State::ACTIVE:
              if(index_  < (int) (Task_rep_ * (N_ + N_rest_))){
                if(index_rep_ >= N_ + N_rest_ || index_rep_ == 0) {
                  index_rep_ = 0;
                  // ILC_FF_torque_update(index_rep_, take_off_pos_, take_off_vel_);
                  reference_extraction(index_rep_, jnt_pos, jnt_vel, jnt_tor);
                  integrative_err_update(take_off_pos_, jnt_pos, d_s);
                  integrative_term_torque_update(jnt_err_, jnt_tor);
                  index_rep_ += 1;
                  }
                else if (index_rep_ >= N_)
                {
                  // ILC_FF_torque_update(0, take_off_pos_, take_off_vel_);
                  reference_extraction(0, jnt_pos, jnt_vel, jnt_tor);
                  integrative_err_update(take_off_pos_, jnt_pos, d_s);
                  integrative_term_torque_update(jnt_err_, jnt_tor);
                  index_rep_ += 1;
                }
                else {
                  jnt_err_.resize(JNT_NUM, 0.0);
                  ILC_FF_torque_update(index_rep_ - 1, take_off_pos_, take_off_vel_); // If this line cumment the ilc_update in the if
                  reference_extraction(index_rep_, jnt_pos, jnt_vel, jnt_tor);
                  index_rep_ += 1;                  
                }
                //RCLCPP_INFO(get_node()->get_logger(),"index value is %d", index_);
              }
              else {
                state_ = Controller_State::ENDED;
                reference_extraction(0, jnt_pos, jnt_vel, jnt_tor);
                RCLCPP_INFO(get_node()->get_logger(),"index value is %d", index_);
              }
              break;
          case Controller_State::ENDED:
               reference_extraction(0, jnt_pos, jnt_vel, jnt_tor);
               break;   
          default:
              reference_extraction(0, jnt_pos, jnt_vel, jnt_tor);
              break;
          }

        for(uint j = 0;j<JNT_NUM;j++)
        { 
          // RCLCPP_INFO(get_node()->get_logger(),"j value is %d",j);
          command_interfaces_[5*j].set_value(jnt_pos[j]);            //Tira fuori le posizioni dei giunti
          // command_interfaces_[5*j].set_value(0.0);
          command_interfaces_[5*j+1].set_value(jnt_vel[j]);          //Tira fuori le velocità dei giunti
          // command_interfaces_[5*j+1].set_value(0.0);          //Tira fuori le velocità dei giunti
          if (std::isnan(jnt_tor[j]))
          {
            command_interfaces_[5*j+2].set_value(0);
          }
          else
          {
            command_interfaces_[5*j+2].set_value(jnt_tor[j]);
          }
          command_interfaces_[5*j+3].set_value(1.0);          
          command_interfaces_[5*j+4].set_value(1.0);
          // if ( jnt_pos[j] || jnt_vel[j] || jnt_tor[j])
          // {
          //   RCLCPP_INFO(get_node()->get_logger(),"SHIT AT j = %d",j);
          //   RCLCPP_INFO(get_node()->get_logger(),"POS = %f",jnt_pos[j]);
          //   RCLCPP_INFO(get_node()->get_logger(),"VEL = %f",jnt_vel[j]);
          //   RCLCPP_INFO(get_node()->get_logger(),"EFF = %f",jnt_tor[j]);
          // }          
        }


        // RCLCPP_INFO(get_node()->get_logger(), "Jnt pos val: %f", jnt_pos[7]);

        //Stream command data on topics
        act_out_.set__position(jnt_pos);
        act_out_.set__velocity(jnt_vel);
        act_out_.set__effort(jnt_tor);
        act_out_.header.set__stamp(time);


        pub_->publish(act_out_);

        // Write data on bags
        joint_ref_.set__position(jnt_pos);
        joint_ref_.set__velocity(jnt_vel);
        joint_ref_.set__effort(jnt_tor);
        joint_ref_.header.set__stamp(time);

        joint_mis_.set__position(take_off_pos_);
        joint_mis_.set__velocity(take_off_vel_);
        joint_mis_.set__effort(take_off_eff_);
        joint_mis_.header.set__stamp(time);
          

        // Write data inside the bags
        // writer_ref_ -> write(joint_ref_, "planner/joint_ref_bag", time);
        // writer_mis_ -> write(joint_mis_, "planner/joint_mis_bag", time);

        index_ += 1;

        return controller_interface::return_type::OK;
      }
      else{
        std::vector<double> final_pose(STATES_NUM);

        reference_extraction(0, jnt_pos, jnt_vel, jnt_tor);
        for(uint j = 0;j<JNT_NUM;j++)
        { 
          // RCLCPP_INFO(get_node()->get_logger(),"j value is %d",j);
          command_interfaces_[5*j].set_value(jnt_pos[j]);            //Tira fuori le posizioni dei giunti
          // command_interfaces_[5*j].set_value(0.0);
          command_interfaces_[5*j+1].set_value(jnt_vel[j]);          //Tira fuori le velocità dei giunti
          // command_interfaces_[5*j+1].set_value(0.0);          //Tira fuori le velocità dei giunti
          if (std::isnan(jnt_tor[j]))
          {
            command_interfaces_[5*j+2].set_value(0);
          }
          else
          {
            command_interfaces_[5*j+2].set_value(jnt_tor[j]);
          }
          command_interfaces_[5*j+3].set_value(1.0);          
          command_interfaces_[5*j+4].set_value(1.0);  
          if (jnt_pos[j] || jnt_vel[j] || jnt_tor[j])
          {
            RCLCPP_INFO(get_node()->get_logger(),"SHIT AT j = %d",j);
            RCLCPP_INFO(get_node()->get_logger(),"POS = %f",jnt_pos[j]);
            RCLCPP_INFO(get_node()->get_logger(),"VEL = %f",jnt_vel[j]);
            RCLCPP_INFO(get_node()->get_logger(),"EFF = %f",jnt_tor[j]);
          }
            
        }
        
        //Stream command data on topics
        act_out_.set__velocity(zeros);
        act_out_.set__effort(zeros);
        act_out_.header.set__stamp(time);
          
        pub_->publish(act_out_);
        return controller_interface::return_type::OK;
      }
    };

    
    controller_interface::InterfaceConfiguration Pi3Hat_Ilc_Sig_Controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(size_t i = 0; i < joint_.size(); i++)
        {     
            // RCLCPP_INFO(get_node()->get_logger(),"State interface i =  %d", (int)i);
            state_interface_config.names.push_back(joint_[i] + "/" + hardware_interface::HW_IF_POSITION);
            state_interface_config.names.push_back(joint_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
            state_interface_config.names.push_back(joint_[i] + "/" + hardware_interface::HW_IF_EFFORT);
            // std::string mystring = joint_[i] + "/" + hardware_interface::HW_IF_POSITION;
            // RCLCPP_INFO(get_node()->get_logger(),"Setting %s", mystring.c_str());
        }
        RCLCPP_INFO(get_node()->get_logger(),"CONFIGURE STATE INTERFACES OK");

        return state_interface_config;
    };

    controller_interface::InterfaceConfiguration Pi3Hat_Ilc_Sig_Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(size_t i = 0; i < joint_.size(); i++)
        {     
            // RCLCPP_INFO(get_node()->get_logger(),"State interface i =  %d", (int)i);
            command_interface_config.names.push_back(joint_[i] + "/" + hardware_interface::HW_IF_POSITION);
            command_interface_config.names.push_back(joint_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
            command_interface_config.names.push_back(joint_[i] + "/" + hardware_interface::HW_IF_EFFORT);
            command_interface_config.names.push_back(joint_[i] + "/" + hardware_interface::HW_IF_KP_SCALE);
            command_interface_config.names.push_back(joint_[i] + "/" + hardware_interface::HW_IF_KD_SCALE);
            // std::string mystring = joint_[i] + "/" + hardware_interface::HW_IF_POSITION;
            // RCLCPP_INFO(get_node()->get_logger(),"Setting %s", mystring.c_str());
        }
        RCLCPP_INFO(get_node()->get_logger(),"CONFIGURE COMMAND INTERFACES OK");
        return command_interface_config;
    };
}

PLUGINLIB_EXPORT_CLASS(
    pi3hat_ilc_sig_controller::Pi3Hat_Ilc_Sig_Controller, controller_interface::ControllerInterface
);
