/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <signal.h>
#include "xarm_api/xarm_driver.h"


void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
    exit(-1);
}

class XArmDriverRunner
{
    public:
        XArmDriverRunner(ros::NodeHandle& root_nh, std::string &server_ip)
        {
            root_nh.getParam("DOF", joint_num_);
            root_nh.getParam("joint_names", joint_name_);
            root_nh.getParam("xarm_report_type", report_type_);
            is_first_cycle_ = true;
            prev_angle_ = new double [joint_num_];

            xarm_driver_.init(root_nh, server_ip);
            xarm_driver_.arm->register_connect_changed_callback(std::bind(&XArmDriverRunner::_report_connect_changed_callback, this, std::placeholders::_1, std::placeholders::_2));
            xarm_driver_.arm->register_report_data_callback(std::bind(&XArmDriverRunner::_report_data_callback, this, std::placeholders::_1));
        }
        void _report_connect_changed_callback(bool connected, bool reported)
        {
            ROS_INFO("[TCP STATUS] CONTROL: %d, REPORT: %d", connected, reported);
            if (!reported) is_first_cycle_ = true;
        }

        void _report_data_callback(XArmReportData *report_data_ptr)
        {
            // ROS_INFO("[2] state: %d, error_code: %d", report_data_ptr->state, report_data_ptr->err);
            last_now_ = now_;
            now_ = ros::Time::now();
            elapsed_ = now_ - last_now_;
            joint_state_msg_.header.stamp = now_;
            joint_state_msg_.header.frame_id = "joint-state data";
            joint_state_msg_.name.resize(joint_num_);
            joint_state_msg_.position.resize(joint_num_);
            joint_state_msg_.velocity.resize(joint_num_);
            joint_state_msg_.effort.resize(joint_num_);
            for(int i = 0; i < joint_num_; i++)
            {
                double d = (double)report_data_ptr->angle[i];
                joint_state_msg_.name[i] = joint_name_[i];
                joint_state_msg_.position[i] = d;

                if (is_first_cycle_)
                {
                    joint_state_msg_.velocity[i] = 0;
                    is_first_cycle_ = false;
                }
                else
                {
                    joint_state_msg_.velocity[i] = (joint_state_msg_.position[i] - prev_angle_[i]) / elapsed_.toSec();
                }

                joint_state_msg_.effort[i] = (double)report_data_ptr->tau[i];

                prev_angle_[i] = d;
            }

            xarm_driver_.pub_joint_state(joint_state_msg_);

            xarm_state_msg_.state = report_data_ptr->state;
            xarm_state_msg_.mode = report_data_ptr->mode;
            xarm_state_msg_.cmdnum = report_data_ptr->cmdnum;
            xarm_state_msg_.err = report_data_ptr->err;
            xarm_state_msg_.warn = report_data_ptr->war;
            xarm_state_msg_.mt_brake = report_data_ptr->mt_brake;
            xarm_state_msg_.mt_able = report_data_ptr->mt_able;
            xarm_state_msg_.angle.resize(joint_num_);

            for(int i = 0; i < joint_num_; i++)
            {
                /* set the float precision*/
                double d = report_data_ptr->angle[i];
                double r;
                char str[8];
                sprintf(str, "%0.3f", d);
                sscanf(str, "%lf", &r);
                xarm_state_msg_.angle[i] = r;
            }
            for(int i = 0; i < 6; i++)
            {
                xarm_state_msg_.pose[i] = report_data_ptr->pose[i];
                xarm_state_msg_.offset[i] = report_data_ptr->tcp_offset[i];
            }
            xarm_state_msg_.header.stamp = now_;
            xarm_driver_.pub_robot_msg(xarm_state_msg_);

            if (report_data_ptr->total_num >= 417) {
                cgpio_state_msg_.header.stamp = now_;
                cgpio_state_msg_.state = report_data_ptr->cgpio_state;
                cgpio_state_msg_.code = report_data_ptr->cgpio_code;
                cgpio_state_msg_.input_digitals[0] = report_data_ptr->cgpio_input_digitals[0];
                cgpio_state_msg_.input_digitals[1] = report_data_ptr->cgpio_input_digitals[1];
                cgpio_state_msg_.output_digitals[0] = report_data_ptr->cgpio_output_digitals[0];
                cgpio_state_msg_.output_digitals[1] = report_data_ptr->cgpio_output_digitals[1];

                cgpio_state_msg_.input_analogs[0] = report_data_ptr->cgpio_input_analogs[0];
                cgpio_state_msg_.input_analogs[1] = report_data_ptr->cgpio_input_analogs[1];
                cgpio_state_msg_.output_analogs[0] = report_data_ptr->cgpio_output_analogs[0];
                cgpio_state_msg_.output_analogs[1] = report_data_ptr->cgpio_output_analogs[1];

                for (int i = 0; i < 16; ++i) {
                    cgpio_state_msg_.input_conf[i] = report_data_ptr->cgpio_input_conf[i];
                    cgpio_state_msg_.output_conf[i] = report_data_ptr->cgpio_output_conf[i];
                }
                xarm_driver_.pub_cgpio_state(cgpio_state_msg_);
            }
            
            // if ((report_type_ == "dev" && report_data_ptr->total_num >= 135) 
            //     || (report_type_ == "rich" && report_data_ptr->total_num >= 481)) {
            //     ftsensor_msg_.header.stamp = now_;
            //     ftsensor_msg_.header.frame_id = "ft_sensor_data";
            //     ftsensor_msg_.wrench.force.x = report_data_ptr->ft_ext_force[0];
		    //     ftsensor_msg_.wrench.force.y = report_data_ptr->ft_ext_force[1];
		    //     ftsensor_msg_.wrench.force.z = report_data_ptr->ft_ext_force[2];
		    //     ftsensor_msg_.wrench.torque.x = report_data_ptr->ft_ext_force[3];
		    //     ftsensor_msg_.wrench.torque.y = report_data_ptr->ft_ext_force[4];
		    //     ftsensor_msg_.wrench.torque.z = report_data_ptr->ft_ext_force[5];
            //     xarm_driver_.pub_ftsensor_state(ftsensor_msg_);
            // }
        }

    private:
        std::string report_type_;
        ros::Time now_, last_now_;
        ros::Duration elapsed_;
        sensor_msgs::JointState joint_state_msg_;
        // geometry_msgs::WrenchStamped ftsensor_msg_;
        xarm_api::XArmDriver xarm_driver_;
        xarm_msgs::RobotMsg xarm_state_msg_;
        xarm_msgs::CIOState cgpio_state_msg_;

        int joint_num_;
        std::vector<std::string> joint_name_;
        bool is_first_cycle_;
        double *prev_angle_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_driver_node");

    // with namespace (ns) specified in the calling launch file (xarm by default)
    ros::NodeHandle n;
    ROS_INFO("namespace: %s", n.getNamespace().c_str());

    std::string robot_ip = "192.168.1.121";
    if (!n.hasParam("xarm_robot_ip"))
    {
        ROS_ERROR("No param named 'xarm_robot_ip'");
        ROS_ERROR("Use default robot ip = 192.168.1.121");
    }
    else
    {
        n.getParam("xarm_robot_ip", robot_ip);
    }

    ROS_INFO("xarm_driver_node start");
    XArmDriverRunner xarm_driver_runner(n, robot_ip);

    signal(SIGINT, exit_sig_handler);
    ros::waitForShutdown();

    ROS_INFO("xarm_driver_node over");

    return 0;
}
