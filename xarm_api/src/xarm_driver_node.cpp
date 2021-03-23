/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
#include <signal.h>
#include <thread>
#include "xarm/core/connect.h"
#include "xarm/core/report_data.h"
#include "xarm/core/debug/debug_print.h"

#define DEBUG_MODE 0
#define DEBUG_DETAIL 0
#define PRINT_HEX_DATA(hex, len, ...)     \
{                                         \
    if (DEBUG_MODE && DEBUG_DETAIL) {     \
        printf(__VA_ARGS__);              \
        for (int i = 0; i < len; ++i) {   \
            printf("%02x ", hex[i]);      \
        }                                 \
        printf("\n");                     \
    }                                     \
}

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
    exit(-1);
}

class XarmRTConnection
{
    public:
        XarmRTConnection(ros::NodeHandle& root_nh, char *server_ip, xarm_api::XARMDriver &drv)
        {
            root_nh.getParam("DOF", joint_num_);
            root_nh.getParam("joint_names", joint_name_);
            root_nh.getParam("xarm_report_type", report_type);
            ip = server_ip;
            xarm_driver = drv;
            xarm_driver.XARMDriverInit(root_nh, server_ip);
            ros::Duration(0.5).sleep();
            std::thread th(thread_proc, (void *)this);
            th.detach();
        }

        bool reConnect(void)
        {
            xarm_driver.closeReportSocket();
            int retryCnt = 0;
            ROS_INFO("try to reconnect to report socket");
            while (retryCnt < 5)
            {
                retryCnt++;
                if (xarm_driver.reConnectReportSocket(ip)) {
                    ROS_INFO("reconnect to report socket success");
                    return true;
                }
                ros::Duration(2).sleep();
            }
            ROS_ERROR("reconnect to report socket failed");
            return false;
        }

        void thread_run(void)
        {
            int ret;
            int err_num = 0;
            int rxcnt;
            int i;
            int first_cycle = 1;
            double d;
            double * prev_angle = new double [joint_num_];

            ros::Rate r(GET_FRAME_RATE_HZ);

            int size = 0;
            int num = 0;
            int offset = 0;
            unsigned char rx_data[1024];
            unsigned char prev_data[1280];
            unsigned char ret_data[1024 * 2];

            int db_read_cnt = 0;
            int db_packet_cnt = 0;
            int db_success_pkt_cnt = 0;
            int db_discard_pkt_cnt = 0;
            int db_failed_pkt_cnt = 0;
            bool prev_pkt_is_not_empty = false;

            ROS_INFO("start to handle the tcp report");

            report_data_ptr = new XArmReportData(report_type);

            while(xarm_driver.isConnectionOK())
            {
                r.sleep();
                ret = xarm_driver.get_frame(rx_data);
                if (ret != 0) continue;
                db_read_cnt += 1;

                num = bin8_to_32(rx_data);
                if (num < 4 && size <= 0) continue;
                if (size <= 0) {
                    size = bin8_to_32(rx_data + 4);
                    bin32_to_8(size, &ret_data[0]);
                }
                if (num + offset < size) {
                    if (DEBUG_MODE)
                        ROS_INFO("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d] The data packet length is insufficient, waiting for the next packet splicing. num=%d, offset=%d, length=%d\n", 
                            db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_failed_pkt_cnt, num, offset, num + offset);
                    memcpy(ret_data + offset + 4, rx_data + 4, num);
                    offset += num;
                    continue;
                }
                else {
                    memcpy(ret_data + offset + 4, rx_data + 4, size - offset);
                    db_packet_cnt += 1;
                    int offset2 = size - offset;
                    while (num - offset2 >= size) {
                        db_discard_pkt_cnt += 1;
                        db_packet_cnt += 1;
                        if (DEBUG_MODE)
                            ROS_INFO("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d] Data packet stick to packets, the previous data packet will be discarded. num=%d, offset=%d\n", 
                                db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_failed_pkt_cnt, num, offset);
                        PRINT_HEX_DATA(ret_data, size + 4, "[%d] Discard Packet: ", db_packet_cnt);
                        memcpy(ret_data + 4, rx_data + 4 + offset2, size);
                        offset2 += size;
                    }
                    
                    int size_of_data = bin8_to_32(ret_data + 4);
                    if (size_of_data != size) {
                        db_failed_pkt_cnt += 2;
                        ROS_WARN("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d] Packet abnormal. num=%d, offset=%d, size=%d, length=%d\n", 
                            db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_failed_pkt_cnt, num, offset, size, size_of_data);
                        PRINT_HEX_DATA(ret_data, size + 4, "[%d] Abnormal Packet: ", db_packet_cnt);
                        if (reConnect()) {
                            size = 0;
                            offset = 0;
                            prev_pkt_is_not_empty = false;
                            first_cycle = 1;
                            continue;
                        };
                        ROS_ERROR("packet abnormal, reconnect failed\n");
                        break;
                    }
                    else {
                        if (prev_pkt_is_not_empty) {
                            PRINT_HEX_DATA(prev_data, size + 4, "[%d] Normal Packet: ", db_packet_cnt);
                            // xarm_driver.update_rich_data(prev_data, size + 4);
                        }
                        memcpy(prev_data, ret_data, size + 4);
                    }
                    // xarm_driver.update_rich_data(ret_data, size + 4);
                    offset = num - offset2;
                    if (offset > 0) {
                        if (DEBUG_MODE)
                            ROS_INFO("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d] Data packets are redundant and will be left for the next packet splicing process. num=%d, offset=%d\n", 
                                db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_failed_pkt_cnt, num, offset);
                        memcpy(ret_data + 4, rx_data + 4 + offset2, offset);
                    }
                    if (!prev_pkt_is_not_empty) {
                        prev_pkt_is_not_empty = true;
                        continue;
                    }
                }

                ret = report_data_ptr->flush_data(prev_data);

                // ret = xarm_driver.flush_report_data(report_data);
                if (ret == 0) {
                    db_success_pkt_cnt++;
                }
                else {
                    db_failed_pkt_cnt++;
                }
                if (DEBUG_MODE && db_packet_cnt % 900 == 2) {
                    // report_data_ptr->print_data();
                    ROS_INFO("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d]", db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_failed_pkt_cnt);
                }

                if (ret == 0)
                {
                    rxcnt++;
                    last_now = now;
                    now = ros::Time::now();
                    elapsed = now - last_now;
                    js_msg.header.stamp = now;
                    js_msg.header.frame_id = "real-time data";
                    js_msg.name.resize(joint_num_);
                    js_msg.position.resize(joint_num_);
                    js_msg.velocity.resize(joint_num_);
                    js_msg.effort.resize(joint_num_);
                    for(i = 0; i < joint_num_; i++)
                    {
                        d = (double)report_data_ptr->angle[i];
                        js_msg.name[i] = joint_name_[i];
                        js_msg.position[i] = d;

                        if (first_cycle)
                        {
                            js_msg.velocity[i] = 0;
                            first_cycle = 0;
                        }
                        else
                        {
                            js_msg.velocity[i] = (js_msg.position[i] - prev_angle[i]) / elapsed.toSec();
                        }

                        js_msg.effort[i] = (double)report_data_ptr->tau[i];

                        prev_angle[i] = d;
                    }

                    xarm_driver.pub_joint_state(js_msg);

                    rm_msg.state = report_data_ptr->state;
                    rm_msg.mode = report_data_ptr->mode;
                    rm_msg.cmdnum = report_data_ptr->cmdnum;
                    rm_msg.err = report_data_ptr->err;
                    rm_msg.warn = report_data_ptr->war;
                    rm_msg.mt_brake = report_data_ptr->mt_brake;
                    rm_msg.mt_able = report_data_ptr->mt_able;
                    rm_msg.angle.resize(joint_num_);

                    for(i = 0; i < joint_num_; i++)
                    {
                        /* set the float precision*/
                        double d = report_data_ptr->angle[i];
                        double r;
                        char str[8];
                        sprintf(str, "%0.3f", d);
                        sscanf(str, "%lf", &r);
                        rm_msg.angle[i] = r;
                    }
                    for(i = 0; i < 6; i++)
                    {
                        rm_msg.pose[i] = report_data_ptr->pose[i];
                        rm_msg.offset[i] = report_data_ptr->tcp_offset[i];
                    }
                    xarm_driver.pub_robot_msg(rm_msg);

                    // publish io state: This line may interfere with servoj execution
                    // xarm_driver.pub_io_state();

                    if (report_type == "rich" && report_data_ptr->total_num >= 417) {
                        cio_msg.state = report_data_ptr->cgpio_state;
                        cio_msg.code = report_data_ptr->cgpio_code;
                        cio_msg.input_digitals[0] = report_data_ptr->cgpio_input_digitals[0];
                        cio_msg.input_digitals[1] = report_data_ptr->cgpio_input_digitals[1];
                        cio_msg.output_digitals[0] = report_data_ptr->cgpio_output_digitals[0];
                        cio_msg.output_digitals[1] = report_data_ptr->cgpio_output_digitals[1];

                        cio_msg.input_analogs[0] = report_data_ptr->cgpio_input_analogs[0];
                        cio_msg.input_analogs[1] = report_data_ptr->cgpio_input_analogs[1];
                        cio_msg.output_analogs[0] = report_data_ptr->cgpio_output_analogs[0];
                        cio_msg.output_analogs[1] = report_data_ptr->cgpio_output_analogs[1];

                        for (int i = 0; i < 16; ++i) {
                            cio_msg.input_conf[i] = report_data_ptr->cgpio_input_conf[i];
                            cio_msg.output_conf[i] = report_data_ptr->cgpio_output_conf[i];
                        }
                        xarm_driver.pub_cgpio_state(cio_msg);
                    }
                }
                else
                {
                    ROS_WARN("[DEBUG] real_data.flush_data failed, ret = %d\n", ret);
                    // printf("Error: real_data.flush_data failed, ret = %d\n", ret);
                    err_num++;
                    break;
                }

            }
            delete report_data_ptr;
            delete [] prev_angle;
            ROS_ERROR("xArm Report Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
        }

        static void* thread_proc(void *arg)
        {
            XarmRTConnection* threadTest=(XarmRTConnection*)arg;
            threadTest->thread_run();
            return (void*)0;
        }

    public:
        char *ip;
        std::string report_type;
        ros::Time now, last_now;
        ros::Duration elapsed;
        SocketPort *arm_report;
        // ReportDataNorm norm_data;
        XArmReportData *report_data_ptr;
        sensor_msgs::JointState js_msg;
        xarm_api::XARMDriver xarm_driver;
        xarm_msgs::RobotMsg rm_msg;
        xarm_msgs::CIOState cio_msg;

        int joint_num_;
        std::vector<std::string> joint_name_;
        constexpr static const double GET_FRAME_RATE_HZ = 1000;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_driver_node");

    // with namespace (ns) specified in the calling launch file (xarm by default)
    ros::NodeHandle n;

    xarm_api::XARMDriver driver;
    ROS_INFO("start xarm driver");

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

    char server_ip[20]={0};
    strcpy(server_ip,robot_ip.c_str());
    XarmRTConnection rt_connect(n, server_ip, driver);

    signal(SIGINT, exit_sig_handler);
    ros::waitForShutdown();

    printf("end");

    return 0;
}
