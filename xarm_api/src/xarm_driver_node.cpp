/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
// #include <xarm/core/linux/thread.h>
#include <signal.h>
#include <thread>
#include "xarm/core/connect.h"
#include "xarm/core/report_data.h"
#include "xarm/core/debug/debug_print.h"

#define DEBUG_MODE 0
#define PRINT_HEX_DATA(hex, len, ...)     \
{                                         \
    if (DEBUG_MODE) {                     \
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
            ip = server_ip;
            xarm_driver = drv;
            xarm_driver.XARMDriverInit(root_nh, server_ip);
            ros::Duration(0.5).sleep();
            std::thread th(thread_proc, this);
            th.detach();
            // thread_id = thread_init(thread_proc, (void *)this);
        }

        bool reConnect(void)
        {
            xarm_driver.closeReportSocket();
            int retryCnt = 5;
            ROS_INFO("try reconnect to report\n");
            while (retryCnt--)
            {
                if (xarm_driver.reConnectReportSocket(ip)) {
                    ROS_INFO("reconnect success\n");
                    return true;
                }
                ros::Duration(2).sleep();
            }
            ROS_ERROR("reconnect failed\n");
            return false;
        }

        void thread_run(void)
        {
            int ret;
            int err_num = 0;
            int rxcnt;
            int i;
            int first_cycle = 1;
            double d, prev_angle[joint_num_];

            ros::Rate r(REPORT_RATE_HZ); // 10Hz

            int size = 0;
            int num = 0;
            int offset = 0;
            unsigned char rx_data[1024];
            unsigned char prev_data[512];
            unsigned char ret_data[1024 * 2];

            int db_read_cnt = 0;
            int db_packet_cnt = 0;
            int db_success_pkt_cnt = 0;
            int db_discard_pkt_cnt = 0;
            int db_faied_pkt_cnt = 0;
            bool prev_pkt_is_not_empty = false;

            // setlocale(LC_CTYPE, "zh_CN.utf8");
            // setlocale(LC_ALL, "");

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
                    ROS_INFO("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d] The data packet length is insufficient, waiting for the next packet splicing. num=%d, offset=%d, length=%d\n", 
                        db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_faied_pkt_cnt, num, offset, num + offset);
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
                        ROS_INFO("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d] Data packet stick to packets, the previous data packet will be discarded. num=%d, offset=%d\n", 
                            db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_faied_pkt_cnt, num, offset);
                        PRINT_HEX_DATA(ret_data, size + 4, "[%d] Discard Packet: ", db_packet_cnt);
                        memcpy(ret_data + 4, rx_data + 4 + offset2, size);
                        offset2 += size;
                    }
                    
                    int size_of_data = bin8_to_32(ret_data + 4);
                    if (size_of_data != size) {
                        db_faied_pkt_cnt += 2;
                        ROS_WARN("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d] Packet abnormal. num=%d, offset=%d, size=%d, length=%d\n", 
                            db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_faied_pkt_cnt, num, offset, size, size_of_data);
                        PRINT_HEX_DATA(ret_data, size + 4, "[%d] Abnormal Packet: ", db_packet_cnt);
                        if (reConnect()) {
                            size = 0;
                            offset = 0;
                            prev_pkt_is_not_empty = false;
                            continue;
                        };
                        ROS_ERROR("packet abnormal, reconnect failed\n");
                        break;
                    }
                    else {
                        if (prev_pkt_is_not_empty) {
                            PRINT_HEX_DATA(prev_data, size + 4, "[%d] Normal Packet: ", db_packet_cnt);
                            xarm_driver.update_rich_data(prev_data, size + 4);
                        }
                        memcpy(prev_data, ret_data, size + 4);
                    }
                    // xarm_driver.update_rich_data(ret_data, size + 4);
                    offset = num - offset2;
                    if (offset > 0) {
                        ROS_INFO("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d] Data packets are redundant and will be left for the next packet splicing process. num=%d, offset=%d\n", 
                            db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_faied_pkt_cnt, num, offset);
                        memcpy(ret_data + 4, rx_data + 4 + offset2, offset);
                    }
                    if (!prev_pkt_is_not_empty) {
                        prev_pkt_is_not_empty = true;
                        continue;
                    }
                }

                ret = xarm_driver.get_rich_data(norm_data);
                if (ret == 0) {
                    db_success_pkt_cnt++;
                }
                else {
                    db_faied_pkt_cnt++;
                }
                if (db_packet_cnt % 900 == 2) {
                    ROS_INFO("[READ:%d][PACKET:%d][SUCCESS:%d][DISCARD:%d][FAILED:%d]", db_read_cnt, db_packet_cnt, db_success_pkt_cnt, db_discard_pkt_cnt, db_faied_pkt_cnt);
                }

                if (ret == 0)
                {
                    rxcnt++;

                    now = ros::Time::now();
                    js_msg.header.stamp = now;
                    js_msg.header.frame_id = "real-time data";
                    js_msg.name.resize(joint_num_);
                    js_msg.position.resize(joint_num_);
                    js_msg.velocity.resize(joint_num_);
                    js_msg.effort.resize(joint_num_);
                    for(i = 0; i < joint_num_; i++)
                    {
                        d = (double)norm_data.angle_[i];
                        js_msg.name[i] = joint_name_[i];
                        js_msg.position[i] = d;

                        if (first_cycle)
                        {
                            js_msg.velocity[i] = 0;
                            first_cycle = 0;
                        }
                        else
                        {
                            js_msg.velocity[i] = (js_msg.position[i] - prev_angle[i])*REPORT_RATE_HZ;
                        }

                        js_msg.effort[i] = (double)norm_data.tau_[i];

                        prev_angle[i] = d;
                    }

                    xarm_driver.pub_joint_state(js_msg);

                    rm_msg.state = norm_data.runing_;
                    rm_msg.mode = norm_data.mode_;
                    rm_msg.cmdnum = norm_data.cmdnum_;
                    rm_msg.err = norm_data.err_;
                    rm_msg.warn = norm_data.war_;
                    rm_msg.mt_brake = norm_data.mt_brake_;
                    rm_msg.mt_able = norm_data.mt_able_;
                    rm_msg.angle.resize(joint_num_);

                    for(i = 0; i < joint_num_; i++)
                    {
                        /* set the float precision*/
                        double d = norm_data.angle_[i];
                        double r;
                        char str[8];
                        sprintf(str, "%0.3f", d);
                        sscanf(str, "%lf", &r);
                        rm_msg.angle[i] = r;
                    }
                    for(i = 0; i < 6; i++)
                    {
                        rm_msg.pose[i] = norm_data.pose_[i];
                        rm_msg.offset[i] = norm_data.tcp_offset_[i];
                    }
                    xarm_driver.pub_robot_msg(rm_msg);

                    // publish io state: This line may interfere with servoj execution
                    // xarm_driver.pub_io_state();

                }
                else
                {
                    ROS_WARN("[DEBUG] real_data.flush_data failed, ret = %d\n", ret);
                    // printf("Error: real_data.flush_data failed, ret = %d\n", ret);
                    err_num++;
                    break;
                }

            }
            ROS_ERROR("xArm Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
        }

        static void* thread_proc(void *arg)
        {
            XarmRTConnection* pThreadTest=(XarmRTConnection*)arg;
            pThreadTest->thread_run();
            pthread_exit(0);
        }

    public:
        // pthread_t thread_id;
        char *ip;
        ros::Time now;
        SocketPort *arm_report;
        ReportDataNorm norm_data;
        sensor_msgs::JointState js_msg;
        xarm_api::XARMDriver xarm_driver;
        xarm_msgs::RobotMsg rm_msg;

        int joint_num_;
        std::vector<std::string> joint_name_;
        constexpr static const double REPORT_RATE_HZ = 1000; /* 10Hz, same with norm_report frequency */
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
