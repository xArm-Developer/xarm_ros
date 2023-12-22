/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/\
#ifndef WIN32
#include <unistd.h>
#endif
#include <signal.h>
#include "ros/ros.h"
#include "xarm/core/connect.h"
#include "xarm/core/report_data.h"

void exit_sig_handler(int signum)
{
  fprintf(stderr, "Ctrl-C caught, exit process...\n");
  exit(-1);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("Please enter IP address\n");
    return 0;
  }
  char *server_ip = argv[1];
  SocketPort *arm_report = connect_tcp_report_norm(server_ip);
  if (arm_report == NULL) return 0;

  ros::init(argc, argv, "example_report_norm");
  ros::NodeHandle nh;

  signal(SIGINT, exit_sig_handler);

  int rxcnt = 0;
  ReportDataNorm *norm_data = new ReportDataNorm();

  unsigned char rx_data[1280];
  int ret;
  int err_num = 0;
  while (1) {
    ros::Duration(0.001).sleep(); //1ms

    ret = arm_report->read_frame(rx_data);
    if (ret != 0) continue;
    ret = norm_data->flush_data(rx_data);

    if (ret == 0) {
      rxcnt++;
      printf("\n【normal report】: len = %d, rxcnt = %d, err_num = %d\n", bin8_to_32(rx_data), rxcnt, err_num);
      norm_data->print_data();
    } else {
      printf("Error: norm_data.flush_data failed, ret = %d\n", ret);
      err_num++;
    }
  }
}
