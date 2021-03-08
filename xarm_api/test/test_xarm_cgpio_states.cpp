/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason <jason@ufactory.cc>
           Vinman <vinman@ufactory.cc> 
 ============================================================================*/
#include "ros/ros.h"
#include <xarm_msgs/CIOState.h>

void chatterCallback(xarm_msgs::CIOState cio_msg)
{
  // printf("[CGPIO] functional_input_difitals: [");
  // for (int i = 0; i < 16; ++i) {
  //   printf("%d,", (cio_msg.input_digitals[0] >> i) & 0x0001);
  // }
  // printf("]\n");
  // printf("[CGPIO] functional_output_difitals: [");
  // for (int i = 0; i < 16; ++i) {
  //   printf("%d,", (cio_msg.output_digitals[0] >> i) & 0x0001);
  // }
  // printf("]\n");

  printf("[INPUT]\n");
  printf("    [CI] ");
  for (int i = 0; i < 8; ++i) { printf("CI%d=%d, ", i, (cio_msg.input_digitals[1] >> i) & 0x0001); }
  printf("\n");
  // printf("    [DI] ");
  // for (int i = 8; i < 16; ++i) { printf("DI%d=%d, ", i-8, (cio_msg.input_digitals[1] >> i) & 0x0001); }
  // printf("\n");
  printf("    [AI] AI0=%f, AI1=%f\n", cio_msg.input_analogs[0], cio_msg.input_analogs[1]);

  printf("[OUTPUT]\n");
  printf("    [CO] ");
  for (int i = 0; i < 8; ++i) { printf("CO%d=%d, ", i, (cio_msg.output_digitals[1] >> i) & 0x0001); }
  printf("\n");
  // printf("    [DO] ");
  // for (int i = 8; i < 16; ++i) { printf("DO%d=%d, ", i-8, (cio_msg.output_digitals[1] >> i) & 0x0001); }
  // printf("\n");
  printf("    [AO] AO0=%f, AO1=%f\n", cio_msg.output_analogs[0], cio_msg.output_analogs[1]);


  printf("[CONF]\n");
  printf("    [CI] ");
    for (int i = 0; i < 8; ++i) { printf("CI%d=%d, ", i, cio_msg.input_conf[i]); }
  printf("\n");
  // printf("    [DI] ");
  //   for (int i = 8; i < 16; ++i) { printf("DI%d=%d, ", i-8, cio_msg.input_conf[i]); }
  // printf("\n");
  printf("    [CO] ");
    for (int i = 0; i < 8; ++i) { printf("CO%d=%d, ", i, cio_msg.output_conf[i]); }
  printf("\n");
  // printf("    [DO] ");
  //   for (int i = 8; i < 16; ++i) { printf("DO%d=%d, ", i-8, cio_msg.output_conf[i]); }
  // printf("\n");

  printf("======================================\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xarm_cgpio_states");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/xarm/xarm_cgpio_states", 1000, chatterCallback);
  ros::spin();

  return 0;
}
