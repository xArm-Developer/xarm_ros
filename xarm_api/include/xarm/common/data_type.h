/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef COMMON_DATA_TYPE_H_
#define COMMON_DATA_TYPE_H_

#include <stdio.h>

inline void bin64_to_8(long long a, unsigned char* b) {
  b[0] = (unsigned char)(a >> 56);
  b[1] = (unsigned char)(a >> 48);
  b[2] = (unsigned char)(a >> 40);
  b[3] = (unsigned char)(a >> 32);
  b[4] = (unsigned char)(a >> 24);
  b[5] = (unsigned char)(a >> 16);
  b[6] = (unsigned char)(a >> 8);
  b[7] = (unsigned char)a;
}

inline void bin32_to_8(int a, unsigned char *b) {
  b[0] = (unsigned char)(a >> 24);
  b[1] = (unsigned char)(a >> 16);
  b[2] = (unsigned char)(a >> 8);
  b[3] = (unsigned char)a;
}

inline void bin16_to_8(int a, unsigned char *b) {
  unsigned short temp = a;
  b[0] = (unsigned char)(temp >> 8);
  b[1] = (unsigned char)temp;
}

inline int bin8_to_32(unsigned char *a) {
  return ((a[0] << 24) + (a[1] << 16) + (a[2] << 8) + a[3]);
}

inline int bin8_to_16(unsigned char *a) {
  signed int tmp = (a[0] << 8) + a[1];
  return tmp;
}

inline int bin8_to_s16(unsigned char *a) {
  return (int)(short)(a[0] << 8) + a[1];
}

inline void bin8_to_ns16(unsigned char *a, int *data, int n) {
  for (int i = 0; i < n; ++i) {
    data[i] = bin8_to_s16(&a[i * 2]);
  }
}

inline void fp32_to_hex(double dataf, unsigned char datahex[4]) {
  union _fp32hex {
    float dataf;
    unsigned char datahex[4];
  } fp32hex;
  fp32hex.dataf = (float)dataf;
  datahex[0] = fp32hex.datahex[0];
  datahex[1] = fp32hex.datahex[1];
  datahex[2] = fp32hex.datahex[2];
  datahex[3] = fp32hex.datahex[3];
}

inline float hex_to_fp32(unsigned char datahex[4]) {
  union _fp32hex {
    float dataf;
    unsigned char datahex[4];
  } fp32hex;
  fp32hex.datahex[0] = datahex[0];
  fp32hex.datahex[1] = datahex[1];
  fp32hex.datahex[2] = datahex[2];
  fp32hex.datahex[3] = datahex[3];
  return (float)fp32hex.dataf;
}

inline void int32_to_hex(int data, unsigned char datahex[4]) {
  union _int32hex {
    int data;
    unsigned char datahex[4];
  } int32hex;
  int32hex.data = data;
  datahex[0] = int32hex.datahex[0];
  datahex[1] = int32hex.datahex[1];
  datahex[2] = int32hex.datahex[2];
  datahex[3] = int32hex.datahex[3];
}

inline  void hex_to_nfp32(unsigned char *datahex, float *dataf, int n) {
  for (int i = 0; i < n; ++i)
  {
    dataf[i] = hex_to_fp32(&datahex[i * 4]);
  }
}

inline void nfp32_to_hex(float *dataf, unsigned char *datahex, int n) {
  for (int i = 0; i < n; ++i)
  {
    fp32_to_hex(dataf[i], &datahex[i * 4]);
  }
}

inline void nint32_to_hex(int *data, unsigned char *datahex, int n) {
  for (int i = 0; i < n; ++i)
  {
    int32_to_hex(data[i], &datahex[i * 4]);
  }
}

#endif
