/* Copyright 2017 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jimy Zhang <jimy92@163.com>
 ============================================================================*/
#ifndef COMMON_DATA_TYPE_H_
#define COMMON_DATA_TYPE_H_

#include <stdio.h>

typedef unsigned char u8;
typedef signed char s8;
typedef unsigned short u16;
typedef signed short s16;
typedef unsigned int u32;
typedef signed int s32;
typedef unsigned long long u64;
typedef signed long long s64;
typedef float fp32;
typedef double fp64;

#define bin64_to_8(a, b)           \
  {                                \
    (b)[0] = (u8)((u64)(a) >> 56); \
    (b)[1] = (u8)((u64)(a) >> 48); \
    (b)[2] = (u8)((u64)(a) >> 40); \
    (b)[3] = (u8)((u64)(a) >> 32); \
    (b)[4] = (u8)((u64)(a) >> 24); \
    (b)[5] = (u8)((u64)(a) >> 16); \
    (b)[6] = (u8)((u64)(a) >> 8);  \
                (b)[7] = (u8)(a)); \
  }

#define bin32_to_8(a, b)           \
  {                                \
    (b)[0] = (u8)((u32)(a) >> 24); \
    (b)[1] = (u8)((u32)(a) >> 16); \
    (b)[2] = (u8)((u32)(a) >> 8);  \
    (b)[3] = (u8)(a);              \
  }

#define bin16_to_8(a, b)          \
  {                               \
    (b)[0] = (u8)((u16)(a) >> 8); \
    (b)[1] = (u8)(a);             \
  }

#define bin8_to_64(a)                                                      \
  ((((a)[0] << 56) | ((a)[1] << 48)) + (((a)[2] << 40) | ((a)[3] << 32)) + \
   (((a)[4] << 24) | ((a)[5] << 16)) + (((a)[6] << 8) | ((a)[7])))

#define bin8_to_32(a) \
  ((((a)[0] << 24) | ((a)[1] << 16)) + (((a)[2] << 8) | ((a)[3])))

#define bin8_to_16(a) (((a)[0] << 8) | ((a)[1]))

inline void fp32_to_hex(fp64 dataf, u8 datahex[4]) {
  union _fp32hex {
    fp32 dataf;
    u8 datahex[4];
  } fp32hex;
  fp32hex.dataf = dataf;
  datahex[0] = fp32hex.datahex[0];
  datahex[1] = fp32hex.datahex[1];
  datahex[2] = fp32hex.datahex[2];
  datahex[3] = fp32hex.datahex[3];
}

inline fp64 hex_to_fp32(u8 datahex[4]) {
  union _fp32hex {
    fp32 dataf;
    u8 datahex[4];
  } fp32hex;
  fp32hex.datahex[0] = datahex[0];
  fp32hex.datahex[1] = datahex[1];
  fp32hex.datahex[2] = datahex[2];
  fp32hex.datahex[3] = datahex[3];
  return (fp64)fp32hex.dataf;
}

inline void hex_to_nfp32(u8 *datahex, fp32 *dataf, u8 n) {
  for (u8 i = 0; i < n; ++i) dataf[i] = hex_to_fp32(&datahex[i * 4]);
}

inline void nfp32_to_hex(fp32 *dataf, u8 *datahex, u8 n) {
  for (u8 i = 0; i < n; ++i) fp32_to_hex(dataf[i], &datahex[i * 4]);
}

#endif
