/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-11-16T06:51:12+0900
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "network.h"
#include "network_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "0xb3a67c3a5ec364e284c2ba960c46b2d7"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2025-11-16T06:51:12+0900"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

static ai_ptr g_network_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_network_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  input_layer_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 3840, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  activation_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  activation_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  add_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  activation_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  zero_padding1d_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8448, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  activation_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  zero_padding1d_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8448, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  add_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  activation_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  zero_padding1d_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8704, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  activation_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  zero_padding1d_5_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8704, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  add_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  activation_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_9_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  zero_padding1d_6_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 9216, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_7_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  activation_7_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  zero_padding1d_7_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 13824, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_8_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  add_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  activation_8_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  zero_padding1d_8_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 15360, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_10_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  activation_9_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  zero_padding1d_9_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 15360, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_11_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#35 */
AI_ARRAY_OBJ_DECLARE(
  add_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#36 */
AI_ARRAY_OBJ_DECLARE(
  activation_10_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#37 */
AI_ARRAY_OBJ_DECLARE(
  global_average_pooling1d_pool_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#38 */
AI_ARRAY_OBJ_DECLARE(
  dense_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#39 */
AI_ARRAY_OBJ_DECLARE(
  dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#40 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4, AI_STATIC)

/* Array#41 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 4, AI_STATIC)

/* Array#42 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 5760, AI_STATIC)

/* Array#43 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#44 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#45 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#46 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#47 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#48 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_3_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#49 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_3_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#50 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_4_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#51 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#52 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_5_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#53 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_5_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#54 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_6_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#55 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_6_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#56 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_9_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#57 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_9_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#58 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_7_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 18432, AI_STATIC)

/* Array#59 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_7_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#60 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_8_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 27648, AI_STATIC)

/* Array#61 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_8_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#62 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_10_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 27648, AI_STATIC)

/* Array#63 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_10_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#64 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_11_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 27648, AI_STATIC)

/* Array#65 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_11_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#66 */
AI_ARRAY_OBJ_DECLARE(
  dense_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 12288, AI_STATIC)

/* Array#67 */
AI_ARRAY_OBJ_DECLARE(
  dense_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#68 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 512, AI_STATIC)

/* Array#69 */
AI_ARRAY_OBJ_DECLARE(
  dense_1_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4, AI_STATIC)

/* Array#70 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 90, AI_STATIC)

/* Array#71 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#72 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_2_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#73 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_9_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  activation_10_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &activation_10_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  activation_1_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &activation_1_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  activation_2_output, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &activation_2_output_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  activation_3_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &activation_3_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  activation_4_output, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &activation_4_output_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  activation_5_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &activation_5_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  activation_6_output, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &activation_6_output_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  activation_7_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &activation_7_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  activation_8_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &activation_8_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  activation_9_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &activation_9_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  activation_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &activation_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  add_1_output, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &add_1_output_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  add_2_output, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &add_2_output_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  add_3_output, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &add_3_output_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  add_4_output, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &add_4_output_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  add_output, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &add_output_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_10_bias, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_10_bias_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_10_output, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_10_output_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_10_weights, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 96, 1, 3, 96), AI_STRIDE_INIT(4, 4, 384, 36864, 36864),
  1, &conv1d_10_weights_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_11_bias, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_11_bias_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_11_output, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_11_output_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_11_weights, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 96, 1, 3, 96), AI_STRIDE_INIT(4, 4, 384, 36864, 36864),
  1, &conv1d_11_weights_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_bias, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_1_bias_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_output, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_1_output_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_scratch0, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 3), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_1_scratch0_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_weights, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 3, 64), AI_STRIDE_INIT(4, 4, 256, 16384, 16384),
  1, &conv1d_1_weights_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_2_bias, AI_STATIC,
  26, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_2_bias_array, NULL)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_2_output, AI_STATIC,
  27, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_2_output_array, NULL)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_2_scratch0, AI_STATIC,
  28, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 3), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_2_scratch0_array, NULL)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_2_weights, AI_STATIC,
  29, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 3, 64), AI_STRIDE_INIT(4, 4, 256, 16384, 16384),
  1, &conv1d_2_weights_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_3_bias, AI_STATIC,
  30, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_3_bias_array, NULL)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_3_output, AI_STATIC,
  31, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_3_output_array, NULL)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_3_weights, AI_STATIC,
  32, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 3, 64), AI_STRIDE_INIT(4, 4, 256, 16384, 16384),
  1, &conv1d_3_weights_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_4_bias, AI_STATIC,
  33, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_4_bias_array, NULL)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_4_output, AI_STATIC,
  34, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_4_output_array, NULL)

/* Tensor #35 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_4_weights, AI_STATIC,
  35, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 3, 64), AI_STRIDE_INIT(4, 4, 256, 16384, 16384),
  1, &conv1d_4_weights_array, NULL)

/* Tensor #36 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_5_bias, AI_STATIC,
  36, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_5_bias_array, NULL)

/* Tensor #37 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_5_output, AI_STATIC,
  37, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_5_output_array, NULL)

/* Tensor #38 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_5_weights, AI_STATIC,
  38, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 3, 64), AI_STRIDE_INIT(4, 4, 256, 16384, 16384),
  1, &conv1d_5_weights_array, NULL)

/* Tensor #39 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_6_bias, AI_STATIC,
  39, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_6_bias_array, NULL)

/* Tensor #40 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_6_output, AI_STATIC,
  40, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_6_output_array, NULL)

/* Tensor #41 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_6_weights, AI_STATIC,
  41, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 3, 64), AI_STRIDE_INIT(4, 4, 256, 16384, 16384),
  1, &conv1d_6_weights_array, NULL)

/* Tensor #42 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_7_bias, AI_STATIC,
  42, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_7_bias_array, NULL)

/* Tensor #43 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_7_output, AI_STATIC,
  43, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_7_output_array, NULL)

/* Tensor #44 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_7_weights, AI_STATIC,
  44, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 3, 96), AI_STRIDE_INIT(4, 4, 256, 24576, 24576),
  1, &conv1d_7_weights_array, NULL)

/* Tensor #45 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_8_bias, AI_STATIC,
  45, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_8_bias_array, NULL)

/* Tensor #46 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_8_output, AI_STATIC,
  46, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_8_output_array, NULL)

/* Tensor #47 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_8_weights, AI_STATIC,
  47, 0x0,
  AI_SHAPE_INIT(4, 96, 1, 3, 96), AI_STRIDE_INIT(4, 4, 384, 36864, 36864),
  1, &conv1d_8_weights_array, NULL)

/* Tensor #48 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_9_bias, AI_STATIC,
  48, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_9_bias_array, NULL)

/* Tensor #49 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_9_output, AI_STATIC,
  49, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 128), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &conv1d_9_output_array, NULL)

/* Tensor #50 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_9_scratch0, AI_STATIC,
  50, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_9_scratch0_array, NULL)

/* Tensor #51 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_9_weights, AI_STATIC,
  51, 0x0,
  AI_SHAPE_INIT(4, 64, 1, 1, 96), AI_STRIDE_INIT(4, 4, 256, 24576, 24576),
  1, &conv1d_9_weights_array, NULL)

/* Tensor #52 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_bias, AI_STATIC,
  52, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_bias_array, NULL)

/* Tensor #53 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_output, AI_STATIC,
  53, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 128), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &conv1d_output_array, NULL)

/* Tensor #54 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_scratch0, AI_STATIC,
  54, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 3), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &conv1d_scratch0_array, NULL)

/* Tensor #55 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_weights, AI_STATIC,
  55, 0x0,
  AI_SHAPE_INIT(4, 30, 1, 3, 64), AI_STRIDE_INIT(4, 4, 120, 7680, 7680),
  1, &conv1d_weights_array, NULL)

/* Tensor #56 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_dense_bias, AI_STATIC,
  56, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &dense_1_dense_bias_array, NULL)

/* Tensor #57 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_dense_output, AI_STATIC,
  57, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &dense_1_dense_output_array, NULL)

/* Tensor #58 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_dense_weights, AI_STATIC,
  58, 0x0,
  AI_SHAPE_INIT(4, 128, 4, 1, 1), AI_STRIDE_INIT(4, 4, 512, 2048, 2048),
  1, &dense_1_dense_weights_array, NULL)

/* Tensor #59 */
AI_TENSOR_OBJ_DECLARE(
  dense_1_output, AI_STATIC,
  59, 0x0,
  AI_SHAPE_INIT(4, 1, 4, 1, 1), AI_STRIDE_INIT(4, 4, 4, 16, 16),
  1, &dense_1_output_array, NULL)

/* Tensor #60 */
AI_TENSOR_OBJ_DECLARE(
  dense_dense_bias, AI_STATIC,
  60, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &dense_dense_bias_array, NULL)

/* Tensor #61 */
AI_TENSOR_OBJ_DECLARE(
  dense_dense_output, AI_STATIC,
  61, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &dense_dense_output_array, NULL)

/* Tensor #62 */
AI_TENSOR_OBJ_DECLARE(
  dense_dense_weights, AI_STATIC,
  62, 0x0,
  AI_SHAPE_INIT(4, 96, 128, 1, 1), AI_STRIDE_INIT(4, 4, 384, 49152, 49152),
  1, &dense_dense_weights_array, NULL)

/* Tensor #63 */
AI_TENSOR_OBJ_DECLARE(
  dense_output, AI_STATIC,
  63, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &dense_output_array, NULL)

/* Tensor #64 */
AI_TENSOR_OBJ_DECLARE(
  global_average_pooling1d_pool_output, AI_STATIC,
  64, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &global_average_pooling1d_pool_output_array, NULL)

/* Tensor #65 */
AI_TENSOR_OBJ_DECLARE(
  input_layer_output, AI_STATIC,
  65, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 128), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &input_layer_output_array, NULL)

/* Tensor #66 */
AI_TENSOR_OBJ_DECLARE(
  zero_padding1d_2_output, AI_STATIC,
  66, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 132), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &zero_padding1d_2_output_array, NULL)

/* Tensor #67 */
AI_TENSOR_OBJ_DECLARE(
  zero_padding1d_3_output, AI_STATIC,
  67, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 132), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &zero_padding1d_3_output_array, NULL)

/* Tensor #68 */
AI_TENSOR_OBJ_DECLARE(
  zero_padding1d_4_output, AI_STATIC,
  68, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 136), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &zero_padding1d_4_output_array, NULL)

/* Tensor #69 */
AI_TENSOR_OBJ_DECLARE(
  zero_padding1d_5_output, AI_STATIC,
  69, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 136), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &zero_padding1d_5_output_array, NULL)

/* Tensor #70 */
AI_TENSOR_OBJ_DECLARE(
  zero_padding1d_6_output, AI_STATIC,
  70, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 144), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &zero_padding1d_6_output_array, NULL)

/* Tensor #71 */
AI_TENSOR_OBJ_DECLARE(
  zero_padding1d_7_output, AI_STATIC,
  71, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 144), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &zero_padding1d_7_output_array, NULL)

/* Tensor #72 */
AI_TENSOR_OBJ_DECLARE(
  zero_padding1d_8_output, AI_STATIC,
  72, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 160), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &zero_padding1d_8_output_array, NULL)

/* Tensor #73 */
AI_TENSOR_OBJ_DECLARE(
  zero_padding1d_9_output, AI_STATIC,
  73, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 160), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &zero_padding1d_9_output_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_layer, 64,
  SM_TYPE, 0x0, NULL,
  sm, forward_sm,
  &dense_1_chain,
  NULL, &dense_1_layer, AI_STATIC, 
  .nl_params = NULL, 
  .axis = AI_SHAPE_CHANNEL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_1_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_1_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_1_dense_weights, &dense_1_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_1_dense_layer, 64,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_1_dense_chain,
  NULL, &dense_1_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_layer, 62,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &dense_chain,
  NULL, &dense_1_dense_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling1d_pool_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &dense_dense_weights, &dense_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_dense_layer, 62,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_dense_chain,
  NULL, &dense_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  global_average_pooling1d_pool_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling1d_pool_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  global_average_pooling1d_pool_layer, 61,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &global_average_pooling1d_pool_chain,
  NULL, &dense_dense_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(1, 128), 
  .pool_stride = AI_SHAPE_2D_INIT(1, 128), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_10_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_10_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_10_layer, 60,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_10_chain,
  NULL, &global_average_pooling1d_pool_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &activation_8_output, &conv1d_11_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_4_layer, 59,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &add_4_chain,
  NULL, &activation_10_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_11_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_11_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_11_weights, &conv1d_11_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_11_layer, 57,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_11_chain,
  NULL, &add_4_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 16), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


AI_STATIC_CONST ai_float zero_padding1d_9_value_data[] = { 0.0 };
AI_ARRAY_OBJ_DECLARE(
    zero_padding1d_9_value, AI_ARRAY_FORMAT_FLOAT,
    zero_padding1d_9_value_data, zero_padding1d_9_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  zero_padding1d_9_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_9_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  zero_padding1d_9_layer, 55,
  PAD_TYPE, 0x0, NULL,
  pad, forward_pad,
  &zero_padding1d_9_chain,
  NULL, &conv1d_11_layer, AI_STATIC, 
  .value = &zero_padding1d_9_value, 
  .mode = AI_PAD_CONSTANT, 
  .pads = AI_SHAPE_INIT(4, 32, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_9_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_9_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_9_layer, 53,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_9_chain,
  NULL, &zero_padding1d_9_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_10_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_10_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_10_weights, &conv1d_10_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_10_layer, 52,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_10_chain,
  NULL, &activation_9_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 16), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


AI_STATIC_CONST ai_float zero_padding1d_8_value_data[] = { 0.0 };
AI_ARRAY_OBJ_DECLARE(
    zero_padding1d_8_value, AI_ARRAY_FORMAT_FLOAT,
    zero_padding1d_8_value_data, zero_padding1d_8_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  zero_padding1d_8_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_8_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  zero_padding1d_8_layer, 50,
  PAD_TYPE, 0x0, NULL,
  pad, forward_pad,
  &zero_padding1d_8_chain,
  NULL, &conv1d_10_layer, AI_STATIC, 
  .value = &zero_padding1d_8_value, 
  .mode = AI_PAD_CONSTANT, 
  .pads = AI_SHAPE_INIT(4, 32, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_8_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_8_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_8_layer, 49,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_8_chain,
  NULL, &zero_padding1d_8_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_9_output, &conv1d_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_3_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_3_layer, 48,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &add_3_chain,
  NULL, &activation_8_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_8_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_8_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_8_weights, &conv1d_8_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_8_layer, 45,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_8_chain,
  NULL, &add_3_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 8), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


AI_STATIC_CONST ai_float zero_padding1d_7_value_data[] = { 0.0 };
AI_ARRAY_OBJ_DECLARE(
    zero_padding1d_7_value, AI_ARRAY_FORMAT_FLOAT,
    zero_padding1d_7_value_data, zero_padding1d_7_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  zero_padding1d_7_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_7_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  zero_padding1d_7_layer, 42,
  PAD_TYPE, 0x0, NULL,
  pad, forward_pad,
  &zero_padding1d_7_chain,
  NULL, &conv1d_8_layer, AI_STATIC, 
  .value = &zero_padding1d_7_value, 
  .mode = AI_PAD_CONSTANT, 
  .pads = AI_SHAPE_INIT(4, 16, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_7_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_7_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_7_layer, 40,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_7_chain,
  NULL, &zero_padding1d_7_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_7_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_7_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_7_weights, &conv1d_7_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_7_layer, 39,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_7_chain,
  NULL, &activation_7_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 8), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


AI_STATIC_CONST ai_float zero_padding1d_6_value_data[] = { 0.0 };
AI_ARRAY_OBJ_DECLARE(
    zero_padding1d_6_value, AI_ARRAY_FORMAT_FLOAT,
    zero_padding1d_6_value_data, zero_padding1d_6_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  zero_padding1d_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_6_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  zero_padding1d_6_layer, 37,
  PAD_TYPE, 0x0, NULL,
  pad, forward_pad,
  &zero_padding1d_6_chain,
  NULL, &conv1d_7_layer, AI_STATIC, 
  .value = &zero_padding1d_6_value, 
  .mode = AI_PAD_CONSTANT, 
  .pads = AI_SHAPE_INIT(4, 16, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_9_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_9_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_9_weights, &conv1d_9_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_9_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_9_layer, 46,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &conv1d_9_chain,
  NULL, &zero_padding1d_6_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_6_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_6_layer, 36,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_6_chain,
  NULL, &conv1d_9_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &activation_4_output, &conv1d_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_2_layer, 35,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &add_2_chain,
  NULL, &activation_6_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_6_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_6_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_6_weights, &conv1d_6_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_6_layer, 33,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_6_chain,
  NULL, &add_2_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 4), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


AI_STATIC_CONST ai_float zero_padding1d_5_value_data[] = { 0.0 };
AI_ARRAY_OBJ_DECLARE(
    zero_padding1d_5_value, AI_ARRAY_FORMAT_FLOAT,
    zero_padding1d_5_value_data, zero_padding1d_5_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  zero_padding1d_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_5_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  zero_padding1d_5_layer, 31,
  PAD_TYPE, 0x0, NULL,
  pad, forward_pad,
  &zero_padding1d_5_chain,
  NULL, &conv1d_6_layer, AI_STATIC, 
  .value = &zero_padding1d_5_value, 
  .mode = AI_PAD_CONSTANT, 
  .pads = AI_SHAPE_INIT(4, 8, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_5_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_5_layer, 29,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_5_chain,
  NULL, &zero_padding1d_5_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_5_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_5_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_5_weights, &conv1d_5_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_5_layer, 28,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_5_chain,
  NULL, &activation_5_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 4), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


AI_STATIC_CONST ai_float zero_padding1d_4_value_data[] = { 0.0 };
AI_ARRAY_OBJ_DECLARE(
    zero_padding1d_4_value, AI_ARRAY_FORMAT_FLOAT,
    zero_padding1d_4_value_data, zero_padding1d_4_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  zero_padding1d_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  zero_padding1d_4_layer, 26,
  PAD_TYPE, 0x0, NULL,
  pad, forward_pad,
  &zero_padding1d_4_chain,
  NULL, &conv1d_5_layer, AI_STATIC, 
  .value = &zero_padding1d_4_value, 
  .mode = AI_PAD_CONSTANT, 
  .pads = AI_SHAPE_INIT(4, 8, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_4_layer, 25,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_4_chain,
  NULL, &zero_padding1d_4_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &activation_2_output, &conv1d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_1_layer, 24,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &add_1_chain,
  NULL, &activation_4_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_4_weights, &conv1d_4_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_4_layer, 22,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_4_chain,
  NULL, &add_1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


AI_STATIC_CONST ai_float zero_padding1d_3_value_data[] = { 0.0 };
AI_ARRAY_OBJ_DECLARE(
    zero_padding1d_3_value, AI_ARRAY_FORMAT_FLOAT,
    zero_padding1d_3_value_data, zero_padding1d_3_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  zero_padding1d_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_3_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  zero_padding1d_3_layer, 20,
  PAD_TYPE, 0x0, NULL,
  pad, forward_pad,
  &zero_padding1d_3_chain,
  NULL, &conv1d_4_layer, AI_STATIC, 
  .value = &zero_padding1d_3_value, 
  .mode = AI_PAD_CONSTANT, 
  .pads = AI_SHAPE_INIT(4, 4, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_3_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_3_layer, 18,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_3_chain,
  NULL, &zero_padding1d_3_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_3_weights, &conv1d_3_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_3_layer, 17,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_3_chain,
  NULL, &activation_3_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


AI_STATIC_CONST ai_float zero_padding1d_2_value_data[] = { 0.0 };
AI_ARRAY_OBJ_DECLARE(
    zero_padding1d_2_value, AI_ARRAY_FORMAT_FLOAT,
    zero_padding1d_2_value_data, zero_padding1d_2_value_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  zero_padding1d_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &zero_padding1d_2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  zero_padding1d_2_layer, 15,
  PAD_TYPE, 0x0, NULL,
  pad, forward_pad,
  &zero_padding1d_2_chain,
  NULL, &conv1d_3_layer, AI_STATIC, 
  .value = &zero_padding1d_2_value, 
  .mode = AI_PAD_CONSTANT, 
  .pads = AI_SHAPE_INIT(4, 4, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_2_layer, 14,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_2_chain,
  NULL, &zero_padding1d_2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &activation_output, &conv1d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_layer, 13,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &add_chain,
  NULL, &activation_2_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_2_weights, &conv1d_2_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_2_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_2_layer, 9,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &conv1d_2_chain,
  NULL, &add_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_1_layer, 7,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_1_chain,
  NULL, &conv1d_2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_1_weights, &conv1d_1_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_1_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_1_layer, 4,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &conv1d_1_chain,
  NULL, &activation_1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 0, 0, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  activation_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &activation_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  activation_layer, 3,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &activation_chain,
  NULL, &conv1d_1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &input_layer_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_weights, &conv1d_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_layer, 2,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &conv1d_chain,
  NULL, &activation_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 803472, 1, 1),
    803472, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 123648, 1, 1),
    123648, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_layer_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &dense_1_output),
  &conv1d_layer, 0xba4fe562, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 803472, 1, 1),
      803472, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 123648, 1, 1),
      123648, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &input_layer_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &dense_1_output),
  &conv1d_layer, 0xba4fe562, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_network_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    input_layer_output_array.data = AI_PTR(g_network_activations_map[0] + 62616);
    input_layer_output_array.data_start = AI_PTR(g_network_activations_map[0] + 62616);
    conv1d_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 77976);
    conv1d_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 77976);
    conv1d_output_array.data = AI_PTR(g_network_activations_map[0] + 78336);
    conv1d_output_array.data_start = AI_PTR(g_network_activations_map[0] + 78336);
    activation_output_array.data = AI_PTR(g_network_activations_map[0] + 78336);
    activation_output_array.data_start = AI_PTR(g_network_activations_map[0] + 78336);
    conv1d_1_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 77568);
    conv1d_1_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 77568);
    conv1d_1_output_array.data = AI_PTR(g_network_activations_map[0] + 44800);
    conv1d_1_output_array.data_start = AI_PTR(g_network_activations_map[0] + 44800);
    activation_1_output_array.data = AI_PTR(g_network_activations_map[0] + 44800);
    activation_1_output_array.data_start = AI_PTR(g_network_activations_map[0] + 44800);
    conv1d_2_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 111104);
    conv1d_2_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 111104);
    conv1d_2_output_array.data = AI_PTR(g_network_activations_map[0] + 43776);
    conv1d_2_output_array.data_start = AI_PTR(g_network_activations_map[0] + 43776);
    add_output_array.data = AI_PTR(g_network_activations_map[0] + 78336);
    add_output_array.data_start = AI_PTR(g_network_activations_map[0] + 78336);
    activation_2_output_array.data = AI_PTR(g_network_activations_map[0] + 78336);
    activation_2_output_array.data_start = AI_PTR(g_network_activations_map[0] + 78336);
    zero_padding1d_2_output_array.data = AI_PTR(g_network_activations_map[0] + 43776);
    zero_padding1d_2_output_array.data_start = AI_PTR(g_network_activations_map[0] + 43776);
    conv1d_3_output_array.data = AI_PTR(g_network_activations_map[0] + 43520);
    conv1d_3_output_array.data_start = AI_PTR(g_network_activations_map[0] + 43520);
    activation_3_output_array.data = AI_PTR(g_network_activations_map[0] + 43520);
    activation_3_output_array.data_start = AI_PTR(g_network_activations_map[0] + 43520);
    zero_padding1d_3_output_array.data = AI_PTR(g_network_activations_map[0] + 42496);
    zero_padding1d_3_output_array.data_start = AI_PTR(g_network_activations_map[0] + 42496);
    conv1d_4_output_array.data = AI_PTR(g_network_activations_map[0] + 42240);
    conv1d_4_output_array.data_start = AI_PTR(g_network_activations_map[0] + 42240);
    add_1_output_array.data = AI_PTR(g_network_activations_map[0] + 42240);
    add_1_output_array.data_start = AI_PTR(g_network_activations_map[0] + 42240);
    activation_4_output_array.data = AI_PTR(g_network_activations_map[0] + 79104);
    activation_4_output_array.data_start = AI_PTR(g_network_activations_map[0] + 79104);
    zero_padding1d_4_output_array.data = AI_PTR(g_network_activations_map[0] + 44288);
    zero_padding1d_4_output_array.data_start = AI_PTR(g_network_activations_map[0] + 44288);
    conv1d_5_output_array.data = AI_PTR(g_network_activations_map[0] + 44032);
    conv1d_5_output_array.data_start = AI_PTR(g_network_activations_map[0] + 44032);
    activation_5_output_array.data = AI_PTR(g_network_activations_map[0] + 44032);
    activation_5_output_array.data_start = AI_PTR(g_network_activations_map[0] + 44032);
    zero_padding1d_5_output_array.data = AI_PTR(g_network_activations_map[0] + 41984);
    zero_padding1d_5_output_array.data_start = AI_PTR(g_network_activations_map[0] + 41984);
    conv1d_6_output_array.data = AI_PTR(g_network_activations_map[0] + 41728);
    conv1d_6_output_array.data_start = AI_PTR(g_network_activations_map[0] + 41728);
    add_2_output_array.data = AI_PTR(g_network_activations_map[0] + 41728);
    add_2_output_array.data_start = AI_PTR(g_network_activations_map[0] + 41728);
    activation_6_output_array.data = AI_PTR(g_network_activations_map[0] + 41728);
    activation_6_output_array.data_start = AI_PTR(g_network_activations_map[0] + 41728);
    conv1d_9_scratch0_array.data = AI_PTR(g_network_activations_map[0] + 41472);
    conv1d_9_scratch0_array.data_start = AI_PTR(g_network_activations_map[0] + 41472);
    conv1d_9_output_array.data = AI_PTR(g_network_activations_map[0] + 74496);
    conv1d_9_output_array.data_start = AI_PTR(g_network_activations_map[0] + 74496);
    zero_padding1d_6_output_array.data = AI_PTR(g_network_activations_map[0] + 37632);
    zero_padding1d_6_output_array.data_start = AI_PTR(g_network_activations_map[0] + 37632);
    conv1d_7_output_array.data = AI_PTR(g_network_activations_map[0] + 24320);
    conv1d_7_output_array.data_start = AI_PTR(g_network_activations_map[0] + 24320);
    activation_7_output_array.data = AI_PTR(g_network_activations_map[0] + 24320);
    activation_7_output_array.data_start = AI_PTR(g_network_activations_map[0] + 24320);
    zero_padding1d_7_output_array.data = AI_PTR(g_network_activations_map[0] + 18176);
    zero_padding1d_7_output_array.data_start = AI_PTR(g_network_activations_map[0] + 18176);
    conv1d_8_output_array.data = AI_PTR(g_network_activations_map[0] + 17792);
    conv1d_8_output_array.data_start = AI_PTR(g_network_activations_map[0] + 17792);
    add_3_output_array.data = AI_PTR(g_network_activations_map[0] + 74496);
    add_3_output_array.data_start = AI_PTR(g_network_activations_map[0] + 74496);
    activation_8_output_array.data = AI_PTR(g_network_activations_map[0] + 74496);
    activation_8_output_array.data_start = AI_PTR(g_network_activations_map[0] + 74496);
    zero_padding1d_8_output_array.data = AI_PTR(g_network_activations_map[0] + 13056);
    zero_padding1d_8_output_array.data_start = AI_PTR(g_network_activations_map[0] + 13056);
    conv1d_10_output_array.data = AI_PTR(g_network_activations_map[0] + 12672);
    conv1d_10_output_array.data_start = AI_PTR(g_network_activations_map[0] + 12672);
    activation_9_output_array.data = AI_PTR(g_network_activations_map[0] + 12672);
    activation_9_output_array.data_start = AI_PTR(g_network_activations_map[0] + 12672);
    zero_padding1d_9_output_array.data = AI_PTR(g_network_activations_map[0] + 384);
    zero_padding1d_9_output_array.data_start = AI_PTR(g_network_activations_map[0] + 384);
    conv1d_11_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    conv1d_11_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    add_4_output_array.data = AI_PTR(g_network_activations_map[0] + 74496);
    add_4_output_array.data_start = AI_PTR(g_network_activations_map[0] + 74496);
    activation_10_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    activation_10_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    global_average_pooling1d_pool_output_array.data = AI_PTR(g_network_activations_map[0] + 49152);
    global_average_pooling1d_pool_output_array.data_start = AI_PTR(g_network_activations_map[0] + 49152);
    dense_dense_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    dense_dense_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    dense_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    dense_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    dense_1_dense_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    dense_1_dense_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    dense_1_output_array.data = AI_PTR(g_network_activations_map[0] + 16);
    dense_1_output_array.data_start = AI_PTR(g_network_activations_map[0] + 16);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_network_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    conv1d_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_weights_array.data = AI_PTR(g_network_weights_map[0] + 0);
    conv1d_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    conv1d_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_bias_array.data = AI_PTR(g_network_weights_map[0] + 23040);
    conv1d_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 23040);
    conv1d_1_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_1_weights_array.data = AI_PTR(g_network_weights_map[0] + 23296);
    conv1d_1_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 23296);
    conv1d_1_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_1_bias_array.data = AI_PTR(g_network_weights_map[0] + 72448);
    conv1d_1_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 72448);
    conv1d_2_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_2_weights_array.data = AI_PTR(g_network_weights_map[0] + 72704);
    conv1d_2_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 72704);
    conv1d_2_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_2_bias_array.data = AI_PTR(g_network_weights_map[0] + 121856);
    conv1d_2_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 121856);
    conv1d_3_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_3_weights_array.data = AI_PTR(g_network_weights_map[0] + 122112);
    conv1d_3_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 122112);
    conv1d_3_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_3_bias_array.data = AI_PTR(g_network_weights_map[0] + 171264);
    conv1d_3_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 171264);
    conv1d_4_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_4_weights_array.data = AI_PTR(g_network_weights_map[0] + 171520);
    conv1d_4_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 171520);
    conv1d_4_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_4_bias_array.data = AI_PTR(g_network_weights_map[0] + 220672);
    conv1d_4_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 220672);
    conv1d_5_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_5_weights_array.data = AI_PTR(g_network_weights_map[0] + 220928);
    conv1d_5_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 220928);
    conv1d_5_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_5_bias_array.data = AI_PTR(g_network_weights_map[0] + 270080);
    conv1d_5_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 270080);
    conv1d_6_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_6_weights_array.data = AI_PTR(g_network_weights_map[0] + 270336);
    conv1d_6_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 270336);
    conv1d_6_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_6_bias_array.data = AI_PTR(g_network_weights_map[0] + 319488);
    conv1d_6_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 319488);
    conv1d_9_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_9_weights_array.data = AI_PTR(g_network_weights_map[0] + 319744);
    conv1d_9_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 319744);
    conv1d_9_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_9_bias_array.data = AI_PTR(g_network_weights_map[0] + 344320);
    conv1d_9_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 344320);
    conv1d_7_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_7_weights_array.data = AI_PTR(g_network_weights_map[0] + 344704);
    conv1d_7_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 344704);
    conv1d_7_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_7_bias_array.data = AI_PTR(g_network_weights_map[0] + 418432);
    conv1d_7_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 418432);
    conv1d_8_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_8_weights_array.data = AI_PTR(g_network_weights_map[0] + 418816);
    conv1d_8_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 418816);
    conv1d_8_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_8_bias_array.data = AI_PTR(g_network_weights_map[0] + 529408);
    conv1d_8_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 529408);
    conv1d_10_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_10_weights_array.data = AI_PTR(g_network_weights_map[0] + 529792);
    conv1d_10_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 529792);
    conv1d_10_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_10_bias_array.data = AI_PTR(g_network_weights_map[0] + 640384);
    conv1d_10_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 640384);
    conv1d_11_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_11_weights_array.data = AI_PTR(g_network_weights_map[0] + 640768);
    conv1d_11_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 640768);
    conv1d_11_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_11_bias_array.data = AI_PTR(g_network_weights_map[0] + 751360);
    conv1d_11_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 751360);
    dense_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_dense_weights_array.data = AI_PTR(g_network_weights_map[0] + 751744);
    dense_dense_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 751744);
    dense_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_dense_bias_array.data = AI_PTR(g_network_weights_map[0] + 800896);
    dense_dense_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 800896);
    dense_1_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_1_dense_weights_array.data = AI_PTR(g_network_weights_map[0] + 801408);
    dense_1_dense_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 801408);
    dense_1_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_1_dense_bias_array.data = AI_PTR(g_network_weights_map[0] + 803456);
    dense_1_dense_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 803456);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 24119008,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xba4fe562,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_network_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 24119008,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0xba4fe562,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_network_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_network_create(network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_network_data_params_get(&params) != true) {
    err = ai_network_get_error(*network);
    return err;
  }
#if defined(AI_NETWORK_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_NETWORK_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_network_init(*network, &params) != true) {
    err = ai_network_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_network_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_network_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= network_configure_weights(net_ctx, params);
  ok &= network_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

