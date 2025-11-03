/**
  ******************************************************************************
  * @file    imu_model.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-11-01T20:42:27+0900
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


#include "imu_model.h"
#include "imu_model_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_imu_model
 
#undef AI_IMU_MODEL_MODEL_SIGNATURE
#define AI_IMU_MODEL_MODEL_SIGNATURE     "0xd8800482decc9e061da6add171f3baef"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2025-11-01T20:42:27+0900"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_IMU_MODEL_N_BATCHES
#define AI_IMU_MODEL_N_BATCHES         (1)

static ai_ptr g_imu_model_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_imu_model_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  x_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 768, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  stem_conv_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  stem_relu_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_conv1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_relu1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_conv2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_add_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_relu2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_conv1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_relu1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_conv2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_add_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_relu2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_conv1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_relu1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_conv2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_add_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_relu2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_conv1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_relu1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_conv2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_add_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_relu2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_conv1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_relu1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_conv2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_add_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_relu2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  gap_pool_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  head_fc_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  head_fc_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  pred_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  pred_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 2, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  stem_conv_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 864, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  stem_conv_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#35 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_conv1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#36 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_conv1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#37 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_conv2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#38 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_conv2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#39 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_conv1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#40 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_conv1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#41 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_conv2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#42 */
AI_ARRAY_OBJ_DECLARE(
  tcn2_conv2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#43 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_conv1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#44 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_conv1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#45 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_conv2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#46 */
AI_ARRAY_OBJ_DECLARE(
  tcn3_conv2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#47 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_conv1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#48 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_conv1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#49 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_conv2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#50 */
AI_ARRAY_OBJ_DECLARE(
  tcn4_conv2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#51 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_conv1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#52 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_conv1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#53 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_conv2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 11520, AI_STATIC)

/* Array#54 */
AI_ARRAY_OBJ_DECLARE(
  tcn5_conv2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#55 */
AI_ARRAY_OBJ_DECLARE(
  head_fc_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 4608, AI_STATIC)

/* Array#56 */
AI_ARRAY_OBJ_DECLARE(
  head_fc_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 96, AI_STATIC)

/* Array#57 */
AI_ARRAY_OBJ_DECLARE(
  pred_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 192, AI_STATIC)

/* Array#58 */
AI_ARRAY_OBJ_DECLARE(
  pred_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)

/* Array#59 */
AI_ARRAY_OBJ_DECLARE(
  stem_conv_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 18, AI_STATIC)

/* Array#60 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_conv1_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#61 */
AI_ARRAY_OBJ_DECLARE(
  tcn1_conv2_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  gap_pool_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &gap_pool_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  head_fc_dense_bias, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &head_fc_dense_bias_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  head_fc_dense_output, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &head_fc_dense_output_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  head_fc_dense_weights, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 48, 96, 1, 1), AI_STRIDE_INIT(4, 4, 192, 18432, 18432),
  1, &head_fc_dense_weights_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  head_fc_output, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 1, 96, 1, 1), AI_STRIDE_INIT(4, 4, 4, 384, 384),
  1, &head_fc_output_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  pred_dense_bias, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &pred_dense_bias_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  pred_dense_output, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &pred_dense_output_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  pred_dense_weights, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 96, 2, 1, 1), AI_STRIDE_INIT(4, 4, 384, 768, 768),
  1, &pred_dense_weights_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  pred_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &pred_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  stem_conv_bias, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &stem_conv_bias_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  stem_conv_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &stem_conv_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  stem_conv_scratch0, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 3), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &stem_conv_scratch0_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  stem_conv_weights, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 6, 1, 3, 48), AI_STRIDE_INIT(4, 4, 24, 1152, 1152),
  1, &stem_conv_weights_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  stem_relu_output, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &stem_relu_output_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_add_output, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_add_output_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_conv1_bias, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_conv1_bias_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_conv1_output, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_conv1_output_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_conv1_scratch0, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 5), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_conv1_scratch0_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_conv1_weights, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn1_conv1_weights_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_conv2_bias, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_conv2_bias_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_conv2_output, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_conv2_output_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_conv2_scratch0, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 5), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_conv2_scratch0_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_conv2_weights, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn1_conv2_weights_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_relu1_output, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_relu1_output_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  tcn1_relu2_output, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn1_relu2_output_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_add_output, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn2_add_output_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_conv1_bias, AI_STATIC,
  26, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn2_conv1_bias_array, NULL)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_conv1_output, AI_STATIC,
  27, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn2_conv1_output_array, NULL)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_conv1_weights, AI_STATIC,
  28, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn2_conv1_weights_array, NULL)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_conv2_bias, AI_STATIC,
  29, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn2_conv2_bias_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_conv2_output, AI_STATIC,
  30, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn2_conv2_output_array, NULL)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_conv2_weights, AI_STATIC,
  31, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn2_conv2_weights_array, NULL)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_relu1_output, AI_STATIC,
  32, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn2_relu1_output_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  tcn2_relu2_output, AI_STATIC,
  33, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn2_relu2_output_array, NULL)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_add_output, AI_STATIC,
  34, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn3_add_output_array, NULL)

/* Tensor #35 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_conv1_bias, AI_STATIC,
  35, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn3_conv1_bias_array, NULL)

/* Tensor #36 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_conv1_output, AI_STATIC,
  36, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn3_conv1_output_array, NULL)

/* Tensor #37 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_conv1_weights, AI_STATIC,
  37, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn3_conv1_weights_array, NULL)

/* Tensor #38 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_conv2_bias, AI_STATIC,
  38, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn3_conv2_bias_array, NULL)

/* Tensor #39 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_conv2_output, AI_STATIC,
  39, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn3_conv2_output_array, NULL)

/* Tensor #40 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_conv2_weights, AI_STATIC,
  40, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn3_conv2_weights_array, NULL)

/* Tensor #41 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_relu1_output, AI_STATIC,
  41, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn3_relu1_output_array, NULL)

/* Tensor #42 */
AI_TENSOR_OBJ_DECLARE(
  tcn3_relu2_output, AI_STATIC,
  42, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn3_relu2_output_array, NULL)

/* Tensor #43 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_add_output, AI_STATIC,
  43, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn4_add_output_array, NULL)

/* Tensor #44 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_conv1_bias, AI_STATIC,
  44, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn4_conv1_bias_array, NULL)

/* Tensor #45 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_conv1_output, AI_STATIC,
  45, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn4_conv1_output_array, NULL)

/* Tensor #46 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_conv1_weights, AI_STATIC,
  46, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn4_conv1_weights_array, NULL)

/* Tensor #47 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_conv2_bias, AI_STATIC,
  47, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn4_conv2_bias_array, NULL)

/* Tensor #48 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_conv2_output, AI_STATIC,
  48, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn4_conv2_output_array, NULL)

/* Tensor #49 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_conv2_weights, AI_STATIC,
  49, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn4_conv2_weights_array, NULL)

/* Tensor #50 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_relu1_output, AI_STATIC,
  50, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn4_relu1_output_array, NULL)

/* Tensor #51 */
AI_TENSOR_OBJ_DECLARE(
  tcn4_relu2_output, AI_STATIC,
  51, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn4_relu2_output_array, NULL)

/* Tensor #52 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_add_output, AI_STATIC,
  52, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn5_add_output_array, NULL)

/* Tensor #53 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_conv1_bias, AI_STATIC,
  53, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn5_conv1_bias_array, NULL)

/* Tensor #54 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_conv1_output, AI_STATIC,
  54, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn5_conv1_output_array, NULL)

/* Tensor #55 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_conv1_weights, AI_STATIC,
  55, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn5_conv1_weights_array, NULL)

/* Tensor #56 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_conv2_bias, AI_STATIC,
  56, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn5_conv2_bias_array, NULL)

/* Tensor #57 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_conv2_output, AI_STATIC,
  57, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn5_conv2_output_array, NULL)

/* Tensor #58 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_conv2_weights, AI_STATIC,
  58, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 5, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn5_conv2_weights_array, NULL)

/* Tensor #59 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_relu1_output, AI_STATIC,
  59, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn5_relu1_output_array, NULL)

/* Tensor #60 */
AI_TENSOR_OBJ_DECLARE(
  tcn5_relu2_output, AI_STATIC,
  60, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn5_relu2_output_array, NULL)

/* Tensor #61 */
AI_TENSOR_OBJ_DECLARE(
  x_output, AI_STATIC,
  61, 0x0,
  AI_SHAPE_INIT(4, 1, 6, 1, 128), AI_STRIDE_INIT(4, 4, 4, 24, 24),
  1, &x_output_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  pred_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &pred_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &pred_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  pred_layer, 46,
  SM_TYPE, 0x0, NULL,
  sm, forward_sm,
  &pred_chain,
  NULL, &pred_layer, AI_STATIC, 
  .nl_params = NULL, 
  .axis = AI_SHAPE_CHANNEL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  pred_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &head_fc_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &pred_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &pred_dense_weights, &pred_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  pred_dense_layer, 46,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &pred_dense_chain,
  NULL, &pred_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  head_fc_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &head_fc_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &head_fc_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  head_fc_layer, 45,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &head_fc_chain,
  NULL, &pred_dense_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  head_fc_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &gap_pool_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &head_fc_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &head_fc_dense_weights, &head_fc_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  head_fc_dense_layer, 45,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &head_fc_dense_chain,
  NULL, &head_fc_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  gap_pool_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_relu2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &gap_pool_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  gap_pool_layer, 44,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &gap_pool_chain,
  NULL, &head_fc_dense_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(1, 128), 
  .pool_stride = AI_SHAPE_2D_INIT(1, 128), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn5_relu2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_add_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_relu2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn5_relu2_layer, 43,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn5_relu2_chain,
  NULL, &gap_pool_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn5_add_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn4_relu2_output, &tcn5_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_add_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn5_add_layer, 42,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &tcn5_add_chain,
  NULL, &tcn5_relu2_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn5_conv2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_relu1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn5_conv2_weights, &tcn5_conv2_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn5_conv2_layer, 41,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn5_conv2_chain,
  NULL, &tcn5_add_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 16), 
  .filter_pad = AI_SHAPE_INIT(4, 32, 0, 32, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn5_relu1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_relu1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn5_relu1_layer, 38,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn5_relu1_chain,
  NULL, &tcn5_conv2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn5_conv1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_relu2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn5_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn5_conv1_weights, &tcn5_conv1_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn5_conv1_layer, 37,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn5_conv1_chain,
  NULL, &tcn5_relu1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 16), 
  .filter_pad = AI_SHAPE_INIT(4, 32, 0, 32, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn4_relu2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_add_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_relu2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn4_relu2_layer, 35,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn4_relu2_chain,
  NULL, &tcn5_conv1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn4_add_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn3_relu2_output, &tcn4_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_add_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn4_add_layer, 34,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &tcn4_add_chain,
  NULL, &tcn4_relu2_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn4_conv2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_relu1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn4_conv2_weights, &tcn4_conv2_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn4_conv2_layer, 33,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn4_conv2_chain,
  NULL, &tcn4_add_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 8), 
  .filter_pad = AI_SHAPE_INIT(4, 16, 0, 16, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn4_relu1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_relu1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn4_relu1_layer, 30,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn4_relu1_chain,
  NULL, &tcn4_conv2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn4_conv1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_relu2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn4_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn4_conv1_weights, &tcn4_conv1_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn4_conv1_layer, 29,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn4_conv1_chain,
  NULL, &tcn4_relu1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 8), 
  .filter_pad = AI_SHAPE_INIT(4, 16, 0, 16, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn3_relu2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_add_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_relu2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn3_relu2_layer, 27,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn3_relu2_chain,
  NULL, &tcn4_conv1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn3_add_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn2_relu2_output, &tcn3_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_add_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn3_add_layer, 26,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &tcn3_add_chain,
  NULL, &tcn3_relu2_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn3_conv2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_relu1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn3_conv2_weights, &tcn3_conv2_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn3_conv2_layer, 25,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn3_conv2_chain,
  NULL, &tcn3_add_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 4), 
  .filter_pad = AI_SHAPE_INIT(4, 8, 0, 8, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn3_relu1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_relu1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn3_relu1_layer, 22,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn3_relu1_chain,
  NULL, &tcn3_conv2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn3_conv1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_relu2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn3_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn3_conv1_weights, &tcn3_conv1_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn3_conv1_layer, 21,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn3_conv1_chain,
  NULL, &tcn3_relu1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 4), 
  .filter_pad = AI_SHAPE_INIT(4, 8, 0, 8, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn2_relu2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_add_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_relu2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn2_relu2_layer, 19,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn2_relu2_chain,
  NULL, &tcn3_conv1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn2_add_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn1_relu2_output, &tcn2_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_add_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn2_add_layer, 18,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &tcn2_add_chain,
  NULL, &tcn2_relu2_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn2_conv2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_relu1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn2_conv2_weights, &tcn2_conv2_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn2_conv2_layer, 17,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn2_conv2_chain,
  NULL, &tcn2_add_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_INIT(4, 4, 0, 4, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn2_relu1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_relu1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn2_relu1_layer, 14,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn2_relu1_chain,
  NULL, &tcn2_conv2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn2_conv1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_relu2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn2_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn2_conv1_weights, &tcn2_conv1_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn2_conv1_layer, 13,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn2_conv1_chain,
  NULL, &tcn2_relu1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_INIT(4, 4, 0, 4, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn1_relu2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_add_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_relu2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn1_relu2_layer, 11,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn1_relu2_chain,
  NULL, &tcn2_conv1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn1_add_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &stem_relu_output, &tcn1_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_add_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn1_add_layer, 10,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &tcn1_add_chain,
  NULL, &tcn1_relu2_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn1_conv2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_relu1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_conv2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn1_conv2_weights, &tcn1_conv2_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn1_conv2_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  tcn1_conv2_layer, 9,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &tcn1_conv2_chain,
  NULL, &tcn1_add_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 0, 2, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn1_relu1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_relu1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn1_relu1_layer, 6,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn1_relu1_chain,
  NULL, &tcn1_conv2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn1_conv1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &stem_relu_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn1_conv1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn1_conv1_weights, &tcn1_conv1_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn1_conv1_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  tcn1_conv1_layer, 5,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &tcn1_conv1_chain,
  NULL, &tcn1_relu1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 0, 2, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  stem_relu_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &stem_conv_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &stem_relu_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  stem_relu_layer, 3,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &stem_relu_chain,
  NULL, &tcn1_conv1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  stem_conv_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &x_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &stem_conv_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &stem_conv_weights, &stem_conv_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &stem_conv_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  stem_conv_layer, 2,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &stem_conv_chain,
  NULL, &stem_relu_layer, AI_STATIC, 
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
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 485960, 1, 1),
    485960, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 55680, 1, 1),
    55680, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_IN_NUM, &x_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_OUT_NUM, &pred_output),
  &stem_conv_layer, 0x1d4ed0b1, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 485960, 1, 1),
      485960, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 55680, 1, 1),
      55680, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_IN_NUM, &x_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_OUT_NUM, &pred_output),
  &stem_conv_layer, 0x1d4ed0b1, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool imu_model_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_imu_model_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    x_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 28776);
    x_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 28776);
    stem_conv_scratch0_array.data = AI_PTR(g_imu_model_activations_map[0] + 31848);
    stem_conv_scratch0_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31848);
    stem_conv_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    stem_conv_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    stem_relu_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    stem_relu_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn1_conv1_scratch0_array.data = AI_PTR(g_imu_model_activations_map[0] + 30144);
    tcn1_conv1_scratch0_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 30144);
    tcn1_conv1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 5568);
    tcn1_conv1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 5568);
    tcn1_relu1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 5568);
    tcn1_relu1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 5568);
    tcn1_conv2_scratch0_array.data = AI_PTR(g_imu_model_activations_map[0] + 30144);
    tcn1_conv2_scratch0_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 30144);
    tcn1_conv2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 4800);
    tcn1_conv2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 4800);
    tcn1_add_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn1_add_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn1_relu2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn1_relu2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn2_conv1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn2_conv1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn2_relu1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn2_relu1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn2_conv2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 5376);
    tcn2_conv2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 5376);
    tcn2_add_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn2_add_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn2_relu2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn2_relu2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn3_conv1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn3_conv1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn3_relu1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn3_relu1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn3_conv2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 4608);
    tcn3_conv2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 4608);
    tcn3_add_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn3_add_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn3_relu2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn3_relu2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn4_conv1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn4_conv1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn4_relu1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn4_relu1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn4_conv2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 3072);
    tcn4_conv2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 3072);
    tcn4_add_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 3072);
    tcn4_add_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 3072);
    tcn4_relu2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn4_relu2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn5_conv1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn5_conv1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn5_relu1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn5_relu1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    tcn5_conv2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn5_conv2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn5_add_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn5_add_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31104);
    tcn5_relu2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn5_relu2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    gap_pool_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 24576);
    gap_pool_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 24576);
    head_fc_dense_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    head_fc_dense_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    head_fc_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 384);
    head_fc_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 384);
    pred_dense_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    pred_dense_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    pred_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 8);
    pred_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 8);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool imu_model_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_imu_model_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    stem_conv_weights_array.format |= AI_FMT_FLAG_CONST;
    stem_conv_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 0);
    stem_conv_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 0);
    stem_conv_bias_array.format |= AI_FMT_FLAG_CONST;
    stem_conv_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 3456);
    stem_conv_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 3456);
    tcn1_conv1_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn1_conv1_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 3648);
    tcn1_conv1_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 3648);
    tcn1_conv1_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn1_conv1_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 49728);
    tcn1_conv1_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 49728);
    tcn1_conv2_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn1_conv2_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 49920);
    tcn1_conv2_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 49920);
    tcn1_conv2_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn1_conv2_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 96000);
    tcn1_conv2_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 96000);
    tcn2_conv1_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn2_conv1_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 96192);
    tcn2_conv1_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 96192);
    tcn2_conv1_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn2_conv1_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 142272);
    tcn2_conv1_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 142272);
    tcn2_conv2_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn2_conv2_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 142464);
    tcn2_conv2_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 142464);
    tcn2_conv2_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn2_conv2_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 188544);
    tcn2_conv2_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 188544);
    tcn3_conv1_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn3_conv1_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 188736);
    tcn3_conv1_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 188736);
    tcn3_conv1_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn3_conv1_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 234816);
    tcn3_conv1_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 234816);
    tcn3_conv2_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn3_conv2_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 235008);
    tcn3_conv2_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 235008);
    tcn3_conv2_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn3_conv2_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 281088);
    tcn3_conv2_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 281088);
    tcn4_conv1_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn4_conv1_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 281280);
    tcn4_conv1_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 281280);
    tcn4_conv1_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn4_conv1_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 327360);
    tcn4_conv1_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 327360);
    tcn4_conv2_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn4_conv2_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 327552);
    tcn4_conv2_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 327552);
    tcn4_conv2_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn4_conv2_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 373632);
    tcn4_conv2_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 373632);
    tcn5_conv1_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn5_conv1_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 373824);
    tcn5_conv1_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 373824);
    tcn5_conv1_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn5_conv1_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 419904);
    tcn5_conv1_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 419904);
    tcn5_conv2_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn5_conv2_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 420096);
    tcn5_conv2_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 420096);
    tcn5_conv2_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn5_conv2_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 466176);
    tcn5_conv2_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 466176);
    head_fc_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    head_fc_dense_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 466368);
    head_fc_dense_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 466368);
    head_fc_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    head_fc_dense_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 484800);
    head_fc_dense_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 484800);
    pred_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    pred_dense_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 485184);
    pred_dense_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 485184);
    pred_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    pred_dense_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 485952);
    pred_dense_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 485952);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_imu_model_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_IMU_MODEL_MODEL_NAME,
      .model_signature   = AI_IMU_MODEL_MODEL_SIGNATURE,
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
      
      .n_macc            = 14966192,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x1d4ed0b1,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_imu_model_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_IMU_MODEL_MODEL_NAME,
      .model_signature   = AI_IMU_MODEL_MODEL_SIGNATURE,
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
      
      .n_macc            = 14966192,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x1d4ed0b1,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_imu_model_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_imu_model_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_imu_model_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_imu_model_create(network, AI_IMU_MODEL_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_imu_model_data_params_get(&params) != true) {
    err = ai_imu_model_get_error(*network);
    return err;
  }
#if defined(AI_IMU_MODEL_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_IMU_MODEL_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_imu_model_init(*network, &params) != true) {
    err = ai_imu_model_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_imu_model_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_imu_model_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_imu_model_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_imu_model_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= imu_model_configure_weights(net_ctx, params);
  ok &= imu_model_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_imu_model_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_imu_model_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_IMU_MODEL_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

