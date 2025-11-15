/**
  ******************************************************************************
  * @file    imu_model.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-11-15T23:03:38+0900
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
#define AI_IMU_MODEL_MODEL_SIGNATURE     "0x869ce66666484bba7b3d65229baf69b5"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2025-11-15T23:03:38+0900"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_IMU_MODEL_N_BATCHES
#define AI_IMU_MODEL_N_BATCHES         (1)

static ai_ptr g_imu_model_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_imu_model_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  imu_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 7680, AI_STATIC)

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
  tcn_block1_conv2_d1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block1_add_d1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block1_out_relu_d1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block2_conv2_d2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block2_add_d2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block2_out_relu_d2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block3_conv2_d4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block3_add_d4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block3_out_relu_d4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6144, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  gap_pool_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  pred_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 5, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  pred_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 5, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  stem_conv_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8640, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  stem_conv_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block1_conv2_d1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6912, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block1_conv2_d1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block2_conv2_d2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6912, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block2_conv2_d2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block3_conv2_d4_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 6912, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block3_conv2_d4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  pred_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 240, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  pred_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 5, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  stem_conv_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 180, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  tcn_block1_conv2_d1_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 144, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  gap_pool_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &gap_pool_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  imu_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 128), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &imu_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  pred_dense_bias, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 1, 1), AI_STRIDE_INIT(4, 4, 4, 20, 20),
  1, &pred_dense_bias_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  pred_dense_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 1, 1), AI_STRIDE_INIT(4, 4, 4, 20, 20),
  1, &pred_dense_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  pred_dense_weights, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 48, 5, 1, 1), AI_STRIDE_INIT(4, 4, 192, 960, 960),
  1, &pred_dense_weights_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  pred_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 5, 1, 1), AI_STRIDE_INIT(4, 4, 4, 20, 20),
  1, &pred_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  stem_conv_bias, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &stem_conv_bias_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  stem_conv_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &stem_conv_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  stem_conv_scratch0, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 60, 1, 3), AI_STRIDE_INIT(4, 4, 4, 240, 240),
  1, &stem_conv_scratch0_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  stem_conv_weights, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 60, 1, 3, 48), AI_STRIDE_INIT(4, 4, 240, 11520, 11520),
  1, &stem_conv_weights_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  stem_relu_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &stem_relu_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block1_add_d1_output, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block1_add_d1_output_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block1_conv2_d1_bias, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block1_conv2_d1_bias_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block1_conv2_d1_output, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block1_conv2_d1_output_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block1_conv2_d1_scratch0, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 3), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block1_conv2_d1_scratch0_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block1_conv2_d1_weights, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 3, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn_block1_conv2_d1_weights_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block1_out_relu_d1_output, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block1_out_relu_d1_output_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block2_add_d2_output, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block2_add_d2_output_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block2_conv2_d2_bias, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block2_conv2_d2_bias_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block2_conv2_d2_output, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block2_conv2_d2_output_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block2_conv2_d2_weights, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 3, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn_block2_conv2_d2_weights_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block2_out_relu_d2_output, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block2_out_relu_d2_output_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block3_add_d4_output, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block3_add_d4_output_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block3_conv2_d4_bias, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 1), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block3_conv2_d4_bias_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block3_conv2_d4_output, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block3_conv2_d4_output_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block3_conv2_d4_weights, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 48, 1, 3, 48), AI_STRIDE_INIT(4, 4, 192, 9216, 9216),
  1, &tcn_block3_conv2_d4_weights_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  tcn_block3_out_relu_d4_output, AI_STATIC,
  26, 0x0,
  AI_SHAPE_INIT(4, 1, 48, 1, 128), AI_STRIDE_INIT(4, 4, 4, 192, 192),
  1, &tcn_block3_out_relu_d4_output_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  pred_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &pred_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &pred_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  pred_layer, 18,
  SM_TYPE, 0x0, NULL,
  sm, forward_sm,
  &pred_chain,
  NULL, &pred_layer, AI_STATIC, 
  .nl_params = NULL, 
  .axis = AI_SHAPE_CHANNEL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  pred_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &gap_pool_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &pred_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &pred_dense_weights, &pred_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  pred_dense_layer, 18,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &pred_dense_chain,
  NULL, &pred_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  gap_pool_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block3_out_relu_d4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &gap_pool_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  gap_pool_layer, 17,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &gap_pool_chain,
  NULL, &pred_dense_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(1, 128), 
  .pool_stride = AI_SHAPE_2D_INIT(1, 128), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block3_out_relu_d4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block3_add_d4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block3_out_relu_d4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn_block3_out_relu_d4_layer, 16,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn_block3_out_relu_d4_chain,
  NULL, &gap_pool_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block3_add_d4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn_block2_out_relu_d2_output, &tcn_block3_conv2_d4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block3_add_d4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn_block3_add_d4_layer, 15,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &tcn_block3_add_d4_chain,
  NULL, &tcn_block3_out_relu_d4_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block3_conv2_d4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block2_out_relu_d2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block3_conv2_d4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn_block3_conv2_d4_weights, &tcn_block3_conv2_d4_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn_block3_conv2_d4_layer, 14,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn_block3_conv2_d4_chain,
  NULL, &tcn_block3_add_d4_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 4), 
  .filter_pad = AI_SHAPE_INIT(4, 4, 0, 4, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block2_out_relu_d2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block2_add_d2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block2_out_relu_d2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn_block2_out_relu_d2_layer, 12,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn_block2_out_relu_d2_chain,
  NULL, &tcn_block3_conv2_d4_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block2_add_d2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn_block1_out_relu_d1_output, &tcn_block2_conv2_d2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block2_add_d2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn_block2_add_d2_layer, 11,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &tcn_block2_add_d2_chain,
  NULL, &tcn_block2_out_relu_d2_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block2_conv2_d2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block1_out_relu_d1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block2_conv2_d2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn_block2_conv2_d2_weights, &tcn_block2_conv2_d2_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn_block2_conv2_d2_layer, 10,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &tcn_block2_conv2_d2_chain,
  NULL, &tcn_block2_add_d2_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 0, 2, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block1_out_relu_d1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block1_add_d1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block1_out_relu_d1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn_block1_out_relu_d1_layer, 8,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &tcn_block1_out_relu_d1_chain,
  NULL, &tcn_block2_conv2_d2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block1_add_d1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &stem_relu_output, &tcn_block1_conv2_d1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block1_add_d1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  tcn_block1_add_d1_layer, 7,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &tcn_block1_add_d1_chain,
  NULL, &tcn_block1_out_relu_d1_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  tcn_block1_conv2_d1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &stem_relu_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &tcn_block1_conv2_d1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &tcn_block1_conv2_d1_weights, &tcn_block1_conv2_d1_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &tcn_block1_conv2_d1_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  tcn_block1_conv2_d1_layer, 6,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &tcn_block1_conv2_d1_chain,
  NULL, &tcn_block1_add_d1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 0), 
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
  NULL, &tcn_block1_conv2_d1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  stem_conv_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &imu_output),
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
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 119252, 1, 1),
    119252, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 49728, 1, 1),
    49728, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_IN_NUM, &imu_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_OUT_NUM, &pred_output),
  &stem_conv_layer, 0x94ffeaca, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 119252, 1, 1),
      119252, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 49728, 1, 1),
      49728, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_IN_NUM, &imu_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_OUT_NUM, &pred_output),
  &stem_conv_layer, 0x94ffeaca, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool imu_model_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_imu_model_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    imu_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 576);
    imu_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 576);
    stem_conv_scratch0_array.data = AI_PTR(g_imu_model_activations_map[0] + 31296);
    stem_conv_scratch0_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 31296);
    stem_conv_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    stem_conv_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    stem_relu_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    stem_relu_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block1_conv2_d1_scratch0_array.data = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block1_conv2_d1_scratch0_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block1_conv2_d1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 25152);
    tcn_block1_conv2_d1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 25152);
    tcn_block1_add_d1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block1_add_d1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block1_out_relu_d1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block1_out_relu_d1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block2_conv2_d2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block2_conv2_d2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block2_add_d2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block2_add_d2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block2_out_relu_d2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block2_out_relu_d2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block3_conv2_d4_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block3_conv2_d4_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block3_add_d4_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block3_add_d4_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    tcn_block3_out_relu_d4_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 24576);
    tcn_block3_out_relu_d4_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 24576);
    gap_pool_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    gap_pool_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    pred_dense_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 192);
    pred_dense_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 192);
    pred_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    pred_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
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
    stem_conv_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 34560);
    stem_conv_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 34560);
    tcn_block1_conv2_d1_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn_block1_conv2_d1_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 34752);
    tcn_block1_conv2_d1_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 34752);
    tcn_block1_conv2_d1_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn_block1_conv2_d1_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 62400);
    tcn_block1_conv2_d1_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 62400);
    tcn_block2_conv2_d2_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn_block2_conv2_d2_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 62592);
    tcn_block2_conv2_d2_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 62592);
    tcn_block2_conv2_d2_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn_block2_conv2_d2_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 90240);
    tcn_block2_conv2_d2_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 90240);
    tcn_block3_conv2_d4_weights_array.format |= AI_FMT_FLAG_CONST;
    tcn_block3_conv2_d4_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 90432);
    tcn_block3_conv2_d4_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 90432);
    tcn_block3_conv2_d4_bias_array.format |= AI_FMT_FLAG_CONST;
    tcn_block3_conv2_d4_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 118080);
    tcn_block3_conv2_d4_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 118080);
    pred_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    pred_dense_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 118272);
    pred_dense_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 118272);
    pred_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    pred_dense_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 119232);
    pred_dense_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 119232);
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
      
      .n_macc            = 3809792,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x94ffeaca,
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
      
      .n_macc            = 3809792,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x94ffeaca,
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

