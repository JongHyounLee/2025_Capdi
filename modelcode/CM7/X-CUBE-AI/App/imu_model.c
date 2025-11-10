/**
  ******************************************************************************
  * @file    imu_model.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-11-05T23:13:59+0900
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
#define AI_IMU_MODEL_MODEL_SIGNATURE     "0x42b1d5b3f5e55d5aeb5ad9e147e9bb37"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "2025-11-05T23:13:59+0900"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_IMU_MODEL_N_BATCHES
#define AI_IMU_MODEL_N_BATCHES         (1)

static ai_ptr g_imu_model_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_imu_model_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  imu_input_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 3000, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  re_lu_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  re_lu_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  add_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  re_lu_2_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  re_lu_3_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  add_1_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  re_lu_4_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 1600, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  global_average_pooling1d_pool_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  dense_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#16 */
AI_ARRAY_OBJ_DECLARE(
  cls_dense_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)

/* Array#17 */
AI_ARRAY_OBJ_DECLARE(
  cls_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 2, AI_STATIC)

/* Array#18 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2400, AI_STATIC)

/* Array#19 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#20 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#21 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#22 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_2_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#23 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_2_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#24 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_3_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#25 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_3_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#26 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_4_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 768, AI_STATIC)

/* Array#27 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_4_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#28 */
AI_ARRAY_OBJ_DECLARE(
  dense_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#29 */
AI_ARRAY_OBJ_DECLARE(
  dense_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 16, AI_STATIC)

/* Array#30 */
AI_ARRAY_OBJ_DECLARE(
  cls_dense_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32, AI_STATIC)

/* Array#31 */
AI_ARRAY_OBJ_DECLARE(
  cls_dense_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 2, AI_STATIC)

/* Array#32 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 150, AI_STATIC)

/* Array#33 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_1_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/* Array#34 */
AI_ARRAY_OBJ_DECLARE(
  conv1d_2_scratch0_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 48, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  add_1_output, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &add_1_output_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  add_output, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &add_output_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  cls_dense_bias, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &cls_dense_bias_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  cls_dense_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &cls_dense_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  cls_dense_weights, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 16, 2, 1, 1), AI_STRIDE_INIT(4, 4, 64, 128, 128),
  1, &cls_dense_weights_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  cls_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 2, 1, 1), AI_STRIDE_INIT(4, 4, 4, 8, 8),
  1, &cls_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_bias, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_1_bias_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_1_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_scratch0, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 3), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_1_scratch0_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_1_weights, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 16, 1, 3, 16), AI_STRIDE_INIT(4, 4, 64, 1024, 1024),
  1, &conv1d_1_weights_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_2_bias, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_2_bias_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_2_output, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_2_output_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_2_scratch0, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 3), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_2_scratch0_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_2_weights, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 16, 1, 3, 16), AI_STRIDE_INIT(4, 4, 64, 1024, 1024),
  1, &conv1d_2_weights_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_3_bias, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_3_bias_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_3_output, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_3_output_array, NULL)

/* Tensor #16 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_3_weights, AI_STATIC,
  16, 0x0,
  AI_SHAPE_INIT(4, 16, 1, 3, 16), AI_STRIDE_INIT(4, 4, 64, 1024, 1024),
  1, &conv1d_3_weights_array, NULL)

/* Tensor #17 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_4_bias, AI_STATIC,
  17, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_4_bias_array, NULL)

/* Tensor #18 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_4_output, AI_STATIC,
  18, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_4_output_array, NULL)

/* Tensor #19 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_4_weights, AI_STATIC,
  19, 0x0,
  AI_SHAPE_INIT(4, 16, 1, 3, 16), AI_STRIDE_INIT(4, 4, 64, 1024, 1024),
  1, &conv1d_4_weights_array, NULL)

/* Tensor #20 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_bias, AI_STATIC,
  20, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_bias_array, NULL)

/* Tensor #21 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_output, AI_STATIC,
  21, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &conv1d_output_array, NULL)

/* Tensor #22 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_scratch0, AI_STATIC,
  22, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 5), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &conv1d_scratch0_array, NULL)

/* Tensor #23 */
AI_TENSOR_OBJ_DECLARE(
  conv1d_weights, AI_STATIC,
  23, 0x0,
  AI_SHAPE_INIT(4, 30, 1, 5, 16), AI_STRIDE_INIT(4, 4, 120, 1920, 1920),
  1, &conv1d_weights_array, NULL)

/* Tensor #24 */
AI_TENSOR_OBJ_DECLARE(
  dense_dense_bias, AI_STATIC,
  24, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &dense_dense_bias_array, NULL)

/* Tensor #25 */
AI_TENSOR_OBJ_DECLARE(
  dense_dense_output, AI_STATIC,
  25, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &dense_dense_output_array, NULL)

/* Tensor #26 */
AI_TENSOR_OBJ_DECLARE(
  dense_dense_weights, AI_STATIC,
  26, 0x0,
  AI_SHAPE_INIT(4, 16, 16, 1, 1), AI_STRIDE_INIT(4, 4, 64, 1024, 1024),
  1, &dense_dense_weights_array, NULL)

/* Tensor #27 */
AI_TENSOR_OBJ_DECLARE(
  dense_output, AI_STATIC,
  27, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &dense_output_array, NULL)

/* Tensor #28 */
AI_TENSOR_OBJ_DECLARE(
  global_average_pooling1d_pool_output, AI_STATIC,
  28, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 1), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &global_average_pooling1d_pool_output_array, NULL)

/* Tensor #29 */
AI_TENSOR_OBJ_DECLARE(
  imu_input_output, AI_STATIC,
  29, 0x0,
  AI_SHAPE_INIT(4, 1, 30, 1, 100), AI_STRIDE_INIT(4, 4, 4, 120, 120),
  1, &imu_input_output_array, NULL)

/* Tensor #30 */
AI_TENSOR_OBJ_DECLARE(
  re_lu_1_output, AI_STATIC,
  30, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &re_lu_1_output_array, NULL)

/* Tensor #31 */
AI_TENSOR_OBJ_DECLARE(
  re_lu_2_output, AI_STATIC,
  31, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &re_lu_2_output_array, NULL)

/* Tensor #32 */
AI_TENSOR_OBJ_DECLARE(
  re_lu_3_output, AI_STATIC,
  32, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &re_lu_3_output_array, NULL)

/* Tensor #33 */
AI_TENSOR_OBJ_DECLARE(
  re_lu_4_output, AI_STATIC,
  33, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &re_lu_4_output_array, NULL)

/* Tensor #34 */
AI_TENSOR_OBJ_DECLARE(
  re_lu_output, AI_STATIC,
  34, 0x0,
  AI_SHAPE_INIT(4, 1, 16, 1, 100), AI_STRIDE_INIT(4, 4, 4, 64, 64),
  1, &re_lu_output_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  cls_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &cls_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &cls_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  cls_layer, 18,
  SM_TYPE, 0x0, NULL,
  sm, forward_sm,
  &cls_chain,
  NULL, &cls_layer, AI_STATIC, 
  .nl_params = NULL, 
  .axis = AI_SHAPE_CHANNEL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  cls_dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &cls_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &cls_dense_weights, &cls_dense_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  cls_dense_layer, 18,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &cls_dense_chain,
  NULL, &cls_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  dense_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_dense_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &dense_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  dense_layer, 17,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &dense_chain,
  NULL, &cls_dense_layer, AI_STATIC, 
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
  dense_dense_layer, 17,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &dense_dense_chain,
  NULL, &dense_layer, AI_STATIC, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  global_average_pooling1d_pool_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &global_average_pooling1d_pool_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  global_average_pooling1d_pool_layer, 16,
  POOL_TYPE, 0x0, NULL,
  pool, forward_ap,
  &global_average_pooling1d_pool_chain,
  NULL, &dense_dense_layer, AI_STATIC, 
  .pool_size = AI_SHAPE_2D_INIT(1, 100), 
  .pool_stride = AI_SHAPE_2D_INIT(1, 100), 
  .pool_pad = AI_SHAPE_INIT(4, 0, 0, 0, 0), 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  re_lu_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_4_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  re_lu_4_layer, 15,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &re_lu_4_chain,
  NULL, &global_average_pooling1d_pool_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &re_lu_2_output, &conv1d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_1_layer, 14,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &add_1_chain,
  NULL, &re_lu_4_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_4_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_4_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_4_weights, &conv1d_4_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_4_layer, 13,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_4_chain,
  NULL, &add_1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 0, 2, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  re_lu_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_3_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  re_lu_3_layer, 11,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &re_lu_3_chain,
  NULL, &conv1d_4_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_3_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_3_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_3_weights, &conv1d_3_bias, NULL),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  conv1d_3_layer, 10,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32_group,
  &conv1d_3_chain,
  NULL, &re_lu_3_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 2), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 0, 2, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  re_lu_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_2_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  re_lu_2_layer, 9,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &re_lu_2_chain,
  NULL, &conv1d_3_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  add_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &re_lu_output, &conv1d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &add_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  add_layer, 8,
  ELTWISE_TYPE, 0x0, NULL,
  eltwise, forward_eltwise,
  &add_chain,
  NULL, &re_lu_2_layer, AI_STATIC, 
  .operation = ai_sum_f32, 
  .buffer_operation = ai_sum_buffer_f32, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_2_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_2_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_2_weights, &conv1d_2_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_2_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_2_layer, 7,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &conv1d_2_chain,
  NULL, &add_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  re_lu_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_1_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  re_lu_1_layer, 5,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &re_lu_1_chain,
  NULL, &conv1d_2_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_1_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_1_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_1_weights, &conv1d_1_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_1_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_1_layer, 4,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &conv1d_1_chain,
  NULL, &re_lu_1_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 1, 0, 1, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  re_lu_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &re_lu_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  re_lu_layer, 2,
  NL_TYPE, 0x0, NULL,
  nl, forward_relu,
  &re_lu_chain,
  NULL, &conv1d_1_layer, AI_STATIC, 
  .nl_params = NULL, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  conv1d_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &imu_input_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &conv1d_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 3, &conv1d_weights, &conv1d_bias, NULL),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &conv1d_scratch0, NULL)
)

AI_LAYER_OBJ_DECLARE(
  conv1d_layer, 1,
  CONV2D_TYPE, 0x0, NULL,
  conv2d, forward_conv2d_if32of32wf32,
  &conv1d_chain,
  NULL, &re_lu_layer, AI_STATIC, 
  .groups = 1, 
  .filter_stride = AI_SHAPE_2D_INIT(1, 1), 
  .dilation = AI_SHAPE_2D_INIT(1, 1), 
  .filter_pad = AI_SHAPE_INIT(4, 2, 0, 2, 0), 
  .in_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_SAME, 
  .out_ch_format = AI_LAYER_FORMAT_CHANNEL_LAST_VALID, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 23432, 1, 1),
    23432, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 13184, 1, 1),
    13184, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_IN_NUM, &imu_input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_OUT_NUM, &cls_output),
  &conv1d_layer, 0x1d20f396, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 23432, 1, 1),
      23432, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 13184, 1, 1),
      13184, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_IN_NUM, &imu_input_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_IMU_MODEL_OUT_NUM, &cls_output),
  &conv1d_layer, 0x1d20f396, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool imu_model_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_imu_model_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    imu_input_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 392);
    imu_input_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 392);
    conv1d_scratch0_array.data = AI_PTR(g_imu_model_activations_map[0] + 12392);
    conv1d_scratch0_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 12392);
    conv1d_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 136);
    conv1d_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 136);
    re_lu_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6592);
    re_lu_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6592);
    conv1d_1_scratch0_array.data = AI_PTR(g_imu_model_activations_map[0] + 12992);
    conv1d_1_scratch0_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 12992);
    conv1d_1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 192);
    conv1d_1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 192);
    re_lu_1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 192);
    re_lu_1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 192);
    conv1d_2_scratch0_array.data = AI_PTR(g_imu_model_activations_map[0] + 12992);
    conv1d_2_scratch0_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 12992);
    conv1d_2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    conv1d_2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    add_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6592);
    add_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6592);
    re_lu_2_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    re_lu_2_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    conv1d_3_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6784);
    conv1d_3_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6784);
    re_lu_3_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6784);
    re_lu_3_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6784);
    conv1d_4_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6528);
    conv1d_4_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6528);
    add_1_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    add_1_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    re_lu_4_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 6400);
    re_lu_4_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 6400);
    global_average_pooling1d_pool_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    global_average_pooling1d_pool_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    dense_dense_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 64);
    dense_dense_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 64);
    dense_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    dense_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
    cls_dense_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 64);
    cls_dense_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 64);
    cls_output_array.data = AI_PTR(g_imu_model_activations_map[0] + 0);
    cls_output_array.data_start = AI_PTR(g_imu_model_activations_map[0] + 0);
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
    
    conv1d_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 0);
    conv1d_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 0);
    conv1d_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 9600);
    conv1d_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 9600);
    conv1d_1_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_1_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 9664);
    conv1d_1_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 9664);
    conv1d_1_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_1_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 12736);
    conv1d_1_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 12736);
    conv1d_2_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_2_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 12800);
    conv1d_2_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 12800);
    conv1d_2_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_2_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 15872);
    conv1d_2_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 15872);
    conv1d_3_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_3_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 15936);
    conv1d_3_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 15936);
    conv1d_3_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_3_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 19008);
    conv1d_3_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 19008);
    conv1d_4_weights_array.format |= AI_FMT_FLAG_CONST;
    conv1d_4_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 19072);
    conv1d_4_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 19072);
    conv1d_4_bias_array.format |= AI_FMT_FLAG_CONST;
    conv1d_4_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 22144);
    conv1d_4_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 22144);
    dense_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    dense_dense_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 22208);
    dense_dense_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 22208);
    dense_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    dense_dense_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 23232);
    dense_dense_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 23232);
    cls_dense_weights_array.format |= AI_FMT_FLAG_CONST;
    cls_dense_weights_array.data = AI_PTR(g_imu_model_weights_map[0] + 23296);
    cls_dense_weights_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 23296);
    cls_dense_bias_array.format |= AI_FMT_FLAG_CONST;
    cls_dense_bias_array.data = AI_PTR(g_imu_model_weights_map[0] + 23424);
    cls_dense_bias_array.data_start = AI_PTR(g_imu_model_weights_map[0] + 23424);
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
      
      .n_macc            = 560432,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x1d20f396,
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
      
      .n_macc            = 560432,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x1d20f396,
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

