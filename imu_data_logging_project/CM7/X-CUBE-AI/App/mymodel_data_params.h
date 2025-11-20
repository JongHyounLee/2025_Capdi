/**
  ******************************************************************************
  * @file    mymodel_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-11-01T14:43:27+0900
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef MYMODEL_DATA_PARAMS_H
#define MYMODEL_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_MYMODEL_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_mymodel_data_weights_params[1]))
*/

#define AI_MYMODEL_DATA_CONFIG               (NULL)


#define AI_MYMODEL_DATA_ACTIVATIONS_SIZES \
  { 55680, }
#define AI_MYMODEL_DATA_ACTIVATIONS_SIZE     (55680)
#define AI_MYMODEL_DATA_ACTIVATIONS_COUNT    (1)
#define AI_MYMODEL_DATA_ACTIVATION_1_SIZE    (55680)



#define AI_MYMODEL_DATA_WEIGHTS_SIZES \
  { 445640, }
#define AI_MYMODEL_DATA_WEIGHTS_SIZE         (445640)
#define AI_MYMODEL_DATA_WEIGHTS_COUNT        (1)
#define AI_MYMODEL_DATA_WEIGHT_1_SIZE        (445640)



#define AI_MYMODEL_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_mymodel_activations_table[1])

extern ai_handle g_mymodel_activations_table[1 + 2];



#define AI_MYMODEL_DATA_WEIGHTS_TABLE_GET() \
  (&g_mymodel_weights_table[1])

extern ai_handle g_mymodel_weights_table[1 + 2];


#endif    /* MYMODEL_DATA_PARAMS_H */
