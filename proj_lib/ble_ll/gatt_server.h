#pragma once

#include "gatt.h"


/** @addtogroup  GATT_Server_Module GATT server
 *  @{
 */

/** @addtogroup  GATT_Server_Types GATT server types
 *  @{
 */

/**
 *  @brief  Definition for client characteristic configuration change callback function
 */

typedef u8* (*att_handler_t)(u8 * p);
typedef int (*att_readwrite_callback_t)(void* p);
typedef struct attribute
{
  u8  attNum;
  u8  uuidLen;
  u8  attrLen;
  u8  attrMaxLen;
  u8* uuid;
  u8* pAttrValue;
  att_readwrite_callback_t w;
  att_readwrite_callback_t r;
} attribute_t;


/** @addtogroup GATT_Attr_Num_Calc GATT attribute number calculation
 * @{
 */
#define GATT_CALC_ATTR_NUM( attrArray )       (sizeof(attrArray) / sizeof(attribute_t))
/** @} end of group GATT_Attr_Num_Calc */

