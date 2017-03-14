
#pragma once

#if(__TL_LIB_8266__ || MCU_CORE_TYPE == MCU_CORE_8266)
#include "rf_drv_8266.h"
#elif(__TL_LIB_8267__ || MCU_CORE_TYPE == MCU_CORE_8267)
#include "rf_drv_8267.h"
#elif(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8263)
#include "rf_drv_8263.h"
#elif(__TL_LIB_8366__ || MCU_CORE_TYPE == MCU_CORE_8366)
#include "rf_drv_8366.h"
#elif(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8368)
#include "rf_drv_8368.h"
#endif

