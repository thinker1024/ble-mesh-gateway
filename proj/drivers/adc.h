/*
 * adc.h
 *
 *  Created on: 2015-12-10
 *      Author: Telink
 */

#ifndef ADC_H_
#define ADC_H_

#if(__TL_LIB_8266__ || MCU_CORE_TYPE == MCU_CORE_8266)
#include "adc_8266.h"
#elif(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8263)
#include "adc_8263.h"
#elif(__TL_LIB_8267__ || MCU_CORE_TYPE == MCU_CORE_8267)
#include "adc_8267.h"
//#elif(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8263)
//#include "adc_8263.h"
#endif



#endif /* ADC_H_ */
