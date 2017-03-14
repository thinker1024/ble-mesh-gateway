
#pragma once

#define AIRMOUSE_CALIBRATION_ADDR   (0x18000)
#define	MPU6050_I2C_ID				(0x68 << 1)

typedef struct {
	s8 btn;
	s8 x;
	s8 y;
	s8 w;
}amouse_data_t;

int airmouse_getxy(amouse_data_t *mouse_data);

