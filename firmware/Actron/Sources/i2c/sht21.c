/*
 * Copyright (c) 2013, Maxim Osipov <maxim.osipov@gmail.com>
 */

#include <stdio.h>
#include "sht21.h"
#include "i2c.h"

extern void delay(int t);

static uint8_t buf[16];

int sht21_temp(void)
{
  int val;

  buf[0] = 0xf3;
  i2c_tx(0x40, 1, buf);

  /* Wait for measurement about 85ms */
  delay(150000);

  i2c_rx(0x40, 3, buf);

  val = (int)(buf[1]<<8 | buf[2]);

  /* return temp * 100 (0.01 deg accuracy) */
  return (-46.85 + (175.72*(val & 0x0000fffc))/0x10000)*100;
}


int sht21_humidity(void)
{
  int val;

  buf[0] = 0xf5;
  i2c_tx(0x40, 1, buf);

  /* Wait for measurement about 85ms */
  delay(150000);

  i2c_rx(0x40, 3, buf);

  val = (int)(buf[1]<<8 | buf[2]);

  /* return relative humidity * 100 (0.04 % accuracy) */
  return (-6.0 + (125.0*(val & 0x0000fffc))/0x10000)*100;
}
