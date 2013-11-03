/*
 * Copyright (c) 2012, Maxim Osipov <maxim.osipov@gmail.com>
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdio.h>
#include "sht21.h"
#include "i2c.h"

extern void delay(int t);

static uint8_t buf[16];

int sht21_temp(void)
{
  int val;

  /* For for about 15ms before the SHT21 can be used */
  delay(15000);

  buf[0] = 0xe3;
  i2c_tx(0x40, 1, buf);

  /* Wait for measurement about 85ms */
  delay(85000);

  i2c_rx(0x40, 3, buf);

  val = (int)(buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3]);

  /* return temp * 100 (0.01 deg accuracy) */
  return (-46.85 + (175.72*((val>>16)&0x0000fffc))/0x10000)*100;
}


int sht21_humidity(void)
{
  int val;

  /* For for about 15ms before the SHT21 can be used */
  delay(15000);

  buf[0] = 0xe5;
  i2c_tx(0x40, 1, buf);

  /* Wait for measurement about 85ms */
  delay(85000);

  i2c_rx(0x40, 3, buf);

  val = (int)(buf[0]<<24 | buf[1]<<16 | buf[2]<<8 | buf[3]);

  /* return relative humidity * 100 (0.04 % accuracy) */
  return (-6.0 + (125.0*((val>>16)&0x0000fffc))/0x10000)*100;
}