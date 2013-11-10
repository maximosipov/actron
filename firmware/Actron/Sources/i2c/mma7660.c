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
#include "i2c.h"
#include "mma7660.h"

extern void delay(int t);

#define ADDR	0x4C

#define XOUT	0x00
#define YOUT	0x01
#define ZOUT	0x02
#define TILT	0x03
#define SRST	0x04
#define SPCNT	0x05
#define INTSU	0x06
#define MODE	0x07
#define SR		0x08
#define PDET	0x09
#define PD		0x0A

volatile mma7660_data_t mma7660_data = {0, 0, 0, 0};

static uint8_t buf[8];

void mma7660_init(void)
{
  buf[0] = INTSU;
  buf[1] = 0x00;
  i2c_tx(ADDR, 2, buf);
  delay(1000);

  buf[0] = MODE;
  buf[1] = 0x01;
  i2c_tx(ADDR, 2, buf);
  delay(1000);

  buf[0] = SR;
  buf[1] = 0x1F;
  i2c_tx(ADDR, 2, buf);
  delay(1000);

  buf[0] = PDET;
  buf[1] = 0xFF;
  i2c_tx(ADDR, 2, buf);
  delay(1000);

  buf[0] = PD;
  buf[1] = 0xFF;
  i2c_tx(ADDR, 2, buf);
  delay(1000);
}

void mma7660_acc_req(void)
{
  /* Read data */
  buf[0] = XOUT;
  i2c_tx(ADDR, 1, buf);
}

void mma7660_acc_resp(void)
{
  int tmp;

  i2c_rx(ADDR, 3, buf);

  tmp = (signed char)((buf[1] & 0x20) ? (buf[1] | 0xC0) : (buf[1] & 0x3F));
  mma7660_data.x = (tmp*150)/32;
  tmp = (signed char)((buf[2] & 0x20) ? (buf[2] | 0xC0) : (buf[2] & 0x3F));
  mma7660_data.y = (tmp*150)/32;
  tmp = (signed char)((buf[3] & 0x20) ? (buf[3] | 0xC0) : (buf[3] & 0x3F));
  mma7660_data.z = (tmp*150)/32;
}
