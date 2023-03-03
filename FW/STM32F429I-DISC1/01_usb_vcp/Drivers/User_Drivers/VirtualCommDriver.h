/**
 * MIT License
 * 
 * Copyright (c) 2019 Brian Amos
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#ifndef _USB_VIRTUAL_COMM_DRIVER_
#define _USB_VIRTUAL_COMM_DRIVER_
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <FreeRTOS.h>
#include <stream_buffer.h>
#include "usbd_cdc.h"


extern USBD_CDC_ItfTypeDef USBD_Interface_fops;
void VirtualCommInit( void );

StreamBufferHandle_t const* GetUsbRxStreamBuff( void );
int32_t TransmitUsbData(uint8_t const* Buff, uint16_t Len);
int32_t TransmitUsbDataLossy(uint8_t const* Buff, uint16_t Len);
int8_t user_usb_tx_cplt(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);


#ifdef __cplusplus
 }
#endif
#endif /* _USB_VIRTUAL_COMM_DRIVER_ */
