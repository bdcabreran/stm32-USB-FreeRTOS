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

#include "VirtualCommDriver.h"
#include <usb_device.h>
#include "usbd_cdc.h"
#include <FreeRTOS.h>
#include <stream_buffer.h>
#include <task.h>
#include <SEGGER_SYSVIEW.h>


/******************************** NOTE *****************************/
/* This code is NOT safe to use across multiple tasks as-is. Also, it will not
 * work properly when included by multiple source files.
 * A new copy of the static variables below will be allocated for each
 * compilation unit the file is included in
 * Some additional work is required to ensure only one copy of these variables
 * is created before this file is able to be used across more than one file.
 */
#define txBuffLen 2048
#define rxBuffLen 2048
static uint8_t usbTxBuff[txBuffLen];
static uint8_t usbRxBuff[rxBuffLen];
static StreamBufferHandle_t txStream = NULL;
static TaskHandle_t usbTaskHandle = NULL;

StreamBufferHandle_t rxStream = NULL;


//hUsbDeviceHS defined in usb_device.c
extern USBD_HandleTypeDef hUsbDeviceHS;

void usbTask( void* NotUsed);

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/********************************** PUBLIC *************************************/

/**
 * Initialize the USB peripheral and HAL-based USB stack.
 * A transmit task, responsible for pulling data out of the stream buffer and
 * pushing it into the USB peripheral is also created.
**/
void VirtualCommInit( void )
{
	MX_USB_DEVICE_Init();
	txStream = xStreamBufferCreate( txBuffLen, 1);
	rxStream  = xStreamBufferCreate( rxBuffLen, 1);
	assert_param( txStream != NULL);
	assert_param( rxStream != NULL);
	assert_param(xTaskCreate(usbTask, "usbTask", 256, NULL, configMAX_PRIORITIES, &usbTaskHandle) == pdPASS);
}

/**
 * return the streamBuffer handle
 * This is wrapped in a function rather than exposed directly
 * so external modules aren't able to change where the handle points
 *
 * NOTE:	This return value will not be valid until VirtualCommInit has
 * 			been run
 */
StreamBufferHandle_t const* GetUsbRxStreamBuff( void )
{
	return &rxStream;
}

/**
 * Transmit data to to USB if space is available in the stream.  If no space is available
 * then the function will return immediately.
 *
 * @param Buff	pointer to the buffer to be transmitted
 * @param Len	number of bytes to transmit
 * @returns number of bytes placed into the buffer
 */
int32_t TransmitUsbDataLossy(uint8_t const* Buff, uint16_t Len)
{
	int32_t numBytesCopied = xStreamBufferSendFromISR(	txStream, Buff, Len, NULL);

	return numBytesCopied;
}

/**
 * Similar to TransmitDataLossy, but adds a bit more convenience by copying as much data as possible
 * within 2 ticks for space to become available - NOT able to be called from within an ISR
 * @param Buff	pointer to the buffer to be transmitted
 * @param Len	number of bytes to transmit
 * @returns number of bytes placed into the buffer
 */
int32_t TransmitUsbData(uint8_t const* Buff, uint16_t Len)
{
	int32_t numBytesCopied = xStreamBufferSend(	txStream, Buff, Len, 1);

	if(numBytesCopied != Len)
	{
		numBytesCopied += xStreamBufferSend(	txStream, Buff+numBytesCopied, Len-numBytesCopied, 1);
	}

	return numBytesCopied;
}

/********************************** PRIVATE *************************************/

/**
 * FreeRTOS task that takes data out of the txStream and pushes
 * data to be transmitted into the USB HAL buffer.
 *
 * This function waits for a task notification, which is sent by a callback generated
 * from the USB stack upon completion of a transmission
 *
 * It then then copies up to 2KB of data from the stream buffer to a
 * local buffer, which is passed to the HAL USB stack.
 *
 */
void usbTask( void* NotUsed)
{
	USBD_CDC_HandleTypeDef *hcdc = NULL;

	while(hcdc == NULL)
	{
		hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
		vTaskDelay(10);
	}
	if (hcdc->TxState == 0)
	{
		//if there is no TX in progress, immediately send a task notification
		//to kick things off
		xTaskNotify( usbTaskHandle, 1, eSetValueWithOverwrite);
	}
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

	//ensure the USB interrupt priority is low enough to allow for
	//FreeRTOS API calls within the ISR
	NVIC_SetPriority(OTG_FS_IRQn, 6);

	while(1)
	{
		SEGGER_SYSVIEW_PrintfHost("waiting for txStream");
		//wait forever for data to become available in the stream buffer
		//txStream.  up to txBuffLen bytes of data will be copied into
		//usbTxBuff when at least 1 byte is available
		uint8_t numBytes = xStreamBufferReceive(	txStream,
													usbTxBuff,
													txBuffLen,
													portMAX_DELAY);
		if(numBytes > 0)
		{
			SEGGER_SYSVIEW_PrintfHost("pulled %d bytes from txStream", numBytes);
			USBD_CDC_SetTxBuffer(&hUsbDeviceHS, usbTxBuff, numBytes);
			USBD_CDC_TransmitPacket(&hUsbDeviceHS);
			//wait forever for a notification, clearing it to 0 when received
			ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
			SEGGER_SYSVIEW_PrintfHost("tx complete");
		}
	}
}

/**
 * ! These functions were extracted from @USB_DEVICE\App\usbd_cdc_if.h
 * 
 */
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, usbTxBuff, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, usbRxBuff);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  // ** These lines are written for the current implementation
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(usbTaskHandle, 1, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* USER CODE END 14 */
  return result;
}
