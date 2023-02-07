/*
 * espdatalogger.c
 *
 *  Created on: 5 Feb 2023
 *      Author: bvick
 */

#include "EspDataLogger/uartringbuffer.h"
#include "EspDataLogger/espdatalogger.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart1;

uint8_t debugBuf1 [40] = {'\0'}; // variable to stored the data to be transmitted


void bufclr (char *buf)
{
	int len = strlen (buf);
	for (int i=0; i<len; i++) buf[i] = '\0';
}


void ESP_Init (char *SSID, char *PASSWD)
{
	char data[80];

	Ringbuf_init();
	//sprintf ((char*)debugBuf1, "Send Data1\r\n");
	//HAL_UART_Transmit (&huart1 , debugBuf1 , sizeof ( debugBuf1 ) , 50) ; // transmit the data
	Uart_sendstring("AT+RST\r\n");
	HAL_Delay(1000);

	Uart_flush();
	//sprintf ((char*)debugBuf1, "Send Data2\r\n");
	//HAL_UART_Transmit (&huart1 , debugBuf1 , sizeof ( debugBuf1 ) , 50) ; // transmit the data
	/********* AT **********/
	Uart_sendstring("AT\r\n");
	while(!(Wait_for("OK\r\n")));
	//sprintf ((char*)debugBuf1, "Send Data3\r\n");
	//HAL_UART_Transmit (&huart1 , debugBuf1 , sizeof ( debugBuf1 ) , 50) ; // transmit the data
	Uart_flush();


	/********* AT+CWMODE=1 **********/
	//Uart_sendstring("AT+CWMODE=1\r\n");
	Uart_sendstring("AT+CWMODE=1\r\n");
	while (!(Wait_for("OK\r\n")));
	//sprintf ((char*)debugBuf1, "Send Data4\r\n");
	//HAL_UART_Transmit (&huart1 , debugBuf1 , sizeof ( debugBuf1 ) , 50) ; // transmit the data
	Uart_flush();

	/********* AT+CWDHCP=1,1 **********/
	//Uart_sendstring("AT+CWMODE=1\r\n");
	Uart_sendstring("AT+CWDHCP=1,1\r\n");
	while (!(Wait_for("OK\r\n")));
	//sprintf ((char*)debugBuf1, "Send Data5\r\n");
	//HAL_UART_Transmit (&huart1 , debugBuf1 , sizeof ( debugBuf1 ) , 50) ; // transmit the data
	Uart_flush();


	/********* AT+CWJAP="SSID","PASSWD" **********/
	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_sendstring(data);
	//while (!(Wait_for("GOT IP\r\n")));
	while (!(Wait_for("WIFI GOT IP\r\n")));
	//sprintf ((char*)debugBuf1, "Send Data6\r\n");
	//HAL_UART_Transmit (&huart1 , debugBuf1 , sizeof ( debugBuf1 ) , 50) ; // transmit the data

	Uart_flush();

	/********* AT+CIPMUX=0 **********/
	Uart_sendstring("AT+CIPMUX=0\r\n");
	while (!(Wait_for("OK\r\n")));
	bufclr((char*)debugBuf1);
	sprintf ((char*)debugBuf1, "ESP WIFI Done\r\n");
	HAL_UART_Transmit (&huart1 , debugBuf1 , sizeof ( debugBuf1 ) , 50) ; // transmit the data
	Uart_flush();

}

void ESP_Send_Data (char *APIkey, int Field_num, int value)
{
	char local_buf[100] = {0};
	char local_buf2[30] = {0};

	Uart_sendstring("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n");
	while (!(Wait_for("OK\r\n")));

	sprintf (local_buf, "GET /update?api_key=%s&field%d=%d\r\n", APIkey, Field_num, value);
	int len = strlen (local_buf);

	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	Uart_sendstring(local_buf2);
	while (!(Wait_for(">")));

	Uart_sendstring (local_buf);
	while (!(Wait_for("SEND OK\r\n")));

	while (!(Wait_for("CLOSED")));

	bufclr(local_buf);
	bufclr(local_buf2);

	Ringbuf_init();

}

void ESP_Send_Multi (char *APIkey, int numberoffileds, int value[])
{
	char local_buf[500] = {0};
	char local_buf2[30] = {0};
	char field_buf[200] = {0};


	Uart_sendstring("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n");
	while (!(Wait_for("OK\r\n")));

	sprintf (local_buf, "GET /update?api_key=%s", APIkey);
	for (int i=0; i<numberoffileds; i++)
	{
		sprintf(field_buf, "&field%d=%d",i+1, value[i]);
		strcat (local_buf, field_buf);
	}

	strcat(local_buf, "\r\n");
	int len = strlen (local_buf);

	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	Uart_sendstring(local_buf2);
	while (!(Wait_for(">")));

	Uart_sendstring (local_buf);
	while (!(Wait_for("SEND OK\r\n")));

	while (!(Wait_for("CLOSED")));

	bufclr(local_buf);
	bufclr(local_buf2);

	Ringbuf_init();

}

