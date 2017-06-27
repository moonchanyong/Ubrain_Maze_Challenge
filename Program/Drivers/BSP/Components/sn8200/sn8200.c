/******************************************************************************/
//  Copyright (C) 2004, SyChip(TM) Inc.
//
// Terms of use of this file:
// (1) This file can not be used in any way for any purpose by any company
//     other than SyChip.
// (2) This file can not be used by any project without prior written authroization 
//     from SyChip.
// (3) The usage of this file is restricted to compile and link only.  
//     No modification is allowed without prior written authorization from SyChip.
// (4) No portion of this file is allowed to be copied in any form without prior
//     written authorization from SyChip.
// (5) The information contained in this file is SyChip confidential and 
//     must not be disclosed without prior written authorization from SyChip.
// (6) Study of this file for the purpose of reverse engineering is 
//     strictly prohibited.
//
// If you don't agree with these terms, please delete this file immediately
// without any further use.
//
// Disclaimer:
//   No warrenty of functionality or performance, implied or explicit, is
//   provided with this file. The user of this file assumes the responsibility
//   for any damage that can result.
//
//*****************************************************************************/

#include "sn8200.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup SPI_FLASH
  * @{
  */  

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SER_ACK_RECD    0x01
#define SER_NAK_RECD    0x02

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char SSID[32];
uint8_t secMode = 0;
char secKey[64];
uint8_t Keylen = 0;
rx_info_t rx_frame;

// static varaibles
static serial_rx_state_t serial_rx_state;
static uint8_t curr_buf_idx = 0;
//static uint16_t rx_frm_payload_index;
//static uint16_t rx_frm_data_index;
static uint8_t rx_frm_chksum;
static cmd_proc_fn_t cmd_proc_fn;

//static uint8_t ack_recv_status;

uint8_t totalscan = 0;
scanlist_t sl[30];

uint8_t client[MAX_CONNECTION_PER_SOCK] = {0,};
static uint8_t WiFi_on = 0;
uint8_t seq = 0;

static uint8_t *TXBuf = NULL;
static uint8_t *buf = NULL;

extern uint32_t selfIP;
extern uint32_t destIP;
extern uint16_t destPort;
extern uint32_t srcIP;
extern uint16_t srcPort;
extern uint8_t mysock;
extern uint8_t StreamOff[6];
extern SPI_HandleTypeDef SpiHandle;
extern UART_HandleTypeDef UartHandle2;

/* IO driver structure initialization */  
WiFi_DrvTypeDef sn8200_wifi_drv = 
{
  sn8200_Start,
	sci_serial_redirect,
	WifiOn,
	WifiScan,
	WifiJoin,
	SnicInit,
	SnicIPConfig,
	SnicGetDhcp,
	tcpCreateSocket,
	getTCPinfo,
	tcpConnectToServer,
	setTCPinfo,
	tcpCreateConnection,
	sendFromSock,
	GetStatus,
	Leave,
	closeSocket,
	SnicCleanup,
	WifiDisconn,
	WifiOff,
	udpCreateSocket,
	udpSendFromSock,
	udpStartRecv,
};

/* Private function prototypes -----------------------------------------------*/
//static uint8_t escape_payload_and_transmit_no_ESC(uint16_t payload_len, uint8_t *payload);
static uint16_t calc_escaped_payload_len_no_ESC(uint16_t payload_len, uint8_t *payload);
static void send_ack_nak(uint8_t ack);
static uint8_t wait_for_ack_nak(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the peripherals used by the SN8200 driver.
  * @param  None
  * @retval None
  */
uint32_t inet_addr(const char *cp)
{
	unsigned int a[4];
	uint32_t ret;
	sscanf(cp, "%d.%d.%d.%d", &a[0], &a[1], &a[2], &a[3]);
	ret = ((a[3]<<24) + (a[2]<<16) + (a[1]<<8) + a[0]);
	return ret;
}

void sn8200_Start(void)
{
	serial_transmit_rx_init(sci_ser_cmd_proc);
	SN8200_ON();
	
	osDelay(500);
	
	if(WiFi_on) 
	{
		WifiOn();
		
		osDelay(10);
	}
	else WiFi_on = 1;
}

void serial_ack_nak_recd(uint8_t ack)
{
	char sz[20];

	if (ack) {
		//ack_recv_status = SER_ACK_RECD;
		//MessageBeep(MB_OK);
		//_stprintf(sz, L"Ack received");
	}
	else {
		//ack_recv_status = SER_NAK_RECD;
		//_stprintf(sz, L"Nak received");
		//MessageBeep(MB_ICONERROR);
	}

	printf("%s", sz);
}

void handle_rx_frame(void)
{
//	printf("bi %d 0x%x %d\n", curr_buf_idx, rx_frame.rx_payload[0] , rx_frame.payload_len );
	rx_frame.available = 1;
	curr_buf_idx = (curr_buf_idx +1) % NUM_RX_BUF;
	//SetEvent(hReadEvent);

	process_rx_frame();
}

//process_rx_frame
// This function is called after receving complte tx packet 
// This function can be used to process received frame
uint8_t process_rx_frame(void)
{        
	if (rx_frame.cmd_id == 0x7F)
	{
		// Received ACK for previous transaction
		serial_ack_nak_recd(1);
	}
	else if (rx_frame.cmd_id == 0x00)
	{
		// Received NAK for previous transaction
		serial_ack_nak_recd(0);
	}
	else {
	// New command
		if (rx_frame.ack_reqd)
		{
			send_ack_nak(rx_frame.ackOk);
		}
		if (rx_frame.ackOk) {
			// This is the function to process
			if (cmd_proc_fn) {	
					(*cmd_proc_fn)(rx_frame.cmd_id, rx_frame.payload_len, rx_frame.rx_payload); 
			}		
		}
	}
	return 0;
}

// top level, handles fragmentation
int8_t serial_transmit(uint8_t cmd_id, uint16_t payload_len, uint8_t *payload, uint8_t ack_required)
{
	if ( payload_len > MAX_PAYLOAD_LEN ) 
return SER_PROC_INVALID_LEN;

	uint16_t len = calc_escaped_payload_len_no_ESC(payload_len, payload);

	return no_ESC_transmit(cmd_id, payload_len, payload, ack_required, len);
}

//rx_process_char
// This function should be called to process each received characted over serial port
// return the amount of character processed
uint16_t rx_process_char_no_ESC(uint8_t *rx_ch)
{
	uint16_t ch=1;
	switch(serial_rx_state)
	{
		case IDLE:
			if (*rx_ch == SOM_CHAR) 
			{
				rx_process_char_no_ESC_RX_SOM:
				serial_rx_state = SOM_RECD;
				//rx_frm_payload_index = 0;
				rx_frm_chksum = 0;
				//rx_frm_data_index = 0;
			}
			else
				ch = 1;
			break;
		case SOM_RECD: 
			if( (*rx_ch & 0x7F) == 0 ) 
			{
				goto rx_process_char_no_ESC_err;
			}
			serial_rx_state=LEN_RECD;
			rx_frame.payload_len = *rx_ch & 0x7F;
			rx_frm_chksum += *rx_ch;
			break;
		case LEN_RECD: 
			/*if((*rx_ch & 0x3F) == 0 ) 
			{
				goto rx_process_char_no_ESC_err;
			}*/
			serial_rx_state=ACK_SEQ_RECD;
			rx_frame.payload_len |= (*rx_ch & 0x3F)<<7;
			//printf("\r\npayload len = %x.", rx_frame.payload_len);
			rx_frame.ack_reqd = (*rx_ch & 0x40);
			//printf("\r\nACK = %x.", rx_frame.ack_reqd);
			rx_frm_chksum += *rx_ch;
			break;
		case ACK_SEQ_RECD: 
			/*if((*rx_ch & 0x7F) == 0 ) 
			{
				goto rx_process_char_no_ESC_err;
			}*/
			serial_rx_state=CMD_RECD;
			rx_frame.cmd_id = *rx_ch & 0x7F;	
			rx_frm_chksum += *rx_ch;
			break;
		case CMD_RECD:
			serial_rx_state=PAYLAD_RX;
		
		case PAYLAD_RX:
			memcpy(rx_frame.rx_payload, (uint8_t *)rx_ch, rx_frame.payload_len);
			// TO DO: for image with checksum implementation, need to validate the checksum here
			rx_frame.ackOk = 1;
			handle_rx_frame();
			serial_rx_state = IDLE;
			ch = rx_frame.payload_len;
			break;

		default:
			rx_process_char_no_ESC_err: 
			serial_rx_state = IDLE;
			if(*rx_ch == SOM_CHAR) 
			{
				goto rx_process_char_no_ESC_RX_SOM;
			}
			break;
	}
	return ch;
}

uint8_t sci_ser_cmd_proc(uint8_t commandId, uint16_t paramLength, uint8_t *params)
{
	switch (commandId) {
		case CMD_ID_SNIC:
			handleRxSNIC(params, paramLength);
			break;
		case CMD_ID_WIFI:
			handleRxWiFi(params, paramLength);
			break;
		case CMD_ID_GEN:
			handleRxGen(params, paramLength);
			break;
		default:
			break;
	}
	return 0;
}

uint8_t serial_transmit_rx_init(cmd_proc_fn_t cmd_processor)
{
	cmd_proc_fn = cmd_processor;
	return 0;
}

uint16_t sci_serial_redirect(uint8_t *data)
{
	uint16_t ret;

	ret = rx_process_char_no_ESC(data);
	
	return ret;
}

int8_t no_ESC_transmit(uint8_t cmd_id, uint16_t payload_len, uint8_t *payload, uint8_t ack_required, uint16_t len)
{
	uint8_t cksum=0, hdr = 0x80;
	
	TXBuf = (uint8_t *)malloc(len+6);
	TXBuf[0] = SOM_CHAR;
	TXBuf[1] = 0x80 | (len&0x7F);
	cksum += (0x80 | (len&0x7F));
	hdr |= (ack_required << 6);
	if (len > 0x7F) hdr |= (len >> 7);
	TXBuf[2] = 0x80 | hdr;
	cksum += (0x80 | hdr);
	TXBuf[3] = 0x80 | cmd_id;
	cksum += (0x80 |cmd_id);
	memcpy((uint8_t *)TXBuf+4, payload, len);
	free(buf);
	buf = NULL;
	TXBuf[4+len] = 0x80 | cksum;
	TXBuf[5+len] = EOM_CHAR;

	//if(HAL_SPI_Transmit(&SpiHandle, (uint8_t *)TXBuf, len+6, 0) != HAL_OK) 
	
	if(HAL_SPI_Transmit_DMA(&SpiHandle, (uint8_t *)TXBuf, len+6) != HAL_OK) 
	{
		printf("\r\nSPI TX error");
		free(TXBuf);
		TXBuf = NULL;
	//	memcpy(TXBuf,"0x01",MAX_PAYLOAD_LEN);
		return SER_PROC_GENERAL_ERR;
	}
	//free(TXBuf);
	//TXBuf = NULL;

	if (ack_required) 
	{
		//ack_recv_status = 0;
		return wait_for_ack_nak();
	}
	else 
		return SER_PROC_SUCCESS;
}

int8_t getTCPinfo(void)
{
	char tempIPstr[32];
	char teststr[8];
	char junk[2];

	gets(junk);

	printf("\r\nEnter server IP to connect: ");
	gets(tempIPstr);
	destIP = inet_addr(tempIPstr);
	if (destIP == INADDR_NONE) {
		return CMD_ERROR;
	}
	
	printf("\r\nEnter server port number: ");
	gets(teststr);
	destPort = strtol(teststr, NULL, 0);
	if (destPort > 0xFFFF)
	{
		printf("\r\nInvalid port, max limit 0xFFFF");
		return CMD_ERROR;
	}

	return 0;
}

int8_t setTCPinfo(void)
{
	//char teststr[8];

	if (selfIP == 0) {
		printf("\r\nIP address has not been obtained.");
		return CMD_ERROR;
	}
	
	srcIP = selfIP;

	//printf("\r\nEnter server port number to set: ");
	//gets(teststr);
	srcPort = (uint16_t)11000;
	/*if (port > 0xFFFF)
	{
		printf("\r\nInvalid port, max limit 0xFFFF ");
		return CMD_ERROR;
	}*/

	return 0;
}

void Leave(void)
{
	uint8_t buf[2] = {0,};

	buf[0] = WIFI_DISCONNECT_REQ;
	buf[1] = seq++;
	serial_transmit(CMD_ID_WIFI, 2, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]Leave");
}

void GetStatus(void)
{
	uint8_t buf[3] = {0,};
	
	buf[0] = WIFI_GET_STATUS_REQ;
	buf[1] = seq++;
	buf[2] = 1;
	serial_transmit(CMD_ID_WIFI, 3, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]GetStatus");
}

void WifiOn(void)
{
	uint8_t buf[4] = {0,};
	
	buf[0] = WIFI_ON_REQ;
	buf[1] = seq++;
	buf[2] = (char)'K';
	buf[3] = (char)'R';	
	serial_transmit(CMD_ID_WIFI, 4, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]WiFiOn");
}

void WifiOff(void)
{
	uint8_t buf[2] = {0,};
	
	buf[0] = WIFI_OFF_REQ;
	buf[1] = seq++;
	serial_transmit(CMD_ID_WIFI, 2, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]WiFiOff");
}

void WifiDisconn(void)
{
	uint8_t buf[2] = {0,};
	
	buf[0] = WIFI_DISCONNECT_REQ;
	buf[1] = seq++;
	serial_transmit(CMD_ID_WIFI, 2, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]Disconnect");
}

void WifiScan(void)
{
	uint8_t buf[12] = {0,};
	
	buf[0] = WIFI_SCAN_REQ;
	buf[1] = seq++;
	buf[3] = 2; // bss type = any
	serial_transmit(CMD_ID_WIFI, 12, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]Scan");
}

void WifiJoin(void)
{
	uint8_t buf[128] = {0,};
	char junk[2];
	char tempstr[2] = {0};
	uint8_t *p = buf;

	*p++ = WIFI_JOIN_REQ; 
	*p++ = seq++; 

	gets(junk);

	printf("\r\nEnter SSID:");
	gets(SSID);
	memcpy(p, SSID, strlen(SSID));

	p += strlen(SSID);
	*p++ = 0x00;

	printf("\r\nEnter Security Mode(i.e. 0 for open, 4 for WPA2 AES):\n");
	gets(tempstr);
	//secMode = atoi(tempstr);
	
	if (secMode) {
		printf("Enter Security Key:\n");
		gets(secKey);
		Keylen = (unsigned char)strlen(secKey);

		if (Keylen <= 0) {
			printf("Invalid Key\n");
			return;
		}
	}

	*p++ = secMode; 
	*p++ = Keylen; 
	
	if (Keylen)
	{
		memcpy(p, secKey, Keylen);
		p += Keylen;
	}
	
	serial_transmit(CMD_ID_WIFI, (uint8_t)(p - buf), (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]Join");
}

void SnicInit(void)
{
	uint8_t buf[4] = {0,};
	uint16_t tmp;

	tmp = swap16(0x0800);
	buf[0] = SNIC_INIT_REQ;
	buf[1] = seq++;
	memcpy(buf+2, (uint8_t*)&tmp, 2);
	serial_transmit(CMD_ID_SNIC, 4, (uint8_t *)buf, ACK_NOT_REQUIRED);
	//printf("\r\n[TX]SNIC init");
}

void SnicCleanup(void)
{
	uint8_t buf[2] = {0,};
	
	buf[0] = SNIC_CLEANUP_REQ;
	buf[1] = seq++;
	serial_transmit(CMD_ID_SNIC, 2, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]SNIC cleanup");
}

void SnicIPConfig(void)
{
	uint8_t buf[4] = {0,};

	buf[0] = SNIC_IP_CONFIG_REQ;
	buf[1] = seq++;
	buf[2] = 1; //STA
	buf[3] = 1; //DHCP

	serial_transmit(CMD_ID_SNIC, 4, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]SNIC IP config");
}

void SnicGetDhcp(void)
{
	uint8_t buf[3] = {0,};

	buf[0] = SNIC_GET_DHCP_INFO_REQ;
	buf[1] = seq++;
	buf[2] = 1; // STA

	serial_transmit(CMD_ID_SNIC, 3, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]SNIC get DHCP");
}

void tcpCreateConnection(uint8_t shortSock, uint8_t maxClient)
{
	uint8_t buf[6] = {0,};
	uint16_t size;
	
	if (maxClient == 0 || maxClient > MAX_CONNECTION_PER_SOCK) maxClient = MAX_CONNECTION_PER_SOCK;

	buf[0] = SNIC_TCP_CREATE_CONNECTION_REQ;
	buf[1] = seq++;
	buf[2] = shortSock;
	size = swap16(0x0800);
	memcpy(buf+3, (uint8_t*)&size, 2);
	
	buf[5] = maxClient;

	serial_transmit(CMD_ID_SNIC, 6, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]TCP create connection");
}

void udpStartRecv(uint8_t sock)
{
	uint8_t buf[5] = {0,};
	uint16_t bufsize;
	bufsize = swap16(0x0800);
	
	buf[0] = SNIC_UDP_START_RECV_REQ;
	buf[1] = seq++;
	buf[2] = sock;
	memcpy(buf+3, (uint8_t*)&bufsize, 2);
	serial_transmit(CMD_ID_SNIC, 5, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]UDP start receive");
}
void udpSendFromSock(uint8_t sock, uint32_t ip, uint16_t port, uint8_t *sendBuf, uint16_t len, uint16_t index, uint8_t conMode)
{
	uint8_t buf[MAX_PAYLOAD_LEN] = {0,};
	
	if (len == 0 || len > MAX_UDP_SEND_DATA_LEN) len = MAX_UDP_SEND_DATA_LEN;
	buf[0] = SNIC_UDP_SEND_FROM_SOCKET_REQ;
	buf[1] = seq++;
	buf[8] = sock;
	buf[9] = conMode;

	port = swap16(port);
	uint16_t mybufsize = swap16((uint16_t)len);
	memcpy(buf+2, (uint8_t*)&ip, 4);
	memcpy(buf+6, (uint8_t*)&port, 2);
	memcpy(buf+10, (uint8_t*)&mybufsize, 2);
	memcpy(buf+12, (uint8_t*)&index, 2);
	memcpy(buf+14, sendBuf, len-2);
	//BSP_SDRAM_ReadData(sendBuf, (uint32_t *)(buf+14), ((len-2) >> 2));
	//BSP_SDRAM_ReadData_DMA(sendBuf, (uint32_t *)(buf+14), ((len-2) >> 2));
	//osDelay(1);
	serial_transmit(CMD_ID_SNIC, 12+len, (uint8_t *)buf, ACK_NOT_REQUIRED);
//	printf("\r\n buf = %", (uint_8 *)buf);
	//printf("\r\n[TX]UDP send");
}

void sendFromSock(uint8_t shortSocket, uint8_t *sendBuf, uint16_t len, uint16_t index)
{
	uint8_t buf[MAX_PAYLOAD_LEN-6] = {0,};
	
	if (len == 0 || len > MAX_TCP_SEND_DATA_LEN) len = MAX_TCP_SEND_DATA_LEN;
	buf[0] = SNIC_SEND_FROM_SOCKET_REQ;
	buf[1] = seq++;
	buf[2] = shortSocket;
	buf[3] = 0;

	uint16_t mybufsize = swap16(len);
	memcpy(buf+4, (uint8_t*)&mybufsize, 2);
	memcpy(buf+6, (uint8_t*)&index, 2);
	memcpy(buf+8, sendBuf, len-2);

	serial_transmit(CMD_ID_SNIC, 6+len, (uint8_t *)buf, ACK_NOT_REQUIRED);
	//printf("\r\n[TX]TCP send");
}

void closeSocket(uint8_t shortSocket)
{
	uint8_t buf[3] = {0,};
	
	buf[0] = SNIC_CLOSE_SOCKET_REQ;
	buf[1] = seq++;
	buf[2] = shortSocket;
	serial_transmit(CMD_ID_SNIC, 3, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]Socket %d closed\n", shortSocket);
}

void tcpConnectToServer(uint8_t shortSock, uint32_t ip, uint16_t port, uint8_t timeout)
{
	uint8_t buf[12] = {0,};
	uint16_t bufsize;

	buf[0] = SNIC_TCP_CONNECT_TO_SERVER_REQ;
	buf[1] = seq++;
	buf[2] = shortSock;

	memcpy(buf+3, (uint8_t*)&ip, 4);
	memcpy(buf+7, (uint8_t*)&port, 2);
	bufsize = swap16(0x0800);
	memcpy(buf+9, (uint8_t*)&bufsize, 2);
	
	buf[11] = timeout;

	serial_transmit(CMD_ID_SNIC, 12, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]TCP connect to server");
}

void udpCreateSocket(uint8_t bindOption, uint32_t ip, uint16_t port)
{
	uint8_t buf[9] = {0,};

	buf[0] = SNIC_UDP_CREATE_SOCKET_REQ;
	buf[1] = seq++;
	buf[2] = bindOption;

	if (bindOption) {
		port = swap16(port);
		memcpy(buf+3, (uint8_t*)&ip, 4);
		memcpy(buf+7, (uint8_t*)&port, 2);
		serial_transmit(CMD_ID_SNIC, 9, (uint8_t *)buf, ACK_NOT_REQUIRED);
	}

	else serial_transmit(CMD_ID_SNIC, 3, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]UDP create socket");
}

void tcpCreateSocket(uint8_t bindOption, uint32_t localIp, uint16_t port)
{
	uint8_t buf[9] = {0,};

	buf[0] = SNIC_TCP_CREATE_SOCKET_REQ;
	buf[1] = seq++;
	buf[2] = bindOption;

	if (bindOption) {
		port = swap16(port);
		memcpy(buf+3, (uint8_t*)&localIp, 4);
		memcpy(buf+7, (uint8_t*)&port, 2);

		serial_transmit(CMD_ID_SNIC, 9, (uint8_t *)buf, ACK_NOT_REQUIRED);
	}

	else serial_transmit(CMD_ID_SNIC, 3, (uint8_t *)buf, ACK_NOT_REQUIRED);
	printf("\r\n[TX]TCP create socket");
}

void handleRxWiFi(uint8_t* buf, uint16_t len)
{
	uint8_t subCmdId = buf[0];
	uint8_t CfmBuf[2] = {0,};
	//seq++ = buf[1];
	uint8_t i, j;
	switch (subCmdId) 
	{
		case WIFI_GET_STATUS_RSP:
			if (buf[2] == MODE_WIFI_OFF) printf("\r\nWiFi Off.");
			else {
				char val[20] = {0};

				for(i=0;i<6;i++) sprintf(val+3*i, "%02X:", buf[3+i]);
				val[strlen(val)-1] = 0;
				printf("\r\nWiFi On.  Mac: %s.  ", val);
					
				if (buf[2] == MODE_NO_NETWORK) printf("\r\nNot joined any network.");
				else if (buf[2] == MODE_STA_JOINED) printf("\r\nJoined SSID: %s", buf+9);
				else if (buf[2] == MODE_AP_STARTED) printf("\r\nAP started");
			}
			// Should post semaphore if the other thread is waiting for the cmd response. 
			// This applies for all the _RSP cases below. See testclient.cpp where the cmd is sent.
			break;

		case WIFI_JOIN_RSP:
			if (buf[2] == WIFI_SUCCESS) printf("\r\n-Join success");
			else printf("\r\n-Join fail");
			break;
				
		case WIFI_NETWORK_STATUS_IND:
			CfmBuf[0] = WIFI_NETWORK_STATUS_IND | 0x80;
			CfmBuf[1] = buf[1];
			serial_transmit(CMD_ID_SNIC, 2, (uint8_t *)CfmBuf, ACK_NOT_REQUIRED);
			if (WIFI_NETWORK_UP == buf[3]) printf("\r\n-Network UP");
			else printf("\r\n-Network Down");
			break;
				
		case WIFI_SCAN_RESULT_IND:
		{
			uint8_t cnt = buf[2];
			uint8_t i = 3;
			uint8_t ch, sec_tmp, type_tmp;
			int8_t rssi;
			uint8_t len = 32;

			if(cnt == 0) 
			{
				CfmBuf[0] = WIFI_SCAN_RESULT_IND | 0x80;
				CfmBuf[1] = buf[1];
				serial_transmit(CMD_ID_SNIC, 2, (uint8_t *)CfmBuf, ACK_NOT_REQUIRED);
				for (j = 0; j < totalscan; j++) 
				{
					printf("\r\nSSID: %20s CH: %2d RSSI: %3d Sec: ", sl[j].SSIDname, sl[j].ch, sl[j].rssi);
					switch(sl[j].sectype)	
					{
						case WIFI_SECURITY_OPEN:
							printf("%s", "OPEN\r\n");
							break;
						case WIFI_SECURITY_WEP_PSK:
							printf("%s", "WEP_PSK\r\n");
							break;
						case WIFI_SECURITY_WPA_TKIP_PSK:
							printf("%s", "WPA_TKIP_PSK\r\n");
							break;
						case WIFI_SECURITY_WPA2_AES_PSK:
							printf("%s", "WPA2_AES_PSK\r\n");
							break;
						case WIFI_SECURITY_WPA2_MIXED_PSK:
							printf("%s", "WPA2_MIXED_PSK\r\n");
							break;
						case WIFI_SECURITY_WPS_OPEN:
							printf("%s", "WPS_OPEN\r\n");
							break;
						case WIFI_SECURITY_WPS_SECURE:
							printf("%s", "WPS_SECURE\r\n");
							break;
						default:
							printf("%s", "UNKNOWN\r\n");
							break;
					}
				}
				memset(sl, 0, totalscan * sizeof(scanlist_t));
				totalscan = 0; //reset current scan result
			}
			else 
			{
				while (cnt--) 
				{
					ch = buf[i++];
					rssi = (int8_t)buf[i++];
					sec_tmp = buf[i++];
					i += 6;
					type_tmp = buf[i++];
					i += 2;
					len = (uint8_t)strlen((char*)buf + i);
					if (len > 32) 
					{
						printf("\r\nCorrupted scan list");
						break;
					}
					strcpy((char*)sl[totalscan].SSIDname, (char*)buf + i);
					sl[totalscan].ch = ch;
					sl[totalscan].rssi = 0xFFFFFF00 | rssi;
					sl[totalscan].sectype = sec_tmp;
					if (len == 0) while (buf[i] == 0) i++;
					else i += len + 1;
					totalscan++;
				}
			}
			break;
		}
		
		case WIFI_SCAN_RSP:
			if (WIFI_SUCCESS == buf[2]) printf("\r\nScan Success");
			else printf("\r\nScan Fail");
			break;
		
		case WIFI_DISCONNECT_RSP:
			if (buf[2] == WIFI_SUCCESS) printf("\r\nDisconnect Success");
			else if (buf[2] == WIFI_ERR_NOT_JOINED) printf("\r\nNot Joined!!!");
			break;
		
		case WIFI_OFF_RSP:
			if (buf[2] == WIFI_SUCCESS) printf("\r\nWi-Fi OFF Success");
			else printf("\r\nWi-Fi OFF Fail");
			break;
		
		case WIFI_ON_RSP:
			if (buf[2] == WIFI_SUCCESS) printf("\r\nWi-Fi ON Success");
			else printf("\r\nWi-Fi ON Fail code : %x", buf[2]);
			break;
		
		default:
			break;
	}
}

void handleRxSNIC(uint8_t* buf, uint16_t len)
{
	uint8_t subCmdId = buf[0];
	uint8_t CfmBuf[2] = {0,};

	//seq++ = buf[1];

	switch (subCmdId) 
	{
		case SNIC_IP_CONFIG_RSP:
			if (buf[2] == SNIC_SUCCESS) printf("\r\nIPConfig OK");
			else if (buf[2] == SNIC_NO_NETWORK) printf("\r\nIPConfig fail : No Network");
			else if (buf[2] == SNIC_NET_IF_FAIL) printf("\r\nIPConfig fail : Network IF Fail");
			else if (buf[2] == SNIC_NET_IF_NOT_UP) printf("\r\nIPConfig fail : Network IF Not Up");
			else if (buf[2] == SNIC_DHCP_START_FAIL) printf("\r\nIPConfig fail : DHCP Start Fail");
			// Should post semaphore here if the other thread is waiting for the cmd response. 
			// This applies for all the _RSP cases below. See testclient.cpp where the cmd is sent.
			break;
			
		case SNIC_GET_DHCP_INFO_RSP:
			if (buf[2] == SNIC_SUCCESS) 
			{
				printf("\r\nIP assigned as %i.%i.%i.%i ", buf[9],buf[10],buf[11],buf[12]);
				//save IP
				memcpy(&selfIP, buf+9, 4);
			}
			else printf("\r\nIP assigned");
			break;

		case SNIC_TCP_CREATE_SOCKET_RSP:
			if (buf[2] == SNIC_SUCCESS) 
			{
				mysock = buf[3];
				printf("\r\nSocket %d opened", mysock);
			}
			else printf("\r\nSocket creation failed");
			break;
		
		case SNIC_CREATE_UDP_SOCKET_RSP:
			if (buf[2] == SNIC_SUCCESS)
			{
				mysock = buf[3];
				printf("\r\nSocket %d opened", mysock);
			}
			else if (buf[2] == SNIC_CREATE_SOCKET_FAIL) printf("\r\nSocket creation failed");
			else if (buf[2] == SNIC_BIND_SOCKET_FAIL) printf("\r\nSocket binding failed");
			break;

		case SNIC_TCP_CONNECT_TO_SERVER_RSP:
			if (buf[2] == SNIC_CONNECT_TO_SERVER_PENDING);
			else printf("\r\nUnable to connect server, make sure it's within same subnet");
			break;

		case SNIC_TCP_CONNECTION_STATUS_IND:
			CfmBuf[0] = SNIC_TCP_CONNECTION_STATUS_IND | 0x80;
			CfmBuf[1] = buf[1];
			serial_transmit(CMD_ID_SNIC, 2, (uint8_t *)CfmBuf, ACK_NOT_REQUIRED);
			if (buf[2] == SNIC_CONNECTION_UP) printf("\r\nSocket connection UP");
			else if (SNIC_CONNECTION_CLOSED == buf[2]) printf("\r\nSocket %i closed", buf[3]);
			break;
			
		case SNIC_SEND_RSP:
			if (buf[2] == SNIC_SUCCESS) 
			{
				uint16_t sentsize = ((uint32_t)buf[3] << 8 | buf[4]);
				printf("\r\n%d bytes sent ", sentsize);
			}
			else if (buf[2] == SNIC_PACKET_TOO_LARGE) printf("\r\nPacket Too Large!!!");
			else if (buf[2] == SNIC_SEND_FAIL) printf("\r\nSend Fail!!!");
			break;
			
		case SNIC_UDP_SEND_FROM_SOCKET_RSP:
			if (buf[2] == SNIC_SUCCESS)
			{
				uint16_t sentsize = ((uint16_t)buf[3] << 8 | buf[4]);
				printf("\r\n%d bytes sent ", sentsize);
			}
			else if (buf[2] == SNIC_CONNECT_TO_SERVER_FAIL) printf("\r\nConnect to Sever Fail!!!");
			else if (buf[2] == SNIC_SEND_FAIL) printf("\r\nSend Fail!!!");
			break;
			
		case SNIC_CONNECTION_RECV_IND:
		{
			CfmBuf[0] = SNIC_CONNECTION_RECV_IND | 0x80;
			CfmBuf[1] = buf[1];
			serial_transmit(CMD_ID_SNIC, 2, (uint8_t *)CfmBuf, ACK_NOT_REQUIRED);
			uint16_t sentsize = ((uint16_t)buf[3] << 8 | buf[4]);
			//uint8_t sock = buf[2];
			//printf("\r\n%d bytes received from socket %d\r\n", sentsize, sock);
			printf("\r\n");
			for(uint16_t i = 0; i < sentsize; i++) printf("%c", buf[5+i]);
			printf("\r\n");
			break;
		}
			
		case SNIC_TCP_CLIENT_SOCKET_IND:
		{
			CfmBuf[0] = SNIC_TCP_CLIENT_SOCKET_IND | 0x80;
			CfmBuf[1] = buf[1];
			serial_transmit(CMD_ID_SNIC, 2, (uint8_t *)CfmBuf, ACK_NOT_REQUIRED);
			static uint8_t i;
			if(i < MAX_CONNECTION_PER_SOCK) 
			{
				client[i] = buf[3];
				i++;
				printf("\r\nAccepted connection from %i.%i.%i.%i", buf[4], buf[5], buf[6], buf[7]);
				printf("\r\nConnection socket: %d", buf[3]);
			}
			else 
			{
				i = 0;
				printf("\r\nClient limit reached!!!");
			}
			break;
		}
			
		case SNIC_UDP_START_RECV_RSP:
			if (buf[2] == SNIC_SUCCESS) printf("\r\nUDP Receive Start");
			else printf("\r\nNot Enough Memory");
			break;
			
		case SNIC_UDP_RECV_IND:
		{
			CfmBuf[0] = SNIC_UDP_RECV_IND | 0x80;
			CfmBuf[1] = buf[1];
			serial_transmit(CMD_ID_SNIC, 2, (uint8_t *)CfmBuf, ACK_NOT_REQUIRED);
			uint16_t receivesize = ((uint16_t)buf[9] << 8 | buf[10]);
			uint8_t sock = buf[2];
			printf("\r\nReceived from %i.%i.%i.%i", buf[3], buf[4], buf[5], buf[6]);
			printf("\r\n%d bytes received from socket %d ", receivesize, sock);
			break;
		}
			
		case SNIC_TCP_CREATE_CONNECTION_RSP:
			if (buf[2] == SNIC_SUCCESS)
			{
				uint16_t Bufsize = ((uint16_t)buf[3] << 8 | buf[4]);
				printf("\r\nListen start! Buf size = %d, Max client = %d", Bufsize, buf[5]);
			}
			else printf("\r\nListen socket fail!");
			break;
		
		case SNIC_CLEANUP_RSP:
			if (buf[2] == SNIC_SUCCESS) printf("\r\nCleanup Success");
			else printf("\r\nCleanup fail!");
			break;
		
		case SNIC_CLOSE_SOCKET_RSP:
			if (buf[2] == SNIC_SUCCESS) printf("\r\nSocket %d closed", mysock);
			else if(buf[2] == SNIC_CLOSE_SOCKET_FAIL) printf("\r\nSocket close fail");
			break;
		
		default :
			break;
	}
}

void handleRxGen(uint8_t* buf, uint16_t len)
{
	uint8_t subCmdId = buf[0];
	uint8_t CfmBuf[2] = {0,};

	//seq++ = buf[1];

	switch (subCmdId) 
	{
		case GEN_PWR_UP_IND:
		{
			CfmBuf[0] = GEN_PWR_UP_IND | 0x80;
			CfmBuf[1] = buf[1];
			serial_transmit(CMD_ID_SNIC, 2, (uint8_t *)CfmBuf, ACK_NOT_REQUIRED);
			uint16_t code = (uint16_t)buf[2] << 8 | buf[3];
			switch(code)
			{
				case 0x0400:
					printf("\r\nPin reset");
					break;
				case 0x0800:
					printf("\r\nPOR/PDR reset");
					break;
				case 0x1000:
					printf("\r\nSoftware reset");
					break;
				case 0x2000:
					printf("\r\nIndependent watchdog reset");
					break;
				case 0x4000:
					printf("\r\nWindow watchdog reset");
					break;
				default:
					break;
			}
			break;
		}		
		default:
			break;
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	free(TXBuf);
	TXBuf = NULL;
	//printf("\r\nSPI TX ok!!!");
}

// escape_payload_and_transmit_no_ESC
// This function is escapes the payload if required and send to the receiver
// This function returns checksum calculated over escaped payload
/*static uint8_t escape_payload_and_transmit_no_ESC(uint16_t payload_len, uint8_t *payload)
{
#if 1	// For image without checksum
	spi_dummy_tx_buffer(payload, payload_len);
	return 0;
#else	// For image with checksum implementation
	uint16_t i;
	uint8_t chksum = 0;
	spi_dummy_tx_buffer(payload, payload_len);
	for (i = 0; i < payload_len; ++i)
	{
		chksum += *payload++;
	}
	return chksum;
#endif
}*/

// calc_escaped_payload_len_no_ESC
// This function calculates length after applying ESC character over payload
// This function returns length
static uint16_t calc_escaped_payload_len_no_ESC(uint16_t payload_len, uint8_t *payload)
{
#if 0
	uint16_t i;
	uint16_t len=0;
	uint8_t c; 

	for (i = 0; i < payload_len; ++i)
	{
		c = *payload++;
		if (c == SOM_CHAR || c == EOM_CHAR || c == ESC_CHAR) 
		{
			len += 2;
		}
		else
		{	
			len ++;
		}
	}
	return len;
#else
	return payload_len;
#endif
}

//send_ack_nak
// This function sends ACK/NAK frame
static void send_ack_nak(uint8_t ack)
{
	/*uint8_t chksum=0;

	SERIAL_TX(SOM_CHAR);
	SERIAL_TX(0x80);
	chksum+=0x80;
	SERIAL_TX(0x80);
	chksum+=0x80;

	if (ack)
	{
		SERIAL_TX(0xFF);
		chksum+=0xff;
	}
	else
	{
		SERIAL_TX(0x80);
		chksum+=0x80;
	}
	SERIAL_TX(0x80|chksum);
	SERIAL_TX(EOM_CHAR);*/
}

//wait_for_ack_nak
// This function waits on a semaphore for ACK from receiver
// Once triggerred it receives data confirming to ACK frame format 
// If the cmd == ACK_CMD then it returs success else failure.
static uint8_t wait_for_ack_nak(void)
{
	// WAIT ON SEMAPHORE
	return 0;
}
