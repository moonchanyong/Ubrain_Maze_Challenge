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
//

#include "..\Common\wifi.h"
#include "main.h"

#ifndef _SPI_DRIVER_H
#define _SPI_DRIVER_H

#ifdef __cplusplus
 extern "C" {
#endif

//Maximum payload len
#define MAX_PAYLOAD_LEN 1484
#define MAX_RX_BUFFER_SIZE 50
#define MAX_TCP_SEND_DATA_LEN 1460
#define MAX_UDP_SEND_DATA_LEN 1472
	 
/* Exported types ------------------------------------------------------------*/
typedef uint8_t (*cmd_proc_fn_t)(uint8_t commandId, uint16_t paramLength, uint8_t *params);

typedef enum {
	MODE_WIFI_OFF,
	MODE_NO_NETWORK,
	MODE_STA_JOINED,
	MODE_AP_STARTED,
	MODE_SNIC_INIT_NOT_DONE,
	MODE_SNIC_INIT_DONE,
	/* Non-mode special values */
	MODE_LIST_END,
	MODE_ANY,
} serial_wifi_mode_t;

typedef enum {
	SNIC_SUCCESS=0,
	SNIC_FAIL,
	SNIC_INIT_FAIL,
	SNIC_CLEANUP_FAIL,
	SNIC_GETADDRINFO_FAIL,
	SNIC_CREATE_SOCKET_FAIL,
	SNIC_BIND_SOCKET_FAIL,
	SNIC_LISTEN_SOCKET_FAIL,
	SNIC_ACCEPT_SOCKET_FAIL,
	SNIC_PARTIAL_CLOSE_FAIL,
	SNIC_CONNECTION_PARTIALLY_CLOSED = 0x0A,
	SNIC_CONNECTION_CLOSED,
	SNIC_CLOSE_SOCKET_FAIL,
	SNIC_PACKET_TOO_LARGE,
	SNIC_SEND_FAIL,
	SNIC_CONNECT_TO_SERVER_FAIL,
	SNIC_NOT_ENOUGH_MEMORY = 0x10,
	SNIC_TIMEOUT,
	SNIC_CONNECTION_UP,
	SNIC_GETSOCKOPT_FAIL,
	SNIC_SETSOCKOPT_FAIL,
	SNIC_INVALID_ARGUMENT,
	SNIC_SEND_ARP_FAIL,
	SNIC_INVALID_SOCKET,
	SNIC_CONNECT_TO_SERVER_PENDING,
	SNIC_SOCKET_NOT_BOUND,
	SNIC_SOCKET_NOT_CONNECTED,
	SNIC_NO_NETWORK = 0x20,
	SNIC_INIT_NOT_DONE,
	SNIC_NET_IF_FAIL,
	SNIC_NET_IF_NOT_UP,
	SNIC_DHCP_START_FAIL,
} SNIC_return_code_e;

enum {
	WIFI_SUCCESS,
	WIFI_FAIL,
	WIFI_ERR_NOT_JOINED = 0x06,
	WIFI_NETWORK_UP = 0x10,
	WIFI_NETWORK_DOWN,
};

typedef enum {
	WIFI_ON_REQ = 0,
	WIFI_OFF_REQ,
	WIFI_JOIN_REQ,
	WIFI_DISCONNECT_REQ,
	WIFI_GET_STATUS_REQ,
	WIFI_SCAN_REQ,
	WIFI_GET_STA_RSSI_REQ,

	WIFI_NETWORK_STATUS_IND = 0x10,
	WIFI_SCAN_RESULT_IND,
	WIFI_RSSI_IND,

	WIFI_ON_RSP = 0x80,
	WIFI_OFF_RSP,
	WIFI_JOIN_RSP,
	WIFI_DISCONNECT_RSP,
	WIFI_GET_STATUS_RSP,
	WIFI_SCAN_RSP,
	WIFI_GET_STA_RSSI_RSP,
}WIFI_subcmd_id_e;

typedef enum {
	GEN_PWR_UP_IND = 0X00,
	
	GEN_SLEEP_CFG_REQ = 0x05,
	
	GEN_FW_VER_GET_REQ = 0x08,
	GEN_RESTORE_REQ,
	GEN_RESET_REQ,
}GEN_subcmd_id_e;


typedef enum {
	CMD_ID_NACK = 0,  // reservered
	CMD_ID_GEN,
	CMD_ID_WIFI = 0x50,
	CMD_ID_SNIC = 0x70,
	CMD_ID_ACK = 0x7F,
}cmd_id_e;

typedef enum {
	SNIC_INIT_REQ = 0,  
	SNIC_CLEANUP_REQ,
	SNIC_SEND_FROM_SOCKET_REQ,
	SNIC_CLOSE_SOCKET_REQ,
	SNIC_SOCKET_PARTIAL_CLOSE_REQ,
	SNIC_GETSOCKOPT_REQ, 
	SNIC_SETSOCKOPT_REQ,
	SNIC_SOCKET_GETNAME_REQ,
	SNIC_SEND_ARP_REQ,
	SNIC_GET_DHCP_INFO_REQ,
	SNIC_RESOLVE_NAME_REQ,
	SNIC_IP_CONFIG_REQ,
	SNIC_DATA_IND_ACK_CONFIG_REQ,

	SNIC_TCP_CREATE_SOCKET_REQ = 0x10,
	SNIC_TCP_CREATE_CONNECTION_REQ,
	SNIC_TCP_CONNECT_TO_SERVER_REQ, 
	
	SNIC_UDP_CREATE_SOCKET_REQ,  
	SNIC_UDP_START_RECV_REQ,
	SNIC_UDP_SIMPLE_SEND_REQ,
	SNIC_UDP_SEND_FROM_SOCKET_REQ,
	SNIC_HTTP_REQ,
	SNIC_HTTP_MORE_REQ,
	SNIC_HTTPS_REQ,
	SNIC_TCP_CREATE_ADV_TLS_SOCKET_REQ,
	SNIC_TCP_CREAET_SIMPLE_TLS_SOCKET_REQ,

	SNIC_TCP_CONNECTION_STATUS_IND = 0x20, 
	SNIC_TCP_CLIENT_SOCKET_IND,    
	SNIC_CONNECTION_RECV_IND,
	SNIC_UDP_RECV_IND,
	SNIC_ARP_REPLY_IND,
	SNIC_HTTP_RSP_IND,

	SNIC_CLEANUP_RSP = 0x81,
	SNIC_SEND_RSP,
	SNIC_CLOSE_SOCKET_RSP,
	SNIC_GET_DHCP_INFO_RSP = 0x89,
	SNIC_IP_CONFIG_RSP = 0x8B,
	SNIC_TCP_CREATE_SOCKET_RSP = 0x90,
	SNIC_TCP_CREATE_CONNECTION_RSP,
	SNIC_TCP_CONNECT_TO_SERVER_RSP,
	SNIC_CREATE_UDP_SOCKET_RSP,
	SNIC_UDP_START_RECV_RSP,
	SNIC_UDP_SEND_FROM_SOCKET_RSP = 0x96,

}SNIC_subcmd_id_e;

enum {
	WIFI_SECURITY_OPEN = 0x0,
	WIFI_SECURITY_WEP_PSK = 0x1,
	WIFI_SECURITY_WPA_TKIP_PSK = 0x02,
	WIFI_SECURITY_WPA2_AES_PSK = 0x04,
	WIFI_SECURITY_WPA2_MIXED_PSK = 0x06,
	WIFI_SECURITY_WPS_OPEN = 0x08,
	WIFI_SECURITY_WPS_SECURE = 0x0C,
};

typedef struct{
	int ch;
	int rssi;
	int sectype;
	char SSIDname[33];
} scanlist_t ;

typedef struct {
	int index;
	int length;
} indices;

typedef struct {
	unsigned short available;
	unsigned short payload_len;
	unsigned char ack_reqd;
	unsigned char cmd_id;
	unsigned char rx_payload[MAX_PAYLOAD_LEN];
	unsigned char chksum;
	unsigned char ackOk;
}rx_info_t;

typedef enum 
{
	IDLE,
	SOM_RECD,
	LEN_RECD,
	ACK_SEQ_RECD,
	CMD_RECD,
	PAYLAD_RX,
	PAYLAD_RX_ESC,
	CHKSUM_RECD,
	EOM_RECD,
	WAIT_FOR_ACK_NAK,
}serial_rx_state_t;

// Special character defines
#define SOM_CHAR 0x02
#define EOM_CHAR 0x04
#define ESC_CHAR 0x10
#define ACK_CMD	0x7F
#define NAK_CMD 0x00

#define ACK_REQUIRED 1
#define ACK_NOT_REQUIRED 0

#define NUM_RX_BUF 10

// Error code
#define SER_PROC_SUCCESS        0
#define SER_PROC_INVALID_LEN    -1
#define SER_PROC_GENERAL_ERR    -2
#define SER_PROC_ACK_NAK_ERR    -3

#define PORT_NONE 0
#define CMD_ERROR -1
#define SUB_CMD_RESP_MASK 0x80 // Bit 7: 0 for original command, 1 for response 
#define MAX_CONNECTION_PER_SOCK 4   // max connection per listening socket

#define INADDR_NONE	0xFFFFFFFF

/* Exported macro ------------------------------------------------------------*/
/* SN8200 ON */
#define SN8200_ON()       BSP_IO_WritePin(IO_I2C_ADDRESS, IO_PIN_0, SET)
/* SN8200 OFF */
#define SN8200_OFF()      BSP_IO_WritePin(IO_I2C_ADDRESS, IO_PIN_0, RESET)

#define swap16(x) (((x & 0x00FF) << 8) | ((x & 0xFF00) >> 8))
#define swap32(x) (((x & 0x000000FF) << 24) | ((x & 0xFF000000) >> 24) | ((x & 0x00FF0000) >> 8) | ((x & 0x0000FF00) << 8))

/* Exported functions ------------------------------------------------------- */
uint32_t inet_addr(const char *cp);
void sn8200_Start(void);

void serial_ack_nak_recd(uint8_t ack);
void handle_rx_frame(void);
uint8_t process_rx_frame(void);
int8_t serial_transmit(uint8_t cmd_id, uint16_t payload_len, uint8_t *payload, uint8_t ack_required);
uint16_t rx_process_char_no_ESC(uint8_t *rx_ch);
uint8_t sci_ser_cmd_proc(uint8_t commandId, uint16_t paramLength, uint8_t *params);
uint8_t serial_transmit_rx_init(cmd_proc_fn_t cmd_processor);
uint16_t sci_serial_redirect(uint8_t *ch);
int8_t no_ESC_transmit(uint8_t cmd_id, uint16_t payload_len, uint8_t *payload, uint8_t ack_required, uint16_t len);
int8_t getTCPinfo(void);
int8_t setTCPinfo(void);
void Leave(void);
void GetStatus(void);
void WifiOn(void);
void WifiOff(void);
void WifiDisconn(void);
void WifiScan(void);
void WifiJoin(void);
void SnicInit(void);
void SnicCleanup(void);
void SnicIPConfig(void);
void SnicGetDhcp(void);
void tcpCreateConnection(uint8_t shortSock, uint8_t maxClient);
void udpStartRecv(uint8_t sock);
void udpSendFromSock(uint8_t sock, uint32_t ip, uint16_t port, uint8_t *sendBuf, uint16_t len, uint16_t index, uint8_t conMode);
void sendFromSock(uint8_t shortSocket, uint8_t * sendBuf, uint16_t len, uint16_t index);
void closeSocket(uint8_t shortSocket);
void tcpConnectToServer(uint8_t shortSock, uint32_t ip, uint16_t port, uint8_t timeout);
void udpCreateSocket(uint8_t bindOption, uint32_t ip, uint16_t port);
void tcpCreateSocket(uint8_t bindOption, uint32_t localIp, uint16_t port);
void handleRxWiFi(uint8_t* buf, uint16_t len);
void handleRxSNIC(uint8_t* buf, uint16_t len);
void handleRxGen(uint8_t* buf, uint16_t len);

/* IO driver structure */
extern WiFi_DrvTypeDef sn8200_wifi_drv;

#ifdef __cplusplus
}
#endif

#endif
