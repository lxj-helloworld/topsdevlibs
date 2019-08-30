#ifndef __ETH_H__
#define __ETH_H__
#include     <stdint.h>

typedef enum
{
    TCP,
    UDP
}PROTOTYPE;

typedef enum
{
    SERVER,
    CLIENT
}CSMODE;

typedef struct
{
    uint32_t local_ip;
    uint32_t sub_mask;
    uint32_t gateway;
    uint8_t  protocol_type; //0: TCP, 1: UDP
    uint8_t  mode;          //0: server mode, 1: client mode
    uint16_t port;          //if server, the port is local port; if client, the port is remote port
    uint32_t remote_ip;     //client mode only
}NET_CONFIG;

void setLocalIp(uint32_t local_ip);
void setSubMask(uint32_t mask);
void setGateway(uint32_t gateway);
void setProtocolType(uint8_t protocol_type);
void setNetMode(uint8_t mode);
void setNetPort(uint16_t port);
void setRemoteIp(uint32_t remote_ip);

uint32_t getLocalIp();
uint32_t getSubMask();
uint32_t getGateway();
uint8_t getProtocolType();
uint8_t getNetMode();
uint16_t getNetPort();
uint32_t getRemoteIp();
int netOpen();
void netClose();
int netIsOpen();
int netCheckConnected();
int netWrite(void *buf, int buf_len);
int netRead(void *buf, int buf_len);
int netReadBlock(void *buf, int buf_len);

#endif
