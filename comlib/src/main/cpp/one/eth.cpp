/********************************** (C) COPYRIGHT *********************************
* File Name       : eth.cpp
* Author          : Wang Lei
* Version         : V1.1
* Date            : 2019/8/26
* Description     :
* 
**********************************************************************************/

#include     <stdio.h>
#include     <stdlib.h>
#include     <unistd.h>
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>
#include     <termios.h>
#include     <errno.h>
#include     <pthread.h>
#include     <string.h>
#include     <stdint.h>
#include     <sched.h>
#include     <sys/ioctl.h>
#include     <linux/spi/spidev.h>
#include     <android/log.h>
#include     "CH395INC.h"
#include     "eth.h"

#define ADB_DEBUG
#ifndef ADB_DEBUG
#define LOG_TAG "CH395"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG ,__VA_ARGS__) // 定义LOGD类型
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG ,__VA_ARGS__) // 定义LOGI类型
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,LOG_TAG ,__VA_ARGS__) // 定义LOGW类型
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG ,__VA_ARGS__) // 定义LOGE类型
#define LOGF(...) __android_log_print(ANDROID_LOG_FATAL,LOG_TAG,__VA_ARGS__) // 定义LOGF类型
#else
#define LOGD(...) printf(__VA_ARGS__)
#define LOGI(...) printf(__VA_ARGS__)
#define LOGW(...) printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)
#define LOGF(...) printf(__VA_ARGS__)
#endif

#define NOTUSED(V)         ((void) V)

#define SPI_SEND_BUF_SIZE 4096
#define SPI_RECV_BUF_SIZE 4096

#define DEF_KEEP_LIVE_IDLE                           (15*1000)        /* 空闲时间 */
#define DEF_KEEP_LIVE_PERIOD                         (20*1000)        /* 间隔为15秒，发送一次KEEPLIVE数据包 */                  
#define DEF_KEEP_LIVE_CNT                            200                /* 重试次数  */

#define CH395_SEND_BUF_LEN (2048)

static const char *spi_dev =  "/dev/spidev5.0";
static uint8_t spi_mode = 1;
static uint8_t bits = 8;
static uint32_t speed = 3000000;
static int spi_fd = -1;
static int net_run_flag   = 0;
static int send_busy_flag = 0;
static uint8_t send_buf[SPI_SEND_BUF_SIZE];
static uint8_t recv_buf[SPI_RECV_BUF_SIZE];
static struct spi_ioc_transfer tr;


static NET_CONFIG config = {
        0xc0a86464,     // local ip: 192.168.100.100
        0xffffff00,     // mask: 255.255.255.0
        0xc0a86401,     // gateway: 192.168.100.1
        TCP,            // protocol: TCP
        SERVER,         // mode: server
        9999,           // port: 9999
        0};

typedef struct
{
    uint8_t *buf;
    uint8_t *put_cur;
    uint8_t *get_cur;
    uint8_t  full_flag;
    int      buf_size;
    pthread_mutex_t mutex;
}RING_BUF;

RING_BUF buf[2];
RING_BUF *ring_wr_buf = &buf[0];
RING_BUF *ring_rd_buf = &buf[1];

void init_ring_buf(void)
{
    ring_wr_buf->buf       = send_buf;
    ring_wr_buf->get_cur   = &send_buf[0];
    ring_wr_buf->put_cur   = &send_buf[0];
    ring_wr_buf->buf_size  = SPI_SEND_BUF_SIZE;
    ring_wr_buf->full_flag = 0;
    ring_wr_buf->mutex     = PTHREAD_MUTEX_INITIALIZER;

    ring_rd_buf->buf       = recv_buf;
    ring_rd_buf->get_cur   = &recv_buf[0];
    ring_rd_buf->put_cur   = &recv_buf[0];
    ring_rd_buf->buf_size  = SPI_RECV_BUF_SIZE;
    ring_rd_buf->full_flag = 0;
    ring_rd_buf->mutex     = PTHREAD_MUTEX_INITIALIZER;
}

void setLocalIp(uint32_t local_ip)
{
    if (config.local_ip != local_ip)
    {
        config.local_ip = local_ip;
    }
}
void setSubMask(uint32_t mask)
{
    if (config.sub_mask != mask)
    {
        config.sub_mask = mask;
    }
}
void setGateway(uint32_t gateway)
{
    if (config.gateway != gateway)
    {
        config.gateway = gateway;
    }
}
void setProtocolType(uint8_t protocol_type)
{
    if (config.protocol_type != protocol_type)
    {
        config.protocol_type = protocol_type;
    }
}
void setNetMode(uint8_t mode)
{
    if (config.mode != mode)
    {
        config.mode = mode;
    }
}
void setNetPort(uint16_t port)
{
    if (config.port != port)
    {
        config.port = port;
    }
}
void setRemoteIp(uint32_t remote_ip)
{
    if (config.remote_ip != remote_ip)
    {
        config.remote_ip = remote_ip;
    }
}

uint32_t getLocalIp()
{
    return config.local_ip;
}

uint32_t getSubMask()
{
    return config.sub_mask;
}
uint32_t getGateway()
{
    return config.gateway;
}
uint8_t getProtocolType()
{
    return config.protocol_type;
}
uint8_t getNetMode()
{
    return config.mode;
}
uint16_t getNetPort()
{
    return config.port;
}
uint32_t getRemoteIp()
{
    return config.remote_ip;
}

void netGpioPowerOn()
{
    system("echo 1 > /sys/class/gpio/gpio56/value");
    system("echo 1 > /sys/class/gpio/gpio91/value");
    system("echo 1 > /sys/class/gpio/gpio62/value");
}

void netGpioPowerOff()
{
    system("echo 0 > /sys/class/gpio/gpio56/value");
    system("echo 0 > /sys/class/gpio/gpio91/value");
    system("echo 0 > /sys/class/gpio/gpio62/value");
}

void netCsOn()
{
    system("echo 0 > /sys/class/gpio/gpio18/value");
}

void netCsOff()
{
    system("echo 1 > /sys/class/gpio/gpio18/value");
}

uint8_t netCheckExist(uint8_t data)
{
    send_buf[0] = CMD_CHECK_EXIST;
    send_buf[1] = data;
    send_buf[2] = 0;
    tr.len = 3;

    netCsOn();
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return recv_buf[2];
}

int netGetCMDStatus(uint8_t *status)
{
    int ret;

    send_buf[0] = CMD_GET_CMD_STATUS;
    send_buf[1] = 0;
    tr.len = 2;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    *status = recv_buf[1];

    return ret;
}

int netGetGlobINTStatus(uint8_t *status)
{
    int ret;

    send_buf[0] = CMD_GET_GLOB_INT_STATUS;
    send_buf[1] = 0;
    tr.len = 2;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    *status = recv_buf[1];

    return ret;
}

int netGetGlobINTStatusAll(uint16_t *status)
{
    int ret;

    send_buf[0] = CMD_GET_GLOB_INT_STATUS_ALL;
    send_buf[1] = 0;
    send_buf[2] = 0;
    tr.len = 3;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    *status = (recv_buf[2]>>8) + recv_buf[1];

    return ret;
}

int netGetSocketINTStatus(uint8_t socket_index, uint8_t *status)
{
    int ret;

    if(socket_index > 7)
    {
        return -1;
    }

    send_buf[0] = CMD_GET_INT_STATUS_SN;
    send_buf[1] = socket_index;
    send_buf[2] = 0;
    tr.len = 3;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    *status = recv_buf[2];

    return ret;
}

int netSetLocalIp(uint32_t local_ip)
{
    int ret;
    send_buf[0] = CMD_SET_IP_ADDR;
    send_buf[1] = (local_ip>>24)&0xff;
    send_buf[2] = (local_ip>>16)&0xff;
    send_buf[3] = (local_ip>>8)&0xff;
    send_buf[4] = local_ip&0xff;
    tr.len = 5;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();
    if (ret < 0)
    {
        LOGE("Set LocalIp failed\n");
        return -1;
    }

    return ret;
}

int netSetSubMask(uint32_t mask_ip)
{
    int ret;

    send_buf[0] = CMD_SET_MASK_ADDR;
    send_buf[1] = (mask_ip>>24)&0xff;
    send_buf[2] = (mask_ip>>16)&0xff;
    send_buf[3] = (mask_ip>>8)&0xff;
    send_buf[4] = mask_ip&0xff;
    tr.len = 5;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();
    if (ret < 0)
    {
        LOGE("Set SubMask failed\n");
        return -1;
    }

    return ret;
}

int netSetGateway(uint32_t gateway)
{
    int ret;

    send_buf[0] = CMD_SET_GWIP_ADDR;
    send_buf[1] = (gateway>>24)&0xff;
    send_buf[2] = (gateway>>16)&0xff;
    send_buf[3] = (gateway>>8)&0xff;
    send_buf[4] = gateway&0xff;
    tr.len = 5;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();
    if (ret < 0)
    {
        LOGE("Set Gateway failed\n");
        return -1;
    }

    return ret;
}

int netSetSourcePort(uint8_t socket_index, uint16_t port)
{
    int ret;

    if(socket_index > 7)
    {
        LOGE("Socket index error\n");
        return -1;
    }
    send_buf[0] = CMD_SET_SOUR_PORT_SN;
    send_buf[1] = socket_index;
    send_buf[2] = port&0xff;
    send_buf[3] = (port>>8)&0xff;
    tr.len = 4;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();
    if (ret < 0)
    {
        LOGE("Set SourcePort failed\n");
        return -1;
    }

    return ret;
}

int netSetRemoteIp(uint8_t socket_index, uint32_t remote)
{
    int ret;

    if(socket_index > 7)
    {
        LOGE("Socket index error\n");
        return -1;
    }
    send_buf[0] = CMD_SET_IP_ADDR_SN;
    send_buf[1] = socket_index;
    send_buf[2] = (remote>>24)&0xff;
    send_buf[3] = (remote>>16)&0xff;
    send_buf[4] = (remote>>8)&0xff;
    send_buf[5] = remote&0xff;
    tr.len = 6;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();
    if (ret < 0)
    {
        LOGE("Set RemoteIp failed\n");
        return -1;
    }

    return ret;
}

int netSetRemotePort(uint8_t socket_index, uint16_t port)
{
    int ret;

    if(socket_index > 7)
    {
        LOGE("Socket index error\n");
        return -1;
    }
    send_buf[0] = CMD_SET_DES_PORT_SN;
    send_buf[1] = socket_index;
    send_buf[2] = port&0xff;
    send_buf[3] = (port>>8)&0xff;
    tr.len = 4;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();
    if (ret < 0)
    {
        LOGE("Set RemotePort failed\n");
        return -1;
    }

    return ret;
}

int netSetProtoType(uint8_t socket_index, uint8_t type)
{
    int ret;

    if(socket_index > 7)
    {
        LOGE("Socket index error\n");
        return -1;
    }

    send_buf[0] = CMD_SET_PROTO_TYPE_SN;
    send_buf[1] = socket_index;
    if(TCP == type)
    {
        send_buf[2] = 0x3;
    }
    else if(UDP == type)
    {
        send_buf[2] = 0x2;
    }
    else
    {
        return -1;
    }
    tr.len = 3;
    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();
    if (ret < 0)
    {
        LOGE("Set ProtoType failed\n");
        return -1;
    }
    return ret;
}

int netGetConf(void)
{
    int ret;

    send_buf[0] = CMD_GET_IP_INF;
    tr.len = 1;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

    memset(send_buf, 0, 20);
    tr.len = 20;
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    LOGI("IPAddr : %x.%x.%x.%x\n", recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3]);
    LOGI("Gateway: %x.%x.%x.%x\n", recv_buf[4], recv_buf[5], recv_buf[6], recv_buf[7]);
    LOGI("Mask   : %x.%x.%x.%x\n", recv_buf[8], recv_buf[9], recv_buf[10], recv_buf[11]);

    return ret;
}

int netCheckCH395Status(void)
{
    int ret;

    send_buf[0] = CMD_CHECK_EXIST;
    send_buf[1] = 0x55;
    send_buf[2] = 0;
    tr.len = 3;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    if(0xAA == recv_buf[2])
    {
        ret = 0;
    }
    LOGI("Check status ret : %d\n", ret);
    return ret;
}

int netInit()
{
    int ret;
    uint8_t status;

    send_buf[0] = CMD_INIT_CH395;
    tr.len = 1;
    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();
    if (ret < 0)
    {
        LOGE("Init ch395 failed\n");
        return -1;
    }

    usleep(350000);
    get_net_init_status:
    netGetCMDStatus(&status);
    if(CH395_ERR_BUSY == status)
    {
        LOGI("Get net init busy\n");
        goto get_net_init_status;
    }
    else
    {
        return status;
    }
}

int netOpenSocket(uint8_t socket_index)
{
    int ret;
    uint8_t status;

    if(socket_index > 7)
    {
        LOGE("Socket index error\n");
        return -1;
    }

    send_buf[0] = CMD_OPEN_SOCKET_SN;
    send_buf[1] = socket_index;
    tr.len = 2;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    get_socket_open_status:
    usleep(2500);
    netGetCMDStatus(&status);
    if(CH395_ERR_BUSY == status)
    {
        LOGI("Open socket busy\n");
        goto get_socket_open_status;
    }
    else
    {
        return status;
    }
}

int netCloseSocket(uint8_t socket_index)
{
    int ret;

    send_buf[0] = CMD_CLOSE_SOCKET_SN;
    send_buf[1] = socket_index;
    tr.len = 2;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}

int netTCPConnect(uint8_t socket_index)
{
    int ret;
    uint8_t status;

    if(socket_index > 7)
    {
        LOGE("Socket index error\n");
        return -1;
    }

    send_buf[0] = CMD_TCP_CONNECT_SN;
    send_buf[1] = socket_index;
    tr.len = 2;
    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    get_tcp_connect_status:
    usleep(2500);
    netGetCMDStatus(&status);
    if(CH395_ERR_BUSY == status)
    {
        LOGI("TCP connect busy\n");
        goto get_tcp_connect_status;
    }
    else
    {
        return status;
    }
}

int netTCPDisconnect(uint8_t socket_index)
{
    int ret;
    uint8_t status;

    if(socket_index > 7)
    {
        LOGE("Socket index error\n");
        return -1;
    }

    send_buf[0] = CMD_TCP_DISNCONNECT_SN;
    send_buf[1] = socket_index;
    tr.len = 2;
    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    get_tcp_disconnect_status:
    usleep(2500);
    netGetCMDStatus(&status);
    if(CH395_ERR_BUSY == status)
    {
        LOGI("TCP connect busy\n");
        goto get_tcp_disconnect_status;
    }
    else
    {
        return status;
    }
}

int netTCPListen(uint8_t socket_index)
{
    int ret;
    uint8_t status;

    send_buf[0] = CMD_TCP_LISTEN_SN;
    send_buf[1] = socket_index;
    tr.len = 2;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    get_listen_status:
    usleep(2500);
    netGetCMDStatus(&status);
    if(CH395_ERR_BUSY == status)
    {
        LOGI("TCP listen busy\n");
        goto get_listen_status;
    }
    else
    {
        return status;
    }
}

int netPingEn(uint8_t en)
{
    int ret;

    send_buf[0] = CMD_PING_ENABLE;
    if(1 == en) {
        send_buf[1] = 1;
    }
    else if(0 == en) {
        send_buf[1] = 0;
    }
    else {
        return -1;
    }

    tr.len = 2;
    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}

int netGetSocketStatus(uint8_t socket_index, uint16_t *status)
{
    int ret;

    if(socket_index > 7)
    {
        return -1;
    }

    send_buf[0] = CMD_GET_SOCKET_STATUS_SN;
    send_buf[1] = socket_index;
    send_buf[2] = 0;
    send_buf[3] = 0;

    tr.len = 4;
    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    *status = (recv_buf[3]>>8) + recv_buf[2];
    return ret;
}

uint8_t netGetPHYStatus(void)
{
    send_buf[0] = CMD_GET_PHY_STATUS;
    send_buf[1] = 0;
    tr.len = 2;

    netCsOn();
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return recv_buf[1];
}

uint16_t netGetRecvLen(uint8_t socket_index)
{
    int ret;
    uint16_t len;

    send_buf[0] = CMD_GET_RECV_LEN_SN;
    send_buf[1] = socket_index;
    send_buf[2] = 0;
    send_buf[3] = 0;
    tr.len = 4;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    //LOGI("RECV   +++   %d   %d\n", recv_buf[2], recv_buf[3]);
    len = (uint16_t)(recv_buf[3]<<8) + recv_buf[2];
    return len;
}

uint16_t netGetRecvBuf(uint8_t socket_index, uint16_t read_len, uint8_t *buf)
{
    int ret;
    uint16_t len, recv_len;

    recv_len = netGetRecvLen(socket_index);
    len = read_len < recv_len ? read_len : recv_len;

    send_buf[0] = CMD_READ_RECV_BUF_SN;
    send_buf[1] = socket_index;
    send_buf[2] = len&0xff;
    send_buf[3] = len>>8;
    memset(&send_buf[4], 0, len);
    tr.len = 4 + len;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    memcpy(buf, &recv_buf[4], len);
    return len;
}


int netClearRecvBuf(uint8_t socket_index)
{
    int ret;

    send_buf[0] = CMD_CLEAR_RECV_BUF_SN;
    send_buf[1] = socket_index;
    tr.len = 2;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}

/********************************************************************************
* Function Name  : netGetUnreachIPPT
* Description    : 获取不可达信息 (IP,Port,Protocol Type)
* Input          : list 保存获取到的不可达
                        第1个字节为不可达代码，请参考 不可达代码(CH395INC.H)
                        第2个字节为IP包协议类型
                        第3-4字节为端口号
                        第4-8字节为IP地址
* Output         : None
* Return         : None
*******************************************************************************/
int netGetUnreachIPPT(uint8_t *buf)
{
    int ret;

    send_buf[0] = CMD_GET_UNREACH_IPPORT;
    memset(&send_buf[1], 0, 8);
    tr.len = 9;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    memcpy(buf, &recv_buf[1], 8);
    return ret;
}

int netSetKeepliveOnOff(uint8_t socket_index, uint8_t enable)
{
    int ret;

    send_buf[0] = CMD_SET_KEEP_LIVE_SN;
    send_buf[1] = socket_index;
    send_buf[1] = enable;
    tr.len = 3;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}

int netSetKeepliveCNT(uint8_t cnt)
{
    int ret;

    send_buf[0] = CMD_SET_KEEP_LIVE_CNT;
    send_buf[1] = cnt;
    tr.len = 2;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}

int netSetKeepliveIDLE(uint32_t time)
{
    int ret;

    send_buf[0] = CMD_SET_KEEP_LIVE_IDLE;
    send_buf[1] = time&0xff;
    send_buf[2] = (time>>8)&0xff;
    send_buf[3] = (time>>16)&0xff;
    send_buf[4] = (time>>24)&0xff;
    tr.len = 5;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}

int netSetKeepliveINTVL(uint32_t time)
{
    int ret;

    send_buf[0] = CMD_SET_KEEP_LIVE_INTVL;
    send_buf[1] = time&0xff;
    send_buf[2] = (time>>8)&0xff;
    send_buf[3] = (time>>16)&0xff;
    send_buf[4] = (time>>24)&0xff;
    tr.len = 5;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}

void keeplive_set(void)
{
    netSetKeepliveCNT(DEF_KEEP_LIVE_CNT);
    //netSetKeepliveIDLE(DEF_KEEP_LIVE_IDLE);
    //netSetKeepliveINTVL(DEF_KEEP_LIVE_PERIOD);
}

int netSetTTL(uint8_t socket_index, uint8_t TTL)
{
    int ret;

    if(TTL > 128)
    {
        LOGI("TTL value too big\n");
        return -1;
    }

    send_buf[0] = CMD_SET_TTL;
    send_buf[1] = socket_index;
    send_buf[2] = TTL;
    tr.len = 3;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}


int netGetRemoteIPP(uint8_t socket_index, uint32_t *ip, uint16_t *port)
{
    int ret;

    send_buf[0] = CMD_GET_REMOT_IPP_SN;
    send_buf[1] = socket_index;
    memset(&send_buf[2], 0, 6);
    tr.len = 8;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    *ip = (uint32_t)recv_buf[2] + (uint32_t)(recv_buf[3]<<8) + 
          (uint32_t)(recv_buf[4]<<16) + (uint32_t)(recv_buf[5]<<24);

    *port = (uint16_t)recv_buf[6] + (uint16_t)(recv_buf[7]<<8);

    return ret;
}

int netSendBuf(uint8_t socket_index, uint8_t *buf, uint16_t length)
{
    int ret;
    int i;

    send_buf[0] = CMD_WRITE_SEND_BUF_SN;
    send_buf[1] = socket_index;
    send_buf[2] = length&0xff;
    send_buf[3] = (length>>8)&0xff;
    memcpy(&send_buf[4], buf, length);
    tr.len = 4 + length;

    netCsOn();
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    netCsOff();

    return ret;
}

/*获取环形buffer中数据个数*/
int getRingDataSize(RING_BUF *ring)
{
    //Don't lock
    int size;
    if(NULL == ring) {
        LOGE("Null pointer\n");
        exit(1);
    }

    if(ring->full_flag) {
        return ring->buf_size;
    }

    if(ring->put_cur >= ring->get_cur) {
        size = ring->put_cur - ring->get_cur;
    }
    else {
        size = ring->buf_size + (ring->put_cur - ring->get_cur);
    }

    return size;
}


int getRingFreeSize(RING_BUF *ring)
{
    //Don't lock
    int size;

    size = ring->buf_size - getRingDataSize(ring);

    return size;
}

int putDataToRingBuf(RING_BUF *ring, uint8_t *data, int data_size)
{
    int tmp;

    if(NULL == ring || NULL == data) {
        return -1;
    }

    pthread_mutex_lock(&ring_wr_buf->mutex);
    if(getRingFreeSize(ring) < data_size) {
        LOGI("Ring buf is full\n");
        return -1;
    }
    if(data_size > ring->buf_size || 0 == data_size) {
        LOGE("Bad data size %d\n", data_size);
        return -1;
    }

    if(ring->put_cur >= ring->get_cur)
    {
        if(ring->buf + ring->buf_size - ring->put_cur >= data_size) {
            memcpy(ring->put_cur, data, data_size);
            ring->put_cur += data_size;
        }
        else {//后半截空闲buffer不够用，需要用前半段
            tmp = ring->buf + ring->buf_size - ring->put_cur;//后半截空闲buffer长度
            memcpy(ring->put_cur, data, tmp);
            memcpy(ring->buf, data + tmp, data_size - tmp);
            ring->put_cur = ring->buf + (data_size - tmp);
        }
    }
    else//put_cur < get_cur
    {
        memcpy(ring->put_cur, data, data_size);
        ring->put_cur += data_size;
    }

    if(ring->put_cur == ring->get_cur) {
        LOGD("Ring is full\n");
        ring->full_flag = 1;
    }
    pthread_mutex_unlock(&ring_wr_buf->mutex);

    return 0;
}

int getDataFromRingBuf(RING_BUF *ring, uint8_t *buf, int buf_len)
{
    int get_size;
    int tmp;

    if(NULL == ring || NULL == buf) {
        return -1;
    }

    pthread_mutex_lock(&ring_wr_buf->mutex);
    get_size = getRingDataSize(ring) < buf_len? getRingDataSize(ring) : buf_len;

    if(ring->put_cur > ring->get_cur)//Put指针在Get之后
    {
        memcpy(buf, ring->get_cur, get_size);
        ring->get_cur += get_size;
    }
    else//Put指针在Get指针之前或者两者相同
    {
        tmp = ring->buf + ring->buf_size - ring->get_cur;//get指针后半截长度
        if(tmp >= get_size) {//仅获取get指针的后半截
            memcpy(buf, ring->get_cur, get_size);
            ring->get_cur += get_size;
        }
        else {//获取get指针的后半截加前半段中的一截
            memcpy(buf, ring->get_cur, tmp);
            memcpy(buf + tmp, ring->buf, get_size - tmp);
            ring->get_cur = ring->buf + get_size - tmp;
        }
        if(ring->full_flag) {
            ring->full_flag = 0;
        }
    }
    pthread_mutex_unlock(&ring_wr_buf->mutex);

    return get_size;
}

/*ch395 socket中断函数*/
void ch395SocketInterrupt(uint8_t socket_index)
{
    uint32_t remoteIP;
    uint16_t remotePort;
    uint8_t  socket_int;
    uint16_t len;
    uint8_t buf[512];
    int loop;

    netGetSocketINTStatus(socket_index, &socket_int);
    LOGI("Socket INT  :  %d\n", socket_int);

    if(socket_int & SINT_STAT_SENBUF_FREE)                       /* 发送缓冲区空闲，可以继续写入要发送的数据 */
    {
        LOGI("Unset send_busy_flag\n");
        send_busy_flag = 0;
    }
    if(socket_int & SINT_STAT_SEND_OK)                           /* 发送完成中断 */
    {
        //LOGI("Socket send OK INT\n");
    }
    if(socket_int & SINT_STAT_RECV)                              /* 接收中断 */
    {
        LOGI("Socket recv INT\n");
        len = netGetRecvLen(socket_index);                          /* 获取当前缓冲区内数据长度 */
        LOGI("Recv len : %d\n", len);
        if(len > 512) {
            len = 512;                                            /* MyBuffer缓冲区长度为512 */
        }
        if(len > getRingFreeSize(ring_rd_buf)) {
            len = getRingFreeSize(ring_rd_buf);
        }
        if(len) {
            netGetRecvBuf(socket_index, len, buf);                        /* 读取数据 */
            putDataToRingBuf(ring_rd_buf, buf, len);
        }
        for(loop = 0; loop < len; loop++)
        {
            LOGI(" %x ,", buf[loop]);
        }
        LOGI("\n");
        //netSendBuf(socket_index, buf, len);
    }
    if(socket_int & SINT_STAT_CONNECT)                          /* 连接中断，仅在TCP模式下有效*/
    {
        LOGI("TCP connected\n");
        netSetKeepliveOnOff(socket_index, 1);                                   /*打开KEEPALIVE保活定时器*/
        netSetTTL(socket_index, 128);                          /*设置TTL*/
        if(SERVER == config.mode)
        {
            netGetRemoteIPP(socket_index, &remoteIP, &remotePort);
            LOGI("Have remote ip: %x, remote port: %d\n", remoteIP, remotePort);
        }
    }
    /*
    **产生断开连接中断和超时中断时，CH395默认配置是内部主动关闭，用户不需要自己关闭该Socket，如果想配置成不主动关闭Socket需要配置
    **SOCK_CTRL_FLAG_SOCKET_CLOSE标志位（默认为0），如果该标志为1，CH395内部不对Socket进行关闭处理，用户在连接中断和超时中断时调用
    **CH395CloseSocket函数对Socket进行关闭，如果不关闭则该Socket一直为连接的状态（事实上已经断开），就不能再去连接了。
    */
    if(socket_int & SINT_STAT_DISCONNECT)                        /* 断开中断，仅在TCP模式下有效 */
    {
        LOGI("TCP disconnected\n");
        netCloseSocket(socket_index);
    }
    if(socket_int & SINT_STAT_TIM_OUT)                           /* 超时中断 */
    {
        LOGI("time out \n");
        //netCloseSocket(socket_index);
    }
}

/*ch395全局中断函数*/
void ch395GlobalInterrupt(void)
{
    uint8_t int_status;
    uint8_t  unreachIPPT[8];

    netGetGlobINTStatus(&int_status);

    if(int_status & GINT_STAT_UNREACH) /* 不可达中断，读取不可达信息 */
    {
        LOGI("Unreach INT\n");
        netGetUnreachIPPT(unreachIPPT);
    }
    if(int_status & GINT_STAT_IP_CONFLI)                            /* 产生IP冲突中断，建议重新修改CH395的 IP，并初始化CH395*/
    {
        LOGI("IP CONFLI INT\n");
    }
    if(int_status & GINT_STAT_PHY_CHANGE)                           /* 产生PHY改变中断*/
    {
        LOGI("PHY change INT\n");
    }
    if(int_status & GINT_STAT_SOCK0)
    {
        LOGI("Socket0 INT\n");
        ch395SocketInterrupt(0);                                     /* 处理socket 0中断*/
    }
    if(int_status & GINT_STAT_SOCK1)                                
    {
        LOGI("Socket1 INT\n");
        ch395SocketInterrupt(1);                                     /* 处理socket 1中断*/
    }
    if(int_status & GINT_STAT_SOCK2)                                
    {
        LOGI("Socket2 INT\n");
        ch395SocketInterrupt(2);                                     /* 处理socket 2中断*/
    }
    if(int_status & GINT_STAT_SOCK3)                                
    {
        LOGI("Socket3 INT\n");
        ch395SocketInterrupt(3);                                     /* 处理socket 3中断*/
    }
    if(int_status & GINT_STAT_SOCK4)
    {
        LOGI("Socket4 INT\n");
        ch395SocketInterrupt(4);                                     /* 处理socket 4中断*/
    }
    if(int_status & GINT_STAT_SOCK5)                                
    {
        LOGI("Socket5 INT\n");
        ch395SocketInterrupt(5);                                     /* 处理socket 5中断*/
    }
    if(int_status & GINT_STAT_SOCK6)                                
    {
        LOGI("Socket6 INT\n");
        ch395SocketInterrupt(6);                                     /* 处理socket 6中断*/
    }
    if(int_status & GINT_STAT_SOCK7)                                
    {
        LOGI("Socket7 INT\n");
        ch395SocketInterrupt(7);                                     /* 处理socket 7中断*/
    }
}

void send_task(void)
{
    int wr_size;
    uint8_t buf[CH395_SEND_BUF_LEN];

    if(send_busy_flag) {
        return;
    }

    wr_size = getRingDataSize(ring_wr_buf);

    if(wr_size > CH395_SEND_BUF_LEN) {
        wr_size = CH395_SEND_BUF_LEN;
    }

    wr_size = getDataFromRingBuf(ring_wr_buf, buf, wr_size);
    if(0 == wr_size) {
        return;
    }

    netSendBuf(0, buf, wr_size);
    send_busy_flag = 1;
}

void *ch395_daemon_func(void *arg)
{
    NOTUSED(arg);
    while(1)
    {
        if(!net_run_flag) {
            LOGI("Close daemon task\n");
            break;
        }
        send_task();
        ch395GlobalInterrupt();
    }

    return NULL;
}

int netOpen()
{
    int ret;
    uint8_t check;
    static pthread_t tid;
    static pthread_attr_t attr;

    if (spi_fd > 0) {
        close(spi_fd);
    }

    spi_fd = open(spi_dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (spi_fd < 0) {
        LOGE("open %s failed\n", spi_dev);
        return spi_fd;
    }

    spi_mode = SPI_MODE_0;
    //spi_mode &= ~SPI_CS_HIGH;

    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (ret == -1) {
        LOGE("set spi write mode failed\n");
        close(spi_fd);
        return ret;
    }
    ret = ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (ret == -1) {
        LOGE("set spi read mode failed\n");
        close(spi_fd);
        return ret;
    }
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1) {
        LOGE("set spi write bit failed\n");
        close(spi_fd);
        return ret;
    }
    ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1) {
        LOGE("set spi read bit failed\n");
        close(spi_fd);
        return ret;
    }
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        LOGE("set spi write speed failed\n");
        close(spi_fd);
        return ret;
    }
    ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        LOGE("set spi read speed failed\n");
        close(spi_fd);
        return ret;
    }

    tr.tx_buf = (unsigned long)send_buf;
    tr.rx_buf = (unsigned long)recv_buf;
    tr.len = 0;
    tr.delay_usecs = 0;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;

    netGpioPowerOn();
    while(0x9a != netCheckExist(0x65))
    {
        LOGE("CH395 Check again.\n");
        usleep(100000);
    }

    ret = netSetLocalIp(config.local_ip);
    if(-1 == ret) {
        LOGE("Set local ip ERR\n");
    }
    else {
        LOGI("Set local ip OK\n");
    }
    ret = netSetGateway(config.gateway);
    if(-1 == ret) {
        LOGE("Set gateway ERR\n");
    }
    else {
        LOGI("Set gateway OK\n");
    }
    ret = netSetSubMask(config.sub_mask);    
    if(-1 == ret) {
        LOGE("Set submask ERR\n");
    }
    else {
        LOGI("Set submask OK\n");
    }
    ret = netInit();
    if(0 == ret) {
        LOGE("Net init OK\n");
    }
    else {
        LOGI("Net init ERR[%d]\n", ret);
    }

    if(TCP == config.protocol_type) {
        keeplive_set();
    }

    while(PHY_DISCONN == netGetPHYStatus())
    {
        LOGI("Get PHY Status\n");
        usleep(200000);
    }
    LOGI("PHY connected\n");

    ret = netSetProtoType(0, config.protocol_type);    
    if(-1 == ret) {
        LOGE("Set prototype ERR\n");
    }
    else {
        LOGI("Set prototype OK\n");
    }

    //Client模式，设置远端ip和端口号
    if(CLIENT == config.mode)
    {
        ret = netSetRemoteIp(0, config.remote_ip);
        if(-1 == ret) {
            LOGE("Set remote IP ERR\n");
        }
        else {
            LOGI("Set remote IP OK\n");
        }
        ret = netSetRemotePort(0, config.port);
        if(-1 == ret) {
            LOGE("Set remote port ERR\n");
        }
        else {
            LOGI("Set remote port OK\n");
        }
        ret = netSetSourcePort(0, config.port - 1);
        if(-1 == ret) {
            LOGE("Set source port ERR\n");
        }
        else {
            LOGI("Set source port OK\n");
        }
    }
    else //Server mode
    {
        ret = netSetSourcePort(0, config.port);
        if(-1 == ret) {
            LOGE("Set source port ERR\n");
        }
        else {
            LOGI("Set source port OK\n");
        }
        if(UDP == config.protocol_type)
        {
            ret = netSetRemoteIp(0, 0xffffffff);
            if(-1 == ret) {
                LOGE("Set remote IP ERR\n");
            }
            else {
                LOGI("Set remote IP OK\n");
            }
        }
    }

    netClearRecvBuf(0);
    init_ring_buf();
    ret = netOpenSocket(0);
    if(0 == ret) {
        LOGE("Open socket OK\n");
    }
    else {
        LOGI("Open socket ERR[%d]\n", ret);
    }

    netGetConf();

    if(TCP == config.protocol_type)
    {
        if(CLIENT == config.mode)
        {
            ret = netTCPConnect(0);
            if(0 == ret) {
                LOGE("TCP connect OK\n");
            }
            else {
                LOGI("TCP connect ERR[%d]\n", ret);
            }
        }
        else//SERVER
        {
            ret = netTCPListen(0);
            if(0 == ret) {
                LOGI("Open TCP listen OK\n");
            }
            else {
                LOGE("Open TCP listen FAIL\n");
            }
        }
    }

    net_run_flag = 1;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    ret = pthread_create(&tid, &attr, ch395_daemon_func, NULL);

    return 0;
}

void netClose(void)
{
    net_run_flag = 0;
    if(spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
    netGpioPowerOff();
}

int netIsOpen(void)
{
    return spi_fd;
}

int netCheckConnected(void)
{
    return net_run_flag;
}

int netWrite(void *buf, int buf_len)
{
    int buf_free_size;
    if(-1 == net_run_flag) {
        LOGE("Net not run yet!\n");
        return -1;
    }
    uint8_t *data     = (uint8_t*)buf;

    while(getRingFreeSize(ring_wr_buf) < buf_len)
    {
        sched_yield();
        usleep(100);
    }
    putDataToRingBuf(ring_wr_buf, data, buf_len);

    return 0;
}

int netRead(void *buf, int buf_len)
{
    if(-1 == net_run_flag) {
        LOGE("Net not run yet!\n");
        return -1;
    }
    uint8_t *data = (uint8_t*)buf;

    return getDataFromRingBuf(ring_rd_buf, data, buf_len);
}

int netReadBlock(void *buf, int buf_len)
{
    int read_len = 0;
    int tmp_len;
    uint8_t *data = (uint8_t*)buf;

    if(-1 == net_run_flag) {
            LOGE("Net not run yet!\n");
            return -1;
        }

    while(read_len < buf_len)
    {
        tmp_len = getDataFromRingBuf(ring_rd_buf, data + read_len, buf_len - read_len);
        read_len += tmp_len;
    }

    return read_len;
}

//TCP Client
int tcp_client(int argc, char **argv)
{
    NOTUSED(argc);
    NOTUSED(argv);
    volatile int ret;
    netGpioPowerOff();
    ret = netOpen();
    int i;
    uint8_t int_status;
    uint16_t socket_status;
    uint16_t recv_len;
    uint8_t msg[]={0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    if(-1 == ret)
    {
        LOGE("Net open ERR\n");
        return -1;
    }
    else
    {
        LOGE("Net open OK\n");
    }
    netGpioPowerOn();

    sleep(2);
    ret = netSetLocalIp(config.local_ip);
    if(-1 == ret) {
        LOGE("Set local ip ERR\n");
    }
    else {
        LOGI("Set local ip OK\n");
    }
    ret = netSetGateway(config.gateway);
    if(-1 == ret) {
        LOGE("Set gateway ERR\n");
    }
    else {
        LOGI("Set gateway OK\n");
    }
    ret = netSetSubMask(config.sub_mask);    
    if(-1 == ret) {
        LOGE("Set submask ERR\n");
    }
    else {
        LOGI("Set submask OK\n");
    }
    ret = netInit();
    if(0 == ret) {
        LOGE("Net init OK\n");
    }
    else {
        LOGI("Net init ERR[%d]\n", ret);
    }

    keeplive_set();

    while(PHY_DISCONN == netGetPHYStatus())
    {
        LOGI("Get PHY Status\n");
        usleep(200000);
    }
    LOGI("PHY connected\n");

    ret = netSetProtoType(0, TCP);    
    if(-1 == ret) {
        LOGE("Set prototype ERR\n");
    }
    else {
        LOGI("Set prototype OK\n");
    }
    ret = netSetRemoteIp(0, 0xc0a86463);
    if(-1 == ret) {
        LOGE("Set remote IP ERR\n");
    }
    else {
        LOGI("Set remote IP OK\n");
    }
    ret = netSetRemotePort(0, 9998);
    if(-1 == ret) {
        LOGE("Set remote port ERR\n");
    }
    else {
        LOGI("Set remote port OK\n");
    }
    ret = netSetSourcePort(0, 9999);
    if(-1 == ret) {
        LOGE("Set source port ERR\n");
    }
    else {
        LOGI("Set source port OK\n");
    }
    ret = netOpenSocket(0);
    if(0 == ret) {
        LOGE("Open socket OK\n");
    }
    else {
        LOGI("Open socket ERR[%d]\n", ret);
    }
    netGetConf();
    sleep(1);
    ret = netTCPConnect(0);
    if(0 == ret) {
        LOGE("TCP connect OK\n");
    }
    else {
        LOGI("TCP connect ERR[%d]\n", ret);
    }

    sleep(3);

    netClearRecvBuf(0);

    while(1)
    {
        ch395GlobalInterrupt();
        ret = netSendBuf(0, msg, sizeof(msg));        
        LOGI("Send ret %d\n", ret);
        LOGI("\n\n");
        sleep(3);
    }
    return 0;

    while(1)
    {
        ch395GlobalInterrupt();
        //ret = netSendBuf(0, msg, sizeof(msg));        
        //LOGI("Send ret %d\n", ret);
        //sleep(3);
    }
    return 0;
}

//TCP Server
int tcp_server(int argc, char **argv)
{
    NOTUSED(argc);
    NOTUSED(argv);
    volatile int ret;
    netGpioPowerOff();
    ret = netOpen();
    uint8_t int_status;
    uint16_t socket_status;
    uint16_t recv_len;
    uint8_t msg[]={0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    if(-1 == ret)
    {
        LOGE("Net open ERR\n");
        return -1;
    }
    else
    {
        LOGE("Net open OK\n");
    }
    netGpioPowerOn();

    sleep(2);
    ret = netSetLocalIp(config.local_ip);
    if(-1 == ret) {
        LOGE("Set local ip ERR\n");
    }
    else {
        LOGI("Set local ip OK\n");
    }
    ret = netSetGateway(config.gateway);
    if(-1 == ret) {
        LOGE("Set gateway ERR\n");
    }
    else {
        LOGI("Set gateway OK\n");
    }
    ret = netSetSubMask(config.sub_mask);    
    if(-1 == ret) {
        LOGE("Set submask ERR\n");
    }
    else {
        LOGI("Set submask OK\n");
    }
    ret = netInit();
    if(0 == ret) {
        LOGE("Net init OK\n");
    }
    else {
        LOGI("Net init ERR[%d]\n", ret);
    }

    keeplive_set();

    while(PHY_DISCONN == netGetPHYStatus())
    {
        LOGI("Get PHY Status\n");
        usleep(200000);
    }
    LOGI("PHY connected\n");

    ret = netSetProtoType(0, TCP);    
    if(-1 == ret) {
        LOGE("Set prototype ERR\n");
    }
    else {
        LOGI("Set prototype OK\n");
    }
    ret = netSetSourcePort(0, 9999);
    if(-1 == ret) {
        LOGE("Set source port ERR\n");
    }
    else {
        LOGI("Set source port OK\n");
    }
    ret = netOpenSocket(0);
    if(0 == ret) {
        LOGI("Open socket OK\n");
    }
    else {
        LOGE("Open socket ERR[%d]\n", ret);
    }

    ret = netTCPListen(0);
    if(0 == ret) {
        LOGI("Open TCP listen OK\n");
    }
    else {
        LOGE("Open TCP listen FAIL\n");
    }
    //netGetConf();
    //netClearRecvBuf(0);
    config.mode = SERVER;
    sleep(1);

    while(1)
    {
        ch395GlobalInterrupt();
        //ret = netSendBuf(0, msg, sizeof(msg));
        //LOGI("Send ret %d\n", ret);
        //sleep(3);
    }
    return 0;
}


int udp_client(void)
{
    volatile int ret;
    int loop;
    uint8_t  int_status;
    uint16_t status;
    uint16_t recv_len, read_len;
    netGpioPowerOff();
    ret = netOpen();
    uint8_t msg[6]={0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    uint8_t buf[30];

    if(-1 == ret) {
        LOGE("Net open ERR\n");
        return -1;
    }
    else {
        LOGE("Net open OK\n");
    }
    netGpioPowerOn();

    sleep(2);
    if(!netCheckCH395Status()) {
        LOGI("Check status OK!\n");
    }
    else {
        LOGE("Check status FAIL!\n");
    }
    ret = netSetLocalIp(config.local_ip);
    if(-1 == ret) {
        LOGE("Set local ip ERR\n");
    }
    else {
        LOGI("Set local ip OK\n");
    }
    ret = netSetGateway(config.gateway);
    if(-1 == ret) {
        LOGE("Set gateway ERR\n");
    }
    else {
        LOGI("Set gateway OK\n");
    }
    ret = netSetSubMask(config.sub_mask);    
    if(-1 == ret) {
        LOGE("Set submask ERR\n");
    }
    else {
        LOGI("Set submask OK\n");
    }
    ret = netInit();
    if(0 == ret) {
        LOGE("Net init OK\n");
    }
    else {
        LOGI("Net init ERR[%d]\n", ret);
    }

    while(PHY_DISCONN == netGetPHYStatus())
    {
        LOGI("Get PHY Status\n");
        usleep(200000);
    }
    LOGI("PHY connected\n");

    ret = netSetProtoType(0, UDP);
    if(-1 == ret) {
        LOGE("Set prototype ERR\n");
    }
    else {
        LOGI("Set prototype OK\n");
    }
    ret = netSetRemoteIp(0, 0xc0a86463);
    if(-1 == ret) {
        LOGE("Set remote IP ERR\n");
    }
    else {
        LOGI("Set remote IP OK\n");
    }
    ret = netSetRemotePort(0, 9998);
    if(-1 == ret) {
        LOGE("Set remote port ERR\n");
    }
    else {
        LOGI("Set remote port OK\n");
    }
    ret = netSetSourcePort(0, 9999);
    if(-1 == ret) {
        LOGE("Set source port ERR\n");
    }
    else {
        LOGI("Set source port OK\n");
    }
    ret = netOpenSocket(0);
    if(0 == ret) {
        LOGE("Open socket OK\n");
    }
    else {
        LOGI("Open socket ERR[%d]\n", ret);
    }
    sleep(1);
    netGetConf();
    //netPingEn(1);

    while(1)
    {
        netGetGlobINTStatus(&int_status);
        LOGI("INT status: %x\n", int_status);

        netGetSocketINTStatus(0,&int_status);
        LOGI("INT socket Status : %x\n", int_status);

        recv_len = netGetRecvLen(0);
        if(recv_len > 0) {
            LOGI("Recv len : %d\n", netGetRecvLen(0));
            read_len = netGetRecvBuf(0, 30, buf);
            for(loop = 0; loop < read_len; loop++)
            {
                LOGI("  %x  ", buf[loop]);
            }
            LOGI("\n\n");
        }
        else {
            LOGI("Recv len : %d\n", recv_len);
        }
        ret = netSendBuf(0, msg, sizeof(msg));
        LOGI("Send ret %d\n", ret);

        LOGI("\n\n\n");
        sleep(5);
    }
    return 0;
}

int udp_server(void)
{
    volatile int ret;
    int loop;
    uint8_t  int_status;
    uint16_t status;
    uint16_t recv_len, read_len;
    netGpioPowerOff();
    ret = netOpen();
    uint8_t msg[6]={0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    uint8_t buf[30];

    if(-1 == ret) {
        LOGE("Net open ERR\n");
        return -1;
    }
    else {
        LOGE("Net open OK\n");
    }
    netGpioPowerOn();

    sleep(2);
    if(!netCheckCH395Status()) {
        LOGI("Check status OK!\n");
    }
    else {
        LOGE("Check status FAIL!\n");
    }

    ret = netSetLocalIp(config.local_ip);
    if(-1 == ret) {
        LOGE("Set local ip ERR\n");
    }
    else {
        LOGI("Set local ip OK\n");
    }
    ret = netSetGateway(config.gateway);
    if(-1 == ret) {
        LOGE("Set gateway ERR\n");
    }
    else {
        LOGI("Set gateway OK\n");
    }
    ret = netSetSubMask(config.sub_mask);    
    if(-1 == ret) {
        LOGE("Set submask ERR\n");
    }
    else {
        LOGI("Set submask OK\n");
    }
    ret = netInit();
    if(0 == ret) {
        LOGE("Net init OK\n");
    }
    else {
        LOGI("Net init ERR[%d]\n", ret);
    }

    while(PHY_DISCONN == netGetPHYStatus())
    {
        LOGI("Get PHY Status\n");
        usleep(200000);
    }
    LOGI("PHY connected\n");

    ret = netSetProtoType(0, UDP);
    if(-1 == ret) {
        LOGE("Set prototype ERR\n");
    }
    else {
        LOGI("Set prototype OK\n");
    }
    ret = netSetRemoteIp(0, 0xffffffff);
    if(-1 == ret) {
        LOGE("Set remote IP ERR\n");
    }
    else {
        LOGI("Set remote IP OK\n");
    }

    ret = netSetSourcePort(0, 9999);
    if(-1 == ret) {
        LOGE("Set source port ERR\n");
    }
    else {
        LOGI("Set source port OK\n");
    }
    ret = netOpenSocket(0);
    if(0 == ret) {
        LOGE("Open socket OK\n");
    }
    else {
        LOGI("Open socket ERR[%d]\n", ret);
    }
    sleep(1);
    netGetConf();
    //netPingEn(1);

    while(1)
    {
        ch395GlobalInterrupt();
    }
    return 0;
}

int main(int argc, char* argv[])
{
    NOTUSED(argc);
    NOTUSED(argv);
    int ret;
    uint8_t buf[5] = {0x00, 0x11, 0x22, 0x33, 0x44};
    uint8_t read_buf[100];
    int read_len, loop;

    if(!strcmp("TCP", argv[1])) {
        LOGI("Set tcp mode\n");
        setProtocolType(TCP);
    }
    else {
        LOGI("Set udp mode\n");
        setProtocolType(UDP);
    }

    if(!strcmp("SERVER", argv[2])) {
        LOGI("Set server mode\n");
        setNetMode(SERVER);
    }
    else{
        LOGI("Set client mode\n");
        setNetMode(CLIENT);
        setRemoteIp(0xc0a86463);
    }
    netClose();
    sleep(1);
    netOpen();

    while(1)
    {
        sleep(3);
        netWrite(buf, sizeof(buf));
        /*read_len = netRead(read_buf, sizeof(read_buf));
        if(read_len > 0)
        {
            LOGI("XXX read data:");
            for(loop = 0; loop < read_len; loop++)
            {
                LOGI("  %x  ", read_buf[loop]);
            }
            LOGI("\n");
        }*/
    }
    return 0;
}


