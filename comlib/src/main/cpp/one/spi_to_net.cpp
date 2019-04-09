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
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

//Android±àÒëÊ±´ò¿ª
#ifndef __ANDROID_BUILD__
//#define __ANDROID_BUILD__
#endif

//#define LOG_NDEBUG 0
#define LOG_TAG "CRC"

#include<android/log.h>

#define TAG "myDemo-jni" // Õâ¸öÊÇ×Ô¶¨ÒåµÄLOGµÄ±êÊ¶
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG ,__VA_ARGS__) // ¶¨ÒåLOGDÀàÐÍ
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG ,__VA_ARGS__) // ¶¨ÒåLOGIÀàÐÍ
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,TAG ,__VA_ARGS__) // ¶¨ÒåLOGWÀàÐÍ
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG ,__VA_ARGS__) // ¶¨ÒåLOGEÀàÐÍ
#define LOGF(...) __android_log_print(ANDROID_LOG_FATAL,TAG ,__VA_ARGS__) // ¶¨ÒåLOGFÀàÐÍ

#define SPI_SEND_BUF_SIZE 4096
#define SPI_RECV_BUF_SIZE 4096
#define SPI_SEND_FRM_SIZE 128
#define SPI_FRM_HEADER_SIZE 3

#define BFM_CTRL_REQUEST      0x01
#define BFM_CTRL_WRITE_HALF_1 0x02
#define BFM_CTRL_WRITE_HALF_2 0x03
#define BFM_CTRL_EXEC         0x04

typedef enum
{
    UP_START,
    UP_HALF_1,
    UP_HALF_2,
    UP_DONE
}UP_STATE;

typedef struct
{
    int state;
    int op_addr;
    int addr;
    char data[2048];
    int data_len;
}UP_FSM_INFO;

typedef union
{
    uint32_t Val;
    uint16_t w[2];
    uint8_t  v[4];
}IPV4_ADDR;

typedef struct
{
    IPV4_ADDR local_ip;
    IPV4_ADDR sub_mask;
    IPV4_ADDR gateway;
    uint8_t   protocol_type; //0: TCP, 1: UDP
    uint8_t   mode;          //0: server mode, 1: client mode
    uint16_t  port;          //if server, the port is local port; if client, the port is remote port
    IPV4_ADDR remote_ip;     //client mode only
}NET_CONFIG;

static const char *spi_dev =  "/dev/spidev5.0";
static uint8_t spi_mode = 1;
static uint8_t bits = 8;
static uint32_t speed = 3000000;
static int spi_fd = -1;
static pthread_mutex_t spi_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t tidFrm;
static pthread_attr_t attrFrm;
static uint8_t send_buf[SPI_SEND_BUF_SIZE];
static uint8_t recv_buf[SPI_RECV_BUF_SIZE];
//static int gpio_export_fd = -1;
static int gpio_cs_direction_fd = -1;
static int gpio_cs_value_fd = -1;
static struct spi_ioc_transfer tr;


//andriod java is big endian, arm C++ is little endian, PIC32 is big endian
static NET_CONFIG config = {
        0xc0a86464,     // local ip: 192.168.100.100
        0xffffff00,     // mask: 255.255.255.0
        0xc0a86401,     // gateway: 192.168.100.1
        0,              // protocol: TCP
        0,              // mode: server
        9999,           // port: 9999
        0};

static uint8_t config_changed = 0;


void setLocalIp(uint32_t local_ip)
{
    if (config.local_ip.Val != local_ip)
    {
        config_changed = 1;
        config.local_ip.Val = local_ip;
    }
}

void setSubMask(uint32_t mask)
{
    if (config.sub_mask.Val != mask)
    {
        config_changed = 1;
        config.sub_mask.Val = mask;
    }
}

void setGateway(uint32_t gateway)
{
    if (config.gateway.Val != gateway)
    {
        config_changed = 1;
        config.gateway.Val = gateway;
    }
}
void setProtocolType(uint8_t protocol_type)
{
    if (config.protocol_type != protocol_type)
    {
        config_changed = 1;
        config.protocol_type = protocol_type;
    }
}
void setNetMode(uint8_t mode)
{
    if (config.mode != mode)
    {
        config_changed = 1;
        config.mode = mode;
    }
}
void setNetPort(uint16_t port)
{
    if (config.port != port)
    {
        config_changed = 1;
        config.port = port;
    }
}
void setRemoteIp(uint32_t remote_ip)
{
    if (config.remote_ip.Val != remote_ip)
    {
        config_changed = 1;
        config.remote_ip.Val = remote_ip;
    }
}

uint32_t getLocalIp()
{
    return config.local_ip.Val;
}

uint32_t getSubMask()
{
    return config.sub_mask.Val;
}

uint32_t getGateway()
{
    return config.gateway.Val;
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
    return config.remote_ip.Val;
}

void *spi_config_func(void *arg)
{
    uint8_t len = 0;
    int ret;

    //sleep 2 seconds for pic32 bootloader software update interval
    sleep(2);

    if (config_changed == 1)
    {
        //Ö¡¸ñÊ½£º 0xAA + ¿ØÖÆÂë + Êý¾Ý³¤¶È + Êý¾ÝÓò + Ð£ÑéÎ» + 0xEE
        send_buf[len++] = 0xAA; //start flag
        send_buf[len++] = 20;   //data len low
        send_buf[len++] = 0;    //data len high

        send_buf[len++] = config.local_ip.v[3];
        send_buf[len++] = config.local_ip.v[2];
        send_buf[len++] = config.local_ip.v[1];
        send_buf[len++] = config.local_ip.v[0];

        send_buf[len++] = config.sub_mask.v[3];
        send_buf[len++] = config.sub_mask.v[2];
        send_buf[len++] = config.sub_mask.v[1];
        send_buf[len++] = config.sub_mask.v[0];

        send_buf[len++] = config.gateway.v[3];
        send_buf[len++] = config.gateway.v[2];
        send_buf[len++] = config.gateway.v[1];
        send_buf[len++] = config.gateway.v[0];

        send_buf[len++] = config.protocol_type;
        send_buf[len++] = config.mode;
        send_buf[len++] = config.port & 0xff;
        send_buf[len++] = config.port >> 8;

        send_buf[len++] = config.remote_ip.v[3];
        send_buf[len++] = config.remote_ip.v[2];
        send_buf[len++] = config.remote_ip.v[1];
        send_buf[len++] = config.remote_ip.v[0];


        tr.len = len;
        ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);//Ö´ÐÐspidev.cÖÐioctlµÄdefault½øÐÐÊý¾Ý´«Êä
        if (ret < 0)
        {
            LOGE("write config failed\n");
            return NULL;
        }
        LOGD("set config sucessfully, local_ip:%x, mask:%x, gateway:%x, protocol:%d, mode:%d, port:%d, remote_ip:%x\n",
             config.local_ip.Val, config.sub_mask.Val, config.gateway.Val,
             config.protocol_type, config.mode, config.port, config.remote_ip.Val);
        sleep(2);
    }

    config_changed = 0;



    //begin to read config
    memset(send_buf, 0, 26);
    send_buf[0] = 0xCC; //start flag
    send_buf[1] = 23;
    send_buf[2] = 0;
    //À­µÍÆ¬Ñ¡Ïß
    tr.len = 26;
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);//Ö´ÐÐspidev.cÖÐioctlµÄdefault½øÐÐÊý¾Ý´«Êä
    //À­¸ßÆ¬Ñ¡Ïß
    if (ret < 0)
    {
        LOGE("read config failed\n");
        return NULL;
    }

    len = 0;
    while (recv_buf[len] != 0xCE && len < 25)
    {
        len++;
    }

    if ((recv_buf[len] == 0xCE) &&
        (recv_buf[len+1] == 20) &&
        (recv_buf[len+2] == 0))
    {
        config.local_ip.v[3] = recv_buf[len+3];
        config.local_ip.v[2] = recv_buf[len+4];
        config.local_ip.v[1] = recv_buf[len+5];
        config.local_ip.v[0] = recv_buf[len+6];
        config.sub_mask.v[3] = recv_buf[len+7];
        config.sub_mask.v[2] = recv_buf[len+8];
        config.sub_mask.v[1] = recv_buf[len+9];
        config.sub_mask.v[0] = recv_buf[len+10];
        config.gateway.v[3]  = recv_buf[len+11];
        config.gateway.v[2]  = recv_buf[len+12];
        config.gateway.v[1]  = recv_buf[len+13];
        config.gateway.v[0]  = recv_buf[len+14];

        config.protocol_type = recv_buf[len+15];
        config.mode          = recv_buf[len+16];

        config.port = recv_buf[len+17] + (uint16_t)(recv_buf[len+18] << 8);

        config.remote_ip.v[3] = recv_buf[len+19];
        config.remote_ip.v[2] = recv_buf[len+20];
        config.remote_ip.v[1] = recv_buf[len+21];
        config.remote_ip.v[0] = recv_buf[len+22];

        LOGD("read config sucessfully, local_ip:%x, mask:%x, gateway:%x, protocol:%d, mode:%d, port:%d, remote_ip:%x\n",
             config.local_ip.Val, config.sub_mask.Val, config.gateway.Val,
             config.protocol_type, config.mode, config.port, config.remote_ip.Val);
    }
    else
    {
        LOGE("read config failed\n");
    }

    return NULL;
}

int net_gpio_power_on()
{
    system("echo 1 > /sys/class/gpio/gpio56/value");
    system("echo 1 > /sys/class/gpio/gpio91/value");

    return 0;
}

int net_gpio_power_off()
{
    system("echo 0 > /sys/class/gpio/gpio56/value");
    system("echo 0 > /sys/class/gpio/gpio91/value");

    return 0;
}

int net_init()
{
    int ret;

    if (spi_fd > 0)
    {
        close(spi_fd);
    }

    spi_fd = open(spi_dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (spi_fd < 0)
    {
        LOGE("open %s failed\n", spi_dev);
        return spi_fd;
    }

    spi_mode |= SPI_MODE_0;     //
    spi_mode &= ~SPI_CS_HIGH;   //µÍµçÆ½Æ¬Ñ¡

    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (ret == -1)
    {
        LOGE("set spi write mode failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (ret == -1)
    {
        LOGE("set spi read mode failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
    {
        LOGE("set spi write bit failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
    {
        LOGE("set spi read bit failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        LOGE("set spi write speed failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
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

    return 0;
}

int netIsOpen(){
    return spi_fd;
}


int netOpen()
{
    int ret;

    spi_fd = open(spi_dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (spi_fd < 0)
    {
        LOGE("open %s failed\n", spi_dev);
        return spi_fd;
    }

    spi_mode |= SPI_MODE_0;     //
    spi_mode &= ~SPI_CS_HIGH;   //µÍµçÆ½Æ¬Ñ¡

    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (ret == -1)
    {
        LOGE("set spi write mode failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (ret == -1)
    {
        LOGE("set spi read mode failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret == -1)
    {
        LOGE("set spi write bit failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret == -1)
    {
        LOGE("set spi read bit failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        LOGE("set spi write speed failed\n");
        close(spi_fd);
        return ret;
    }
    ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret == -1)
    {
        LOGE("set spi read speed failed\n");
        close(spi_fd);
        return ret;
    }

    tr.tx_buf = (unsigned long)send_buf;
    tr.rx_buf = (unsigned long)recv_buf;
    tr.len = sizeof(send_buf);
    tr.delay_usecs = 0;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;

    //gpioÉÏµç
    ret = net_gpio_power_on();
    if (ret == -1)
    {
        LOGE("set gpio power on failed\n");
        close(spi_fd);
        return ret;
    }

    //create thread to config ip address
    pthread_attr_init(&attrFrm);
    pthread_attr_setdetachstate(&attrFrm, PTHREAD_CREATE_DETACHED);
    ret = pthread_create(&tidFrm, &attrFrm, spi_config_func, NULL);

    return spi_fd;
}

int netClose(void)
{
    int ret = -1;

    if(spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }

    //gpioÏÂµç
    ret = net_gpio_power_off();

    return 0;
}

int netWrite(void *buf, int buf_len)
{
    uint8_t done = 0;
    uint16_t idx;
    uint16_t len = buf_len;
    uint16_t write_len;
    uint16_t ret;
    uint8_t *tmp = (uint8_t *)buf;

    if (tmp == NULL)
    {
        LOGE("input buffer null\n");
        return -1;
    }

    while (len > 0)
    {
        if (len > SPI_SEND_FRM_SIZE)
        {
            write_len = SPI_SEND_FRM_SIZE;
            memcpy(&send_buf[3], tmp, SPI_SEND_FRM_SIZE);
            tmp += SPI_SEND_FRM_SIZE;
            len -= SPI_SEND_FRM_SIZE;
        }
        else
        {
            write_len = len;
            memcpy(&send_buf[3], tmp, len);
            done = 1;
        }

        tr.len = 0;
        send_buf[tr.len++] = 0xBB;
        send_buf[tr.len++] = write_len & 0xFF;
        send_buf[tr.len++] = (write_len >> 8) & 0xFF;

        tr.len += write_len;

        ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);//Ö´ÐÐspidev.cÖÐioctlµÄdefault½øÐÐÊý¾Ý´«Êä
        if (ret < 0)
        {
            LOGE("write spi failed\n");
            return -1;
        }

        if (done == 1)
        {
            break;
        }
    }

    return buf_len;
}


int netRead(void *buf, int buf_size)
{
    uint16_t ret;
    uint16_t frm_len;
    uint16_t out_len = 0;
    uint8_t *recv = &recv_buf[3];
    uint8_t *end;

    if (buf == NULL)
    {
        LOGE("input buffer null\n");
        return -1;
    }

    tr.len = buf_size;
    memset(send_buf, 0, tr.len);
    send_buf[0] = 0xDD;
    send_buf[1] = (tr.len - SPI_FRM_HEADER_SIZE) & 0xFF;
    send_buf[2] = ((tr.len - SPI_FRM_HEADER_SIZE) >> 8) &0xFF;


    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);//Ö´ÐÐspidev.cÖÐioctlµÄdefault½øÐÐÊý¾Ý´«Êä
    if (ret < 0)
    {
        LOGE("read spi failed\n");
        return ret;
    }

    end = recv_buf+buf_size;
    while (recv < end)
    {
        if (*recv != 0xEE)
        {
            recv++;
            continue;
        }

        frm_len = *(recv+1) + (uint16_t)((*(recv+2))<<8);
        if ((frm_len == 0) || (frm_len > (SPI_RECV_BUF_SIZE-6)))
        {
            recv += SPI_FRM_HEADER_SIZE;
            continue;
        }

        if ((recv + SPI_FRM_HEADER_SIZE + frm_len) > (end))
        {
            LOGE("invalid frm_len:%d, data was truncated\n",frm_len);
            memcpy((uint8_t *)buf+out_len, recv+SPI_FRM_HEADER_SIZE, end - recv - SPI_FRM_HEADER_SIZE);
            out_len += frm_len;
            break;
        }

        memcpy((uint8_t *)buf+out_len, recv+SPI_FRM_HEADER_SIZE, frm_len);
        out_len += frm_len;
        recv += (SPI_FRM_HEADER_SIZE + frm_len);
    }

    return out_len;
}

int netReadVersion(int v[])
{
    int ret;

    ret = net_init();
    if (ret < 0)
    {
        return -1;
    }

    net_gpio_power_on();

    sleep(2);

    tr.len = 4;
    send_buf[0] = 0xD0;
    send_buf[1] = 0;
    send_buf[2] = 0;
    send_buf[3] = 0;
    recv_buf[1] = 0;
    recv_buf[2] = 0;
    recv_buf[3] = 0;

    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (recv_buf[1] == 0x99)
    {
        v[0] = recv_buf[2];
        v[1] = recv_buf[3];


        ret = 0;
    }
    else
    {
        ret = -1;
    }

    net_gpio_power_off();
    return ret;
}

// 1 - connected, 0 - not connected, -1 - check error
int netCheckConnected()
{
    int ret;

    if (spi_fd <= 0)
    {
        ret = netOpen();
        if (ret < 0)
        {
            return -1;
        }

        sleep(3);
    }

    tr.len = 3;
    send_buf[0] = 0xD1;
    send_buf[1] = 0;
    send_buf[2] = 0;
    recv_buf[1] = 0;
    recv_buf[2] = 0;

    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    LOGE("recv_buf_1 = %d",recv_buf[1]);
    LOGE("recv_buf_2 = %d",recv_buf[2]);
    if (recv_buf[1] == 0x99)
    {
        if (recv_buf[2] > 0)
        {
            ret = 1;
        }
        else
        {
            ret = 0;
        }
    }
    else
    {
        ret = -1;
    }

    return ret;
}

void decode_ecrypt_data(char *data, int len, const char *pwd)
{
    int pwd_len;
    int idx;

    if (NULL == pwd)
    {
        pwd = "pic32mx664_ethernet_software_topscomm-shandong-qingdao-china,www.topscomm.com";
    }

    pwd_len = strlen(pwd);

    for (idx=0; idx < len; idx++)
    {
        data[idx] = data[idx] ^ (pwd[idx%pwd_len] - 0x30);
    }
}

char ascii_to_num(char a)
{
    if (a >= '0' && a <= '9')
    {
        return (a - '0');
    }
    else if (a >= 'a' && a <= 'f')
    {
        return (a - 'a' + 0xa);
    }
    else if (a >= 'A' && a <= 'F')
    {
        return (a - 'A' + 0xa);
    }
    else
    {
        return 0;
    }
}

int up_request1()
{
    uint8_t ret;
    int i = 0;

    tr.len = 0;
    send_buf[tr.len++] = 0x55;
    send_buf[tr.len++] = 0x55;
    send_buf[tr.len++] = 0x55;
    send_buf[tr.len++] = BFM_CTRL_REQUEST;
    tr.len++;   //last byte for ACK

    //delay 250ms to wait slave device up
    usleep(250000);

    //keep try for one second until slave accept upgrade
    while (i++ < 100)
    {
        recv_buf[tr.len - 1] = 0;

        ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

        ret = recv_buf[tr.len - 1];
        if (ret == 0x99)
        {
            printf("up request success\n");
            return ret;
        }
        else
        {
            usleep(10000);
        }
    }

    return 0;
}

int up_write(UP_FSM_INFO *fsm, int type)
{
    uint8_t *ret;
    int i = 0;

    tr.len = 0;
    send_buf[tr.len++] = 0x55;
    send_buf[tr.len++] = 0x55;
    send_buf[tr.len++] = 0x55;

    if (type == BFM_CTRL_WRITE_HALF_1)
    {
        //wait for previous page burn
        usleep(20000);

        send_buf[tr.len++] = BFM_CTRL_WRITE_HALF_1;
        send_buf[tr.len++] = (fsm->addr >> 16) & 0xff;
        send_buf[tr.len++] = (fsm->addr >> 8) & 0xff;
        send_buf[tr.len++] = fsm->addr & 0xff;

        memcpy(&send_buf[tr.len], fsm->data, 2048);

        tr.len += 2049; //2048 for data, 1 for ACK
    }
    else if (type == BFM_CTRL_WRITE_HALF_2)
    {
        send_buf[tr.len++] = BFM_CTRL_WRITE_HALF_2;

        memcpy(&send_buf[tr.len], fsm->data, 2048);

        tr.len += 2049;
    }

    ret = &recv_buf[tr.len - 1];

    while (i++ < 100)
    {
        *ret = 0;

        ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

        if (*ret == 0x99)
        {
            printf("write 0x%x success, type=%d, ret=%x\n", fsm->addr, type, *ret);
            return *ret;
        }
        usleep(1000);
    }

    LOGE("write 0x%x failed, type=%d\n", fsm->addr, type);

    return 0;
}

void up_exec(UP_FSM_INFO *fsm)
{
    int i = 0;
    uint8_t ret;

    tr.len = 0;
    send_buf[tr.len++] = 0x55;
    send_buf[tr.len++] = 0x55;
    send_buf[tr.len++] = 0x55;
    send_buf[tr.len++] = BFM_CTRL_EXEC;

    send_buf[tr.len++] = (fsm->op_addr >> 16) & 0xff;
    send_buf[tr.len++] = (fsm->op_addr >> 8) & 0xff;
    send_buf[tr.len++] = fsm->op_addr & 0xff;
    tr.len++;   //for ACK

    while (i++ < 100)
    {
        recv_buf[tr.len - 1] = 0;

        ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

        ret = recv_buf[tr.len - 1];
        if (ret == 0x99)
        {
            break;
        }
        else
        {
            usleep(1000);
        }
    }
}

int up_fsm(UP_FSM_INFO *fsm, int addr, char *data, int len)
{
    int i;
    char byte;
    int ret;

    switch (fsm->state)
    {
        case UP_START:
            fsm->addr = (addr & (~4095));
            fsm->op_addr = fsm->addr;
            fsm->data_len = 0;
            memset(fsm->data, 0xff, 2048);

            if (addr < (fsm->addr+0x800))
            {
                if (addr > fsm->addr)
                {
                    fsm->data_len = addr - fsm->addr;
                }
                for (i=0; i<len; i++)
                {
                    byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                    fsm->data[fsm->data_len++] = byte;
                }
                fsm->state = UP_HALF_1;
            }
            else
            {
                ret = up_write(fsm, BFM_CTRL_WRITE_HALF_1);
                if (ret == 0)
                {
                    return -1;
                }
                if (addr > (fsm->addr+0x800))
                {
                    fsm->data_len = addr - fsm->addr - 0x800;
                }
                for (i=0; i<len; i++)
                {
                    byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                    fsm->data[fsm->data_len++] = byte;
                }
                fsm->state = UP_HALF_2;
            }
            break;
        case UP_HALF_1:
            if (addr >= 0x20000)
            {
                ret = up_write(fsm, BFM_CTRL_WRITE_HALF_1);
                if (ret == 0)
                {
                    return -1;
                }
                memset(fsm->data, 0xff, 2048);
                ret = up_write(fsm, BFM_CTRL_WRITE_HALF_2);
                if (ret == 0)
                {
                    return -1;
                }
                fsm->state = UP_DONE;
                break;
            }
            if (addr < (fsm->addr + 0x800))
            {
                if (addr > (fsm->addr+fsm->data_len))
                {
                    fsm->data_len = addr - fsm->addr;
                }
                for (i=0; i<len; i++)
                {
                    byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                    fsm->data[fsm->data_len++] = byte;
                }
            }
            else if (addr < (fsm->addr + 0x1000))
            {
                ret = up_write(fsm, BFM_CTRL_WRITE_HALF_1);
                if (ret == 0)
                {
                    return -1;
                }
                fsm->data_len = 0;
                memset(fsm->data, 0xff, 2048);
                if (addr > (fsm->addr + 0x800))
                {
                    fsm->data_len = addr - (fsm->addr + 0x800);
                }

                for (i=0; i<len; i++)
                {
                    byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                    fsm->data[fsm->data_len++] = byte;
                }
                fsm->state = UP_HALF_2;
            }
            else
            {
                ret = up_write(fsm, BFM_CTRL_WRITE_HALF_1);
                if (ret == 0)
                {
                    return -1;
                }
                memset(fsm->data, 0xff, 2048);
                ret = up_write(fsm, BFM_CTRL_WRITE_HALF_2);
                if (ret == 0)
                {
                    return -1;
                }

                fsm->addr = (addr & (~4095));
                fsm->data_len = 0;
                memset(fsm->data, 0xff, 2048);

                if (addr < (fsm->addr+0x800))
                {
                    if (addr > fsm->addr)
                    {
                        fsm->data_len = addr - fsm->addr;
                    }
                    for (i=0; i<len; i++)
                    {
                        byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                        fsm->data[fsm->data_len++] = byte;
                    }
                    fsm->state = UP_HALF_1;
                }
                else
                {
                    memset(fsm->data, 0xff, 2048);
                    ret = up_write(fsm, BFM_CTRL_WRITE_HALF_1);
                    if (ret == 0)
                    {
                        return -1;
                    }
                    if (addr > (fsm->addr+0x800))
                    {
                        fsm->data_len = addr - fsm->addr - 0x800;
                    }
                    for (i=0; i<len; i++)
                    {
                        byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                        fsm->data[fsm->data_len++] = byte;
                    }
                    fsm->state = UP_HALF_2;
                }
            }
            break;
        case UP_HALF_2:
            if (addr >= 0x20000)
            {
                ret = up_write(fsm, BFM_CTRL_WRITE_HALF_2);
                if (ret == 0)
                {
                    return -1;
                }
                fsm->state = UP_DONE;
                break;
            }

            if (addr < (fsm->addr + 0x1000))
            {
                if ((addr - 0x800) > (fsm->addr+fsm->data_len))
                {
                    fsm->data_len = addr - fsm->addr - 0x800;
                }
                for (i=0; i<len; i++)
                {
                    byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                    fsm->data[fsm->data_len++] = byte;
                }
            }
            else
            {
                ret = up_write(fsm, BFM_CTRL_WRITE_HALF_2);
                if (ret == 0)
                {
                    return -1;
                }

                fsm->addr = (addr & (~4095));
                fsm->data_len = 0;
                memset(fsm->data, 0xff, 2048);

                if (addr < (fsm->addr+0x800))
                {
                    if (addr > fsm->addr)
                    {
                        fsm->data_len = addr - fsm->addr;
                    }
                    for (i=0; i<len; i++)
                    {
                        byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                        fsm->data[fsm->data_len++] = byte;
                    }
                    fsm->state = UP_HALF_1;
                }
                else
                {
                    memset(fsm->data, 0xff, 2048);
                    ret = up_write(fsm, BFM_CTRL_WRITE_HALF_1);
                    if (ret == 0)
                    {
                        return -1;
                    }
                    if (addr > (fsm->addr+0x800))
                    {
                        fsm->data_len = addr - fsm->addr - 0x800;
                    }
                    for (i=0; i<len; i++)
                    {
                        byte = (ascii_to_num(data[i*2])<<4) + ascii_to_num(data[i*2+1]);
                        fsm->data[fsm->data_len++] = byte;
                    }
                    fsm->state = UP_HALF_2;
                }
            }
            break;
        case UP_DONE:
        default:
            break;
    }

    return 0;
}

int netUpgrade(void *path, void *pwd = NULL)
{
    FILE *fp;
    char *data;
    int len, hex_len;
    int idx = 0, i;
    int ret;
    int addr, addr_1, addr_2, addr_3;
    char crc;
    UP_FSM_INFO fsm;

    memset(&fsm, 0, sizeof(UP_FSM_INFO));

    fp = fopen(reinterpret_cast<char *>(path), "r");
    if (NULL == fp)
    {
        LOGE("open file(%s) failed\n", reinterpret_cast<char *>(path));
        return -1;
    }

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    data = (char*)malloc(len+1);
    fseek(fp, 0, SEEK_SET);
    fread(data, len, 1, fp);
    data[len] = '\0';
    fclose(fp);

    decode_ecrypt_data(data, len, reinterpret_cast<char *>(pwd));

    net_gpio_power_off();
    net_gpio_power_on();

    ret = net_init();
    if (ret < 0)
    {
        free(data);
        net_gpio_power_off();
        return ret;
    }

    //check if slave device is ready for upgrade
    if ((ret = up_request1()) == 0)
    {
        LOGE("Slave not ready for upgrade\n");
        free(data);
        return -1;
    }

    addr_1 = -1;
    while (idx < len)
    {
        if (data[idx] != ':')
        {
            idx++;
            continue;
        }

        hex_len = (ascii_to_num(data[idx+1]) << 4) + ascii_to_num(data[idx+2]);
        crc = 0;
        for (i=0; i<(hex_len+4); i++)
        {
            crc += ((ascii_to_num(data[idx+1+i*2]) << 4) + ascii_to_num(data[idx+1+i*2+1]));
        }
        if (((0x100 - crc)&0xff) != ((ascii_to_num(data[idx+1+i*2]) << 4) + ascii_to_num(data[idx+1+i*2+1])))
        {
            LOGE("crc error, file was destoryed\n");
            close(spi_fd);
            free(data);
            net_gpio_power_off();
            return -1;
        }

        if (data[idx+8] == '4')
        {
            addr_1++;
            idx = idx + (2*hex_len + 11);
            continue;
        }
        else if (data[idx+8] == '0')
        {
            addr = (addr_1 << 16) + ((ascii_to_num(data[idx+3])) << 12) + (ascii_to_num(data[idx+4]) << 8) + (ascii_to_num(data[idx+5]) << 4) + ascii_to_num(data[idx+6]);
            if ((ret = up_fsm(&fsm, addr, &data[idx+9], hex_len)) != 0)
            {
                LOGE("Slave upgrade failed\n");
                close(spi_fd);
                free(data);
                net_gpio_power_off();
                return ret;
            }
            if (fsm.state == UP_DONE)
            {
                LOGD("Slave upgrade success\n");
                up_exec(&fsm);
                close(spi_fd);
                free(data);
                net_gpio_power_off();
                return 0;
            }
            idx = idx + (2*hex_len + 11);
            continue;
        }

        idx++;
    }

    close(spi_fd);
    free(data);
    net_gpio_power_off();
    return 0;
}

