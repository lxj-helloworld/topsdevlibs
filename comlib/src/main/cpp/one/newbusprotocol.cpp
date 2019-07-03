/********************************************************
 * Copyright (C), 2008-2019, Topscomm Co., Ltd.
 * File name: fp.c
 * Author: Wanglei
 * Version: V1.0
 * Date: 2019/5/31
 * Description:
 *
 * Others:
 * Function List:
 * History:
 * 2019/5/31 - For temp upgrade ver1.3 of Fplus(Baudrate is 4800)
 * 2019/6/6  - For new 1.25M+1.25M ver2.0, can diablo!
 ********************************************************/
#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <string.h>
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <pthread.h>
#include     <string.h>
#include     <sys/time.h>
#include     <sys/timerfd.h>
#include     <android/log.h>
#include     <signal.h>

//Android编译时打开
#ifndef __ANDROID_BUILD__
#define __ANDROID_BUILD__
#endif

//#define ADB_DEBUG
#ifndef ADB_DEBUG
#define LOG_TAG "FPLUS"
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

typedef unsigned short uint16;
typedef unsigned long long uint64;
typedef unsigned char uint8;
typedef unsigned int uint32;

typedef struct
{
    int up_baudrate;
    int upgrade_time_out_sec;
    int request_time_out_init_sec;
    int request_time_out_init_nsec;
    int request_time_out_period_sec;
    int request_time_out_period_nsec;
    int exec_time_out_init_sec;
    int exec_time_out_init_nsec;
    int exec_time_out_period_sec;
    int exec_time_out_period_nsec;
} bus_param_t;

#define BUF_SIZE_MASK 4095
#define BUF_SIZE (BUF_SIZE_MASK + 1)
#define CHAN_MASK 0xF0
#define ACK_SEQ_MASK 0xF0
#define DATA_SEQ_MASK 0x0F

#define HEAD 0xAA
#define TAIL 0x16

#define HEAD_LEN 1
#define CTRL_LEN 1
#define CHAN_LEN 1
#define DATALENTH_LEN 2
#define CS_LEN 2
#define TAIL_LEN 1

#define CHANNEL_485 0x10
#define CHANNEL_IR 0x20
#define CHANNEL_NFC 0x30
#define CHANNEL_MGR 0x00

#define QUEUE_MAX 0x0F

struct frame_struct{
    uint8 data[BUF_SIZE];
    uint16 data_lenth;
    uint8 channel;
#define ACK_SEQ 0xF0
#define DATA_SEQ 0x0F
    uint8 seq;
#define IS_ACTIVE (1 << 7)
#define IS_DATA (1 << 6)
#define IS_ACK (1 << 5)
#define NEED_REPEAT (1 << 4)
    uint8 flag;
    int timerfd;
    int count;
//    struct frame_struct *prev;
    struct frame_struct *next;
};


struct report_data_struct{
    uint8 data[BUF_SIZE];
    uint16 data_lenth;
    uint8 channel;
//    struct report_data_struct *prev;
    struct report_data_struct *next;
};


/*=========升级相关宏定义=========*/

#define PAGE_SIZE       (0x200)
#define APP_OFFSET_ADDR (0x1000)

#define MAIN_CMD_UPGRADE      1
typedef enum
{
    SUB_CMD_UP_REQUEST    = 0x00,
    SUB_CMD_UP_PAGE_WRITE = 0x01,
    SUB_CMD_UP_CHECK      = 0x02,
    SUB_CMD_UP_ERASE      = 0x03,
    SUB_CMD_UP_LAUNCH     = 0x04
}sub_cmd;

#define ACK 0
#define NCK 1
#define RET_OK             0xAA
#define RET_FAIL           0xEE
#define RET_ERR_ADDR       0xCC

#define FRM_LEN_POS           0
#define FRM_MAIN_CMD_POS      2
#define FRM_SUB_CMD_POS       3
#define FRM_ADDR_POS          4
#define FRM_DATA_LEN_POS      7
#define FRM_DATA_POS          9
/*================================*/


/*For android app use*/
int upgrade_fplus(char *path);
int tryOpenTty(void);
void ttyClose(void);
int writeMGR(const uint8 * buf, int len);
int write485(const uint8 *buf, int len);
int writeIR(const uint8 * buf, int len);
int writeNFC(const uint8 * buf, int len);
int readMGR(uint8 * buf);
int read485(uint8 * buf);
int readIR(uint8 * buf);
int readNFC(uint8 * buf);

/*Self use*/
uint16 CRC16_2(const uint8 *puchMsg , uint16 usDataLen, uint8 uchCRCLo, uint8 uchCRCHi);
report_data_struct *report_queue_take_one(uint8 chan);
report_data_struct *report_queue_take_one(uint8 chan);
void frame_queue_add(frame_struct *new_node);
frame_struct *frame_queue_reduce(void);
int queueWrite(const uint8 *buf, int len, uint8 channel);
int queueRead(uint8 *buf, uint8 channel);
report_data_struct *report_queue_take_one(uint8 chan);
void report_queue_add(report_data_struct *new_node);
void frame_queue_add(frame_struct *new_node);
void repeat_queue_add(frame_struct *new_node);
frame_struct *repeat_queue_reduce(void);
void got_ack(uint8 recv_seq);
void send_frame(struct frame_struct *fr_st);
void *frame_send_loop(void*);
void *frame_receive_loop(void*);
int _upgrade_fplus(char *path, int baudrate);

static volatile int ttyfd = -1;
static int report_count = 0;
static uint8 global_seq = 0;
static pthread_mutex_t tty_mutex = PTHREAD_MUTEX_INITIALIZER;

static pthread_t tidSend = -1;
static pthread_t tidReceive = -1;
static pthread_attr_t attrSend, attrReceive;

static int gpio_export_fd = -1;

bus_param_t bus_param = {0};
int save_baudrate = 0;

void gpio_power_on(void)
{
    system("echo 1 > /sys/class/gpio/gpio55/value"); //5V_EN
    system("echo 1 > /sys/class/gpio/gpio93/value"); //NFC_VCC EN
}

void gpio_power_off(void)
{
    system("echo 0 > /sys/class/gpio/gpio55/value");
    system("echo 0 > /sys/class/gpio/gpio93/value");
}

int _ttyOpen(int speed)
{
    int ret;
    struct termios2	options;

    if(0 == pthread_mutex_trylock(&tty_mutex)) {
        if(ttyfd < 0) {
            ttyfd = open("/dev/ttyHSL1", O_RDWR | O_NOCTTY | O_NDELAY);//O_NONBLOCK);
            if(ttyfd < 0) {
                LOGE("ttyOpen failed, errno[%d]\n", errno);
                pthread_mutex_unlock(&tty_mutex);
                return ttyfd;
            }

            memset(&options, 0, sizeof(options));
            ioctl(ttyfd, TCGETS2,&options);

            options.c_cflag &= ~CBAUD;
            options.c_cflag |= BOTHER;

            options.c_ispeed = speed;
            options.c_ospeed = speed;
            //修改控制模式，保证程序不会占用串口
            options.c_cflag |= CLOCAL;
            //修改控制模式，使得能够从串口中读取输入数据
            options.c_cflag |= CREAD;

            options.c_cflag &= ~CRTSCTS;//不使用流控制
            //屏蔽其他标志位
            options.c_cflag &= ~CSIZE;
            //8位数据长度
            options.c_cflag |= CS8;
            //无奇偶校验
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            //1停止位
            options.c_cflag &= ~CSTOPB;
            //修改输出模式，原始数据输出
            options.c_oflag &= ~(OPOST | ONLCR);
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            options.c_iflag &= ~(ICRNL | IXON);

            options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
            options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
            //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
            tcflush(ttyfd,TCIFLUSH);

            if (ioctl(ttyfd, TCSETS2, &options)) {
                LOGE("com set error!\n");
                return -1;
            }

            pthread_attr_init(&attrReceive);
            pthread_attr_setdetachstate(&attrReceive, PTHREAD_CREATE_DETACHED);
            if(-1 == tidReceive) {
                ret = pthread_create(&tidReceive, &attrReceive, frame_receive_loop, NULL);
                LOGD("create thread frame_receive_loop, ret:%d\n", ret);
                if (ret < 0) {
                    LOGE("pthread_create frame_receive_loop failed\n");
                    close(ttyfd);
                    ttyfd = -1;
                    return -1;
                }
            }

            if(-1 == tidSend) {
                pthread_attr_init(&attrSend);
                pthread_attr_setdetachstate(&attrSend, PTHREAD_CREATE_DETACHED);
                ret = pthread_create(&tidSend, &attrSend, frame_send_loop, NULL);
                LOGD("create thread frame_send_loop, ret:%d\n", ret);
                if (ret < 0) {
                    LOGE("pthread_create frame_send_loop failed\n");
                    close(ttyfd);
                    ttyfd = -1;
                    return -1;
                }
            }
        }
        LOGD("tty open success, fd[%d]\n", ttyfd);
        return ttyfd;
    }
    else {
        LOGE("ttyOpen already be inited\n");
        return -1;
    }

}

void _ttyClose(void)
{
    LOGD("_ttyClose\n");
    if(ttyfd >= 0) {
        LOGD("ttyClose fd %d\n", ttyfd);
        close(ttyfd);
        ttyfd = -1;
    }
    pthread_mutex_trylock(&tty_mutex);
    pthread_mutex_unlock(&tty_mutex);
}

void ttyClose(void)
{
    LOGD("ttyClose\n");
    gpio_power_off();
    _ttyClose();
}

void commMGR(uint8 *send, int send_len, uint8 *recv, int *recv_len, int ms_delay);
int tryOpenTty(void)
{
    int ret;

    gpio_power_on();
    ret = _ttyOpen(1250000);
    if(ret >= 0) {
        LOGD("ttyOpen success\n");
    }
    else {
        LOGE("ttyOpen failed\n");
        ttyClose();
    }
    return ret;
}

int write485(const uint8 *buf, int len)
{
    return queueWrite(buf, len, CHANNEL_485);
}

int writeIR(const uint8 *buf, int len)
{
    return queueWrite(buf, len, CHANNEL_IR);
}

int writeNFC(const uint8 *buf, int len)
{
    return queueWrite(buf, len, CHANNEL_NFC);
}

int writeMGR(const uint8 *buf, int len)
{
    return queueWrite(buf, len, CHANNEL_MGR);
}

int read485(uint8 *buf)
{
    return queueRead(buf, CHANNEL_485);
}

int readIR(uint8 *buf)
{
    return queueRead(buf, CHANNEL_IR);
}

int readNFC(uint8 *buf)
{
    return queueRead(buf, CHANNEL_NFC);
}

int readMGR(uint8 *buf)
{
    return queueRead(buf, CHANNEL_MGR);
}


int queueWrite(const uint8 *buf, int len, uint8 channel)
{
    struct frame_struct *new_frame;

    if(len > BUF_SIZE)
        return -1;

    new_frame = (struct frame_struct*) malloc(sizeof(struct frame_struct));

    if(new_frame == NULL)
        return -2;

    new_frame->channel = channel;
    new_frame->flag = IS_DATA;
    new_frame->data_lenth = len;
    memcpy(new_frame->data, buf, len);
    frame_queue_add(new_frame);

    return 0;
}

int queueRead(uint8 *buf, uint8 channel)
{
    struct report_data_struct *temp;
    int ret = -1;

    temp = report_queue_take_one(channel);
    if(temp == NULL) {
        //LOGD("%s, %s, %d\n", __FILE__, __func__, __LINE__);
        return ret;
    }

    ret = temp->data_lenth;
    memcpy(buf, temp->data, ret);
    free(temp);
    return ret;
}


#define RECV_BUF_LEN (BUF_SIZE + 9)
uint8 receive_buf[RECV_BUF_LEN];


/*上报队列*/
struct report_data_struct *report_queue_head;
struct report_data_struct *report_queue_tail;
static pthread_mutex_t report_p_mutex = PTHREAD_MUTEX_INITIALIZER;

void report_queue_add(report_data_struct *new_node)
{
    if(new_node == NULL)
        return;

    pthread_mutex_lock(&report_p_mutex);
    if(report_count > QUEUE_MAX) {
        //LOGD("%s, %s, %d\n", __FILE__, __func__, __LINE__);
        free(new_node);
        pthread_mutex_unlock(&report_p_mutex);
        return;
    }
    report_count++;
    if(report_queue_head == NULL) {
        report_queue_head = new_node;
        //new_node->prev = NULL;
        report_queue_tail = report_queue_head;
    }
    else {
        report_queue_head->next = new_node;
        //new_node->prev = report_queue_head;
        report_queue_head = new_node;
    }
    new_node->next = NULL;
    pthread_mutex_unlock(&report_p_mutex);
}

report_data_struct *report_queue_take_one(uint8 chan)
{
    report_data_struct *temp, *tempbak;
    pthread_mutex_lock(&report_p_mutex);
    if(report_queue_tail == NULL) {
        pthread_mutex_unlock(&report_p_mutex);
        return NULL;
    }

    for(temp = report_queue_tail, tempbak = NULL; temp != NULL;
        tempbak = temp, temp = temp->next) {
        if(temp->channel == chan) {
            break;
        }
    }
    if(temp == NULL) {
        pthread_mutex_unlock(&report_p_mutex);
        return NULL;
    }
    if(report_count > 0)
        report_count--;

    if(report_queue_head == report_queue_tail) {//temp指向队列唯一的节点
        temp->next = NULL;
        //report_queue_head->prev = NULL;
        report_queue_head = NULL;
        report_queue_tail = NULL;
    }else if(temp == report_queue_tail) {
        report_queue_tail = temp->next;
        //report_queue_tail->prev = NULL;
        temp->next = NULL;
    }else if(temp == report_queue_head) {
        report_queue_head = tempbak;
        tempbak->next = NULL;
    }else {
        tempbak->next = temp->next;
    }
    pthread_mutex_unlock(&report_p_mutex);
    return temp;
}
/*上报队列*/


/*发送队列*/
struct frame_struct *frame_queue_head;
struct frame_struct *frame_queue_tail;
static pthread_mutex_t frame_p_mutex = PTHREAD_MUTEX_INITIALIZER;

void frame_queue_add(frame_struct *new_node)
{
    if(new_node == NULL)
        return;

    pthread_mutex_lock(&frame_p_mutex);

    new_node->seq = (++global_seq > DATA_SEQ_MASK) ? (1) : (global_seq);
    if(frame_queue_head == NULL) {
        frame_queue_head = new_node;
        //new_node->prev = NULL;
        frame_queue_tail = frame_queue_head;
    }
    else {
        frame_queue_head->next = new_node;
        //new_node->prev = frame_queue_head;
        frame_queue_head = new_node;
    }
    new_node->next = NULL;
    pthread_mutex_unlock(&frame_p_mutex);
}

frame_struct *frame_queue_reduce(void)
{
    frame_struct *temp;

    pthread_mutex_lock(&frame_p_mutex);

    if(frame_queue_tail == NULL) {
        pthread_mutex_unlock(&frame_p_mutex);
        return NULL;
    }

    temp = frame_queue_tail;
    if(frame_queue_head == frame_queue_tail) {
        temp->next = NULL;
        //frame_queue_head->prev = NULL;
        frame_queue_head = NULL;
        frame_queue_tail = NULL;
    }else {
        frame_queue_tail = temp->next;
        //frame_queue_tail->prev = NULL;
        temp->next = NULL;
    }
    pthread_mutex_unlock(&frame_p_mutex);
    return temp;
}

/*重发队列*/
struct frame_struct *repeat_queue_head;
struct frame_struct *repeat_queue_tail;
static pthread_mutex_t repeat_p_mutex = PTHREAD_MUTEX_INITIALIZER;

void repeat_queue_add(frame_struct *new_node)
{
    if(new_node == NULL)
        return;

    pthread_mutex_lock(&repeat_p_mutex);

    if(repeat_queue_head == NULL) {
        repeat_queue_head = new_node;
        //new_node->prev = NULL;
        repeat_queue_tail = frame_queue_head;
    }
    else {
        repeat_queue_head->next = new_node;
        //new_node->prev = frame_queue_head;
        repeat_queue_head = new_node;
    }
    new_node->next = NULL;
    pthread_mutex_unlock(&repeat_p_mutex);
}

frame_struct *repeat_queue_reduce(void)
{
    frame_struct *temp;

    pthread_mutex_lock(&repeat_p_mutex);

    if(repeat_queue_tail == NULL) {
        pthread_mutex_unlock(&repeat_p_mutex);
        return NULL;
    }

    temp = repeat_queue_tail;
    if(repeat_queue_head == repeat_queue_tail) {
        temp->next = NULL;
        //frame_queue_head->prev = NULL;
        repeat_queue_head = NULL;
        repeat_queue_tail = NULL;
    }
    else {
        repeat_queue_tail = temp->next;
        //frame_queue_tail->prev = NULL;
        temp->next = NULL;
    }
    pthread_mutex_unlock(&repeat_p_mutex);
    return temp;
}

void got_ack(uint8 recv_seq)
{
    frame_struct *temp;

    pthread_mutex_lock(&repeat_p_mutex);

    temp = repeat_queue_tail;
    for(;;) {
        if(temp == NULL)
            break;
        if((temp->seq & DATA_SEQ) == ((recv_seq & ACK_SEQ) >> 4)) {
            temp->flag &= ~NEED_REPEAT;
            break;
        }
        temp = temp->next;
    }
    pthread_mutex_unlock(&repeat_p_mutex);
}
/*重发队列*/


/*组帧发送*/
void send_frame(struct frame_struct *fr_st)
{
    uint8 *p = NULL;
    uint16 cs, n = 0, count = 0;
    int ret;
    if(fr_st == NULL || ttyfd < 0) {
        return;
    }
    p = fr_st->data;
    memmove(p + HEAD_LEN + CTRL_LEN + CHAN_LEN + DATALENTH_LEN,
            p, fr_st->data_lenth);
    p[0] = HEAD;
    p[HEAD_LEN] = fr_st->seq;
    p[HEAD_LEN + CTRL_LEN] = fr_st->channel & CHAN_MASK;
    p[HEAD_LEN + CTRL_LEN + CHAN_LEN] = fr_st->data_lenth & 0x00FF;
    p[HEAD_LEN + CTRL_LEN + CHAN_LEN + 1] = fr_st->data_lenth >> 8;
    cs = CRC16_2(p, fr_st->data_lenth + HEAD_LEN +
                    CTRL_LEN + CHAN_LEN + DATALENTH_LEN, 0xFF, 0xFF);
    p[fr_st->data_lenth + HEAD_LEN + CTRL_LEN + CHAN_LEN +
      DATALENTH_LEN] = cs & 0x00FF;
    p[fr_st->data_lenth + HEAD_LEN + CTRL_LEN + CHAN_LEN +
      DATALENTH_LEN + 1] = cs >> 8;
    p[fr_st->data_lenth + HEAD_LEN + CTRL_LEN + CHAN_LEN +
      DATALENTH_LEN + CS_LEN] = TAIL;
    n = fr_st->data_lenth + HEAD_LEN + CTRL_LEN + CHAN_LEN +
        DATALENTH_LEN + CS_LEN + TAIL_LEN;
    LOGD("Send Frame: ");
    for(int i=0;i<n;i++)
        LOGD("%02X ",p[i]);
    LOGD("\n");
    do {
        if(ttyfd > 0)
        {
            ret = write(ttyfd, p + count, n);
            if (ret >= 0) {
                count += ret;
                n -= count;
            }
        }
    } while(n == 0);
    LOGD("Send Finish\n");
}

/*循环发送线程*/
void *frame_send_loop(void *arg)
{
    struct frame_struct *temp;
    uint64 timerfd_timeout_buf;
    uint8 ret;
    struct itimerspec time_out;

    for(;;) {
        if(-1 == tidSend) {
            break;
        }
        temp = frame_queue_reduce();
        if(temp != NULL) {
            //LOGD("TEMP : %ul ,  len:%d, channel:%d\n", temp, temp->data_lenth, temp->channel);
            send_frame(temp);
            if(temp->flag & IS_DATA) {
                temp->flag |= NEED_REPEAT;
                temp->timerfd = timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK);
                if(temp->timerfd >= 0) {
                    time_out.it_interval.tv_sec = 0;
                    time_out.it_interval.tv_nsec = 40000000;//400000000;//400ms周期

                    time_out.it_value.tv_sec = 0;
                    time_out.it_value.tv_nsec = 40000000;//400000000;//400ms初始

                    timerfd_settime(temp->timerfd,0,&time_out,NULL);
                    repeat_queue_add(temp);
                }
            }
        }

        temp = repeat_queue_reduce();
        if(temp != NULL) {
            if((temp->flag & NEED_REPEAT) != NEED_REPEAT) {
                if(temp->timerfd >= 0) {
                    close(temp->timerfd);
                    temp->timerfd = -1;
                }
                free(temp);
            }
            else {
                ret = read(temp->timerfd,
                           &timerfd_timeout_buf,
                           sizeof(timerfd_timeout_buf));
                if(ret == sizeof(timerfd_timeout_buf)) {
                    temp->count += timerfd_timeout_buf;
                    if(temp->count > 3) {
                        temp->flag &= ~NEED_REPEAT;
                    }
                    else {
                        send_frame(temp);
                    }
                }
                repeat_queue_add(temp);
            }
        }
    }
    return NULL;
}

/*  CRC16  Tabs */
const uint8 auchCRCHi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;

const uint8 auchCRCLo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
        0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
        0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
        0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
        0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
        0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
        0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
        0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
        0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
        0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
        0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
        0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
        0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
        0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
        0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
        0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
        0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
        0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};


/*+++
  功能:  CRC16计算函数
  参数:
         uint8 *puchMsg       待计算CRC缓冲区
         uint16 usDataLen     待计算CRC缓冲区的长度
         uint8 uchCRCHi       CRC检验码初始值高字节
         uint8 uchCRCLo       CRC校验码初始值低字节
  返回:
         结果CRC校验码
  描述:
---*/
uint16 CRC16_2(const uint8 *puchMsg , uint16 usDataLen, uint8 uchCRCLo, uint8 uchCRCHi )
{
    uint8 uIndex ;
    while ( usDataLen-- )
    {
        uIndex = uchCRCHi ^ *puchMsg++;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return ( ( (uint16) uchCRCLo<< 8 ) | uchCRCHi);
}


/*解析数据包，按类别填入发送队列(ack)和上报队列(data)*/
//  帧头  控制码  地址  数据长度  数据域  校验码  帧尾
//  AAH   1Byte   1Byte   2Byte    ...    2byte   16H
bool isValidPacket(const unsigned char* p, int len){

    struct report_data_struct  *temp_report;

    //数据域的数据长度值
    uint16 length = p[HEAD_LEN + CTRL_LEN + CHAN_LEN] +
                    ((uint16)p[HEAD_LEN + CTRL_LEN + CHAN_LEN + 1] << 8);

    //校验域的校验值
    uint16 frame_cs = p[HEAD_LEN + CTRL_LEN + CHAN_LEN +
                        DATALENTH_LEN + length]
                      + ((uint16)p[HEAD_LEN + CTRL_LEN + CHAN_LEN +
                                   DATALENTH_LEN + length + 1] << 8);

    //LOGD("Bus frame_cs : %X\n", frame_cs);

    for(int i=0;i<len;i++){
        //LOGD("p[%d] = %X\n", i, p[i]);
    }


    //尾部
    if (p[len -1 ] != TAIL){
        LOGD("p[len - 1] : %d\n", len);
        LOGD("p[len - 1] : %X\n", p[len - 1]);
        LOGD("Bus The tail is not 0x16\n");
        return false;
    }

    //CRC校验
    uint16 crcCheck = 0;
    crcCheck = CRC16_2(p,len - 3 ,0xFF ,0xFF);
    //LOGD("BUS crc : %X", crcCheck);

    if ((crcCheck&0xffFF) != frame_cs){
        LOGD("crcCheck:%X\n",crcCheck);
        LOGD("target frame_cs:%X\n",frame_cs);
        return false;
    }
    //LOGD("isValidPacket True\n");

    if(!(p[HEAD_LEN] & ACK_SEQ_MASK)) {
        got_ack(p[HEAD_LEN] & ACK_SEQ_MASK);
    }

    if((!(p[HEAD_LEN] & DATA_SEQ_MASK)) || (length > 0)) {
        /*add to report queue*/
        temp_report = (struct report_data_struct*)malloc(sizeof(struct report_data_struct));
        if(temp_report == NULL) {
            LOGE("malloc failed\n");
            return true;
        }
        temp_report->data_lenth = length;
        memcpy(temp_report->data, p + HEAD_LEN + CTRL_LEN +
                                  CHAN_LEN + DATALENTH_LEN, length);
        temp_report->channel = p[HEAD_LEN + CTRL_LEN] & CHAN_MASK;
        report_queue_add(temp_report);

    }
    return true;
}

//帧头  控制码  地址  数据长度  数据域  校验码  帧尾
//AAH  1Byte   1Byte   2Byte    ...     2byte   16H
void *frame_receive_loop(void *arg)
{
    int n = -1, useless_count = 0, movcount, temp;
    uint64 timeout;
    int timeoutfd = -1;
    struct itimerspec time_out;
    unsigned char* pBufHead = receive_buf;
    unsigned char* recv_data_tail = pBufHead;
    unsigned char* pTmp = NULL;
    uint16 data_lenth, frame_lenth, recv_lenth;

    for (;;){
        if (ttyfd < 0) {
            //LOGD("Read data continue\n");
            continue;
        }

        //do {
        n = read(ttyfd, recv_data_tail, RECV_BUF_LEN - (recv_data_tail - pBufHead));
        //} while (n < 0 && errno == EINTR);//(n < 0 && errno == EINTR);//while (n < 0);

        if (n > 0) {
            LOGD("recv:\n");
            for (int x = 0; x < n; x++)
                LOGD("%X ", recv_data_tail[x]);
            LOGD("\n");
            recv_data_tail += n;
        }
        else {
            //LOGD("Read continue %d\n", n);
            continue;
        }

        recv_lenth = recv_data_tail - pBufHead;
        for (useless_count = 0; useless_count < recv_lenth; useless_count++) {
            if(pBufHead[useless_count] == HEAD) {
                //LOGD("Bus find HEAD, recv_length %d,  useless_count %d\n", recv_lenth, useless_count);
                if(timeoutfd == -1) {
                    timeoutfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
                    if(timeoutfd >= 0) {
                        time_out.it_interval.tv_sec = 0;
                        time_out.it_interval.tv_nsec = 500000;//1s周期

                        time_out.it_value.tv_sec = 0;
                        time_out.it_value.tv_nsec = 500000;//1s初始

                        timerfd_settime(timeoutfd,0,&time_out,NULL);
                    }
                }
                else {
                    temp = read(timeoutfd, &timeout, sizeof(timeout));
                    if(temp == sizeof(timeout)) {
                        close(timeoutfd);
                        timeoutfd = -1;
                        continue;
                    }
                }
                break;
            }
        }
        movcount = recv_data_tail - pBufHead - useless_count;
        memmove(pBufHead, pBufHead + useless_count, movcount);//解析前，剔除buf头部无用的数据
        recv_data_tail -= useless_count;
        if (n == useless_count)
            continue;
        /*
        **现在我们已经找到了一个帧头字节，并将此后的数据搬移到了
        **buf头部位置。计算出接收到的有效数据长度。
        */
        if(recv_data_tail != pBufHead){
            //LOGD("BUS recv_data_tail : %u\n", recv_data_tail);
            //LOGD("BUS pBufHead : %u\n", pBufHead);
            //LOGD("XXXXXXX: ");
            //for(pTmp = pBufHead; pTmp < recv_data_tail; pTmp++)
            {
                //  LOGD(" %X", *pTmp);
            }
            //LOGD("\n");
        }

        recv_lenth = recv_data_tail - pBufHead;

        /*尝试获取数据域长度*/
        if (recv_lenth >= HEAD_LEN + CTRL_LEN + CHAN_LEN + DATALENTH_LEN) {

            data_lenth = (pBufHead[HEAD_LEN + CTRL_LEN + CHAN_LEN]) + ((uint16)pBufHead[HEAD_LEN + CTRL_LEN + CHAN_LEN + 1] << 8);
            //LOGD("BUS data_len : %X\n", data_lenth);
        }
        else {
            continue;
        }

        /*根据数据域长度，计算整帧长度*/
        frame_lenth = data_lenth + HEAD_LEN + CTRL_LEN + CHAN_LEN +
                      DATALENTH_LEN + CS_LEN + TAIL_LEN;
        //LOGD("FRAME len : %d\n", frame_lenth);
        if (recv_lenth >= frame_lenth) {
            if(isValidPacket(pBufHead, frame_lenth)) {
                memmove(pBufHead, pBufHead + frame_lenth, frame_lenth);
                ///
                recv_data_tail = pBufHead;
                ///
            }
            else {
                ///
                recv_data_tail = pBufHead;
                ///
            }
            if(timeoutfd >= 0) {
                close(timeoutfd);
                timeoutfd = -1;
                ///
                recv_data_tail = pBufHead;
                ///
            }
        }
        else {
            continue;
        }
    }
    return NULL;
}

void mystrncpy(char *dest, const char *src, int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        dest[i] = src[i];
    }
}

void make_frame(unsigned char *out_buf, unsigned int *out_len, sub_cmd cmd, unsigned int addr, unsigned int data_len, const unsigned char *data)
{
    if(SUB_CMD_UP_REQUEST == cmd)
    {
        out_buf[FRM_LEN_POS]        = 0x1;
        out_buf[FRM_LEN_POS+1]      = 0x0;
        out_buf[FRM_MAIN_CMD_POS]   = MAIN_CMD_UPGRADE;
        out_buf[FRM_SUB_CMD_POS]    = cmd;
        *out_len = 4;
    }
    else if(SUB_CMD_UP_PAGE_WRITE == cmd)
    {
        out_buf[FRM_LEN_POS]        = (data_len+6) & 0xFF;
        out_buf[FRM_LEN_POS+1]      = ((data_len+6)>>8) & 0xFF;
        out_buf[FRM_MAIN_CMD_POS]   = MAIN_CMD_UPGRADE;
        out_buf[FRM_SUB_CMD_POS]    = cmd;
        out_buf[FRM_ADDR_POS]       = addr & 0xFF;
        out_buf[FRM_ADDR_POS+1]     = (addr>>8) & 0xFF;
        out_buf[FRM_ADDR_POS+2]     = (addr>>16) & 0xFF;
        out_buf[FRM_DATA_LEN_POS]   = data_len & 0xFF;
        out_buf[FRM_DATA_LEN_POS+1] = (data_len>>8) & 0xFF;
        //strncpy((char*)(out_buf+FRM_DATA_POS), (char*)data, data_len);
        mystrncpy((char*)(out_buf+FRM_DATA_POS), (char*)data, data_len);
        *out_len = data_len + 9;
    }
    else if(SUB_CMD_UP_LAUNCH == cmd)
    {
        out_buf[FRM_LEN_POS]      = 0x01;
        out_buf[FRM_LEN_POS+1]    = 0x00;
        out_buf[FRM_MAIN_CMD_POS] = MAIN_CMD_UPGRADE;
        out_buf[FRM_SUB_CMD_POS]  = cmd;
        *out_len =4;
    }
    else
    {
        *out_len = 0;
    }
}

int up_request(int repeat_num)
{
    unsigned char send_buf[8]      = "\0";
    unsigned int send_len          = 0;
    unsigned char recv_buf[512]    = "\0";
    int  time_out_fd      = -1;
    int  temp             = -1;
    struct itimerspec time_out_set;
    struct itimerspec time_out_get;
    unsigned int repeat_cnt = 0;
    int read_cnt   = 0;
    uint64 timeout = 0;

    make_frame(send_buf, &send_len, SUB_CMD_UP_REQUEST, (unsigned int)0, (unsigned int)0, (unsigned char*)0);

    writeMGR(send_buf, (int)send_len);

    for(;;)
    {

        if(-1 == time_out_fd)
        {
            time_out_fd = timerfd_create(CLOCK_MONOTONIC, 0);
            if(time_out_fd >= 0)
            {
                time_out_set.it_interval.tv_sec = bus_param.request_time_out_period_sec;
                time_out_set.it_interval.tv_nsec = bus_param.request_time_out_period_nsec;//周期

                time_out_set.it_value.tv_sec = bus_param.request_time_out_init_sec;
                time_out_set.it_value.tv_nsec = bus_param.request_time_out_init_nsec;//初始

                timerfd_settime(time_out_fd,0,&time_out_set,NULL);
                continue;
            }
            else
            {
                LOGE("Create time_out_fd failed\n");
                return -1;
            }
        }
        else
        {
            temp = read(time_out_fd, &timeout, sizeof(timeout));
            if(temp == sizeof(timeout))
            {
                repeat_cnt++;
                if(repeat_num == repeat_cnt)
                {
                    close(time_out_fd);
                    return -1;
                }
            }
        }

        read_cnt = readMGR(recv_buf);
        LOGD("readMGR : %d\n", read_cnt);
        if(read_cnt > 0)
        {
            LOGD("RECV read_cnt : %d\n", read_cnt);
            if(recv_buf[FRM_MAIN_CMD_POS] == MAIN_CMD_UPGRADE && recv_buf[FRM_SUB_CMD_POS] == SUB_CMD_UP_REQUEST && recv_buf[FRM_DATA_POS] == ACK)
            {
                close(time_out_fd);
                return 0;
            }
        }
        writeMGR(send_buf, (int)send_len);
    }
}

int diablo_request(int repeat_num)
{
    unsigned char send_buf[7]      = {0x00, 0x04, 0x07, 0xAA, 0xBB, 0xCC, 0xDD};
    unsigned char recv_buf[512]    = "\0";
    int  time_out_fd      = -1;
    int  temp             = -1;
    struct itimerspec time_out_set;
    struct itimerspec time_out_get;
    unsigned int repeat_cnt = 0;
    int read_cnt   = 0;
    uint64 timeout = 0;

    writeMGR(send_buf, 7);

    for(;;)
    {

        if(-1 == time_out_fd)
        {
            time_out_fd = timerfd_create(CLOCK_MONOTONIC, 0);
            if(time_out_fd >= 0)
            {
                time_out_set.it_interval.tv_sec = bus_param.request_time_out_period_sec;
                time_out_set.it_interval.tv_nsec = bus_param.request_time_out_period_nsec;//周期

                time_out_set.it_value.tv_sec = bus_param.request_time_out_init_sec;
                time_out_set.it_value.tv_nsec = bus_param.request_time_out_init_nsec;//初始

                timerfd_settime(time_out_fd,0,&time_out_set,NULL);
                continue;
            }
            else
            {
                LOGE("Create time_out_fd failed\n");
                return -1;
            }
        }
        else
        {
            temp = read(time_out_fd, &timeout, sizeof(timeout));
            if(temp == sizeof(timeout))
            {
                repeat_cnt++;
                if(repeat_num == repeat_cnt)
                {
                    close(time_out_fd);
                    return -1;
                }
            }
        }

        read_cnt = readMGR(recv_buf);
        LOGD("readMGR : %d\n", read_cnt);
        if(read_cnt > 0)
        {
            LOGD("RECV read_cnt : %d\n", read_cnt);
            if(recv_buf[FRM_MAIN_CMD_POS] == 0x07)
            {
                close(time_out_fd);
                return 0;
            }
        }
        writeMGR(send_buf, 7);
    }
}


int upgrade_exec(char *path)
{
    unsigned char page_buf[PAGE_SIZE]  = "\0";
    unsigned char send_buf[1024] = "\0";
    unsigned int send_len = 0;
    unsigned char recv_buf[256]  = "\0";
    FILE *fp   = NULL;
    int  wt_count = 0;
    int  rd_count = 0;
    unsigned int wt_addr = APP_OFFSET_ADDR;
    unsigned int rd_addr = 0;
    char write_ok_flag = 1;

    time_t start_tm, tm;

    int timeoutfd = -1;
    uint64 timeout   = -1;
    struct itimerspec time_out;
    int temp = -1;
    int read_cnt = 0;

    int err_cnt = 0;

    fp = fopen(path, "r");
    if(NULL == fp)
    {
        LOGE("Open dat file failed\n");
        return -1;
    }
    else
    {
        LOGE("Open file : %s\n", path);
    }

    start_tm = time(NULL);

    while(1)
    {
        tm = time(NULL);
        if(tm - start_tm > bus_param.upgrade_time_out_sec)
        {
            if(timeoutfd != -1)
            {
                close(timeoutfd);
            }
            LOGE("Out Out Out Out of time\n");
            return -1;
        }

        if(err_cnt > 50)
        {
            LOGE("Write error 20 times! break!");
            return -1;
        }

        if(1 == write_ok_flag)
        {
            wt_count = fread(page_buf, sizeof(char), PAGE_SIZE, fp);
        }
        if(0 == wt_count)
        {
            break;
        }
        make_frame(send_buf, &send_len, SUB_CMD_UP_PAGE_WRITE, wt_addr, (unsigned int)wt_count, page_buf);
        writeMGR(send_buf, (int)send_len);

        for(;;)
        {
            if(timeoutfd == -1)
            {
                LOGD("Create time_out FD\n");
                timeoutfd = timerfd_create(CLOCK_MONOTONIC, 0);
                if(timeoutfd >= 0)
                {
                    LOGD("\nCreate time_out FD OK\n");
                    time_out.it_interval.tv_sec = bus_param.exec_time_out_period_sec;
                    time_out.it_interval.tv_nsec = bus_param.exec_time_out_period_nsec;//周期

                    time_out.it_value.tv_sec = bus_param.exec_time_out_init_sec;
                    time_out.it_value.tv_nsec = bus_param.exec_time_out_init_nsec;//初始

                    timerfd_settime(timeoutfd,0,&time_out,NULL);
                }
                else
                {
                    LOGE("Create time_out FD failed [fd:%d] [%d]\n", timeoutfd, __LINE__);
                }
                continue;
            }
            else
            {
                LOGD("Read timeoutfd\n");
                temp = read(timeoutfd, &timeout, sizeof(timeout));
                LOGD("Read timeoutfd  A TIME [%d]\n", temp);
                if(temp == sizeof(timeout))
                {
                    read_cnt++;
                    if(5 == read_cnt)//超时循环5次未收到ACK信号,退出for循环
                    {
                        LOGD("Not receive ACK , exit 5 loop\n");
                        close(timeoutfd);
                        timeoutfd = -1;
                        write_ok_flag = 0;
                        read_cnt = 0;
                        break;
                    }
                }
            }
            rd_count = readMGR(recv_buf);

            if(rd_count > 0)
            {
                if(recv_buf[FRM_MAIN_CMD_POS] != MAIN_CMD_UPGRADE)
                {
                    LOGD("!!!!!!!!!!!!!!!!!!!!!!Not upgrade main\n");
                    continue;
                }
                if(SUB_CMD_UP_PAGE_WRITE == recv_buf[FRM_SUB_CMD_POS])
                {
                    rd_addr = recv_buf[FRM_ADDR_POS] + (recv_buf[FRM_ADDR_POS+1]<<8) + (recv_buf[FRM_ADDR_POS+2]<<16);
                    LOGD("rd_addr : %x\n", rd_addr);
                    LOGD("wt_addr : %x\n", wt_addr);
                    if(wt_addr == rd_addr)
                    {
                        if(RET_OK == recv_buf[FRM_DATA_POS])
                        {
                            LOGD("!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ADDR  :  %x  OK\n", rd_addr);
                            read_cnt = 0;
                            wt_addr += PAGE_SIZE;
                            write_ok_flag = 1;
                            //Reopen timeout fd
                            close(timeoutfd);
                            timeoutfd = timerfd_create(CLOCK_MONOTONIC, 0);
                            if(timeoutfd >= 0) {
                                LOGD("\nReopen time_out FD OK\n");
                                time_out.it_interval.tv_sec = bus_param.exec_time_out_period_sec;
                                time_out.it_interval.tv_nsec = bus_param.exec_time_out_period_nsec;//周期

                                time_out.it_value.tv_sec = bus_param.exec_time_out_init_sec;
                                time_out.it_value.tv_nsec = bus_param.exec_time_out_init_nsec;//初始

                                timerfd_settime(timeoutfd,0,&time_out,NULL);
                            }
                            else {
                                LOGE("Reopen time_out FD failed [fd:%d] [%d]\n", timeoutfd, __LINE__);
                            }
                            break;
                        }
                        else if(RET_FAIL == recv_buf[FRM_DATA_POS])
                        {
                            err_cnt++;
                            write_ok_flag = 0;
                            //writeMGR(send_buf, wt_count+9);
                            break;
                        }
                        else if(RET_ERR_ADDR == recv_buf[FRM_DATA_POS])
                        {
                            err_cnt++;
                            LOGE("The write addr is error\n");
                            //Some case ?
                        }
                    }
                    else
                    {
                        LOGD("!!!!!!!!!!!!!!!!!!!!!!Read and Write ADDR not fix\n");
                    }
                }
            }
        }
    }
    fclose(fp);
    return 0;
}

int diablo_exec(char *path)
{
    unsigned char page_buf[PAGE_SIZE]  = "\0";
    unsigned char send_buf[1024] = "\0";
    unsigned int send_len = 0;
    unsigned char recv_buf[256]  = "\0";
    FILE *fp   = NULL;
    int  wt_count = 0;
    int  rd_count = 0;
    unsigned int wt_addr = 0x0000;
    unsigned int rd_addr = 0;
    char write_ok_flag = 1;

    time_t start_tm, tm;

    int timeoutfd = -1;
    uint64 timeout   = -1;
    struct itimerspec time_out;
    int temp = -1;
    int read_cnt = 0;
    int err_cnt = 0;
    int diablo = 0;

    fp = fopen(path, "r");
    if(NULL == fp)
    {
        LOGE("Open dat file failed\n");
        return -1;
    }
    else
    {
        LOGE("Open file : %s\n", path);
    }

    start_tm = time(NULL);

    while(1)
    {
        tm = time(NULL);
        if(tm - start_tm > bus_param.upgrade_time_out_sec)
        {
            if(timeoutfd != -1)
            {
                close(timeoutfd);
            }
            LOGE("Out Out Out Out of time\n");
            return -1;
        }

        if(err_cnt > 50)
        {
            LOGE("Write error 20 times! break!");
            return -1;
        }

        if(1 == write_ok_flag)
        {
            wt_count = fread(page_buf, sizeof(char), PAGE_SIZE, fp);
        }
        if(0 == wt_count)
        {
            break;
        }
        else
        {
            diablo += wt_count;
            printf("wt_count:%x, temp:%x\n", wt_count, diablo);
        }
        make_frame(send_buf, &send_len, SUB_CMD_UP_PAGE_WRITE, wt_addr, (unsigned int)wt_count, page_buf);
        writeMGR(send_buf, (int)send_len);

        for(;;)
        {
            if(timeoutfd == -1)
            {
                LOGD("Create time_out FD\n");
                timeoutfd = timerfd_create(CLOCK_MONOTONIC, 0);
                if(timeoutfd >= 0)
                {
                    LOGD("\nCreate time_out FD OK\n");
                    time_out.it_interval.tv_sec = bus_param.exec_time_out_period_sec;
                    time_out.it_interval.tv_nsec = bus_param.exec_time_out_period_nsec;//周期

                    time_out.it_value.tv_sec = bus_param.exec_time_out_init_sec;
                    time_out.it_value.tv_nsec = bus_param.exec_time_out_init_nsec;//初始

                    timerfd_settime(timeoutfd,0,&time_out,NULL);
                }
                else
                {
                    LOGE("Create time_out FD failed [fd:%d] [%d]\n", timeoutfd, __LINE__);
                }
                continue;
            }
            else
            {
                LOGD("Read timeoutfd\n");
                temp = read(timeoutfd, &timeout, sizeof(timeout));
                LOGD("Read timeoutfd  A TIME [%d]\n", temp);
                if(temp == sizeof(timeout))
                {
                    read_cnt++;
                    if(5 == read_cnt)//超时循环5次未收到ACK信号,退出for循环
                    {
                        LOGD("Not receive ACK , exit 5 loop\n");
                        close(timeoutfd);
                        timeoutfd = -1;
                        write_ok_flag = 0;
                        read_cnt = 0;
                        break;
                    }
                }
            }
            rd_count = readMGR(recv_buf);

            if(rd_count > 0)
            {
                if(recv_buf[FRM_MAIN_CMD_POS] != MAIN_CMD_UPGRADE)
                {
                    LOGD("!!!!!!!!!!!!!!!!!!!!!!Not upgrade main\n");
                    continue;
                }
                if(SUB_CMD_UP_PAGE_WRITE == recv_buf[FRM_SUB_CMD_POS])
                {
                    rd_addr = recv_buf[FRM_ADDR_POS] + (recv_buf[FRM_ADDR_POS+1]<<8) + (recv_buf[FRM_ADDR_POS+2]<<16);
                    LOGD("rd_addr : %x\n", rd_addr);
                    LOGD("wt_addr : %x\n", wt_addr);
                    if(wt_addr == rd_addr)
                    {
                        if(RET_OK == recv_buf[FRM_DATA_POS])
                        {
                            LOGD("!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ADDR  :  %x  OK\n", rd_addr);
                            read_cnt = 0;
                            wt_addr += PAGE_SIZE;
                            write_ok_flag = 1;
                            //Reopen timeout fd
                            close(timeoutfd);
                            timeoutfd = timerfd_create(CLOCK_MONOTONIC, 0);
                            if(timeoutfd >= 0) {
                                LOGD("\nReopen time_out FD OK\n");
                                time_out.it_interval.tv_sec = bus_param.exec_time_out_period_sec;
                                time_out.it_interval.tv_nsec = bus_param.exec_time_out_period_nsec;//周期

                                time_out.it_value.tv_sec = bus_param.exec_time_out_init_sec;
                                time_out.it_value.tv_nsec = bus_param.exec_time_out_init_nsec;//初始

                                timerfd_settime(timeoutfd,0,&time_out,NULL);
                            }
                            else {
                                LOGE("Reopen time_out FD failed [fd:%d] [%d]\n", timeoutfd, __LINE__);
                            }
                            break;
                        }
                        else if(RET_FAIL == recv_buf[FRM_DATA_POS])
                        {
                            err_cnt++;
                            write_ok_flag = 0;
                            //writeMGR(send_buf, wt_count+9);
                            break;
                        }
                        else if(RET_ERR_ADDR == recv_buf[FRM_DATA_POS])
                        {
                            err_cnt++;
                            LOGE("The write addr is error\n");
                            //Some case ?
                        }
                    }
                    else
                    {
                        LOGD("!!!!!!!!!!!!!!!!!!!!!!Read and Write ADDR not fix\n");
                    }
                }
            }
        }
    }
    fclose(fp);
    return 0;
}

int launch_app()
{
    unsigned char send_buf[8]      = "\0";
    unsigned int send_len          = 0;
    unsigned char recv_buf[512]    = "\0";
    int  time_out_fd      = -1;
    int  temp             = -1;
    struct itimerspec time_out_set;
    struct itimerspec time_out_get;
    unsigned int repeat_cnt = 0;
    int read_cnt   = 0;
    uint64 timeout = 0;

    make_frame(send_buf, &send_len, SUB_CMD_UP_LAUNCH, (unsigned int)0, (unsigned int)0, (unsigned char*)0);

    writeMGR(send_buf, (int)send_len);

    for(;;)
    {

        if(-1 == time_out_fd)
        {
            time_out_fd = timerfd_create(CLOCK_MONOTONIC, 0);
            if(time_out_fd >= 0)
            {
                time_out_set.it_interval.tv_sec = 0;
                time_out_set.it_interval.tv_nsec = 100000000;//20ms周期

                time_out_set.it_value.tv_sec = 0;
                time_out_set.it_value.tv_nsec = 100000000;//20ms初始

                timerfd_settime(time_out_fd,0,&time_out_set,NULL);
                continue;
            }
            else
            {
                LOGE("Create time_out_fd failed\n");
                return -1;
            }
        }
        else
        {
            temp = read(time_out_fd, &timeout, sizeof(timeout));
            if(temp == sizeof(timeout))
            {
                repeat_cnt++;
                if(10 == repeat_cnt)
                {
                    close(time_out_fd);
                    return -1;
                }
            }
        }

        read_cnt = readMGR(recv_buf);

        if(read_cnt > 0)
        {
            if(recv_buf[FRM_MAIN_CMD_POS] == MAIN_CMD_UPGRADE && recv_buf[FRM_SUB_CMD_POS] == SUB_CMD_UP_LAUNCH && recv_buf[FRM_DATA_POS] == ACK)
            {
                close(time_out_fd);
                return 0;
            }
        }
        writeMGR(send_buf, (int)send_len);
    }
}



void commMGR(uint8 *send, int send_len, uint8 *recv, int *recv_len, int ms_sleep)
{
    int rd_len = 0;
    int try_tms = 0;
    int rd_pos  = 0;
    writeMGR(send, send_len);
    LOGD("*******Send a frame  len:%d******.\n", send_len);
    usleep(ms_sleep * 1000);

    while(1)
    {
        rd_len = readMGR((uint8*)(recv + rd_pos));
        if(-1 != rd_len)
        {
            LOGD("Read len: %d\n", rd_len);
            rd_pos += rd_len;
            try_tms = 0;
        }
        else
        {
            try_tms++;
        }

        if(4 == try_tms)
        {
            LOGD("!!!!!!!!Break!!!!!!!!!\n");
            break;
        }
        usleep(50 * 1000);
    }

    *recv_len = rd_pos;
}

void commIR(uint8 *send, int send_len, uint8 *recv, int *recv_len)
{
    int rd_len = 0;
    int try_tms = 0;
    int rd_pos  = 0;
    writeIR(send, send_len);
    usleep(200 * 1000);

    while(1)
    {
        rd_len = readIR((uint8*)(recv + rd_pos));
        if(-1 != rd_len)
        {
            rd_pos += rd_len;
            try_tms = 0;
        }
        else
        {
            try_tms++;
        }

        if(8 == try_tms)
        {
            break;
        }
        usleep(100 * 1000);
    }

    *recv_len = rd_pos;
}

void comm485(uint8 *send, int send_len, uint8 *recv, int *recv_len)
{
    int rd_len = 0;
    int try_tms = 0;
    int rd_pos  = 0;
    write485(send, send_len);
    usleep(300 * 1000);

    while(1)
    {
        rd_len = read485((uint8*)(recv + rd_pos));
        if(-1 != rd_len)
        {
            rd_pos += rd_len;
            try_tms = 0;
        }
        else
        {
            try_tms++;
        }

        if(7 == try_tms)
        {
            break;
        }
        usleep(100 * 1000);
    }

    *recv_len = rd_pos;
}

void commNFC(uint8 *send, int send_len, uint8 *recv, int *recv_len)
{
    int rd_len = 0;
    int try_tms = 0;
    int rd_pos  = 0;
    writeNFC(send, send_len);
    usleep(300 * 1000);

    while(1)
    {
        rd_len = readNFC((uint8*)(recv + rd_pos));
        if(-1 != rd_len)
        {
            rd_pos += rd_len;
            try_tms = 0;
        }
        else
        {
            try_tms++;
        }

        if(7 == try_tms)
        {
            break;
        }
        usleep(100 * 1000);
    }

    *recv_len = rd_pos;
}

int readVersion()
{
    const uint8 ver_commd[3]= {0x00, 0x00, 0x00};
    uint8 rd_data[30] = {0};
    int try_tms = 0;
    int rd      = 0;
    RED_VER:
    writeMGR((uint8*)ver_commd, 3);
    LOGD("*******Send readVersion*******\n");

    while(1)
    {
        rd = readMGR((uint8*)((uint8*)rd_data));
        if(rd > 0)
        {
            LOGD("Read len: %d\n", rd);
            try_tms = 0;
        }
        else
        {
            try_tms++;
        }

        if(rd > 0)
        {
            LOGD("Try times %d OK\n", try_tms);
            break;
        }
        else if(try_tms >= 10)
        {
            LOGD("!!!!!!!!Break!!!!!!!!!\n");
            try_tms = 0;
            break;
        }
        usleep(100 * 1000);
    }


    if(rd > 0)
    {
        if(0 == rd_data[2])
        {
            printf("Ver rd : %x.%x\n", rd_data[3], rd_data[4]);
        }
    }
    else
    {
        goto RED_VER;
    }

    return 0;
}

int readmeter()
{
    const uint8 ver_commd[3]= {0x00, 0x00, 0x00};
    const uint8 sel_ir_commd[4] = {0x01, 0x00, 0x05, 0x01};
    const uint8 data[12] = {0x68,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x68,0x13,0x00,0xDF,0x16};
    const uint8 re_data[22]= {0xFE, 0xFE, 0xFE, 0xFE, 0x68, 0x76, 0x93, 0x76, 0x03, 0x00, 0x00, 0x68, 0x93, 0x06, 0xA9, 0xC6, 0xA9, 0x36, 0x33, 0x33, 0x9F, 0x16};
    uint8 rd_data[30] = {0};
    int  rd_len = 0;
    int  try_tms= 0;
    int  loop   = 0;
    int  cnt = 0;

    sleep(1);

    RED_VER:
    commMGR((uint8*)ver_commd, 3, (uint8*)rd_data, &rd_len, 200);
    if(rd_len > 0)
    {
        if(0 == rd_data[2])
        {
            printf("Ver rd : %x.%x\n", rd_data[3], rd_data[4]);
        }
    }
    else
    {
        goto RED_VER;
    }


    rd_len = 0;
    SEL_SRC:
    commMGR((uint8*)sel_ir_commd, 4, (uint8*)rd_data, &rd_len, 200);
    if(rd_len > 0)
    {
        if(5 == rd_data[2])
        {
            printf("Sel success\n");
        }
    }
    else
    {
        goto SEL_SRC;
    }


    while(1)
    {
        //sleep(1);
        memset(rd_data, 0, 30);
        commIR((uint8*)data, 12, (uint8*)rd_data, &rd_len);
        if(rd_len == 22)
        {
            if (strncmp((char*)re_data, (char*)rd_data, 13))
            {
                printf("Error data  EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE cnt[%d]\n", cnt);
                for(loop =0; loop < 13; loop++)
                {
                    printf("%x\n", rd_data[loop]);
                }
                //return -1;
            }
            else
            {
                //printf("Ok\n");
            }
        }
        else if(rd_len > 22)
        {
            if (strncmp((char*)re_data, (char*)rd_data, 13))
            {
                printf("Error data  LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE  rd_len[%d], cnt[%d]\n", rd_len, cnt);
                for(loop =0; loop < 13; loop++)
                {
                    printf("%x\n", rd_data[loop]);
                }
                continue;
            }
            printf("Error too long  rd_len[%d], cnt[%d] LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL\n", rd_len, cnt);
            //return -1;
        }
        else
        {
            if (strncmp((char*)re_data, (char*)rd_data, rd_len))
            {
                printf("Error data  SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE  rd_len[%d], cnt[%d]\n", rd_len, cnt);
                for(loop =0; loop < 13; loop++)
                {
                    printf("%x\n", rd_data[loop]);
                }
                continue;
            }
            printf("Error too short rd_len[%d], cnt[%d] SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS\n", rd_len, cnt);
            //return -1;
        }
        cnt++;
        //if(!(cnt%1000))
        {
            printf("Success cnt: %d\n", cnt);
        }
    }

    return 0;
}

int test_readNfc()
{
    const uint8 ver_commd[3]= {0x00, 0x00, 0x00};
    const uint8 sel_ir_commd[4] = {0x01, 0x00, 0x05, 0x01};
    const uint8 data[12] = {0x68,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0x68,0x13,0x00,0xDF,0x16};
    const uint8 WUPA[1]= {0x52};
    uint8 rd_data[30] = {0};
    int  rd_len = 0;
    int  try_tms= 0;
    int  loop   = 0;

    sleep(1);

    RED_VER:
    commMGR((uint8*)ver_commd, 3, (uint8*)rd_data, &rd_len, 200);
    if(rd_len > 0) {
        if(0 == rd_data[2]) {
            printf("Ver rd : %x.%x\n", rd_data[3], rd_data[4]);
        }
    }
    else {
        goto RED_VER;
    }


    rd_len = 0;
    SEL_SRC:
    commMGR((uint8*)sel_ir_commd, 4, (uint8*)rd_data, &rd_len, 200);
    if(rd_len > 0) {
        if(5 == rd_data[2]) {
            printf("Sel success\n");
        }
    }
    else {
        goto SEL_SRC;
    }


    while(1)
    {
        sleep(1);
        memset(rd_data, 0, 30);
        commIR((uint8*)WUPA, 1, (uint8*)rd_data, &rd_len);
        if(rd_len) {
            printf("Read data len %d\n", rd_len);
            for(loop=0; loop < rd_len; loop++)
            {
                printf("  %x", rd_data[rd_len]);
            }
            printf("\n");
        }
    }
}

int test_readmeter()
{
    int ret;
    ret = tryOpenTty();
    if(ret >= 0)
    {
        readmeter();
    }
    else
    {
        printf("TryOpen Failed\n");
    }
    return 0;
}

void testUpgrade(void)
{
    int ret = 0;

    ret = tryOpenTty();
    if(ret < 0)
    {
        LOGE("tryTTYOpen failed\n");
    }
    else
    {
        sleep(1);
        _upgrade_fplus("/data/fp.dat", 1250000);
    }
    ttyClose();
}

void set_upgrade_bus_param(int baud)
{
    if(4800 == baud)
    {
        LOGI("Set 4800.\n");
        bus_param.up_baudrate = 4800;
        bus_param.upgrade_time_out_sec         = 500;
        bus_param.request_time_out_init_sec    = 0;
        bus_param.request_time_out_init_nsec   = 100000000;
        bus_param.request_time_out_period_sec  = 0;
        bus_param.request_time_out_period_nsec = 100000000;
        bus_param.exec_time_out_init_sec       = 1;
        bus_param.exec_time_out_init_nsec      = 300000000;
        bus_param.exec_time_out_period_sec     = 0;
        bus_param.exec_time_out_period_nsec    = 500000000;
    }
    else if(1250000 == baud)
    {
        LOGI("Set 1250000.\n");
        bus_param.up_baudrate = 1250000;
        bus_param.upgrade_time_out_sec         = 50;
        bus_param.request_time_out_init_sec    = 0;
        bus_param.request_time_out_init_nsec   = 5000000;//5000000;
        bus_param.request_time_out_period_sec  = 0;
        bus_param.request_time_out_period_nsec = 5000000;//5000000;
        bus_param.exec_time_out_init_sec       = 0;
        bus_param.exec_time_out_init_nsec      = 50000000;
        bus_param.exec_time_out_period_sec     = 0;
        bus_param.exec_time_out_period_nsec    = 20000000;
    }
    else
    {
        LOGE("set_upgrade_bus_param invalid baudrate\n");
    }
}

int upgrade_fplus(char *path)
{
    _upgrade_fplus(path, 1250000);
}

int _upgrade_fplus(char *path, int baudrate)
{
    int ret = 0;

    set_upgrade_bus_param(baudrate);
    ttyClose();
    sleep(1);
    _ttyOpen(baudrate);
    gpio_power_on();
    ret = up_request(20);
    if(-1 == ret) {
        LOGE("Upgrade request failed\n");
        ttyClose();
        return -1;
    }
    else {
        LOGI("Request success start to upgrade\n");
    }
    ret = upgrade_exec(path);
    if(-1 == ret) {
        ttyClose();
        LOGE("Upgrade %s failed\n", path);
        return -2;
    }
    else {
        LOGD("Upgrade %s success\n", path);
    }

    ttyClose();
    return 0;
}


int diablo(char *path, int baudrate)
{
    int ret = 0;

    set_upgrade_bus_param(baudrate);
    ttyClose();
    sleep(1);
    _ttyOpen(baudrate);
    gpio_power_on();
    sleep(3);
    ret = diablo_request(20);
    if(-1 == ret) {
        LOGE("Diablo request 4800 failed\n");
        ttyClose();
        return -1;
    }
    else {
        LOGI("Diablo success start to upgrade\n");
    }
    set_upgrade_bus_param(1250000);
    ret = diablo_exec(path);
    if(-1 == ret) {
        ttyClose();
        LOGE("Diablo %s failed\n", path);
        return -2;
    }
    else {
        LOGD("Upgrade %s success\n", path);
    }

    ttyClose();
    return 0;
}


int test_set_baudrate()
{
    const uint8 sel_ir_commd[4] = {0x01, 0x00, 0x05, 0x00};
    uint8 set_param[10]         = {0x07, 0x00, 0x03, 0x00, 0xC0, 0x12, 0x00, 0x08, 0x02, 0x01};
    uint8 rd_data[30]           = {0};
    const uint8 send_buf[4]     = {0x00, 0x11, 0x22, 0x33};
    int  rd_len = 0;


    SEL_SRC:
    commMGR((uint8*)sel_ir_commd, 4, (uint8*)rd_data, &rd_len, 200);
    if(rd_len > 0) {
        if(5 == rd_data[2]) {
            printf("Sel success\n");
        }
    }
    else {
        goto SEL_SRC;
    }

    SET_PAR:
    commMGR((uint8*)set_param, 10, (uint8*)rd_data, &rd_len, 200);
    if(rd_len > 0) {
        if(3 == rd_data[2]) {
            printf("Set 4800 success\n");
        }
    }
    else {
        goto SET_PAR;
    }


    while(1)
    {
        write485(send_buf, sizeof(send_buf));
        sleep(1);
    }
}

int diablo_tool(int argc, char **argv)
{
    int ret;

    if(argc < 2)
    {
        LOGE("Please input param 'app' or 'diablo'\n");
        return -1;
    }
    if(!strcmp("app", *(argv+1)))
    {
        ret = _upgrade_fplus("/data/app.dat", 4800);
        if(ret)
        {
            return -1;
        }
        return 0;
    }
    else if(!strcmp("diablo", *(argv+1)))
    {
        ret = diablo("/data/diablo.dat", 1250000);
        if(ret)
        {
            return -2;
        }
        return 0;
    }
    else if(!strcmp("leon", *(argv+1)))
    {
        ret = _upgrade_fplus("/data/leon.dat", 1250000);
        if(ret)
        {
            return -1;
        }
        return 0;
    }
    else if(!strcmp("readver", *(argv+1)))
    {
        if(argc < 3)
        {
            printf(
                    "Please input delay param\n");
            return -1;
        }
        printf("Delay %d\n", atoi(*(argv+2)));
        tryOpenTty();
        usleep(atoi(*(argv+2)));
        readVersion();
        ttyClose();
        return -1;
    }
    else
    {
        LOGE("Wrong param\n");
        LOGE("Please input param 'app' or 'diablo'\n");
        return -1;
    }
}
