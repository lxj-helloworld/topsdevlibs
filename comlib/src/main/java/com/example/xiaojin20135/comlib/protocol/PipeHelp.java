package com.example.xiaojin20135.comlib.protocol;

/**
 * Created by lixiaojin on 2018-09-13 10:39.
 * 功能描述：
 */

public enum PipeHelp {
    PIPE_HELP;

    public static final String CONTROL_BYTE = "control";//控制码字段
    public static final String PIPE_CONTROL_BYTE = "pieControl";//管理通道命令
    public static final String BAUD_RATE = "baud_rate";//波特率
    public static final String CRC = "crc";//校验
    public static final String DATA_BIT = "data_bit";//数据位
    public static final String STOP_BIT = "stop_bit";//停止位

    //网口
    public static final String IP_ARR = "ip_arr";//IP地址
    public static final String SUB_ARR = "sub_arr";//子网掩码
    public static final String GATE_ARR = "gate_arr";//网关

    //TCP/UDP参数
    public static final String PROTOCOL_TYPE = "protocol_type";//协议类型
    public static final String SERVER_MODEL = "server_model";//模式（客户端/服务器）
    public static final String PORT = "port";//端口号
    public static final String REMOTE_IP = "remote_ip";//远端IP地址

    public static final byte READ_VERSION = 0x00;//读版本
    public static final byte UPDATE_COMMAND = 0x01;//升级命令
    public static final byte SOFT_RESET = 0x02;//子卡软件复位（F+和PIC芯片同时复位）
    public static final byte PARAS_SET = 0x03;//参数配置
    public static final byte PARAS_STATE = 0x04;//参数/状态查询
    public static final byte CHANNEL_SET = 0x05;//通道设置
    public static final byte LED_CONTROL = 0x06;//LED亮灭控制


    //设置当前收发通道
    public static final byte CHANNEL_485 = 0; //485通道
    public static final byte CHANNEL_IR = 1; //红外通道
    public static final byte CHANNEL_ETH = 2; //网口通道

    //LED 命令字
    public static final String LED = "led";
    public static final String LED_DATA = "led_data";
    public static final byte LED_CLOSE = 0; //LED灭
    public static final byte LED_OPEN = 1; //LED亮
    public static final byte LED_TWINKLE = 2; //LED闪烁






}
