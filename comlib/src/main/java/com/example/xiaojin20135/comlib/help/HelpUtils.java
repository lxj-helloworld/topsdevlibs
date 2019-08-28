package com.example.xiaojin20135.comlib.help;


/**
 * Created by lixiaojin on 2018-09-06 14:50.
 * 功能描述：
 */

public enum HelpUtils {
    HELP_UTILS;
        //最大发送次数
    public static final int maxWriteCount = 5;
    //最大读取次数
    public static int maxReadCount = 20;
    //收到报文后，连续收不到多少次后停止
    public static final int maxSerialCount = 2;

    //串口最小通信间隔 单位毫秒
    public static int minSerialInterval = 100;
    //每次发送接收时的最大字节个数
    public static final int normalByteSize = 1024;
    //当前通道
    /**
     * 0 管理通道
     * 1 485
     * 2 红外
     * 3 网口
     * 4 LoRa
     */
    public static  int currentChannel = 0;
    //管理通道
    public static final int channelManage = 0;
    //485通道
    public static final int channel485 = 1;

    //红外通道
    public static final int channelFirared = 2;
    //网口通道
    public static final int channelNetPort = 3;
    //LoRA
    public static final int channelLoRa = 4;
    //NFC通道
    public static final int channelNFC = 5;
    //载波通道
    public static final int channelZb = 6;
    //LoRa音频通道
    public static final int channelLoRaAudio = 7;

    //标识当前传输的报文类型
    public static int protocolType = 0;

    //应用协议
    public static final int TYPE_SYSTEM = 0;
    //485口协议
    public static final int TYPE_485 = 1;
    //网口小板协议
    public static final int TYPE_NET = 2;
    //LoRa 协议
    public static final int TYPE_LORA = 3;
    //NFC协议
    public static final int TYPE_NFC = 4;
    //是否允许一直接收数据
    public static boolean canReceiving = false;


    public static int getMaxReadCount() {
        return maxReadCount;
    }

    public static void setMaxReadCount(int maxReadCount) {
        HelpUtils.maxReadCount = maxReadCount;
    }

    public static int getMinSerialInterval() {
        return minSerialInterval;
    }

    public static void setMinSerialInterval(int minSerialInterval) {
        HelpUtils.minSerialInterval = minSerialInterval;
    }

    public static int getMaxWriteCount() {
        return maxWriteCount;
    }

    public static int getMaxSerialCount() {
        return maxSerialCount;
    }
}
