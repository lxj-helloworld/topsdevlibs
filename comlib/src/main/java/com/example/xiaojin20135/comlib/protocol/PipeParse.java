package com.example.xiaojin20135.comlib.protocol;

import android.util.Log;


import java.util.HashMap;
import java.util.Map;

import static com.example.xiaojin20135.comlib.protocol.PipeHelp.BAUD_RATE;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.CHANNEL_SET;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.CRC;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.DATA_BIT;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.GATE_ARR;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.IP_ARR;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.PARAS_SET;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.PARAS_STATE;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.PORT;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.PROTOCOL_TYPE;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.READ_VERSION;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.REMOTE_IP;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.SERVER_MODEL;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.SOFT_RESET;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.STOP_BIT;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.SUB_ARR;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.UPDATE_COMMAND;

/**
 * Created by lixiaojin on 2018-09-13 14:56.
 * 功能描述：
 * 管理通道帧格式
 */

public enum PipeParse {
    PIPE_PARSE;
    private static final String TAG = "PipeParse";
    private Map dataMap = new HashMap();
    private int pos = 0;
    private byte[] frame;

    public Map parse(byte[] recieve){
        frame = recieve;
        dataMap = new HashMap();
        this.pos = 0;
        if(frame == null){
            return dataMap;
        }
        //解析长度
        if(frame.length < 3){
            return dataMap;
        }
//        dataMap.put("datas",MethodsUtils.METHODS_UTILS.byteToHexString(frame));
        int len = frame[pos++] + (frame[pos++] << 8) + 3;
        Log.d (TAG,"len = " + len);
        //长度不匹配，返回
        if(len > frame.length){
            dataMap.put("message","长度错误");
            return dataMap;
        }
        //解析控制码
        byte controlByte = frame[pos++];
        //管理通道命令
        switch (controlByte){
            case READ_VERSION: //读版本
                prase_version();
                break;
            case UPDATE_COMMAND: //升级命令
                break;
            case SOFT_RESET: //子卡软件复位
                break;
            case PARAS_SET: //参数配置
                parse_read_paras();
                break;
            case PARAS_STATE: //参数/状态查询
                parse_paras();
                break;
            case CHANNEL_SET://通道设置
                channelSet();
                break;
        }
        return dataMap;
    }



    /*
     * @author lixiaojin
     * create on 2018/12/20 08:22
     * description: 版本解析
     */
    private void prase_version() {
        ///F+芯片软件版本
        String softVersion = Integer.toHexString(frame[pos++] & 0xFF) + "." + Integer.toHexString(frame[pos++] & 0xFF);
        //F+芯片软件日期
        String softDate = Integer.toBinaryString(frame[pos++] & 0xFF) + ".";
        softDate = softDate + Integer.toBinaryString(frame[pos++] & 0xFF) + ".";
        softDate = softDate + Integer.toBinaryString(frame[pos++] & 0xFF) + ".";
        softDate = softDate + Integer.toBinaryString(frame[pos++] & 0xFF);
        dataMap.put("softVersion",softVersion);
        dataMap.put("softDate",softDate);
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 15:11
     * @Describe 参数解析
     */
    private void parse_paras(){
        byte pieControl = frame[pos++];
        switch (pieControl){
            //串口
            case 0x00:
                parse_485();
                break;
            //红外参数设置
            case 0x01:
                parse_ir();
                break;
            //网口参数设置
            case 0x02:
                parse_net();
                break;
            //TCP/UDP参数设置
            case 0x03:
                parse_tcp();
                break;
        }
//        parse_485();
    }

    /*
     * @author lixiaojin
     * create on 2019/4/1 09:45
     * description: 参数读取
     */

    private void parse_read_paras(){

    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 15:14
     * @Describe 485串口解析
     */
    private void parse_485(){
        int baud_rate = (frame[pos++] & 0xFF) + (((frame[pos++] & 0xFF)<<8) + (((frame[pos++] & 0xFF)<<16)));
        int data_bit = frame[pos++];
        int crc = frame[pos++];
        int stop_bit = frame[pos++];
        dataMap.put (BAUD_RATE,baud_rate);
        dataMap.put (DATA_BIT,data_bit);
        dataMap.put (CRC,crc);
        dataMap.put (STOP_BIT,stop_bit);
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 15:14
     * @Describe 红外串口解析
     */
    private void parse_ir(){
        parse_485();
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 15:14
     * @Describe 网口参数解析
     */
    private void parse_net(){
        byte[] ipArr = new byte[4];
        for(int i=0;i<4;i++){
            ipArr[i] = frame[pos++];
        }
        byte[] subArr = new byte[4];
        for(int i=0;i<4;i++){
            subArr[i] = frame[pos++];
        }
        byte[] gate_arr = new byte[4];
        for(int i=0;i<4;i++){
            gate_arr[i] = frame[pos++];
        }
        dataMap.put (IP_ARR,ipArr);
        dataMap.put (SUB_ARR,ipArr);
        dataMap.put (GATE_ARR,ipArr);
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 15:14
     * @Describe TCP/UDP参数解析
     */
    private void parse_tcp(){
        byte temp = frame[pos++];
        //协议类型
        byte protocol_type = (byte)(temp & 0x03);
        //模式，客户端/服务器
        byte server_model = (byte)(temp >> 4);
        //端口
        int port = (frame[pos++] & 0xFF) + (frame[pos++] << 8);
        //远程地址
        if(pos < frame.length){
            byte[] remote_ip = new byte[4];
            for(int i=0;i<4;i++){
                remote_ip[i] = frame[pos++];
                dataMap.put (REMOTE_IP,remote_ip);
            }
        }
        dataMap.put (PORT,port);
        dataMap.put (SERVER_MODEL,server_model);
        dataMap.put (PROTOCOL_TYPE,protocol_type);
    }


    private void channelSet() {
        if(frame[this.pos] == 0){
            dataMap.put("open",true);
        }else{
            dataMap.put("open",false);
            dataMap.put("message",frame[this.pos] & 0xFF);
        }
    }
}
