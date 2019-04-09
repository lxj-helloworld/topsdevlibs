package com.example.xiaojin20135.comlib.protocol;

import android.util.Log;

import java.util.Map;

import static com.example.xiaojin20135.comlib.protocol.PipeHelp.CONTROL_BYTE;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.READ_VERSION;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.BAUD_RATE;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.CHANNEL_SET;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.CONTROL_BYTE;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.CRC;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.DATA_BIT;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.GATE_ARR;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.IP_ARR;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.LED;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.LED_CONTROL;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.LED_DATA;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.PARAS_SET;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.PARAS_STATE;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.PIPE_CONTROL_BYTE;
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
 * Created by lixiaojin on 2018-09-13 10:24.
 * 功能描述：
 * 管理通道帧 组帧部分
 */

public enum PiPeFrame {
    PI_PE_FRAME;
    private static final String TAG = "PiPeFrame";
    private byte[] frame = null;
    private int pos = 0;

    /**
     * @author lixiaojin
     * @createon 2018-09-13 10:38
     * @Describe
     * 帧格式
     * 数据域长度、控制码、数据域（1字节管理通道命令+命令内容）
     *  2          1      1+n
     */
    public byte[] frameMake(Map paraMap){
        Log.d (TAG,"paraMpa = " + paraMap.toString ());
        frame = new byte[1024];
        if(paraMap.get (CONTROL_BYTE) == null){
            return null;
        }
        //跳过两个字节的数据长度
        pos = 2;
        //一个字节 控制码
        byte controlByte = (byte)paraMap.get (CONTROL_BYTE);
        frame[pos++] = controlByte;
        //数据域
        //管理通道命令
        switch (controlByte){
            case READ_VERSION: //读版本
                break;
            case UPDATE_COMMAND: //升级命令
                break;
            case SOFT_RESET: //子卡软件复位
                break;
            case PARAS_SET: //参数配置
                paras_set(paraMap);
                break;
            case PARAS_STATE: //参数/状态查询
                paras_read(paraMap);
                break;
            case CHANNEL_SET: //通道设置
                channel_set(paraMap);
                break;
            case LED_CONTROL://LED亮灭控制
                frame[pos++] = (byte)paraMap.get (LED);
                frame[pos++] = (byte)paraMap.get (LED_DATA);
                break;
        }
        completeFrame ();
        return frame;
    }

    /*
     * @author lixiaojin
     * create on 2019/1/28 18:13
     * description: 485参数读取
     */
    private void paras_read(Map paraMap){
        int pieControl = (int)paraMap.get (PIPE_CONTROL_BYTE);
        frame[pos++] = (byte)(pieControl & 0xFF);
    }


    /**
     * @author lixiaojin
     * @createon 2018-09-13 11:12
     * @Describe 参数配置
     */
    private void paras_set(Map paraMap){
        int pieControl = (byte)paraMap.get (PIPE_CONTROL_BYTE);
        frame[pos++] = (byte)(pieControl & 0xFF);
        switch (pieControl){
            //串口
            case 0x00:
                frame_485(paraMap);
                break;
            //红外参数设置
            case 0x01:
                frame_ir(paraMap);
                break;
            //网口参数设置
            case 0x02:
                frame_net(paraMap);
                break;
            //TCP/UDP参数设置
            case 0x03:
                frame_tcp(paraMap);
                break;
        }
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 11:26
     * @Describe 485口
     * 串口发送参数5个字节，
     * 字节1-2 波特率
     * 字节3 字节比特数
     * 字节4 校验 0-无校验；1-奇校验；2-偶校验
     * 字节5 停止位
     */
    private void frame_485(Map paraMap){
        //波特率
        int baud_rate =Integer.parseInt (paraMap.get (BAUD_RATE)+"");
        frame[pos++] = (byte)(baud_rate & 0xFF);
        frame[pos++] = (byte)((baud_rate >> 8) & 0xFF);
        frame[pos++] = (byte)((baud_rate >> 16) & 0xFF);
        //比特字节数
        frame[pos++] = Byte.parseByte (paraMap.get (DATA_BIT)+"");
        //校验位
        frame[pos++] = Byte.parseByte (paraMap.get (CRC)+"");
        //停止位
        frame[pos++] = Byte.parseByte (paraMap.get (STOP_BIT)+"");
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 14:32
     * @Describe 红外参数设置，同485串口
     */
    private void frame_ir(Map paraMap){
        frame_485(paraMap);
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 11:26
     * @Describe 网口
     * IP地址 4字节
     * 子网掩码 4字节
     * 网关 4字节
     */
    private void frame_net(Map paraMap){
        byte[] ipArr = paraMap.get(IP_ARR) == null ? null : (byte[])paraMap.get (IP_ARR);
        byte[] subArr = paraMap.get(SUB_ARR) == null ? null : (byte[])paraMap.get (SUB_ARR);
        byte[] gate_arr = paraMap.get(GATE_ARR) == null ? null : (byte[])paraMap.get (GATE_ARR);
        if(ipArr != null){
            frame[pos++] = ipArr[0];
            frame[pos++] = ipArr[1];
            frame[pos++] = ipArr[2];
            frame[pos++] = ipArr[3];
        }
        if(subArr != null){
            frame[pos++] = subArr[0];
            frame[pos++] = subArr[1];
            frame[pos++] = subArr[2];
            frame[pos++] = subArr[3];
        }
        if(gate_arr != null){
            frame[pos++] = gate_arr[0];
            frame[pos++] = gate_arr[1];
            frame[pos++] = gate_arr[2];
            frame[pos++] = gate_arr[3];
        }
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-13 11:27
     * @Describe TCP/UDP参数设置
     */
    private void frame_tcp(Map paraMap){
        byte protocol_type = Byte.parseByte (paraMap.get (PROTOCOL_TYPE).toString ());
        byte server_model = Byte.parseByte (paraMap.get (SERVER_MODEL).toString ());
        frame[pos++] = (byte)(protocol_type | server_model);
        int port = (byte)paraMap.get (PORT);
        frame[pos++] = (byte)(port & 0xFF);
        frame[pos++] = (byte)((port >> 8) & 0xFF);
        byte[] remote_ip = paraMap.get(REMOTE_IP) == null ? null : (byte[])paraMap.get (REMOTE_IP);
        if(remote_ip != null){
            frame[pos++] = remote_ip[0];
            frame[pos++] = remote_ip[1];
            frame[pos++] = remote_ip[2];
            frame[pos++] = remote_ip[3];
        }
    }



    /*
     * @author lixiaojin
     * create on 2018/12/25 10:04
     * description: 通道设置
     */

    private void channel_set(Map paraMap){
        if(paraMap.get("channel").toString().equals("0")){
            frame[pos++] = 0x00;
        }else if(paraMap.get("channel").toString().equals("1")){
            frame[pos++] = 0x01;
        }else if(paraMap.get("channel").toString().equals("2")){
            frame[pos++] = 0x02;
        }

    }


    /**
     * @author lixiaojin
     * @createon 2018-09-13 14:22
     * @Describe 设置报文长度
     */
    private void completeFrame(){
        int len = pos - 3;
        Log.d (TAG,"len = " + len);
        frame[1] = (byte)((len << 8));
        frame[0] = (byte)(len & 0xFF);
        byte[] completeArr = new byte[pos];
        System.arraycopy (frame,0,completeArr,0,pos);
        frame = completeArr;
    }
}


