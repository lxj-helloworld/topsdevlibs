package com.example.xiaojin20135.comlib.jni;

import java.io.File;
import java.io.IOException;

/**
 * Created by lixiaojin on 2018-09-06 14:02.
 * 功能描述：
 */

public class JniMethods {
    /**
     * @author lixiaojin
     * @createon 2018-09-06 8:05
     * @Describe 加载库文件，文件名与实现所需的库函数的C代码文件名相同
     */
    static {
        System.loadLibrary ("native-lib");
    }

    public static void initPort(File device) throws SecurityException, IOException {
		/* Check access permission */
        if (!device.canRead() || !device.canWrite()) {
            try {
				/* Missing read/write permission, trying to chmod the file */
                Process su;
                su = Runtime.getRuntime().exec("su");
                String cmd = "chmod 777 " + device.getAbsolutePath() + "\n"
                    + "exit\n";
                su.getOutputStream().write(cmd.getBytes());
                if ((su.waitFor() != 0) || !device.canRead()
                    || !device.canWrite()) {
                    throw new SecurityException();
                }
            } catch (Exception e) {
                e.printStackTrace();
                throw new SecurityException();
            }
        }
    }

    public static native int tryOpenTty ();
    public static native int ttyClose ();
    public static native int read485(byte[] bytesArr,int len);
    public static native int write485(byte[] bytesArr,int len);
    public static native int readIR(byte[] bytesArr,int len);
    public static native int writeIR(byte[] bytesArr,int len);

    public static native int readNFC(byte[] bytesArr,int len);
    public static native int writeNFC(byte[] bytesArr,int len);

    public static native int writeMGR(byte[] bytesArr,int len);
    public static native int readMGR(byte[] bytesArr,int len);

    //网口操作部分
    //打开网口
    public static native int openEth ();
    //检查网口连接状态
    public static native int netCheckConnected();
    //检查网口是否已经打开
    public static native int netIsOpen();
    //关闭网口
    public static native int closeEth ();
    //配置网口参数
    //设置本地IP地址
    public static native int setLocalIp(int ip);
    //设置本地端口或者远程端口
    public static native int setNetPort(int port);
    //协议类型
    public static native int setProtocolType(int protocol_type);
    //设置子网掩码
    public static native int setSubMask(int mask);
    //设置默认网关
    public static native int setGateway(int gateway);

    //设置本机工作模式
    public static native int setNetMode(int mode);
    //设远程IP地址
    public static native int setRemoteIp(int remote_ip);

    //读取本地IP地址
    public static native int getLocalIp();
    //读取子网掩码
    public static native int getSubMask();
    //读取网关地址
    public static native int getGateway();
    //获取协议类型
    public static native int getProtocolType();
    //获取本机工作模式
    public static native int getNetMode();
    //获取端口
    public static native int getNetPort();
    //获取远端IP地址
    public static native int getRemoteIp();

    public static native int readEth(byte[] bytesArr,int len);
    public static native int writeEth(byte[] bytesArr,int len);

    //LoRa打开
    public static native int loraOpen();
    //LoRa关闭
    public static native int loraClose();
    public static native int LoraRead(byte[] bytesArr,int len);
    public static native int LoraWrite(byte[] bytesArr,int len);


    //设置中心频率
    public static native int LoRaSetFrequency(int freq);
    //设置扩频因子
    public static native int LoRaSetSpreadingFactor(int factor);
    //设置带宽
    public static native int LoRaSetBandWidth(int bw);
    //
    public static native int LoRaSetPower(int power);

    public static native int LoRaSetErrorCoding(int value);

    public static native int LoRaSetPacketCrcOn(int value);

    public static native int LoRaSetPreambleLength(int value);

    public static native int LoRaSetImplicitHeaderOn(int value);

    public static native int LoRaSetPayloadLength(int value);

    public static native int LoRaSetPaRamp(int value);

    public static native int LoRaSetLowDatarateOptimize(int value);

    public static native int LoRaSetSymbTimeOut(int value);




    //读取中心频率
    public static native int LoRaGetFrequency();
    //读取扩频因子
    public static native int LoRaGetSpreadingFactor();
    //读取带宽
    public static native int LoRaGetBandWidth();



    //网口升级相关
    public static native int netReadVersion(int[] bytesArr);
    public static native int netUpgrade(String path, String pwd);

    //F+芯片升级
    public static native int upgradeFpuls(String path);




    //载波口  打开
    public static native int ttyUSBOpen (int baud);
    //载波口  关闭
    public static native int ttyUSBClose ();
    //载波口  发送
    public static native int ttyUSBWrite(byte[] bytesArr,int len);
    //载波口  读取
    public static native int ttyUSBRead(byte[] bytesArr,int len);

}
