#include <jni.h>
#include <sys/timerfd.h>
#include<android/log.h>
#include "newbusprotocol.cpp"
#include "lora.cpp"
#include "ttyusb.cpp"
#include "eth.cpp"

#include <android/log.h>
#define   LOG_TAG    "LOG_TEST"
#define   LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define   LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define   LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)


extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_readTest(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* buffer = (unsigned char *) new char[len];
    int length = read485(buffer);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_writeTest(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* buffer = (unsigned char *) new char[len];
    int length = write485(buffer,len);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}

//打开串口
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_open(JNIEnv *env, jclass type,jint baudrate) {
    int i = tryOpenTty();
    LOGE("%d", i);
    return i;
}

//尝试打开串口
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_tryOpenTty(JNIEnv *env, jclass type) {
    int i = tryOpenTty();
    LOGE("%d", i);
    return i;
}

//打开串口
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_ttyClose(JNIEnv *env, jclass type) {
    ttyClose();
    return 1;
}



//读取485
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_read485(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* buffer = (unsigned char *) new char[len];
    int length = read485(buffer);
    LOGE("485读取长度  %d", length);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}

//写入485
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_write485(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* bBuffer=(unsigned char*)(env)->GetByteArrayElements(array, 0);
    return write485(bBuffer,len);
}



//写入管理通道
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_writeMGR(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* bBuffer=(unsigned char*)(env)->GetByteArrayElements(array, 0);
    return writeMGR(bBuffer,len);
}

//读取管理通道
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_readMGR(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* buffer = (unsigned char *) new char[len];
    int length = readMGR(buffer);
    LOGE("length=%d" , length);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}


//写入红外
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_writeIR(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* bBuffer=(unsigned char*)(env)->GetByteArrayElements(array, 0);
    return writeIR(bBuffer,len);
}

//读取红外
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_readIR(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* buffer = (unsigned char *) new char[len];
    int length = readIR(buffer);
    LOGE("length=%d" , length);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}


//写入NFC
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_writeNFC(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* bBuffer=(unsigned char*)(env)->GetByteArrayElements(array, 0);
    return writeNFC(bBuffer,len);
}

//读取NFC
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_readNFC(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* buffer = (unsigned char *) new char[len];
    int length = readNFC(buffer);
    LOGE("length=%d" , length);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}


//网口相关net_gpio_power_on
//打开网口
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_openEth(JNIEnv *env, jclass type) {
    int i = netOpen();
    LOGE("打开网口 %d", i);
    return i;
}

/*
* @author lixiaojin
* create on 2019/2/23 14:21
* description: 检查网口是否已经打开
*/

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_netIsOpen(JNIEnv *env, jclass type) {
    int i = netIsOpen();
    LOGE("检查网口是否已经打开 %d", i);
    return i;
//    return 0;
}
/*
* @author lixiaojin
* create on 2019/2/23 13:48
* description: 读取网口连接状态
 * 1 连接成功
 * 0 未连接
 * -1 错误
*/
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_netCheckConnected(JNIEnv *env, jclass type) {
    int i = netCheckConnected();
    LOGE("网络连接状态 %d", i);
    return i;
//    return 0;
}


//关闭网口
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_closeEth(JNIEnv *env, jclass type) {
    netClose();
    LOGE("关闭网口 ");
    return 0;
}

//设置本机工作模式
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_setNetMode(JNIEnv *env, jclass type,jint mode) {
    setNetMode(mode);
    LOGE("设置本机工作模式 mode = %d",mode);
    return 0;
}

//设置协议类型
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_setProtocolType(JNIEnv *env, jclass type,jint protocol_type) {
    setProtocolType(protocol_type);
    LOGE("设置协议类型 protocol_type = %d",protocol_type);
    return 0;
}

//设置IP地址
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_setLocalIp(JNIEnv *env, jclass type,jint ip) {
    setLocalIp(ip);
    LOGE("设置IP地址 ip = %d",ip);
    return 0;
}

//设置子网掩码
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_setSubMask(JNIEnv *env, jclass type,jint mask) {
    setSubMask(mask);
    LOGE("设置子网掩码 mask = %d",mask);
    return 0;
}

//设置默认网关
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_setGateway(JNIEnv *env, jclass type,jint gateway) {
    setGateway(gateway);
    LOGE("设置默认网关 gateway = %d",gateway);
    return 0;
}

//设置端口
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_setNetPort(JNIEnv *env, jclass type,jint port) {
    setNetPort(port);
    LOGE("设置端口 port = %d",port);
    return 0;
}

//设置远程IP地址
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_setRemoteIp(JNIEnv *env, jclass type,jint remote_ip) {
    setRemoteIp(remote_ip);
    LOGE("设置远程IP地址 remote_ip = %d",remote_ip);
    return 0;
}

//读取本地IP地址
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_getLocalIp(JNIEnv *env, jclass type) {
    return getLocalIp();
}

//读取子网掩码
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_getSubMask(JNIEnv *env, jclass type) {
    return getSubMask();
}

//读取网关地址
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_getGateway(JNIEnv *env, jclass type) {
    return getGateway();
}

//获取协议类型
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_getProtocolType(JNIEnv *env, jclass type) {
    return getProtocolType();
}

//获取本机工作模式
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_getNetMode(JNIEnv *env, jclass type) {
    return getNetMode();
}

//获取端口
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_getNetPort(JNIEnv *env, jclass type) {
    return getNetPort();
}

//获取远端IP地址
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_getRemoteIp(JNIEnv *env, jclass type) {
    return getRemoteIp();
}

//读取网口数据
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_readEth(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* buffer = (unsigned char *) new char[len];
    int length = netRead(buffer,1024);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}

//写入网口数据
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_writeEth(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* bBuffer=(unsigned char*)(env)->GetByteArrayElements(array, 0);
    return netWrite(bBuffer,len);
}




//LoRa相关
//打开LoRa
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_loraOpen(JNIEnv *env, jclass type) {
    int i = loraOpen();
    LOGE("打开LoRa %d", i);
    return i;
}


//关闭LoRa
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_loraClose(JNIEnv *env, jclass type) {
    loraClose();
    LOGE("关闭LoRa ");
    return 0;
}


//读取LoRa数据
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoraRead(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    char* buffer = (char *) new char[len];
    int length = LoraRead(buffer,len);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}

//写入LoRa数据
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoraWrite(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    char* bBuffer=( char*)(env)->GetByteArrayElements(array, 0);
    return LoraWrite(bBuffer,len);
}


//读取LoRa数据
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_AudioRead(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    char* buffer = (char *) new char[len];
    int length = AudioRead(buffer,len);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}

//写入LoRa数据
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_AudioWrite(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    char* bBuffer=( char*)(env)->GetByteArrayElements(array, 0);
    return AudioWrite(bBuffer,len);
}



//配置LoRa通信参数
// LoRaSetFrequency
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetFrequency(JNIEnv *env, jclass type,jint freq) {
    LoRaSetFrequency(freq);
    LOGE(" freq = %d",freq);
    return 0;
}


//LoRaSetPower
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetPower(JNIEnv *env, jclass type,jint power) {
    LoRaSetPower(power);
    LOGE(" power = %d",power);
    return 0;
}


//LoRaSetBandWidth
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetBandWidth(JNIEnv *env, jclass type,jint bw) {
    LoRaSetBandWidth(bw);
    LOGE(" bw = %d",bw);
    return 0;
}

//LoRaSetSpreadingFactor
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetSpreadingFactor(JNIEnv *env, jclass type,jint factor) {
    LoRaSetSpreadingFactor(factor);
    LOGE(" factor = %d",factor);
    return 0;
}


//LoRaSetErrorCoding
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetErrorCoding(JNIEnv *env, jclass type,jint value) {
    LoRaSetErrorCoding(value);
    LOGE(" value = %d",value);
    return 0;
}

//LoRaSetPacketCrcOn
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetPacketCrcOn(JNIEnv *env, jclass type,jint value) {
    LoRaSetPacketCrcOn(value);
    LOGE(" value = %d",value);
    return 0;
}


//LoRaSetPreambleLength
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetPreambleLength(JNIEnv *env, jclass type,jint value) {
    LoRaSetPreambleLength(value);
    LOGE(" value = %d",value);
    return 0;
}


//LoRaSetImplicitHeaderOn
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetImplicitHeaderOn(JNIEnv *env, jclass type,jint value) {
    LoRaSetImplicitHeaderOn(value);
    LOGE(" value = %d",value);
    return 0;
}


//LoRaSetPayloadLength
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetPayloadLength(JNIEnv *env, jclass type,jint value) {
    LoRaSetPayloadLength(value);
    LOGE(" value = %d",value);
    return 0;
}


//LoRaSetPaRamp
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetPaRamp(JNIEnv *env, jclass type,jint value) {
    LoRaSetPaRamp(value);
    LOGE(" value = %d",value);
    return 0;
}


//LoRaSetLowDatarateOptimize
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetLowDatarateOptimize(JNIEnv *env, jclass type,jint value) {
    LoRaSetLowDatarateOptimize(value);
    LOGE(" value = %d",value);
    return 0;
}

//LoRaSetSymbTimeOut
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaSetSymbTimeOut(JNIEnv *env, jclass type,jint value) {
    LoRaSetSymbTimeOut(value);
    LOGE(" value = %d",value);
    return 0;
}

//读取带宽
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaGetBandWidth(JNIEnv *env, jclass type) {
    return LoRaGetBandWidth();
//    return 0;
}

//读取扩频因子
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaGetSpreadingFactor(JNIEnv *env, jclass type) {
    return LoRaGetSpreadingFactor();
//    return 0;
}

//读取中心频率
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_LoRaGetFrequency(JNIEnv *env, jclass type) {
    return LoRaGetFrequency();
//    return 0;
}



//网口升级相关
//读取网口芯片版本号 netReadVersion

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_netReadVersion(JNIEnv *env, jclass type, jintArray bytesArr_) {
//    jint* buffer = new jint[2];
//    jint result = netReadVersion(buffer);
//    LOGE("读取网口芯片程序结果  result=%d" , result);
//    if(result > -1){
//        (*env).SetIntArrayRegion(bytesArr_, 0, 2, (jint *)buffer);
//    }
//    return result;
    return 0;
}

//升级网口芯片程序 netUpgrade

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_netUpgrade(JNIEnv *env, jclass type, jstring path,jstring pwd) {
//    void *strContent = (void *) env->GetStringUTFChars(path, JNI_FALSE);
//    jint result = netUpgrade(strContent);
//    LOGE("升级网口芯片程序结果  result=%d" , result);
//
//    return result;
    return 0;
}



//升级F+芯片程序 netUpgrade
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_upgradeFpuls(JNIEnv *env, jclass type,jstring path_) {
    char *strContent = (char *) env->GetStringUTFChars(path_, JNI_FALSE);
    jint result = upgrade_fplus(strContent);
    LOGE("升级F+芯片程序   result=%d" , result);
    return result;
}




//载波口 打开
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_ttyUSBOpen(JNIEnv *env, jclass type,jint baud) {
    int i = ttyUSBOpen(baud);
    LOGE("载波口 %d", i);
    return i;
}

//载波口 关闭
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_ttyUSBClose(JNIEnv *env, jclass type) {
    int i = ttyUSBClose();
    LOGE("载波口 %d", i);
    return i;
}


//载波口  发送数据
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_ttyUSBWrite(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* bBuffer=(unsigned char*)(env)->GetByteArrayElements(array, 0);
    return ttyUSBWrite(bBuffer,len);
}


//载波口 读取数据
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_xiaojin20135_comlib_jni_JniMethods_ttyUSBRead(JNIEnv *env, jclass type,jbyteArray array,jint len) {
    unsigned char* buffer = (unsigned char *) new char[len];
    int length = ttyUSBRead(buffer,1024);
    if(length > 0){
        (*env).SetByteArrayRegion(array, 0, length, (jbyte*)buffer);
    }
    return length;
}

