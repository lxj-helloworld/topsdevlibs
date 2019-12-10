package com.example.xiaojin20135.comlib.help;

import com.example.xiaojin20135.comlib.jni.JniMethods;

/*
* @author lixiaojin
* create on 2019-12-10 17:08
* description: 断电处理
*/
public enum Power {
    POWER;

    public void powerOff(){
        JniMethods.ttyClose();
        JniMethods.closeEth();
        JniMethods.loraClose();
        JniMethods.ttyUSBClose();
    }
}
