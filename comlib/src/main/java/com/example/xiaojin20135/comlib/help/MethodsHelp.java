package com.example.xiaojin20135.comlib.help;

import java.util.Date;

public enum MethodsHelp {
    METHODS_HELP;

    public String byteToHexString(byte[] bytes, int len) {
        if(bytes != null && bytes.length >= len){
            new Date();
            StringBuilder frameStr = new StringBuilder();
            if (bytes == null) {
                return "";
            } else {
                for(int i = 0; i < len; ++i) {
                    frameStr.append((Integer.toHexString(bytes[i] & 255).length() < 2 ? "0" + Integer.toHexString(bytes[i] & 255) : Integer.toHexString(bytes[i] & 255)) + " ");
                }
                new Date();
                return frameStr.toString();
            }
        }else{
            return "";
        }
    }

    /*
    * @author lixiaojin
    * create on 2019/2/28 16:18
    * description: int转String[]
    */
    public String[] intToStringArr(int ip){
        String[] resultArr = new String[4];
        resultArr[0] = ((ip >> 24) & 0xFF) + "";
        resultArr[1] = ((ip >> 16) & 0xFF) + "";
        resultArr[2] = ((ip >> 8) & 0xFF) + "";
        resultArr[3] = ((ip >> 0) & 0xFF) + "";
        return resultArr;
    }
}
