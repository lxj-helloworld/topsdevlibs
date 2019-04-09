package com.example.xiaojin20135.comlib.help;

/**
 * Created by lixiaojin on 2018-09-17 18:12.
 * 功能描述：
 * 待发送报文
 */

public enum DataSendBuffer {
    DATA_SEND_BUFFER;
    private byte[] datasSendArr = null;

    public byte[] getDatasSendArr () {
        return datasSendArr;
    }

    public void setDatasSendArr (byte[] datasSendArr) {
        this.datasSendArr = datasSendArr;
    }
}
