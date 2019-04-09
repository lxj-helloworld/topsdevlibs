package com.example.xiaojin20135.comlib.help;

import java.util.ArrayList;

/**
 * Created by lixiaojin on 2018-09-10 14:06.
 * 功能描述：
 * 收到的报文
 */

public enum DataRecieveBuffer {
    DATA_RECIEVE_BUFFER;
    private ArrayList<byte[]> receiveList = new ArrayList<>();

    /**
     * @author lixiaojin
     * @createon 2018-09-10 14:07
     * @Describe 添加新收到的报文
     */
    public byte[] addItem (byte[] receiveItem) {
        receiveList.add (receiveItem);
        //校验当前报文列表是否存在完整的报文，如果有返回，否则返回空
        return completeItem();
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-10 14:11
     * @Describe 校验第一帧完整报文
     * 根据报文编写
     */
    private byte[] completeItem(){
        byte[] temp = receiveList.get (0);
        if(temp != null){
            receiveList.remove(0);
        }
        return temp;
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-10 14:12
     * @Describe 当前收到的报文清理
     */
    public void resetBuffer(){
        receiveList.clear ();
    }
}
