package com.example.xiaojin20135.topsdevlibs;

import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.example.xiaojin20135.comlib.fragment.Base485IRFragment;
import com.example.xiaojin20135.comlib.fragment.BaseReadFragment;
import com.example.xiaojin20135.comlib.help.DataSendBuffer;
import com.example.xiaojin20135.comlib.help.HelpUtils;
import com.example.xiaojin20135.comlib.help.MethodsHelp;
import com.example.xiaojin20135.comlib.jni.JniMethods;

import java.util.HashMap;
import java.util.Map;

import static com.example.xiaojin20135.comlib.help.HelpUtils.TYPE_485;
import static com.example.xiaojin20135.comlib.help.HelpUtils.TYPE_LORA;
import static com.example.xiaojin20135.comlib.help.HelpUtils.TYPE_SYSTEM;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channel485;

/**
 * A simple {@link Fragment} subclass.
 */
public class MyReadFragment extends Base485IRFragment {
    private LinearLayout content_485_LL;
    private Button open_485_btn,send_btn;
    private TextView result_TV;
    public static MyReadFragment myReadFragment;

    public MyReadFragment() {

    }

    public static MyReadFragment getInstance() {
        if(myReadFragment == null){
            myReadFragment = new MyReadFragment();
        }
        return myReadFragment;
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_my_read, container, false);
        initView(view);
        initEvents();
        return view;
    }

    private void initView(View view){
        content_485_LL = (LinearLayout)view.findViewById(R.id.content_485_LL);
        open_485_btn = (Button)view.findViewById(R.id.open_485_btn);
        result_TV = (TextView)view.findViewById(R.id.result_TV);
        send_btn = (Button)view.findViewById(R.id.send_btn);
        getActivity().setTitle("485通道");
    }


    private void initEvents(){
        open_485_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                open485();
            }
        });

        send_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sendData();
            }
        });
    }

    /*
    * @author lixiaojin
    * create on 2019-10-26 11:08
    * description: 报文发送
    */
    private void sendData(){
        Log.d(TAG,"" + new HashMap().get("1313").toString());
        byte[] frame = new byte[]{0x68,(byte)0xAA,(byte) 0xAA,(byte)0xAA,(byte)0xAA,(byte)0xAA,(byte)0xAA,0x68,0x11,0x04,0x35,0x34,0x33,0x37,(byte)0xB4,0x16};
        DataSendBuffer.DATA_SEND_BUFFER.setDatasSendArr (frame);
        HelpUtils.currentChannel = channel485;
        send();

    }

    /**
     * 数据展示
     * @param map
     */
    @Override
    public void showResult(Map map) {
        this.map = map;
    }

    /**
     * 报文接收完成后调用
     */
    @Override
    public void readDone() {
        super.readDone();
        if(hasData){ //如果有数据
            result_TV.setText(map.get("datas").toString());
        }else{
            Toast.makeText(getActivity(),"无数据",Toast.LENGTH_LONG).show();
        }
    }

    /**
     * 485报文解析
     * @param receiveBytes
     * @return
     */
    @Override
    public Map parse485(byte[] receiveBytes) {
        Map map = new HashMap();
        map.put("datas",MethodsHelp.METHODS_HELP.byteToHexString(receiveBytes,receiveBytes.length));
        return map;
    }

    @Override
    public void serialPowerSuccess() {
        super.serialPowerSuccess();
        open485();
    }


    @Override
    public void onSaveInstanceState(@NonNull Bundle outState) {
        super.onSaveInstanceState(outState);
        Log.d(TAG,"MyReadFragment  onSaveInstanceState");
    }
}
