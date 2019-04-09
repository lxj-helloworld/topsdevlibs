package com.example.xiaojin20135.topsdevlibs;


import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;

import com.example.xiaojin20135.comlib.fragment.BaseReadFragment;
import com.example.xiaojin20135.comlib.help.DataSendBuffer;
import com.example.xiaojin20135.comlib.help.HelpUtils;
import com.example.xiaojin20135.comlib.help.MethodsHelp;
import com.example.xiaojin20135.comlib.jni.JniMethods;

import java.util.Map;

import static com.example.xiaojin20135.comlib.help.HelpUtils.TYPE_485;
import static com.example.xiaojin20135.comlib.help.HelpUtils.TYPE_LORA;
import static com.example.xiaojin20135.comlib.help.HelpUtils.TYPE_SYSTEM;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channel485;


/**
 * A simple {@link Fragment} subclass.
 */
public class MyReadFragment extends BaseReadFragment {

    private LinearLayout content_485_LL;
    private Button open_485_btn,send_btn;
    public static MyReadFragment myReadFragment;


    public MyReadFragment() {
        // Required empty public constructor
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
        send_btn = (Button)view.findViewById(R.id.send_btn);
        JniMethods.open();
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

    private void sendData(){
        byte[] frame = new byte[]{0x00,0x01,0x02,0x03,0x04};
        DataSendBuffer.DATA_SEND_BUFFER.setDatasSendArr (frame);
        HelpUtils.currentChannel = channel485;
        HelpUtils.protocolType = TYPE_LORA;
        send();
    }

    @Override
    public void showResult(Map map) {

    }

    @Override
    public void readDone() {
        super.readDone();

    }
}
