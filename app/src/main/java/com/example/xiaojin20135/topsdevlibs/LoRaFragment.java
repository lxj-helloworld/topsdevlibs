package com.example.xiaojin20135.topsdevlibs;


import android.content.Context;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.example.xiaojin20135.comlib.fragment.BaseReadFragment;
import com.example.xiaojin20135.comlib.help.DataSendBuffer;
import com.example.xiaojin20135.comlib.help.HelpUtils;
import com.example.xiaojin20135.comlib.jni.JniMethods;

import java.util.Map;

import static com.example.xiaojin20135.comlib.help.HelpUtils.channelLoRa;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelZb;


/**
 * A simple {@link Fragment} subclass.
 */
public class LoRaFragment extends BaseReadFragment {
    private Button send_btn;
    private TextView result_TV;


    public static LoRaFragment loRaFragment;

    public LoRaFragment() {

    }

    public static LoRaFragment getInstance() {
        loRaFragment = new LoRaFragment();
        return loRaFragment;
    }


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_lo_ra, container, false);
        initView(view);
        initEvents();
        return view;

    }

    @Override
    public void showResult(Map map) {
        this.map = map;
    }

    private void initView(View view) {
        send_btn = (Button) view.findViewById(R.id.send_btn);
        result_TV = (TextView) view.findViewById(R.id.result_TV);
    }

    private void initEvents() {
        send_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sendData();
            }
        });
    }

    private void sendData() {
        byte[] frame = new byte[]{0x00, 0x01, 0x02, 0x03, 0x04};
        DataSendBuffer.DATA_SEND_BUFFER.setDatasSendArr(frame);
        HelpUtils.currentChannel = channelLoRa;

        send();
    }

    @Override
    public void readDone() {
        super.readDone();
        if (hasData) { //如果有数据
            result_TV.setText(map.get("datas").toString());
        } else {
            Toast.makeText(getActivity(), "无数据", Toast.LENGTH_LONG).show();
        }
    }

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        //打开载波通道
        JniMethods.loraOpen();
    }

    @Override
    public void onDetach() {
        super.onDetach();
        //关闭载波通道
        JniMethods.loraClose();
    }
}