package com.example.xiaojin20135.topsdevlibs;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.TextView;

import com.example.xiaojin20135.comlib.fragment.Base485IRFragment;
import com.example.xiaojin20135.comlib.fragment.BaseReadFragment;
import com.example.xiaojin20135.comlib.help.DataSendBuffer;
import com.example.xiaojin20135.comlib.help.HelpUtils;
import com.example.xiaojin20135.comlib.help.MethodsHelp;

import java.util.HashMap;
import java.util.Map;

import static com.example.xiaojin20135.comlib.help.HelpUtils.channel485;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelFirared;

/**
 * A simple {@link Fragment} subclass.
 */
public class MyIrReadFragment extends Base485IRFragment {

    private Button open_ir_btn,send_btn;
    private TextView result_TV;
    private static MyIrReadFragment myIrReadFragment;

    public MyIrReadFragment() {

    }


    public static MyIrReadFragment getInstance(){
        if(myIrReadFragment == null){
            myIrReadFragment = new MyIrReadFragment();
        }
        return myIrReadFragment;
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_my_ir_read, container, false);
        initView(view);
        initEvents();
        return view;
    }

    private void initView(View view){
        open_ir_btn = (Button)view.findViewById(R.id.open_ir_btn);
        send_btn = (Button)view.findViewById(R.id.send_btn);
        result_TV = (TextView) view.findViewById(R.id.result_TV);
        getActivity().setTitle("红外通道");
    }


    private void initEvents(){
        open_ir_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                openIr();
            }
        });
        send_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sendByIr();
            }
        });
    }

    private void sendByIr(){
        byte[] frame = new byte[]{0x00,0x01,0x02,0x03,0x04};
        DataSendBuffer.DATA_SEND_BUFFER.setDatasSendArr (frame);
        HelpUtils.currentChannel = channelFirared;
        send();
    }

    @Override
    public void showResult(Map map) {
        this.map = map;
    }

    @Override
    public void readDone() {
        super.readDone();
        if(hasData){
            if(map != null && map.get("datas") != null){
                result_TV.setText(map.get("datas").toString());
            }
        }else{

        }
    }

    @Override
    public Map parseLoRa(byte[] receiveBytes) {
        Map map = new HashMap();
        map.put("datas",MethodsHelp.METHODS_HELP.byteToHexString(receiveBytes,receiveBytes.length));
        return map;
    }
}
