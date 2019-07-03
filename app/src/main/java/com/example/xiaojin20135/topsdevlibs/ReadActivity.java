package com.example.xiaojin20135.topsdevlibs;

import android.support.v4.app.Fragment;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.FrameLayout;

import static com.example.xiaojin20135.comlib.help.HelpUtils.channel485;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelFirared;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelLoRa;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelNetPort;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelZb;

public class ReadActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_read);
        initView();
    }



    private void initView(){
        Fragment fragment = null;
        int channel = getIntent().getIntExtra("channel",0);
        if(channel == channel485){ //485
            fragment = MyReadFragment.getInstance();
        }else if(channel == channelFirared){ //红外
            fragment = MyIrReadFragment.getInstance();
        }else if(channel == channelZb){ //载波
            fragment = ZbFragment.getInstance();
        }else if(channel == channelLoRa){//LoRa
            fragment = LoRaFragment.getInstance();
        }else if(channel == channelNetPort){ //网口
            fragment = NetPortFragment.getInstance();
        }
        if(fragment != null){
            getSupportFragmentManager().beginTransaction().replace(R.id.fragment_content,fragment).commit();
        }

    }
}
