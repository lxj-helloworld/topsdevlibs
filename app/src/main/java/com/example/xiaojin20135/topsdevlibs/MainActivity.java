package com.example.xiaojin20135.topsdevlibs;

import android.content.Intent;
import android.support.annotation.BinderThread;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;

import static com.example.xiaojin20135.comlib.help.HelpUtils.channel485;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelFirared;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelLoRa;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelNetPort;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelZb;

public class MainActivity extends AppCompatActivity {


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    public void test(View view) {
        Intent intent = null;
        switch (view.getId()){
            case R.id.test_485_btn:
                intent = new Intent(MainActivity.this,ReadActivity.class);
                intent.putExtra("channel",channel485);
                break;
            case R.id.test_ir_btn:
                intent = new Intent(MainActivity.this,ReadActivity.class);
                intent.putExtra("channel",channelFirared);
                break;
            case R.id.test_lora_btn:
                intent = new Intent(MainActivity.this,ReadActivity.class);
                intent.putExtra("channel",channelLoRa);
                break;
            case R.id.test_zb_btn:
                intent = new Intent(MainActivity.this,ReadActivity.class);
                intent.putExtra("channel",channelZb);
                break;
            case R.id.test_net_btn:
                intent = new Intent(MainActivity.this,ReadActivity.class);
                intent.putExtra("channel",channelNetPort);
                break;
        }
        if(intent != null){
            startActivity(intent);
        }
    }

}
