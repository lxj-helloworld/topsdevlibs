package com.example.xiaojin20135.topsdevlibs;

import android.support.v4.app.Fragment;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.FrameLayout;

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
        if(channel == 0){
            fragment = MyReadFragment.getInstance();
        }else if(channel == 1){
            fragment = MyReadFragment.getInstance();
        }

        if(fragment != null){
            getSupportFragmentManager().beginTransaction().replace(R.id.fragment_content,fragment).commit();
        }

    }
}
