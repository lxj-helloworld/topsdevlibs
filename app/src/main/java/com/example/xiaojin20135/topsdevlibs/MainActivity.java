package com.example.xiaojin20135.topsdevlibs;

import android.content.Intent;
import android.support.annotation.BinderThread;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;

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
                intent.putExtra("channel",0);
                break;
            case R.id.test_ir_btn:
                intent = new Intent(MainActivity.this,ReadActivity.class);
                intent.putExtra("channel",1);
                break;
        }
        if(intent != null){
            startActivity(intent);
        }
    }

}
