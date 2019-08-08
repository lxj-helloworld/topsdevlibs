package com.example.xiaojin20135.comlib.fragment;

import android.content.Context;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;

import com.example.xiaojin20135.comlib.R;
import com.example.xiaojin20135.comlib.jni.JniMethods;

/**
 * A simple {@link Fragment} subclass.
 */
public abstract class Base485IRFragment extends BaseReadFragment {
    //F+芯片上电状态
    private int result = 0;

    @Override
    public void onResume() {
        super.onResume();
        result = JniMethods.tryOpenTty();
        if(result > 0){
            Log.d(TAG,"上电成功！");
            serialPowerSuccess();
        }else{
            serialPowerFailed();
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        if(result > 0){
            JniMethods.ttyClose();
            Log.d(TAG,"断电成功");
        }else{
            Log.d(TAG,"断电失败");
        }
    }

    /*
    * @author lixiaojin
    * create on 2019/7/31 13:46
    * description: 上电成功
    */
    public void serialPowerSuccess(){

    }

    /*
    * @author lixiaojin
    * create on 2019/7/31 13:46
    * description: 上电失败
    */
    public void serialPowerFailed(){
        Toast.makeText(getContext(),"上电失败！",Toast.LENGTH_LONG).show();
    }



}
