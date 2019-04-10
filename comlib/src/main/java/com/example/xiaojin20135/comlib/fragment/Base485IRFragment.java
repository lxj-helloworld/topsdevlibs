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

    private int result = 0;

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        Log.d(TAG,"onAttach");
        result = JniMethods.open();
        if(result > 0){
            Log.d(TAG,"串口打开成功！");
        }else{
            Toast.makeText(getContext(),"串口打开失败！",Toast.LENGTH_LONG).show();
        }
    }

    @Override
    public void onDetach() {
        super.onDetach();
        Log.d(TAG,"onDetach");
        if(result > 0){
            JniMethods.ttyClose();
            Log.d(TAG,"串口关闭");
        }
    }
}
