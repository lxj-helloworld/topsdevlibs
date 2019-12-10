package com.example.xiaojin20135.comlib.crash;

import android.app.Application;
import android.os.Looper;
import android.os.Process;
import android.os.SystemClock;
import android.util.Log;
import android.widget.Toast;

import com.example.xiaojin20135.comlib.help.Power;

/*
* @author lixiaojin
* create on 2019-12-10 16:58
* description: 异常崩溃处理
*/
public class MyCrashHandler implements Thread.UncaughtExceptionHandler {
    private static final String TAG = "MyCrashHandler";
    private Application mApplication;
    private static MyCrashHandler sMyCrashHandler = new MyCrashHandler();

    public MyCrashHandler(){

    }

    public static MyCrashHandler getInstance(){
        return sMyCrashHandler;
    }

    public void init(Application application){
        this.mApplication = application;
        Thread.setDefaultUncaughtExceptionHandler(sMyCrashHandler);
    }

    @Override
    public void uncaughtException(Thread t, Throwable e) {
        Log.d(TAG,"捕获异常");

        //弹出即将崩溃提示
        new Thread(){
            @Override
            public void run() {
                Looper.prepare();
                Toast.makeText(mApplication,"程序遇到了问题，将会退出，请重新开启",Toast.LENGTH_SHORT).show();
                Looper.loop();
            }
        }.start();

        //断电处理
        Power.POWER.powerOff();

        SystemClock.sleep(3000); //延时3秒退出
        Log.d(TAG,"Process.myPid() = " + Process.myPid() + "   ");
        Process.killProcess(Process.myPid());
        System.exit(0);
    }

}
