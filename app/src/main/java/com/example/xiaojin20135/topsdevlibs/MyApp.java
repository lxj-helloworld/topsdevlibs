package com.example.xiaojin20135.topsdevlibs;

import android.app.Application;

import com.example.xiaojin20135.comlib.crash.MyCrashHandler;

public class MyApp extends Application {
    @Override
    public void onCreate() {
        super.onCreate();
        MyCrashHandler.getInstance().init(this);
    }
}
