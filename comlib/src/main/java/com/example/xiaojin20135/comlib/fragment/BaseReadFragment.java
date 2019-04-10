package com.example.xiaojin20135.comlib.fragment;

import android.app.ProgressDialog;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;


import com.example.xiaojin20135.comlib.help.DataRecieveBuffer;
import com.example.xiaojin20135.comlib.help.DataSendBuffer;
import com.example.xiaojin20135.comlib.help.HelpUtils;
import com.example.xiaojin20135.comlib.help.MethodsHelp;
import com.example.xiaojin20135.comlib.help.ReadDatas;
import com.example.xiaojin20135.comlib.protocol.PiPeFrame;
import com.example.xiaojin20135.comlib.protocol.PipeParse;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import io.reactivex.Observable;
import io.reactivex.ObservableSource;
import io.reactivex.android.schedulers.AndroidSchedulers;
import io.reactivex.disposables.CompositeDisposable;
import io.reactivex.functions.Action;
import io.reactivex.functions.Consumer;
import io.reactivex.functions.Function;
import io.reactivex.functions.Predicate;
import io.reactivex.observers.DisposableObserver;
import io.reactivex.schedulers.Schedulers;

import static com.example.xiaojin20135.comlib.help.HelpUtils.TYPE_SYSTEM;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelManage;
import static com.example.xiaojin20135.comlib.help.HelpUtils.maxWriteCount;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.CHANNEL_SET;
import static com.example.xiaojin20135.comlib.protocol.PipeHelp.CONTROL_BYTE;

/**
 * A simple {@link Fragment} subclass.
 */
public abstract class BaseReadFragment extends Fragment {
    public static String TAG = "BaseReadFragment";

    protected CompositeDisposable compositeDisposable = new CompositeDisposable ();
    protected Observable<Integer> observableWrite = new ReadDatas ().observableWrite;
    protected Observable<byte[]> observableRead = new ReadDatas ().observableRead;
    protected Observable<byte[]> observableReadAlways = new ReadDatas ().observableReadAlways;
    protected DisposableObserver<byte[]> disposableObserverRead;
    protected DataRecieveBuffer dataRecieveBuffer = DataRecieveBuffer.DATA_RECIEVE_BUFFER;

    public int sendFailedCount = 0;//发送失败次数

    //0：管理通道 ； 1：485；  2： 红外 ：
    public int channel = 0;//
    public boolean hasData = false;//标识是否有数据
    public Map map = new HashMap();

    public static ProgressDialog progressDialog;
    public  boolean isShowProgressDialog=true;


    public BaseReadFragment() {
        TAG = this.getClass().getName();
        Log.d(TAG,"TAG = " + TAG);
    }

    @Override
    public View onCreateView (LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        return super.onCreateView (inflater,container,savedInstanceState);
    }

    @Override
    public void onStart () {
        super.onStart ();
    }
    @Override
    public void onDestroy () {
        super.onDestroy ();
        compositeDisposable.dispose ();
    }
    /**
     * @author lixiaojin
     * @createon 2018-09-10 13:56
     * @Describe 初始化
     */
    private void init(){
        sendFailedCount = 0;
        hasData = false;
        //读取
        disposableObserverRead = new DisposableObserver<byte[]> () {
            @Override
            public void onNext (byte[] bytes) {
                Log.d(TAG,"bytes = " + bytes);
                    hasData = true;
                    Log.d(TAG,"收到数据  hasData = " + hasData);
                    parseBytes (bytes);
            }
            @Override
            public void onError (Throwable e) {
            }
            @Override
            public void onComplete () {
                Log.d (TAG,"完成");
            }
        };
    }
    
    /*
    * @author lixiaojin
    * create on 2019/4/9 17:18
    * description: 开启485通道
    */
    public void open485(){
        open(0);
    }

    /*
    * @author lixiaojin
    * create on 2019/4/9 17:19
    * description:开启红外通道
    */
    public void openIr(){
        open(1);
    }

    /*
    * @author lixiaojin
    * create on 2019/4/9 17:20
    * description: 开启NFC
    */
    public void openNfc(){
        open(2);
    }
    

    private void open(int index){
        Map paraMap = new HashMap ();
        paraMap.put (CONTROL_BYTE,CHANNEL_SET); //控制码
        if(index == 1){
            paraMap.put ("channel","1"); //红外
            HelpUtils.currentChannel = channelManage;
        }else if(index == 0){
            paraMap.put ("channel","0"); //485
            HelpUtils.currentChannel = channelManage;
        }else if(index == 2){
            paraMap.put ("channel","2"); //NFC
            HelpUtils.currentChannel = channelManage;
        }
        Log.d(TAG,"paraMap = " + paraMap.toString());
        byte[] frame  = PiPeFrame.PI_PE_FRAME.frameMake (paraMap);
        DataSendBuffer.DATA_SEND_BUFFER.setDatasSendArr (frame);
        Log.d (TAG,"frame = " + MethodsHelp.METHODS_HELP.byteToHexString (frame,frame.length));
        HelpUtils.protocolType = TYPE_SYSTEM;
        send();
    }
    /**
     * @author lixiaojin
     * @createon 2018-09-10 13:59
     * @Describe 启动发送任务
     */
    public void send(){
        init();
        showProgress();
        observableWrite
            .subscribeOn (Schedulers.single ()) //指定Observable的subscribe方法在后台线程执行
            .observeOn (AndroidSchedulers.mainThread ()) //指定observer的回调方法运行在主线程
            .doOnNext (new Consumer<Integer> () {
                @Override
                public void accept (Integer length) throws Exception {
                    Log.d (TAG,"length = " + length);
                    if(length >= 0){
                        Log.d (TAG,"发送成功，返回发送长度，启动读取任务");
                    }else{
                        Log.d (TAG,"发送失败，即将重试");
                        sendFailedCount++;
                    }
                }
            })
            .doFinally (new Action () {
                @Override
                public void run () throws Exception {
                    Log.d (TAG,"写任务结束 sendFailedCount = " +sendFailedCount);
                    if(sendFailedCount >= maxWriteCount){ //如果超过最大允许发送次数，提示错误信息
                        showError ("发送失败，失败次数：" + sendFailedCount);
                    }
                }
            })
            .observeOn (Schedulers.io ())
            .skipWhile (new Predicate<Integer> () {
                @Override
                public boolean test (Integer integer) throws Exception {
                    //如果条件为true，跳过
                    Log.d(TAG,"integer = " + integer);
                    return integer < 0;
                }
            })
            .flatMap (new Function<Integer, ObservableSource<byte[]>> () {
                @Override
                public ObservableSource<byte[]> apply (Integer length) throws Exception {
                    //启动读取任务
                    Log.d(TAG,"启动读取任务");
                    return observableRead;
                }
            })
            .observeOn (AndroidSchedulers.mainThread ())
            .doFinally (new Action () {
                @Override
                public void run () throws Exception {
                    Log.d (TAG,"结束！hasData = " + hasData);
                    if(HelpUtils.currentChannel != HelpUtils.channelManage){ //如果当前不是管理通道
                        readDone();
                    }else{
                        dismissProgress();
                    }
                }
            })
            .subscribe (disposableObserverRead);
        compositeDisposable.add (disposableObserverRead);
    }


    /*
    * @author lixiaojin
    * create on 2019/1/25 08:12
    * description: 启动接收任务
    */

    public void receiveStart(){
        init();
        observableReadAlways.subscribeOn(Schedulers.single ())
                .observeOn(AndroidSchedulers.mainThread ())
        .subscribe(disposableObserverRead);
        compositeDisposable.add (disposableObserverRead);
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-10 14:30
     * @Describe 显示错误信息
     */
    private void showError(String errorInfo){
        Toast.makeText (getActivity (),errorInfo,Toast.LENGTH_LONG).show ();
    }

    private void parseBytes(byte[] bytes){

        byte[] receiveBytes = dataRecieveBuffer.addItem (bytes);
        Log.d(TAG,"in parseBytes receiveBytes = " + MethodsHelp.METHODS_HELP.byteToHexString(receiveBytes,receiveBytes.length));
        Map map = new HashMap();
        if(receiveBytes != null){
            if(HelpUtils.currentChannel == HelpUtils.channelManage){ //管理通道通道
                map = PipeParse.PIPE_PARSE.parse(receiveBytes);
            }else if(HelpUtils.currentChannel == HelpUtils.channel485){ //485通道
                map = parse485(receiveBytes);
            }else if(HelpUtils.currentChannel == HelpUtils.channelFirared){ //红外
                map = parseIr(receiveBytes);
            }else if(HelpUtils.currentChannel == HelpUtils.channelNetPort){ //网口通道
                map = parseNet(receiveBytes);
            }else if(HelpUtils.currentChannel == HelpUtils.channelLoRa){ //LoRa通道
                map = parseLoRa(receiveBytes);
            }else if(HelpUtils.currentChannel == HelpUtils.channelNFC){//NFC通道
                map = parseNfc(receiveBytes);
            }
            Log.d(TAG,"map = " + map.toString());
            if(HelpUtils.currentChannel == HelpUtils.channelManage){
                showSysMessage(map);
            }else{
                showResult (map);
            }
        }else{
            showResult (map);
        }
    }

    private void showSysMessage(Map map){
        if(map != null && map.get("open") != null){
            Toast.makeText(getActivity(),"操作成功！",Toast.LENGTH_LONG).show();
        }else{
            Toast.makeText(getActivity(),"操作失败，请重试！",Toast.LENGTH_LONG).show();
        }
    }


    /**
     * @author lixiaojin
     * @createon 2018-09-10 14:52
     * @Describe 展示结果
     */
    public abstract void showResult(Map map);

    /**
     * 读取完成，无数据
     */
    public void readDone(){
        dismissProgress();
    }

    public int getChannel() {
        return channel;
    }

    public void setChannel(int channel) {
        this.channel = channel;
    }

    /*
    * @author lixiaojin
    * create on 2019/1/25 08:42
    * description: 接受中
    */
    public void receiving(){

    }

    /*
    * @author lixiaojin
    * create on 2019/4/9 10:01
    * description: 485口协议解析
    */
    public Map parse485(byte[] receiveBytes){
        return map;
    }


    /*
     * @author lixiaojin
     * create on 2019/4/9 10:01
     * description: 红外口协议解析
     */
    public Map parseIr(byte[] receiveBytes){
        return map;
    }


    /*
     * @author lixiaojin
     * create on 2019/4/9 10:01
     * description: 网口协议解析
     */
    public Map parseNet(byte[] receiveBytes){
        return map;
    }

    /*
     * @author lixiaojin
     * create on 2019/4/9 10:01
     * description: LoRa口协议解析
     */
    public Map parseLoRa(byte[] receiveBytes){
        return map;
    }

    /*
     * @author lixiaojin
     * create on 2019/4/9 10:01
     * description: NFC口协议解析
     */
    public Map parseNfc(byte[] receiveBytes){
        return map;
    }


    public void showProgress () {
        //等待框
        if(isShowProgressDialog){
            if(progressDialog == null || !progressDialog.isShowing ()){
                progressDialog = new ProgressDialog(getContext());
            }
            progressDialog.show();
        }
    }

    public void dismissProgress () {
        if(progressDialog != null){
            progressDialog.hide();
            progressDialog.dismiss();
        }
    }

    public boolean isShowProgressDialog() {
        return isShowProgressDialog;
    }

    public void setShowProgressDialog(boolean showProgressDialog) {
        isShowProgressDialog = showProgressDialog;
    }
}
