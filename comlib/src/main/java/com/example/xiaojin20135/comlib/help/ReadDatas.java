package com.example.xiaojin20135.comlib.help;

import android.util.Log;

import com.example.xiaojin20135.comlib.jni.JniMethods;

import java.util.concurrent.TimeUnit;

import io.reactivex.Observable;
import io.reactivex.ObservableEmitter;
import io.reactivex.ObservableOnSubscribe;
import io.reactivex.ObservableSource;
import io.reactivex.functions.Action;
import io.reactivex.functions.Consumer;
import io.reactivex.functions.Function;

import static com.example.xiaojin20135.comlib.help.HelpUtils.canReceiving;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channel485;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelFirared;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelLoRa;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelLoRaAudio;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelManage;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelNFC;
import static com.example.xiaojin20135.comlib.help.HelpUtils.channelNetPort;
import static com.example.xiaojin20135.comlib.help.HelpUtils.maxReadCount;
import static com.example.xiaojin20135.comlib.help.HelpUtils.maxSerialCount;
import static com.example.xiaojin20135.comlib.help.HelpUtils.maxWriteCount;
import static com.example.xiaojin20135.comlib.help.HelpUtils.minSerialInterval;
import static com.example.xiaojin20135.comlib.help.HelpUtils.normalByteSize;

/**
 * Created by lixiaojin on 2018-09-06 16:23.
 * 功能描述：
 */
public class ReadDatas {
    private static final String TAG = "ReadDatas";


    /**
     * @author lixiaojin
     * @createon 2018-09-07 11:27
     * @Describe 启动从串口读取数据部分
     * 固定时延轮询
     * intervalRange 参数说明：
     * start：发送数据的起始值
     * count:总共发送多少数据
     * initalDelay:发送第一个数据项的起始延时
     * period:两项数据之前的间隔时间
     * TimeUnit：时间单位
     * intervalRange (0,5,0,3000,TimeUnit.MILLISECONDS)
     * 表示从0开始输出5个数据，延迟0秒执行，每隔3秒执行一次
     * take:表示只取前n项。这里用take和interval操作符联合使用，由于一旦interval计时开始除了解绑就无法停止，
     * 使用take操作符就简单很多了，它的意思是只释放前n项，过后Observable流就自动终止
     */
    public Observable<Long> simpleObservable = Observable
        .intervalRange (0,10,0,3000,TimeUnit.MILLISECONDS)
        .take (5)
        .doOnNext (new Consumer<Long> () {
            @Override
            public void accept (Long aLong) throws Exception {
                Log.d (TAG,"in doOnNext accept. along = " + aLong);
                doWork ();
            }
        })
        .doAfterNext (new Consumer<Long> () {
            @Override
            public void accept (Long aLong) throws Exception {
                Log.d (TAG,"in doAfterNext accept. along = " + aLong);
            }
        });

    private void doWork() {
        long workTime = (long) (Math.random() * 500) + 500;
        try {
            Log.d(TAG, "doWork start, threadId=" + Thread.currentThread().getId());
            Thread.sleep(workTime);
            Log.d(TAG, "doWork finished");
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * @author lixiaojin
     * @createon 2018-09-08 17:00
     * @Describe 串口发送数据任务
     */
    public Observable<Integer> observableWrite = Observable.create (new ObservableOnSubscribe<Integer> () {
        @Override
        public void subscribe (ObservableEmitter<Integer> emitter) throws Exception {
            int datasLen = 0;
            int count = maxWriteCount;
            for(int i=0;i<count;i++){
                if(!emitter.isDisposed ()){
                    Log.d (TAG,"发送数据任务 i  = " + i);
                    try {
                        byte[] toSendArr = DataSendBuffer.DATA_SEND_BUFFER.getDatasSendArr ();
                        if(toSendArr != null){
//                            Log.d (TAG,"toSendArr = " + MethodsUtils.METHODS_UTILS.byteToHexString (toSendArr));
                            if(HelpUtils.currentChannel == 0){  //管理通道
                                datasLen = JniMethods.writeMGR (toSendArr,toSendArr.length);
                                Log.d (TAG,"管理通道 dataLen = " + datasLen);
                            }else if(HelpUtils.currentChannel == 1){ //485通道
                                datasLen = JniMethods.write485 (toSendArr,toSendArr.length);
                                Log.d (TAG,"485通道 dataLen = " + datasLen);
                            }else if(HelpUtils.currentChannel == 2){ //红外通道
                                datasLen = JniMethods.writeIR (toSendArr,toSendArr.length);
                                Log.d (TAG,"红外通道 dataLen = " + datasLen);
                            }else if(HelpUtils.currentChannel == channelNFC){//NFC通道
                                datasLen = JniMethods.writeNFC (toSendArr,toSendArr.length);
                                Log.d (TAG," NFC dataLen = " + datasLen);
                            }else if(HelpUtils.currentChannel == 3){ //网口通道
                                datasLen = JniMethods.writeEth(toSendArr,toSendArr.length);
                                Log.d (TAG,"网口通道 dataLen = " + datasLen);
                            }else if(HelpUtils.currentChannel == channelLoRa){
                                datasLen = JniMethods.LoraWrite(toSendArr,toSendArr.length);
                                Log.d (TAG,"LoRa通道 发送 dataLen = " + datasLen);
                            }else if(HelpUtils.currentChannel == channelLoRaAudio){
                                datasLen = JniMethods.AudioWrite(toSendArr,toSendArr.length);
                                Log.d (TAG,"LoRa音频通道 发送 dataLen = " + datasLen);
                            }
                        }
                        Thread.sleep (0);
                    }catch (InterruptedException e){
                        if(!emitter.isDisposed ()){
                            emitter.onError (e);
                        }
                    }
                    if(datasLen >= 0){
                        count = 0;//停止当前任务
                        //如果发送成功，清理当前已经发送的报文
                        emitter.onNext (datasLen);
                    }else{
                        emitter.onNext (0);
                    }
                }else{
                    Log.d (TAG,"发送数据任务尝试销毁");
                    break;
                }
            }
            if(!emitter.isDisposed ()){
                emitter.onComplete ();
            }
            Log.d (TAG,"发送数据任务完成");
        }
    });


    /**
     * @author lixiaojin
     * @createon 2018-09-07 9:34
     * @Describe 读取操作
     */
    public Observable<byte[]> observableRead = Observable.create (new ObservableOnSubscribe<byte[]> () {
        @Override
        public void subscribe (ObservableEmitter<byte[]> emitter) throws Exception {
            byte[] datas = new byte[normalByteSize];
            Log.d(TAG,"初始化接收数组" );
            byte[] receivedArr = null;
            int length = 0;
            int currentLen = 0;
            int count = maxReadCount;
            receivedArr = new byte[normalByteSize];
            for(int i=0;i<count;i++){
                if(!emitter.isDisposed ()){
                    Log.d (TAG,"读取任务 i  = " + i);
                    try {
                        if(HelpUtils.currentChannel == channelManage){ //管理通道
                            length = JniMethods.readMGR (datas,normalByteSize);
                            Log.d (TAG," 管理通道 length = " +length);
                        }else if(HelpUtils.currentChannel == channel485){ //485通道
                            length = JniMethods.read485 (datas,normalByteSize);
                            Log.d (TAG,"485通道 length = " +length);
                        }else if(HelpUtils.currentChannel == channelFirared){//红外通道
                            length = JniMethods.readIR (datas,normalByteSize);
                            Log.d (TAG," 红外通道 length = " +length);
                        }else if(HelpUtils.currentChannel == channelNFC){//NFC通道
                            length = JniMethods.readNFC (datas,normalByteSize);
                            Log.d (TAG," NFC length = " +length);
                        }else if(HelpUtils.currentChannel == channelNetPort){ //网口通道
                            length = JniMethods.readEth(datas,normalByteSize);
                            Log.d (TAG," 网口通道 length = " +length);
                        }else if(HelpUtils.currentChannel == channelLoRa){
                            datas = new byte[255];
                            length = JniMethods.LoraRead(datas,255);
                            Log.d (TAG,"LoRa通道 接收 length = " + length);
                        }else if(HelpUtils.currentChannel == channelLoRaAudio){
                            datas = new byte[255];
                            length = JniMethods.AudioRead(datas,255);
                            Log.d (TAG,"LoRa音频通道 接收 length = " + length);
                        }
                        if(length > 0){
                            Log.d(TAG,"当前已收到的报文长度 currentLen = " + currentLen);
                            Log.d(TAG,"本次收到 = " + MethodsHelp.METHODS_HELP.byteToHexString(datas,length));
                            System.arraycopy(datas,0,receivedArr,currentLen,length);
                            currentLen = currentLen + length;
                            Log.d(TAG,"拼接后的 = " + MethodsHelp.METHODS_HELP.byteToHexString(receivedArr,currentLen));
                            Log.d(TAG,"拼接本次报文后的新报文长度 currentLen = " + currentLen);
                        }
                        Thread.sleep (minSerialInterval);
                    }catch (InterruptedException e){
                        if(!emitter.isDisposed ()){
                            emitter.onError (e);
                        }
                    }
                    if(length > 0){
                        i = 0;
                        count = maxSerialCount;//停止当前任务
                        Log.d(TAG,"count设置为" + count);
                    }else{
                    }
                    if(i >= (count - 1)){
                        if(currentLen > 0){
                            byte[] resultArr = new byte[currentLen];
                            System.arraycopy(receivedArr,0,resultArr,0,currentLen);
                            Log.d(TAG,"停止");
                            emitter.onNext (resultArr);
                        }
                    }
                }else{
                    Log.d (TAG,"读取任务尝试销毁");
                    break;
                }
            }
            if(!emitter.isDisposed ()){
                emitter.onComplete ();
            }
            Log.d (TAG,"读取任务完成");
        }

    });


    /*
    * @author lixiaojin
    * create on 2019/1/25 15:54
    * description: 一直接收数据
    */
    public Observable<byte[]> observableReadAlways = Observable.create (new ObservableOnSubscribe<byte[]> () {
        @Override
        public void subscribe (ObservableEmitter<byte[]> emitter) throws Exception {
            byte[] datas = new byte[normalByteSize];
            byte[] receivedArr = null;
            int length = 0;
            while (canReceiving){
                if(!emitter.isDisposed ()){
                    Log.d(TAG,"读取中......");
                    try {
                        if(HelpUtils.currentChannel == channelManage){ //管理通道
                            length = JniMethods.readMGR (datas,normalByteSize);
                            Log.d (TAG," 管理通道 length = " +length);
                        }else if(HelpUtils.currentChannel == channel485){ //485通道
                            length = JniMethods.read485 (datas,normalByteSize);
                            Log.d (TAG,"485通道 length = " +length);
                        }else if(HelpUtils.currentChannel == channelFirared){//红外通道
                            length = JniMethods.readIR (datas,normalByteSize);
                            Log.d (TAG," 红外通道 length = " +length);
                        }else if(HelpUtils.currentChannel == channelNFC){//NFC通道
                            length = JniMethods.readIR (datas,normalByteSize);
                            Log.d (TAG," NFC length = " +length);
                        }else if(HelpUtils.currentChannel == channelNetPort){ //网口通道
                            length = JniMethods.readEth(datas,normalByteSize);
                            Log.d (TAG," 网口通道 length = " +length);
                        }else if(HelpUtils.currentChannel == channelLoRa){
                            datas = new byte[255];
                            length = JniMethods.LoraRead(datas,255);
                            Log.d (TAG,"LoRa通道 接收 length = " + length);
                        }else if(HelpUtils.currentChannel == channelLoRa){
                            datas = new byte[255];
                            length = JniMethods.LoraRead(datas,255);
                            Log.d (TAG,"LoRa通道 接收 length = " + length);
                        }
                        if(length > 0){
                            Log.d(TAG,"本次收到 = " + MethodsHelp.METHODS_HELP.byteToHexString(datas,length));
                            receivedArr = new byte[length];
                            System.arraycopy(datas,0,receivedArr,0,length);
                            Log.d(TAG,"拼接后 = " + MethodsHelp.METHODS_HELP.byteToHexString(receivedArr,receivedArr.length));
                            emitter.onNext (receivedArr);
                        }else{
                            emitter.onNext (new byte[0]);
                        }
                        Thread.sleep (minSerialInterval);
                    }catch (InterruptedException e){
                        if(!emitter.isDisposed ()){
                            emitter.onError (e);
                        }
                    }
                }else{
                    Log.d (TAG,"读取任务尝试销毁");
                    break;
                }
            }
            if(!emitter.isDisposed ()){
                emitter.onComplete ();
            }
            Log.d (TAG,"读取任务完成");
        }

    });


    private byte[] testData(long i){
        byte[] datas = new byte[]{1,2,3,4,5,6};
        if(i == 6){
            return datas;
        }else{
            return null;
        }
    }



    public Observable<Long> observablePoll = Observable
        .just (0L)
        .doOnComplete (new Action () {
            @Override
            public void run () throws Exception {
                Log.d (TAG,"doOnComplete run");
                doWork();
            }
        })
        .repeat ()
        .repeatWhen (new Function<Observable<Object>, ObservableSource<Long>> () {
            private long mRepeatCount;
            @Override
            public ObservableSource<Long> apply (Observable<Object> objectObservable) throws Exception {
                return objectObservable.flatMap (new Function<Object, ObservableSource<Long>> () {
                    @Override
                    public ObservableSource<Long> apply (Object o) throws Exception {
                        Log.d (TAG,"mRepeatCount = " + mRepeatCount);
                        if(++mRepeatCount > 10){
                            return Observable.empty ();
                        }
                        return Observable.timer (1000 + mRepeatCount * 10, TimeUnit.MILLISECONDS);
                    }
                });
            }
        });
}
