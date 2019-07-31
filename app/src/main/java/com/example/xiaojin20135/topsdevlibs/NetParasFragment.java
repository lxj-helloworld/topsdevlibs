package com.example.xiaojin20135.topsdevlibs;


import android.content.Context;
import android.content.DialogInterface;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.Fragment;
import android.text.TextUtils;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.Spinner;

import com.example.xiaojin20135.basemodule.activity.BaseActivity;
import com.example.xiaojin20135.basemodule.util.MethodsUtils;
import com.example.xiaojin20135.basemodule.view.edittext.ip.IPView;
import com.example.xiaojin20135.comlib.fragment.BaseReadFragment;
import com.example.xiaojin20135.comlib.help.DataSendBuffer;
import com.example.xiaojin20135.comlib.help.HelpUtils;
import com.example.xiaojin20135.comlib.jni.JniMethods;

import java.util.Map;

import butterknife.BindView;
import butterknife.ButterKnife;

import static com.example.xiaojin20135.comlib.help.HelpUtils.channelNetPort;


/**
 * A simple {@link Fragment} subclass.
 */
public class NetParasFragment extends BaseReadFragment {
    ///协议类型
    @BindView(R.id.protocol_type_SP)
    Spinner protocol_type_SP;
    //模式
    @BindView (R.id.mode_SP)
    Spinner mode_SP;
    //设置
    @BindView (R.id.set_btn)
    Button set_btn;
    //打开
    @BindView(R.id.open_btn)
    Button open_btn;
    //关闭
    @BindView (R.id.close_btn)
    Button close_btn;
    //发送命令
    @BindView (R.id.read_time_btn)
    Button read_time_btn;
    //当前参数
    @BindView(R.id.current_paras_btn)
    Button current_paras_btn;

    //本地参数
    @BindView(R.id.local_LL)
    LinearLayout local_LL;
    //本地IP地址
    @BindView (R.id.local_ip_IPV)
    IPView local_ip_IPV;
    //本地端口
    @BindView (R.id.local_port_ET)
    EditText local_port_ET;
    //网关地址
    @BindView(R.id.gateway_IPV)
    IPView gateway_IPV;
    //子网掩码
    @BindView(R.id.subnet_IPV)
    IPView subnet_IPV;

    //远端参数
    @BindView(R.id.remote_LL)
    LinearLayout remote_LL;
    @BindView(R.id.remote_ip_IPV)
    IPView remote_ip_IPV;
    @BindView(R.id.remote_port_ET)
    EditText remote_port_ET;

    public static NetParasFragment netFragment;
    private BaseActivity baseActivity;
    //协议类型0：TCP；1：UDP
    private int protocol_type = 0;
    //服务器模式  0  客户端 1、
    private int mode = 0;
    private static final int serverMode = 0; //服务器模式
    private static final int clientMode = 1; //客户端模式
    private byte[] localIpArr,remoteIpArr;
    private int localPort,remotePort;
    //默认网关
    private int gateway = 0;
    //子网掩码
    private int subnet = 0;

    int connect = 0;

    //仅设置参数
    private boolean onlyForParas = false;

    public NetParasFragment() {
        // Required empty public constructor
    }

    public static NetParasFragment getInstance(BaseActivity baseActivity){
        netFragment = new NetParasFragment ();
        netFragment.setBaseActivity (baseActivity);
        return netFragment;
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        View view = inflater.inflate (R.layout.fragment_net_paras, container, false);
        initView (view);
        initEvents (view);
        return view;
    }

    protected void initView (View view) {
        ButterKnife.bind (this,view);
        if(onlyForParas){
            open_btn.setVisibility(View.GONE);
            close_btn.setVisibility(View.GONE);
            read_time_btn.setVisibility(View.GONE);
        }
    }


    public void initPatasView(){
        onlyForParas = true;
    }

    protected void initEvents (View view) {
        protocol_type_SP.setOnItemSelectedListener (new AdapterView.OnItemSelectedListener () {
            @Override
            public void onItemSelected (AdapterView<?> parent, View view, int position, long id) {
                protocol_type = position;
            }
            @Override
            public void onNothingSelected (AdapterView<?> parent) {

            }
        });
        //客户端服务器模式
        mode_SP.setOnItemSelectedListener (new AdapterView.OnItemSelectedListener () {
            @Override
            public void onItemSelected (AdapterView<?> parent, View view, int position, long id) {
                mode = position;
                if(serverMode == mode){ //如果是服务器模式
                    local_LL.setVisibility(View.VISIBLE);
                    remote_LL.setVisibility(View.GONE);
                }else if(clientMode == mode){ //如果是客户端模式
                    local_LL.setVisibility(View.GONE);
                    remote_LL.setVisibility(View.VISIBLE);
                }
            }
            @Override
            public void onNothingSelected (AdapterView<?> parent) {

            }
        });

        //设置网口参数
        set_btn.setOnClickListener (new View.OnClickListener () {
            @Override
            public void onClick (View v) {
                setParas();
            }
        });

        //设置成功后打开网口
        open_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                int openEth  = JniMethods.openEth();
                connect = 0;
                if(openEth > 0){
                    showProgress(false,"尝试建立网络连接",false);
                    final Handler handler = new Handler();
                    handler.postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            Log.d(TAG,"connect = " + connect);
                            int result = 0;
                            if(connect++ < 20){
                                result = JniMethods.netCheckConnected();
                                Log.d(TAG,"result = " + result);
                                if(result == 1){
                                    connect = 20;
                                }else{
                                    handler.postDelayed(this,1000);
                                }
                            }
                            dismissProgress();
                            if(result == 1){
                                Log.d(TAG,"连接成功");
                            }else if(result == 0){
                                Log.d(TAG,"失败");
                            }else if(result == -1){
                                Log.d(TAG,"错误");
                            }
                        }
                    },3000);
                }else{
                    showAlertDialog(getActivity(),"打开网口失败！返回值：" + openEth);
                }
            }
        });

        //关闭网口
        close_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                int closeEth  =JniMethods.closeEth();
                showAlertDialog(getActivity(),"关闭返回值：" + closeEth);
            }
        });

        //发送命令
        read_time_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                byte[] frame  = new byte[300];
                for(int i=0;i<frame.length;i++){
                    frame[i] = (byte) (i+1);
                }
                DataSendBuffer.DATA_SEND_BUFFER.setDatasSendArr (frame);
                Log.d (TAG,"frame = " + MethodsUtils.METHODS_UTILS.byteToHexString (frame));
                HelpUtils.currentChannel = channelNetPort;
                send();
            }
        });

        //当前参数
        current_paras_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showProgress();
                if(JniMethods.netIsOpen() > 0){
                    readParas();
                }else{
                    int open = JniMethods.openEth();
                    if(open > 0){
                        new Handler().postDelayed(new Runnable() {
                            @Override
                            public void run() {
                                Log.d(TAG,"读取");
                                readParas();
                                Log.d(TAG,"关闭");
                                JniMethods.closeEth();
                            }
                        },2500);
                    }else{
                        showAlertDialog(getActivity(),"打开网口失败！");
                    }
                }
            }
        });

    }

    /*
     * @author lixiaojin
     * create on 2019/2/23 14:23
     * description: 读取网口参数
     */
    private void readParas(){
        dismissProgress();
        //本地IP地址
        int localIP = JniMethods.getLocalIp();
        //子网掩码
        int subMask = JniMethods.getSubMask();
        //网关地址
        int gateWay = JniMethods.getGateway();
        //协议类型
        int protocolType = JniMethods.getProtocolType();
        //本机工作模式
        int netMode = JniMethods.getNetMode();
        //端口
        int port = JniMethods.getNetPort();
        //远端IP地址
        int remoteIp = JniMethods.getRemoteIp();

        //本地IP地址
        String[] localIpStrArr = MethodsHelp.METHODS_HELP.intToStringArr(localIP);
        //远端IP地址
        String[] remoteIpStrArr = MethodsHelp.METHODS_HELP.intToStringArr(remoteIp);
        //默认网关
        String[] gatewayStrArr = MethodsHelp.METHODS_HELP.intToStringArr(gateWay);
        //子网掩码
        String[] subnetStrArr = MethodsHelp.METHODS_HELP.intToStringArr(subMask);

        String protocolTypeStr = protocolType == 0 ? "TCP" : "UDP";
        String netModeStr = netMode == 0 ? "服务器模式" : "客户端模式";
        String remoteIpStr = ((remoteIp >> 24) & 0xFF) + "." + ((remoteIp >> 16) & 0xFF) + "." + ((remoteIp >> 8) & 0xFF) + "." + (remoteIp & 0xFF);

        //协议类型
        protocol_type_SP.setSelection(protocolType);
        //模式
        mode_SP.setSelection(netMode);
        //本机IP地址
        local_ip_IPV.setGatewayText(localIpStrArr);
        //本机端口号
        local_port_ET.setText(port + "");
        //远端IP地址
        remote_ip_IPV.setGatewayText(remoteIpStrArr);
        //远端端口号
        remote_port_ET.setText(port + "");
        //默认网关
        gateway_IPV.setGatewayText(gatewayStrArr);
        //子网掩码
        subnet_IPV.setGatewayText(subnetStrArr);

        Log.d(TAG,"protocolType = " + protocolType);
        Log.d(TAG,"netMode = " + netMode
        );
    }


    public void setBaseActivity (BaseActivity baseActivity) {
        this.baseActivity = baseActivity;
    }


    //设置参数 作为服务端或者作为客户端
    private void setParas(){
        if(mode == serverMode){ //如果是作为服务端
            //如果是本地IP地址为空，或者是端口号为空，提示错误信息,校验端口号是否符合数字格式
            if(!local_ip_IPV.checkInputValue() || local_port_ET.getText().equals("")){
                showAlertDialog(getActivity(),"请输入本地IP地址、端口号不能为空！");
                return;
            }else{
                if(!TextUtils.isDigitsOnly(local_port_ET.getText().toString())){
                    showAlertDialog(getActivity(),"端口号只能为数字！");
                    return;
                }
            }

            //判断子网掩码
            if(!setGateAndSub()){
                return;
            }
            localIpArr = local_ip_IPV.getBytesWithIP ();
            localPort = Integer.parseInt(local_port_ET.getText().toString());

            int localIp = (((localIpArr[0] & 0xFF) << 24) + ((localIpArr[1] & 0xFF) << 16) + ((localIpArr[2] & 0xFF) << 8) + (localIpArr[3] & 0xFF) & 0xFFFFFFFF);
            Log.d(TAG,"localIp = " + localIp);
            //协议类型
            JniMethods.setProtocolType(protocol_type);
            //模式客户端或者服务器模式
            JniMethods.setNetMode(mode);
            //设置IP地址
            JniMethods.setLocalIp(localIp);
            //设置端口
            JniMethods.setNetPort(localPort);
            //设置子网掩码
            JniMethods.setSubMask(subnet);
            //设置默认网关
            JniMethods.setGateway(gateway);

            showAlertDialog(getActivity(),"设置成功！");
        }else if(mode == clientMode){ //如果是作为客户端
            if(!local_ip_IPV.checkInputValue() || !remote_ip_IPV.checkInputValue() || remote_port_ET.getText().equals("")){
                showAlertDialog(getActivity(),"本地IP地址、远端IP地址、远端端口不能为空！");
                return;
            }else{
                if(!TextUtils.isDigitsOnly(remote_port_ET.getText().toString())){
                    showAlertDialog(getActivity(),"端口号只能为数字！");
                    return;
                }
            }
            if(!setGateAndSub()){
                return;
            }
            //本机IP地址
            localIpArr = local_ip_IPV.getBytesWithIP();
            int localIp = (((localIpArr[0] & 0xFF) << 24) + ((localIpArr[1] & 0xFF) << 16) + ((localIpArr[2] & 0xFF) << 8) + (localIpArr[3] & 0xFF) & 0xFFFFFFFF);
            Log.d(TAG,"localIp = " + localIp);
            //远端IP地址和端口
            remoteIpArr = remote_ip_IPV.getBytesWithIP ();
            remotePort = Integer.parseInt(remote_port_ET.getText().toString());
            int remoteIp = (((remoteIpArr[0] & 0xFF) << 24) + ((remoteIpArr[1] & 0xFF) << 16) + ((remoteIpArr[2] & 0xFF) << 8) + (remoteIpArr[3] & 0xFF) & 0xFFFFFFFF);
            Log.d(TAG,"remoteIp = " + remoteIp);
            //协议类型
            JniMethods.setProtocolType(protocol_type);
            //模式客户端或者服务器模式
            JniMethods.setNetMode(mode);
            //设置IP地址
            JniMethods.setRemoteIp(remoteIp);
            //设置端口
            JniMethods.setNetPort(remotePort);
            //设置本机IP地址
            JniMethods.setLocalIp(localIp);
            //设置子网掩码
            JniMethods.setSubMask(subnet);
            //设置默认网关
            JniMethods.setGateway(gateway);

            showAlertDialog(getActivity(),"设置成功！");
        }


        android.support.v7.app.AlertDialog.Builder builder = new android.support.v7.app.AlertDialog.Builder(getActivity());
        builder.setMessage("设置成功");
        builder.setCancelable(false);
        builder.setPositiveButton("确定", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                if(onlyForParas){
                    getActivity().finish();
                }
            }
        });
        builder.show();


    }

    /*
     * @author lixiaojin
     * create on 2019/2/28 15:55
     * description: 校验子网掩码和默认网关
     */
    private boolean setGateAndSub(){
        if(!gateway_IPV.checkInputValue() || !subnet_IPV.checkInputValue()){
            showAlertDialog(getActivity(),"默认网关或者子网掩码不能为空！");
            return false;
        }else{
            //默认网关
            byte[] gatewayArr = gateway_IPV.getBytesWithIP ();
            gateway = (((gatewayArr[0] & 0xFF) << 24) + ((gatewayArr[1] & 0xFF) << 16) + ((gatewayArr[2] & 0xFF) << 8) + (gatewayArr[3] & 0xFF) & 0xFFFFFFFF);
            Log.d(TAG,"gatewayArr = " + gatewayArr);
            //子网掩码
            byte[] subnetArr = subnet_IPV.getBytesWithIP ();
            subnet = (((subnetArr[0] & 0xFF) << 24) + ((subnetArr[1] & 0xFF) << 16) + ((subnetArr[2] & 0xFF) << 8) + (subnetArr[3] & 0xFF) & 0xFFFFFFFF);
            Log.d(TAG,"subnet = " + subnet);
            return true;
        }
    }

    @Override
    public void showResult(Map map) {
        showAlertDialog(getActivity(),"收到响应数据map = " + map.toString());
    }


    @Override
    public void onDestroy() {
        super.onDestroy();
        //退出页面关闭网口
        int closeEth  =JniMethods.closeEth();
        Log.d(TAG,"closeEth = " + closeEth);
    }

    public void showAlertDialog(Context context, String text){
        if(getActivity ()!=null) {
            ((BaseActivity) getActivity ()).showAlertDialog (context, text);
        }
    }

    public void showProgress (boolean hideTitle, String message, boolean cancled) {
        if(getActivity ()!=null) {
            ((BaseActivity) getActivity ()).showProgress (hideTitle, message, cancled);
        }
    }


}
