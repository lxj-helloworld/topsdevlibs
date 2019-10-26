# topsdevlibs

1、使用前配置：

Step 1. Add the JitPack repository to your build file
Add it in your root build.gradle at the end of repositories:


	allprojects {
		repositories {
			...
			maven { url 'https://jitpack.io' }
		}
	}
	

Step 2. Add the dependency

    dependencies {
         compile 'com.github.lxj-helloworld:topsdevlibs:1.14'  
    }


2、以485通信为例，说明使用步骤以及要求：

Step 1. 创建Fragment，并继承Base485IRFragment，该类是一个抽象类，可按要求实现抽象方法，该类封装了上电、下电、报文发送和接收部分，以下是报文发送和接收示例:


	 /*
    * @author lixiaojin
    * create on 2019-10-26 11:08
    * description: 报文发送
    */
    private void sendData(){
        byte[] frame = new byte[]{0x68,(byte)0xAA,(byte) 0xAA,(byte)0xAA,(byte)0xAA,(byte)0xAA,(byte)0xAA,0x68,0x11,0x04,0x35,0x34,0x33,0x37,(byte)0xB4,0x16};
        DataSendBuffer.DATA_SEND_BUFFER.setDatasSendArr (frame);
        HelpUtils.currentChannel = channel485;
        send();
    }

    /**
     * 数据展示
     * @param map
     */
    @Override
    public void showResult(Map map) {
        this.map = map;
    }

    /**
     * 报文接收完成后调用
     */
    @Override
    public void readDone() {
        super.readDone();
        if(hasData){ //如果有数据
            result_TV.setText(map.get("datas").toString());
        }else{
            Toast.makeText(getActivity(),"无数据",Toast.LENGTH_LONG).show();
        }
    }

    /**
     * 485报文解析
     * @param receiveBytes
     * @return
     */
    @Override
    public Map parse485(byte[] receiveBytes) {
        Map map = new HashMap();
        map.put("datas",MethodsHelp.METHODS_HELP.byteToHexString(receiveBytes,receiveBytes.length));
        return map;
    }
	


