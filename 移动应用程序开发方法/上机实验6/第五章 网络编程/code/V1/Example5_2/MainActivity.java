package cn.edu.zjut.test4;

import android.os.Handler;
import android.os.Message;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class MainActivity extends ActionBarActivity {
    private TextView tv1;
    private Button button1, button2;
    private Handler handler;
    private boolean isRunning = false;
    private static final String MESSAGE_TITLE = "result";

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        tv1 = (TextView) findViewById(R.id.text1);
        button1 = (Button) findViewById(R.id.button1);
        button2 = (Button) findViewById(R.id.button2);
        handler = new Handler() {
            public void handleMessage(Message msg) {
                String result = msg.getData().getString(MESSAGE_TITLE);
                tv1.setText(result);
            }
        };
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                isRunning = true;
                Thread thread = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        while (isRunning) {
                            String result = "当前时间戳为：" + System.currentTimeMillis() + "！";
                            Message msg = new Message();
                            Bundle bundle = new Bundle();
                            bundle.putString(MESSAGE_TITLE, result);
                            msg.setData(bundle);
                            handler.sendMessage(msg);
                            try {
                                Thread.sleep(1000);
                            } catch (InterruptedException ex) {
                                ex.printStackTrace();
                            }
                        }
                    }
                });
                thread.start();
            }
        });
        button2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                isRunning = false;
            }
        });
    }
}