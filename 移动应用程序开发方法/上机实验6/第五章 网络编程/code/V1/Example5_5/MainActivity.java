package cn.edu.zjut.test4;

import android.os.Handler;
import android.os.Message;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import org.apache.http.HttpResponse;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.util.EntityUtils;

import java.io.IOException;


public class MainActivity extends ActionBarActivity {
    private TextView tv2, tv4, tv9;
    private Button button1, button2, button3;
    private EditText edit1, edit2, edit3, edit4, edit5;
    private Handler handler1, handler2, handler3;
    private static final String IP = "192.168.1.100";
    private static final String LIST_URL = "http://" + IP + ":8080/example5_5/list.do";
    private static final String LIST_MESSAGE = "result1";
    private static final String VIEW_URL = "http://" + IP + ":8080/example5_5/view.do";
    private static final String VIEW_MESSAGE = "result2";
    private static final String ADD_URL = "http://" + IP + ":8080/example5_5/add.do";
    private static final String ADD_MESSAGE = "result3";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        this.initViews();
        this.initHandlers();
        this.initListeners();
    }

    private void initViews() {
        tv2 = (TextView) findViewById(R.id.text2);
        tv4 = (TextView) findViewById(R.id.text4);
        tv9 = (TextView) findViewById(R.id.text9);
        button1 = (Button) findViewById(R.id.button1);
        button2 = (Button) findViewById(R.id.button2);
        button3 = (Button) findViewById(R.id.button3);
        edit1 = (EditText) findViewById(R.id.edit1);
        edit2 = (EditText) findViewById(R.id.edit2);
        edit3 = (EditText) findViewById(R.id.edit3);
        edit4 = (EditText) findViewById(R.id.edit4);
        edit5 = (EditText) findViewById(R.id.edit5);
    }

    private void initHandlers() {
        handler1 = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                String result = msg.getData().getString(LIST_MESSAGE);
                tv2.setText(result);
            }
        };
        handler2 = new Handler() {
            public void handleMessage(Message msg) {
                String result = msg.getData().getString(VIEW_MESSAGE);
                tv4.setText(result);
            }
        };
        handler3 = new Handler() {
            public void handleMessage(Message msg) {
                String result = msg.getData().getString(ADD_MESSAGE);
                tv9.setText(result);
            }
        };
    }

    private void initListeners() {
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Thread thread = new Thread() {
                    public void run() {
                        HttpGet get = new HttpGet(LIST_URL);
                        try {
                            HttpResponse response = new DefaultHttpClient().execute(get);
                            if (response.getStatusLine().getStatusCode() == 200) {
                                String result = EntityUtils.toString(response.getEntity());
                                Message msg = new Message();
                                Bundle bundle = new Bundle();
                                bundle.putString(LIST_MESSAGE, result);
                                msg.setData(bundle);
                                handler1.sendMessage(msg);
                            }
                        } catch (IOException ex) {
                            ex.printStackTrace();
                            String result = ex.toString();
                            Message msg = new Message();
                            Bundle bundle = new Bundle();
                            bundle.putString(LIST_MESSAGE, result);
                            msg.setData(bundle);
                            handler1.sendMessage(msg);
                        }
                    }
                };
                thread.start();
            }
        });
        button2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Thread thread = new Thread() {
                    public void run() {
                        String sId = edit1.getText().toString();
                        String url = VIEW_URL + "?id=" + sId;
                        HttpGet get = new HttpGet(url);
                        try {
                            HttpResponse response = new DefaultHttpClient().execute(get);
                            if (response.getStatusLine().getStatusCode() == 200) {
                                String result = EntityUtils.toString(response.getEntity());
                                Message msg = new Message();
                                Bundle bundle = new Bundle();
                                bundle.putString(VIEW_MESSAGE, result);
                                msg.setData(bundle);
                                handler2.sendMessage(msg);
                            }
                        } catch (IOException ex) {
                            ex.printStackTrace();
                            String result = ex.toString();
                            Message msg = new Message();
                            Bundle bundle = new Bundle();
                            bundle.putString(VIEW_MESSAGE, result);
                            msg.setData(bundle);
                            handler2.sendMessage(msg);
                        }
                    }
                };
                thread.start();
            }
        });
        button3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Thread thread = new Thread() {
                    public void run() {
                        String sId = edit2.getText().toString();
                        String sName = edit3.getText().toString();
                        String major = edit4.getText().toString();
                        String sCredit = edit5.getText().toString();
                        String url = ADD_URL + "?id=" + sId + "&sName=" + sName + "&major=" + major + "&credit=" + sCredit;
                        HttpGet get = new HttpGet(url);
                        try {
                            HttpResponse response = new DefaultHttpClient().execute(get);
                            if (response.getStatusLine().getStatusCode() == 200) {
                                String result = "新增成功！";
                                Message msg = new Message();
                                Bundle bundle = new Bundle();
                                bundle.putString(ADD_MESSAGE, result);
                                msg.setData(bundle);
                                handler3.sendMessage(msg);
                            }
                        } catch (IOException ex) {
                            ex.printStackTrace();
                            String result = ex.toString();
                            Message msg = new Message();
                            Bundle bundle = new Bundle();
                            bundle.putString(ADD_MESSAGE, result);
                            msg.setData(bundle);
                            handler3.sendMessage(msg);
                        }
                    }
                };
                thread.start();
            }
        });
    }
}
