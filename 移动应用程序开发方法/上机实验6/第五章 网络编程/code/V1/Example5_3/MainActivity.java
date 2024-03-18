package cn.edu.zjut.test4;

import android.os.Handler;
import android.os.Message;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;


public class MainActivity extends ActionBarActivity {
    private EditText edit1;
    private Button button1;
    private TextView tv3;
    private Handler handler;
    private static final String SURL = "http://192.168.1.100:8080/example5_3/index.jsp";
    private static final String MESSAGE_TITLE = "result";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        this.initViews();
        this.initHandler();
        this.initListener();
    }

    private void initViews() {
        edit1 = (EditText) findViewById(R.id.edit01);
        button1 = (Button) findViewById(R.id.button01);
        tv3 = (TextView) findViewById(R.id.text03);
    }

    private void initHandler() {
        handler = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                String result = msg.getData().getString(MESSAGE_TITLE);
                tv3.setText(result);
            }
        };
    }

    private void initListener() {
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Thread thread = new Thread() {
                    @Override
                    public void run() {
                        String query = edit1.getText().toString();
                        String sUrl = SURL + "?query=" + query;
                        try {
                            URL url = new URL(sUrl);
                            HttpURLConnection urlConn = (HttpURLConnection) url.openConnection();
                            InputStreamReader in = new InputStreamReader(urlConn.getInputStream());
                            BufferedReader reader = new BufferedReader(in);
                            String result = "";
                            String inputLine = null;
                            while ((inputLine = reader.readLine()) != null) {
                                result += inputLine + "\n";
                            }
                            Message msg = new Message();
                            Bundle bundle = new Bundle();
                            bundle.putString(MESSAGE_TITLE, result);
                            msg.setData(bundle);
                            handler.sendMessage(msg);
                            in.close();
                            urlConn.disconnect();
                        } catch (MalformedURLException ex) {
                            ex.printStackTrace();
                            String result = ex.toString();
                            Message msg = new Message();
                            Bundle bundle = new Bundle();
                            bundle.putString(MESSAGE_TITLE, result);
                            msg.setData(bundle);
                            handler.sendMessage(msg);
                        } catch (IOException ex) {
                            ex.printStackTrace();
                            String result = ex.toString();
                            Message msg = new Message();
                            Bundle bundle = new Bundle();
                            bundle.putString(MESSAGE_TITLE, result);
                            msg.setData(bundle);
                            handler.sendMessage(msg);
                        }
                    }
                };
                thread.start();
            }
        });
    }
}