package cn.edu.zjut.test4;

import android.os.Handler;
import android.os.Message;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;


public class MainActivity extends ActionBarActivity {
    private Button button1;
    private TextView tv2, tv3;
    private Handler handler;
    private static final String SURL = "http://www.weather.com.cn/data/cityinfo/101210101.html";
    private static final String JSON_MESSAGE = "jsonResult";
    private static final String FINAL_MESSAGE = "finalResult";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        this.initViews();
        this.initHandler();
        this.initListener();
    }

    private void initViews() {
        button1 = (Button) findViewById(R.id.button01);
        tv2 = (TextView) findViewById(R.id.text02);
        tv3 = (TextView) findViewById(R.id.text03);
    }

    private void initHandler() {
        handler = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                Bundle bundle = msg.getData();
                String jsonResult = bundle.getString(JSON_MESSAGE);
                String result = bundle.getString(FINAL_MESSAGE);
                tv2.setText(jsonResult);
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
                        try {
                            URL url = new URL(SURL);
                            HttpURLConnection urlConn = (HttpURLConnection) url.openConnection();
                            InputStreamReader in = new InputStreamReader(urlConn.getInputStream());
                            BufferedReader reader = new BufferedReader(in);
                            String jsonResult = "";
                            String inputLine = null;
                            while ((inputLine = reader.readLine()) != null) {
                                jsonResult += inputLine + "\n";
                            }
                            String result = parseJsonObject(jsonResult);
                            Message msg = new Message();
                            Bundle bundle = new Bundle();
                            bundle.putString(JSON_MESSAGE, jsonResult);
                            bundle.putString(FINAL_MESSAGE, result);
                            msg.setData(bundle);
                            handler.sendMessage(msg);
                        } catch (MalformedURLException ex) {
                            ex.printStackTrace();
                        } catch (IOException ex) {
                            ex.printStackTrace();
                        } catch (JSONException ex) {
                            ex.printStackTrace();
                        }
                    }
                };
                thread.start();
            }
        });
    }

    private String parseJsonObject(String jsonStr) throws JSONException {
        JSONObject jsonObject1 = new JSONObject(jsonStr);
        JSONObject jsonObject2 = jsonObject1.getJSONObject("weatherinfo");
        String city = jsonObject2.optString("city");
        String highTemp = jsonObject2.optString("temp1");
        String lowTemp = jsonObject2.optString("temp2");
        String weather = jsonObject2.optString("weather");
        String result = "城市：" + city + "\n" + "温度：" + lowTemp + "-" + highTemp + "\n" + "天气：" + weather;
        return result;
    }
}