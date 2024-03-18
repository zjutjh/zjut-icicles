package edu.zjut.example.example5_2_1;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;


public class MainActivity extends ActionBarActivity {
    private TextView tv1, tv2;
    private Button button1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        this.initViews();
    }

    private void initViews() {
        tv1 = (TextView) findViewById(R.id.text01);
        tv2 = (TextView) findViewById(R.id.text02);
        button1 = (Button) findViewById(R.id.button01);
        tv1.setText(JSONWord.WORD1);
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                List<Place> places = parseJsonWord(JSONWord.WORD1);
                if (places != null) {
                    String result = "";
                    for (Place place : places) {
                        result += place.toString() + "\n\n";
                    }
                    tv2.setText(result);
                }
                else {
                    tv2.setText("解析失败");
                }
            }
        });
    }

    private List<Place> parseJsonWord(String jsonWord) {
        List<Place> places = new ArrayList<Place>();
        try {
            JSONObject jsonObject1 = new JSONObject(jsonWord);
            JSONArray jsonArray = jsonObject1.optJSONArray("results");
            for (int i=0; i<jsonArray.length(); i++) {
                JSONObject jsonObject2 = jsonArray.optJSONObject(i);
                String name = jsonObject2.optString("name");
                JSONObject jsonObject3 = jsonObject2.optJSONObject("location");
                double lat = jsonObject3.optDouble("lat");
                double lng = jsonObject3.optDouble("lng");
                String address = jsonObject2.optString("address");
                String telephone = jsonObject2.optString("telephone");
                Place place = new Place();
                place.setName(name);
                place.setLng(lng);
                place.setLat(lat);
                place.setAddress(address);
                place.setTelephone(telephone);
                places.add(place);
            }
            return places;
        } catch (JSONException ex) {
            ex.printStackTrace();
            return null;
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }
}
