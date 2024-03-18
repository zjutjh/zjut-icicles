package edu.zjut.example.example2_4;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.TextView;


public class MainActivity extends ActionBarActivity {
    private Spinner spinner1, spinner2;
    private Button button1;
    private TextView text4;
    private static final String[] PROVINCES = {"浙江省", "广东省"};
    private static final String[] ZJ_CITIES = {"杭州市", "宁波市", "温州市"};
    private static final String[] GD_CITIES = {"广州市", "深圳市", "中山市"};
    private ArrayAdapter<String> adapter1, adapter2, adapter3;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        spinner1 = (Spinner) findViewById(R.id.spinner01);
        spinner2 = (Spinner) findViewById(R.id.spinner02);
        button1 = (Button) findViewById(R.id.button01);
        text4 = (TextView) findViewById(R.id.text04);
        adapter1 = new ArrayAdapter<String>(MainActivity.this, android.R.layout.simple_spinner_item, PROVINCES);
        adapter2 = new ArrayAdapter<String>(MainActivity.this, android.R.layout.simple_spinner_item, ZJ_CITIES);
        adapter3 = new ArrayAdapter<String>(MainActivity.this, android.R.layout.simple_spinner_item, GD_CITIES);
        spinner1.setAdapter(adapter1);
        spinner1.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (position == 0) {
                    spinner2.setAdapter(adapter2);
                }
                else if (position == 1) {
                    spinner2.setAdapter(adapter3);
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                text4.setText("任何选项都没被选！");
            }
        });
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                int position1 = spinner1.getSelectedItemPosition();
                int position2 = spinner2.getSelectedItemPosition();
                String province = adapter1.getItem(position1);
                String city = "";
                if (position1 == 0) {
                    city = adapter2.getItem(position2);
                }
                else if (position1 == 1) {
                    city = adapter3.getItem(position2);
                }
                String result = "所选省份：" + province + "\n" + "所选市区：" + city;
                text4.setText(result);
            }
        });
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
