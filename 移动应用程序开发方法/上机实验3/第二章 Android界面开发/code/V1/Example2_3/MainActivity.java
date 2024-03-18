package edu.zjut.example.example2_3;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;


public class MainActivity extends ActionBarActivity {
    private Button button1;
    private RadioButton radio1, radio2;
    private CheckBox check1, check2, check3;
    private TextView text4;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        button1 = (Button) findViewById(R.id.button01);
        radio1 = (RadioButton) findViewById(R.id.radio01);
        radio2 = (RadioButton) findViewById(R.id.radio02);
        check1 = (CheckBox) findViewById(R.id.check01);
        check2 = (CheckBox) findViewById(R.id.check02);
        check3 = (CheckBox) findViewById(R.id.check03);
        text4 = (TextView) findViewById(R.id.text04);
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String result = "性别：";
                if (radio1.isChecked()) {
                    result += "男\n";
                }
                else if (radio2.isChecked()) {
                    result += "女\n";
                }
                result += "爱好：";
                if (check1.isChecked()) {
                    result += "音乐 ";
                }
                if (check2.isChecked()) {
                    result += "旅游 ";
                }
                if (check3.isChecked()) {
                    result += "电影";
                }
                text4.setText(result.trim());
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
