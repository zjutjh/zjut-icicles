package edu.zjut.example.example2_5;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.DatePicker;
import android.widget.ImageView;
import android.widget.RatingBar;
import android.widget.TextView;
import android.widget.ToggleButton;

import java.util.Calendar;


public class MainActivity extends ActionBarActivity {
    private ImageView image1;
    private ToggleButton button1;
    private DatePicker picker;
    private RatingBar rating;
    private Button button2, button3;
    private TextView text1, text2;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        image1 = (ImageView) findViewById(R.id.image01);
        picker = (DatePicker) findViewById(R.id.datePicker);
        rating = (RatingBar) findViewById(R.id.ratingBar);
        text1 = (TextView) findViewById(R.id.text01);
        text2 = (TextView) findViewById(R.id.text02);
        button1 = (ToggleButton) findViewById(R.id.button01);
        button2 = (Button) findViewById(R.id.button02);
        button3 = (Button) findViewById(R.id.button03);
        button1.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                //button1.setChecked(isChecked);
                if (isChecked) {
                    image1.setImageResource(R.drawable.light_on);
                }
                else {
                    image1.setImageResource(R.drawable.light_off);
                }
            }
        });
        button2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                int year = picker.getYear();
                int month = picker.getMonth();
                int day = picker.getDayOfMonth();
                String result = "选择的日期为：" + year + "-" + month + "-" + day;
                text1.setText(result);
            }
        });
        button3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                float start = rating.getRating();
                String result = "选择的星级为：" + start;
                text2.setText(result);
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
