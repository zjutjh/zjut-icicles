package cn.edu.zjut.example2_1;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.RadioButton;
import android.widget.Toast;


public class MainActivity extends ActionBarActivity {
    private Button button1;
    private RadioButton radio1, radio2, radio3, radio4;
    private CheckBox check1, check2, check3, check4;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        radio1 = (RadioButton) findViewById(R.id.rb1);
        radio2 = (RadioButton) findViewById(R.id.rb2);
        radio3 = (RadioButton) findViewById(R.id.rb3);
        radio4 = (RadioButton) findViewById(R.id.rb4);
        check1 = (CheckBox) findViewById(R.id.check1);
        check2 = (CheckBox) findViewById(R.id.check2);
        check3 = (CheckBox) findViewById(R.id.check3);
        check4 = (CheckBox) findViewById(R.id.check4);
        Button button1 = (Button) findViewById(R.id.button1);
        View.OnClickListener listener1 = new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                StringBuffer buffer = new StringBuffer();
                buffer.append("第1题：");
                if (radio2.isChecked()) {
                    buffer.append("正确！");
                }
                else {
                    buffer.append("错误！");
                }
                buffer.append("\n").append("第2题：");
                String result2 = "错误！";
                if (check2.isChecked()) {
                    if (check3.isChecked()) {
                        result2 = "正确！";
                    }
                }
                buffer.append(result2);
                Toast.makeText(MainActivity.this, buffer.toString(), Toast.LENGTH_LONG).show();
            }
        };
        button1.setOnClickListener(listener1);
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
