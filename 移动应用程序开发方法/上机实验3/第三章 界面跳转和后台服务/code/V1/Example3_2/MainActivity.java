package cn.edu.zjut.test1;

import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;


public class MainActivity extends ActionBarActivity {
    private Button button1, button2;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        button1 = (Button) findViewById(R.id.button01);
        button2 = (Button) findViewById(R.id.button02);
        View.OnClickListener listener = new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (v.getId() == button1.getId()) {
                    if (MusicService.isplay == false) {
                        Toast.makeText(MainActivity.this, "开始播放背景音乐！", Toast.LENGTH_SHORT).show();
                        Intent intent = new Intent(MainActivity.this, MusicService.class);
                        startService(intent);
                    }
                }
                else if (v.getId() == button2.getId()) {
                    Toast.makeText(MainActivity.this, "停止播放背景音乐！", Toast.LENGTH_SHORT).show();
                    Intent intent = new Intent(MainActivity.this, MusicService.class);
                    stopService(intent);
                }
            }
        };
        button1.setOnClickListener(listener);
        button2.setOnClickListener(listener);
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