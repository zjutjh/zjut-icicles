package edu.zjut.example.example2_10;

import android.app.AlertDialog;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.Toast;


public class MainActivity extends ActionBarActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
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
        if (id == R.id.item01) {
            new AlertDialog.Builder(MainActivity.this).setTitle("提示").setMessage("开始游戏").show();
        }
        else if (id == R.id.item02) {
            new AlertDialog.Builder(MainActivity.this).setTitle("提示").setMessage("继续游戏").show();
        }
        else if (id == R.id.item31) {
            Toast.makeText(MainActivity.this, "设置视频", Toast.LENGTH_SHORT).show();
        }
        else if (id == R.id.item32) {
            Toast.makeText(MainActivity.this, "设置音频", Toast.LENGTH_SHORT).show();
        }

        return super.onOptionsItemSelected(item);
    }
}
