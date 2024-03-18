package edu.zjut.example.example4_3;

import android.os.Environment;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;


public class MainActivity extends ActionBarActivity {
    private EditText edit1, edit2;
    private Button button1, button2;
    private static final String FILE_NAME = "file2.txt";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        this.initViews();
        this.initListeners();
    }

    private void initViews() {
        edit1 = (EditText) findViewById(R.id.edit01);
        edit2 = (EditText) findViewById(R.id.edit02);
        button1 = (Button) findViewById(R.id.button01);
        button2 = (Button) findViewById(R.id.button02);
    }

    private void initListeners() {
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
                    FileOutputStream fos = null;
                    try {
                        File file = new File(Environment.getExternalStorageDirectory().getPath(), FILE_NAME);
                        if (!file.exists()) {
                            file.createNewFile();
                        }
                        fos = new FileOutputStream(file);
                        String text = edit1.getText().toString();
                        fos.write(text.getBytes());
                        fos.flush();
                        edit1.setText("");
                        Toast.makeText(MainActivity.this, "保存文件到SD卡", Toast.LENGTH_SHORT).show();
                    } catch (IOException ex) {
                        ex.printStackTrace();
                    } finally {
                        try {
                            fos.close();
                        } catch (IOException ex) {
                            ex.printStackTrace();
                        }
                    }
                }
            }
        });
        button2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
                    FileInputStream fis = null;
                    try {
                        File file = new File(Environment.getExternalStorageDirectory().getPath(), FILE_NAME);
                        if (!file.exists()) {
                            Toast.makeText(MainActivity.this, "未找到该文件", Toast.LENGTH_SHORT).show();
                            return;
                        }
                        fis = new FileInputStream(file);
                        byte[] buffer = new byte[fis.available()];
                        fis.read(buffer);
                        String text = new String(buffer);
                        edit2.setText(text);
                    } catch (IOException ex) {
                        ex.printStackTrace();
                    } finally {
                        try {
                            fis.close();
                        } catch (IOException ex) {
                            ex.printStackTrace();
                        }
                    }
                }
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
