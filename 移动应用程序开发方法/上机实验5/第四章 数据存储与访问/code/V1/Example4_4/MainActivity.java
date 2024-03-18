package edu.zjut.example.example4_4;

import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;


public class MainActivity extends ActionBarActivity {
    private MyDBHelper helper;
    private SQLiteDatabase mydb;
    private EditText edit1, edit2, edit3, edit4, edit5, edit6;
    private Button button1, button2, button3, button4;
    private TextView tv10;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        this.initDB();
        this.initViews();
        this.initListeners();
    }

    private void initDB() {
        helper = new MyDBHelper(MainActivity.this);
        mydb = helper.getWritableDatabase();
    }

    private void initViews() {
        edit1 = (EditText) findViewById(R.id.edit01);
        edit2 = (EditText) findViewById(R.id.edit02);
        edit3 = (EditText) findViewById(R.id.edit03);
        edit4 = (EditText) findViewById(R.id.edit04);
        edit5 = (EditText) findViewById(R.id.edit05);
        edit6 = (EditText) findViewById(R.id.edit06);
        button1 = (Button) findViewById(R.id.button01);
        button2 = (Button) findViewById(R.id.button02);
        button3 = (Button) findViewById(R.id.button03);
        button4 = (Button) findViewById(R.id.button04);
        tv10 = (TextView) findViewById(R.id.text10);
    }

    private void initListeners() {
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                int id = Integer.parseInt(edit1.getText().toString());
                String name = edit2.getText().toString();
                double credit = Double.parseDouble(edit3.getText().toString());
                insertData(id, name, credit);
                edit1.setText("");
                edit2.setText("");
                edit3.setText("");
                Toast.makeText(MainActivity.this, "插入数据成功", Toast.LENGTH_SHORT).show();
            }
        });
        button2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                int id = Integer.parseInt(edit4.getText().toString());
                double credit = Double.parseDouble(edit5.getText().toString());
                updateData(id, credit);
                edit4.setText("");
                edit5.setText("");
                Toast.makeText(MainActivity.this, "修改数据成功", Toast.LENGTH_SHORT).show();
            }
        });
        button3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                int id = Integer.parseInt(edit6.getText().toString());
                removeData(id);
                edit6.setText("");
                Toast.makeText(MainActivity.this, "删除数据成功", Toast.LENGTH_SHORT).show();
            }
        });
        button4.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                queryData();
            }
        });
    }

    private void insertData(int id, String name, double credit) {
        String sql = "INSERT INTO " + helper.TABLE_NAME + " (id,name,credit) VALUES (" + id + ",'" + name + "'," + credit + ")";
        mydb.execSQL(sql);
    }

    private void updateData(int id, double credit) {
        String sql = "UPDATE " + helper.TABLE_NAME + " SET credit=" + credit + " WHERE id=" + id;
        mydb.execSQL(sql);
    }

    private void removeData(int id) {
        String sql = "DELETE FROM " + helper.TABLE_NAME + " WHERE id=" + id;
        mydb.execSQL(sql);
    }

    private void queryData() {
        String sql = "SELECT * FROM " + helper.TABLE_NAME;
        Cursor cursor = mydb.rawQuery(sql, null);
        String result = null;
        if (cursor.getCount() == 0) {
            result = "无数据";
        }
        else {
            result = "";
            while (cursor.moveToNext()) {
                int id = cursor.getInt(0);
                String name = cursor.getString(1);
                double credit = cursor.getDouble(2);
                String temp = id + ", " + name + ", " + credit + "\n";
                result += temp;
            }
        }
        tv10.setText(result);
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
