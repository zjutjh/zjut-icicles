package cn.edu.zjut.test4;

import android.content.ContentValues;
import android.content.Intent;
import android.database.sqlite.SQLiteDatabase;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.Toast;

public class AddActivity extends ActionBarActivity {
    private DBOpenHelper dbOpenHelper;
    private EditText etWord, etExplain;
    private ImageButton btn_Save, btn_Cancel;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_add);
        //创建DBOpenHelper对象,指定名称、版本号并保存在databases目录下
        dbOpenHelper = new DBOpenHelper(AddActivity.this, "dict.db", null, 1);

        etWord = (EditText) findViewById(R.id.add_word);
        etExplain = (EditText) findViewById(R.id.add_interpret);
        btn_Save = (ImageButton) findViewById(R.id.save_btn);
        btn_Cancel = (ImageButton) findViewById(R.id.cancel_btn1);

        //实现将添加的单词解释保存在数据库中
        btn_Save.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                String word = etWord.getText().toString();
                String explain = etExplain.getText().toString();
                if (word.equals("") || explain.equals("")){
                    Toast.makeText(AddActivity.this, "填写的单词或解释为空", Toast.LENGTH_SHORT).show();
                }else {
                    insertData(dbOpenHelper.getReadableDatabase(), word, explain);
                    etWord.setText("");
                    etExplain.setText("");
                    Toast.makeText(AddActivity.this, "添加生词成功！", Toast.LENGTH_LONG).show();
                }

            }
        });
        //实现返回查询单词界面
        btn_Cancel.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Intent intent = new Intent(AddActivity.this, MainActivity.class);
                startActivity(intent);
            }
        });
    }

    private void insertData(SQLiteDatabase readableDatabase, String word, String explain) {
        ContentValues values = new ContentValues();
        values.put("word", word);
        values.put("detail", explain);
        readableDatabase.insert("dict", null, values);
    }
}
