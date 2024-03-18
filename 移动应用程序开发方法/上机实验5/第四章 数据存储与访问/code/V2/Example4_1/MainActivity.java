package cn.edu.zjut.test4;

import android.content.Intent;
import android.database.Cursor;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.SimpleAdapter;
import android.widget.Toast;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MainActivity extends ActionBarActivity {
    private DBOpenHelper dbOpenHelper;
    private ListView listView;
    private EditText etSearch;
    private ImageButton btnSearch;
    private Button btn_add;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        dbOpenHelper = new DBOpenHelper(MainActivity.this, "dict.db", null, 1);

        listView = (ListView) findViewById(R.id.result_listView);
        etSearch = (EditText) findViewById(R.id.search_et);
        btnSearch = (ImageButton) findViewById(R.id.search_btn);
        btn_add = (Button) findViewById(R.id.btn_add);

        //单击添加生词按钮，实现跳转到添加生词的界面
        btn_add.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Intent intent = new Intent(MainActivity.this, AddActivity.class);
                startActivity(intent);
            }
        });

        //单击查询按钮，实现查询词库中的单词
        btnSearch.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                String key = etSearch.getText().toString();
                //查询单词
                Cursor cursor = dbOpenHelper.getReadableDatabase().query("dict", null, "word = ?", new String[]{key}, null, null, null);
                //创建ArrayList对象，用于保存查询出的结果
                List<Map<String, String>> resultList = new ArrayList<Map<String, String>>();
                while (cursor.moveToNext()) {
                    Map<String, String> map = new HashMap<>();  // 将一个元组存入HashMap中
                    map.put("word", cursor.getString(1));
                    map.put("interpret", cursor.getString(2));
                    resultList.add(map);
                }

                if (resultList == null || resultList.size() == 0) {
                    Toast.makeText(MainActivity.this, "很遗憾，没有相关记录！", Toast.LENGTH_LONG).show();
                } else {
                    // 将查询的结果显示到ListView列表中
                    SimpleAdapter simpleAdapter = new SimpleAdapter(MainActivity.this, resultList, R.layout.result_main, new String[]{"word", "interpret"}, new int[]{R.id.result_word, R.id.result_interpret});
                    listView.setAdapter(simpleAdapter);
                }
            }
        });
    }

    protected void onDestroy() {
        super.onDestroy();
        if (dbOpenHelper != null) {
            dbOpenHelper.close();
        }
    }
}
