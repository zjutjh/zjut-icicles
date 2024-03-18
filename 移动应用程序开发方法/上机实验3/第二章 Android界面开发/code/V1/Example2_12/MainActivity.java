package edu.zjut.example.example2_12;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.ListView;
import android.widget.SimpleAdapter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


public class MainActivity extends ActionBarActivity {
    private ListView list;
    private int[] IDS = new int[] {1, 2, 3, 4, 5};
    private String[] NAMES = new String[] {"张三", "李四", "王五", "吴六", "陈七"};
    private String[] MAJORS = new String[] {"计算机", "管理", "电子", "机械", "英语"};

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        list = (ListView) findViewById(R.id.listView);
        List<HashMap<String, String>> myList = new ArrayList<HashMap<String, String>>();
        for (int i=0; i<5; i++) {
            HashMap<String, String> map = new HashMap<String, String>();
            map.put("itemId", String.valueOf(IDS[i]));
            map.put("itemName", NAMES[i]);
            map.put("itemMajor", MAJORS[i]);
            myList.add(map);
        }
        SimpleAdapter adapter = new SimpleAdapter(MainActivity.this, myList, R.layout.item_list, new String[] {"itemId", "itemName", "itemMajor"}, new int[] {R.id.text01, R.id.text02, R.id.text03});
        list.setAdapter(adapter);
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
