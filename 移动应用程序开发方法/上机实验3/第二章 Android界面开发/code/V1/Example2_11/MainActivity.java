package edu.zjut.example.example2_11;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.AdapterView;
import android.widget.GridView;
import android.widget.SimpleAdapter;
import android.widget.Toast;

import java.util.*;


public class MainActivity extends ActionBarActivity {
    private int images[] = null;
    private String texts[] = null;
    private GridView grid1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        grid1 = (GridView) findViewById(R.id.grid01);
        images = new int[] {R.drawable.p1, R.drawable.p2, R.drawable.p3, R.drawable.p4, R.drawable.p5, R.drawable.p6, R.drawable.p7, R.drawable.p8, R.drawable.p9};
        texts = new String[] {"选项1", "选项2", "选项3", "选项4", "选项5", "选项6", "选项7", "选项8", "选项9"};
        List<HashMap<String, Object>> itemList = new ArrayList<HashMap<String, Object>>();
        for (int i=0; i<9; i++) {
            HashMap<String, Object> map = new HashMap<String, Object>();
            map.put("itemImage", images[i]);
            map.put("itemText", texts[i]);
            itemList.add(map);
        }
        SimpleAdapter adapter = new SimpleAdapter(MainActivity.this, itemList, R.layout.grid_item, new String[] {"itemImage", "itemText"}, new int[] {R.id.itemImage, R.id.itemText});
        grid1.setAdapter(adapter);
        grid1.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                int option = position + 1;
                Toast.makeText(MainActivity.this, "你选择了选项" + option, Toast.LENGTH_SHORT).show();
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
