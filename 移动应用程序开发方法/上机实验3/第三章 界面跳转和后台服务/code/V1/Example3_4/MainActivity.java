package cn.edu.zjut.test3;

import android.app.Activity;
import android.app.AlarmManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import java.util.Calendar;

public class MainActivity extends Activity {
    private Button button1;
    private Calendar c;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        c = Calendar.getInstance();
        button1 = (Button) findViewById(R.id.button1);
        button1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                AlarmManager alarm = (AlarmManager) getSystemService(Context.ALARM_SERVICE);
                int second = c.get(Calendar.SECOND);
                c.set(Calendar.SECOND, second + 10);
                Intent intent = new Intent(MainActivity.this, AlarmActivity.class);
                PendingIntent pi = PendingIntent.getActivity(MainActivity.this, 0, intent, 0);
                alarm.set(AlarmManager.RTC_WAKEUP, c.getTimeInMillis(), pi);
                Toast.makeText(MainActivity.this, "闹钟设置成功", Toast.LENGTH_SHORT).show();
            }
        });
    }
}

