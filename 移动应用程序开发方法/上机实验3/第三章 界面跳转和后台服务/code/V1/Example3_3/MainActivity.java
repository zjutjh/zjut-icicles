package cn.edu.zjut.test2;

import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.Intent;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;

public class MainActivity extends ActionBarActivity {
    final int NOTIFYID = 0x123;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Notification.Builder notification = new Notification.Builder(this);
        notification.setAutoCancel(true); // 设置打开该通知，该通知自动消失
        notification.setSmallIcon(R.drawable.packet); // 设置通知的图标
        notification.setContentTitle("通知的标题！"); // 设置通知的标题
        notification.setContentText("通知的内容，点击查看详情！"); // 设置通知的内容
        notification.setDefaults(Notification.DEFAULT_SOUND | Notification.DEFAULT_VIBRATE); //设置使用系统默认的声音、默认震动
        notification.setWhen(System.currentTimeMillis()); //设置发送时间
        Intent intent = new Intent(MainActivity.this, NotifyActivity.class);
        PendingIntent pi = PendingIntent.getActivity(MainActivity.this, 0, intent, 0);
        notification.setContentIntent(pi); //设置通知栏点击跳转

        NotificationManager manager = (NotificationManager) getSystemService(NOTIFICATION_SERVICE);
        manager.notify(NOTIFYID, notification.build()); //发送通知
    }
}
