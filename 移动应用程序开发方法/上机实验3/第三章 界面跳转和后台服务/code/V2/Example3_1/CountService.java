package edu.zjut.example.example3_3;

import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;

public class CountService extends Service {
    private boolean threadDisable;
    private int count;
    private ServiceBinder serviceBinder = new ServiceBinder();

    public class ServiceBinder extends Binder implements ICountService {
        public int getCount() {
            return count;
        }
    }

    public CountService() {
    }

    @Override
    public IBinder onBind(Intent intent) {
        return serviceBinder;
    }

    @Override
    public void onCreate() {
        super.onCreate();
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (!threadDisable) {
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                    count++;
                }
            }
        }).start();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        threadDisable = true;
    }
}
