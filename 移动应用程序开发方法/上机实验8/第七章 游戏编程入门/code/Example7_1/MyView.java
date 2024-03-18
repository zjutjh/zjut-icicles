package edu.zjut.lmq.example7_1;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.view.View;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by Administrator on 2016/4/14.
 */
public class MyView extends View {
    private int screenWidth, screenHeight;
    private List<Bitmap> pics = new ArrayList<Bitmap>();
    private int curIndex;
    private Timer timer;
    private TimerTask task;

    public MyView(Context context) {
        super(context);
        Bitmap pic1 = BitmapFactory.decodeResource(getResources(), R.drawable.run1);
        Bitmap pic2 = BitmapFactory.decodeResource(getResources(), R.drawable.run2);
        Bitmap pic3 = BitmapFactory.decodeResource(getResources(), R.drawable.run3);
        Bitmap pic4 = BitmapFactory.decodeResource(getResources(), R.drawable.run4);
        pics.add(pic1);
        pics.add(pic2);
        pics.add(pic3);
        pics.add(pic4);
        this.initTimerTask();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        Paint paint = new Paint();
        Bitmap curPic = pics.get(curIndex);
        canvas.drawBitmap(curPic, 0, 0, paint);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        screenWidth = w;
        screenHeight = h;
        super.onSizeChanged(w, h, oldw, oldh);
    }

    private void initTimerTask() {
        timer = new Timer();
        task = new TimerTask() {
            @Override
            public void run() {
                curIndex++;
                if (curIndex == 4) {
                    curIndex = 0;
                }
                postInvalidate();
            }
        };
        timer.schedule(task, 3000, 200);
    }
}
