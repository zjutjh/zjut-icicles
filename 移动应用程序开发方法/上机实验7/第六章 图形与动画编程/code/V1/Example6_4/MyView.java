package edu.zjut.example.example6_4;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.view.View;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Created by Administrator on 2016/3/26.
 */
public class MyView extends View {
    private int screenWidth, screenHeight;
    private Bitmap pic1;
    private int pic1Width, pic1Height;
    private int pic1X, pic1Y;
    private int direction;
    private Timer timer;
    private TimerTask task;

    public MyView(Context context) {
        super(context);
        pic1 = BitmapFactory.decodeResource(getResources(), R.drawable.pic1);
        pic1Width = pic1.getWidth();
        pic1Height = pic1.getHeight();
        pic1X = 0;
        direction = 1;
        timer = new Timer();
        task = new TimerTask() {
            @Override
            public void run() {
                if (direction == 1) {
                    pic1X += 20;
                }
                else if (direction == 2) {
                    pic1X -= 20;
                }
                if (pic1X >= (screenWidth - pic1Width)) {
                    direction = 2;
                }
                else if (pic1X <= 0) {
                    direction = 1;
                }
                postInvalidate();
            }
        };
        timer.schedule(task, 5000, 200);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        screenWidth = w;
        screenHeight = h;
        pic1Y = screenHeight/2 - pic1Height/2;
        super.onSizeChanged(w, h, oldw, oldh);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        Paint paint = new Paint();
        canvas.drawBitmap(pic1, pic1X, pic1Y, paint);
    }
}
