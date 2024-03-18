package edu.zjut.lmq.example7_7;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

/**
 * Created by Administrator on 2016/4/21.
 */
public class MySurfaceView extends SurfaceView implements SurfaceHolder.Callback, Runnable {
    private SurfaceHolder sfh;
    private Paint paint;
    private Canvas canvas;
    private Thread th;
    private int curX1 = 0;
    private int curY1 = 0;
    private int direction = 1;
    private int curX3 = 10;
    private int curY3 = 10;
    private int screenWidth, screenHeight;
    private boolean flag;
    private Bitmap pic1, pic3, bg;
    private static final int REFRESH_INTERVAL = 50;

    public MySurfaceView(Context context) {
        super(context);
        sfh = getHolder();
        sfh.addCallback(this);
        paint = new Paint();
        pic1 = BitmapFactory.decodeResource(getResources(), R.drawable.pic1);
        pic3 = BitmapFactory.decodeResource(getResources(), R.drawable.pic3);
        bg = BitmapFactory.decodeResource(getResources(), R.drawable.bg);
        setFocusable(true);
    }

    @Override
    public void run() {
        while (flag) {
            long start = System.currentTimeMillis();
            myDraw();
            long end = System.currentTimeMillis();
            long interval = end - start;
            try {
                if (interval < REFRESH_INTERVAL) {
                    Thread.sleep(REFRESH_INTERVAL - interval);
                }
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        screenWidth = getWidth();
        screenHeight = getHeight();
        flag = true;
        th = new Thread(this);
        th.start();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        curX3 = (int) event.getX();
        curY3 = (int) event.getY();
        return true;
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        flag = false;
    }

    private void myDraw() {
        try {
            canvas = sfh.lockCanvas();
            if (canvas != null) {
                canvas.drawBitmap(bg, new Rect(0, 0, bg.getWidth(), bg.getHeight()), new Rect(0, 0, screenWidth, screenHeight), paint);
                if (direction == 1) {
                    curX1 += 5;
                }
                else if (direction == 2) {
                    curX1 -= 5;
                }
                if (curX1 > (screenWidth - pic1.getWidth())) {
                    direction = 2;
                }
                else if (curX1 < 0) {
                    direction = 1;
                }
                canvas.drawBitmap(pic1, curX1, curY1, paint);
                canvas.drawBitmap(pic3, curX3, curY3, paint);
            }
        } finally {
            if (canvas != null) {
                sfh.unlockCanvasAndPost(canvas);
            }
        }
    }
}
