package edu.zjut.lmq.example7_8;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

/**
 * Created by Administrator on 2016/4/23.
 */
public class MySurfaceView extends SurfaceView implements SurfaceHolder.Callback, Runnable {
    private int x1 = 10, y1 = 110;
    private int w1 = 40, h1 = 40;
    private int x2 = 100, y2 = 110;
    private int w2 = 40, h2 = 40;
    private boolean flag;
    private boolean isCollsion;
    private Thread th;
    private SurfaceHolder sfh;
    private Canvas canvas;
    private Paint paint;
    private static final long REFRESH_INTERVAL = 50;

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
        flag = true;
        th = new Thread(this);
        th.start();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        flag = false;
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        x1 = (int) event.getX() - w1/2;
        y1 = (int) event.getY() - h1/2;
        if (isRectCollsion()) {
            isCollsion = true;
        }
        else {
            isCollsion = false;
        }
        myDraw();
        return true;
    }

    public MySurfaceView(Context context) {
        super(context);
        sfh = getHolder();
        sfh.addCallback(this);
        paint = new Paint();
        setFocusable(true);
    }

    private void myDraw() {
        try {
            canvas = sfh.lockCanvas();
            if (canvas != null) {
                if (isCollsion) {
                    paint.setColor(Color.RED);
                }
                else {
                    paint.setColor(Color.WHITE);
                }
                canvas.drawColor(Color.BLACK);
                canvas.drawRect(x1, y1, x1+w1, y1+h1, paint);
                canvas.drawRect(x2, y2, x2+w2, y2+h2, paint);
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        } finally {
            if (canvas != null) {
                sfh.unlockCanvasAndPost(canvas);
            }
        }
    }

    private boolean isRectCollsion() {
        if (x1 >= x2 + w2) {
            return false;
        }
        else if (x1 + w1 <= x2) {
            return false;
        }
        else if (y1 >= y2 + h2) {
            return false;
        }
        else if (y1 + h1 <= y2) {
            return false;
        }
        else {
            return true;
        }
    }
}
