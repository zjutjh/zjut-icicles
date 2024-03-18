package edu.zjut.lmq.example7_6;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

/**
 * Created by Administrator on 2016/4/21.
 */
public class MySurfaceView extends SurfaceView implements SurfaceHolder.Callback {
    private int curX = 10;
    private int curY = 10;
    private Bitmap pic1;
    private SurfaceHolder sfh;
    private Paint paint;

    public MySurfaceView(Context context) {
        super(context);
        sfh = getHolder();
        sfh.addCallback(this);
        pic1 = BitmapFactory.decodeResource(getResources(), R.drawable.pic1);
        paint = new Paint();
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        curX = (int) event.getX();
        curY = (int) event.getY();
        myDraw();
        return true;
    }

    private void myDraw() {
        Canvas canvas = sfh.lockCanvas();
        canvas.drawColor(Color.WHITE);
        canvas.drawBitmap(pic1, curX, curY, paint);
        sfh.unlockCanvasAndPost(canvas);
    }
}
