package edu.zjut.lmq.example7_3;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import android.view.MotionEvent;
import android.view.View;

/**
 * Created by Administrator on 2016/4/17.
 */
public class MyView extends View {
    private Bitmap pic1, pic2;
    private int screenWidth = 0;
    private int screenHeight = 0;
    private int curX = 0;
    private int curY = 0;
    private Rect upArea = new Rect(80,487,125,550);
    private Rect leftArea = new Rect(15,550,80,600);
    private Rect rightArea = new Rect(125,550,190,600);
    private Rect downArea = new Rect(80,600,125,667);
    private int stepSize = 20;

    public MyView(Context context) {
        super(context);
        pic1 = BitmapFactory.decodeResource(getResources(), R.drawable.pic1);
        pic2 = BitmapFactory.decodeResource(getResources(), R.drawable.pic2);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        Paint paint = new Paint();
        canvas.drawBitmap(pic2, 0, screenHeight - pic2.getHeight(), paint);
        canvas.drawBitmap(pic1, curX, curY, paint);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        screenWidth = w;
        screenHeight = h;
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        int touchX = (int)event.getX();
        int touchY = (int)event.getY();
        System.out.println(touchX + ", " + touchY);
        if (upArea.contains(touchX, touchY)) {
            curY -= stepSize;
            if (curY < 0) {
                curY = 0;
            }
            postInvalidate();
        }
        else if (leftArea.contains(touchX, touchY)) {
            curX -= stepSize;
            if (curX < 0) {
                curX = 0;
            }
            postInvalidate();
        }
        else if (rightArea.contains(touchX, touchY)) {
            curX += stepSize;
            if (curX + pic1.getWidth() > screenWidth) {
                curX = screenWidth - pic1.getWidth();
            }
            postInvalidate();
        }
        else if (downArea.contains(touchX, touchY)) {
            curY += stepSize;
            if (curY + pic1.getHeight() > screenHeight) {
                curY = screenHeight - pic1.getHeight();
            }
            postInvalidate();
        }
        return super.onTouchEvent(event);
    }
}
