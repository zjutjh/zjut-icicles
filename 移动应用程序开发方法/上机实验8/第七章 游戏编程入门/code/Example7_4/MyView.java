package edu.zjut.lmq.example7_4;

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
    private Bitmap pic3;
    private int curX, curY;
    private Rect area;
    private boolean isDrag = false;

    public MyView(Context context) {
        super(context);
        pic3 = BitmapFactory.decodeResource(getResources(), R.drawable.pic3);
        curX = 0;
        curY = 0;
        area = new Rect(0, 0, pic3.getWidth(), pic3.getHeight());
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        Paint paint = new Paint();
        canvas.drawBitmap(pic3, curX, curY, paint);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        int touchX = (int) event.getX();
        int touchY = (int) event.getY();
        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            if (area.contains(touchX, touchY)) {
                isDrag = true;
            }
        }
        else if (event.getAction() == MotionEvent.ACTION_UP) {
            if (isDrag) {
                curX = touchX - pic3.getWidth()/2;
                curY = touchY - pic3.getHeight()/2;
                postInvalidate();
                isDrag = false;
                area.set(curX, curY, curX + pic3.getWidth(), curY + pic3.getHeight());
            }
        }
        else if (event.getAction() == MotionEvent.ACTION_MOVE) {
            if (isDrag) {
                curX = touchX - pic3.getWidth()/2;
                curY = touchY - pic3.getHeight()/2;
                postInvalidate();
            }
        }
        return true;
    }
}
