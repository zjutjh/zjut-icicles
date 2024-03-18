package edu.zjut.lmq.example7_2;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.MotionEvent;
import android.view.View;

/**
 * Created by Administrator on 2016/4/17.
 */
public class MyView extends View {
    private Bitmap pic1;
    private int curX = 0;
    private int curY = 0;

    public MyView(Context context) {
        super(context);
        setBackgroundColor(Color.WHITE);
        pic1 = BitmapFactory.decodeResource(getResources(), R.drawable.pic1);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        Paint paint = new Paint();
        canvas.drawBitmap(pic1, curX, curY, paint);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        float x = event.getX();
        float y = event.getY();
        this.curX = (int) x;
        this.curY = (int) y;
        postInvalidate();
        return true;
    }
}
