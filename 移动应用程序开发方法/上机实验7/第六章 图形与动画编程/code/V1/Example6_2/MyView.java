package edu.zjut.example.example6_2;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Rect;
import android.view.View;

/**
 * Created by Administrator on 2016/3/26.
 */
public class MyView extends View {
    private Bitmap pic1, pic2;
    private int screenWidth, screenHeight;
    private int pic1Width, pic1Height;
    private int pic2Width, pic2Height;

    public MyView(Context context) {
        super(context);
        pic1 = BitmapFactory.decodeResource(getResources(), R.drawable.pic1);
        pic1Width = pic1.getWidth();
        pic1Height = pic1.getHeight();
        pic2 = BitmapFactory.decodeResource(getResources(), R.drawable.map);
        pic2Width = pic2.getWidth();
        pic2Height = pic2.getHeight();
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        screenWidth = w;
        screenHeight = h;
        super.onSizeChanged(w, h, oldw, oldh);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        Paint paint = new Paint();
        Rect rect2 = new Rect(pic2Width/2 - screenWidth/2, pic2Height/2 - screenHeight, pic2Width/2 + screenWidth/2, pic2Height/2);
        Rect rect3 = new Rect(0, 0, screenWidth, screenHeight);
        canvas.drawBitmap(pic2, rect2, rect3, paint);
        canvas.drawBitmap(pic1, 0, 0, paint);
        Rect rect1 = new Rect(pic1Width, 0, pic1Width + pic1Width/2, pic1Height/2);
        canvas.drawBitmap(pic1, null, rect1, paint);
    }
}
