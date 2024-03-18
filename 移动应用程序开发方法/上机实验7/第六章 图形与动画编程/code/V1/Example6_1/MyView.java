package edu.zjut.example.example6_1;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.view.View;

/**
 * Created by Administrator on 2016/3/26.
 */
public class MyView extends View {
    private int width;
    private int height;

    public MyView(Context context) {
        super(context);
        this.setBackgroundResource(R.drawable.bg);
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        width = w;
        height = h;
        super.onSizeChanged(w, h, oldw, oldh);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStrokeWidth(5);
        paint.setAlpha(50);
        paint.setStyle(Paint.Style.STROKE);
        canvas.drawLine(0, 0, 100, 100, paint);
        canvas.drawRect(50, 100, 150, 200, paint);
        canvas.drawCircle(100, 250, 50, paint);
        paint.setTextSize(30);
        paint.setStrokeWidth(0);
        paint.setAlpha(255);
        canvas.drawText("屏幕分辨率：" + width + "*" + height, 100, 50, paint);
        paint.setStyle(Paint.Style.FILL);
        Path path = new Path();
        path.moveTo(width/2, height/2);
        path.lineTo(width, height*3/4);
        path.lineTo(width/2, height);
        path.lineTo(0, height*3/4);
        path.close();
        canvas.drawPath(path, paint);
    }
}
