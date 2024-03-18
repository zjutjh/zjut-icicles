package edu.zjut.lmq.example7_5;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

/**
 * Created by Administrator on 2016/4/21.
 */
public class MySurfaceView extends SurfaceView implements SurfaceHolder.Callback {
    private SurfaceHolder sfh;

    public MySurfaceView(Context context) {
        super(context);
        sfh = getHolder();
        sfh.addCallback(this);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        myDraw();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }

    private void myDraw() {
        Canvas canvas = sfh.lockCanvas();
        Paint paint = new Paint();
        paint.setColor(Color.WHITE);
        paint.setTextSize(60);
        canvas.drawText("GAME", 30, 100, paint);
        sfh.unlockCanvasAndPost(canvas);
    }
}
