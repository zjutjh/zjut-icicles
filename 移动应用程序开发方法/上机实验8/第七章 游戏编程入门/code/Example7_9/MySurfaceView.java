package edu.zjut.lmq.example7_9;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.media.AudioManager;
import android.media.MediaPlayer;
import android.media.SoundPool;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import java.io.IOException;

/**
 * Created by Administrator on 2016/4/23.
 */
public class MySurfaceView extends SurfaceView implements SurfaceHolder.Callback {
    private Context context;
    private MediaPlayer player;
    private SoundPool pool;
    private SurfaceHolder sfh;
    private Canvas canvas;
    private Paint paint;
    private Bitmap pic4, pic5;
    private int pic4X = 40, pic4Y = 80;
    private int pic5X = 100, pic5Y = 250;
    private Rect region4, region5;
    private int soundId1, soundId2;

    public MySurfaceView(Context context) {
        super(context);
        this.context = context;
        sfh = getHolder();
        sfh.addCallback(this);
        paint = new Paint();
        this.initPictures();
        this.initSounds();
    }

    private void initPictures() {
        pic4 = BitmapFactory.decodeResource(getResources(), R.drawable.pic4);
        pic5 = BitmapFactory.decodeResource(getResources(), R.drawable.pic5);
        region4 = new Rect(pic4X, pic4Y, pic4X+pic4.getWidth(), pic4Y+pic4.getHeight());
        region5 = new Rect(pic5X, pic5Y, pic5X+pic5.getWidth(), pic5Y+pic5.getHeight());
    }

    private void initSounds() {
        pool = new SoundPool(5, AudioManager.STREAM_SYSTEM, 100);
        soundId1 = pool.load(context, R.raw.sound1, 1);
        soundId2 = pool.load(context, R.raw.sound2, 1);
        player = MediaPlayer.create(context, R.raw.bg1);
        player.setLooping(true);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        myDraw();
        player.start();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        int touchX = (int) event.getX();
        int touchY = (int) event.getY();
        if (region4.contains(touchX, touchY)) {
            pool.play(soundId2, 1, 1, 0, 0, 1);
        }
        else if (region5.contains(touchX, touchY)) {
            pool.play(soundId1, 1, 1, 0, 0, 1);
        }
        return true;
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        player.stop();
    }

    private void myDraw() {
        canvas = sfh.lockCanvas();
        canvas.drawColor(Color.WHITE);
        canvas.drawBitmap(pic4, 40, 80, paint);
        canvas.drawBitmap(pic5, 100, 250, paint);
        sfh.unlockCanvasAndPost(canvas);
    }
}
