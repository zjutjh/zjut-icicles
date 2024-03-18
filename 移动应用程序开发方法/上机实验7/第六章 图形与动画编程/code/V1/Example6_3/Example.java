package edu.zjut.lmq;

import java.util.*;

public class Example {
	private int count1 = 1;
	private int count2 = 1;
	private Timer timer1, timer2;
	
	public void startPeriodicalTask() {
		timer1 = new Timer();
		TimerTask task1 = new TimerTask() {
			public void run() {
				System.out.println("第一个计数为：" + count1);
				count1++;
				if (count1 == 10) {
					timer1.cancel();
				}
			}
		};
		timer1.schedule(task1, 2000, 1000);
	}
	
	public void startTimingTask() {
		timer2 = new Timer();
		TimerTask task2 = new TimerTask() {
			public void run() {
				System.out.println("第二个计数为：" + count2);
				count2++;
				if (count2 == 10) {
					timer2.cancel();
				}
			}
		};
		Date startTime = this.getAssignTime();
		timer2.scheduleAtFixedRate(task2, startTime, 1000);
	}
	
	public Date getAssignTime() {
		Calendar cal = Calendar.getInstance();
		cal.add(Calendar.SECOND, 14);
		return cal.getTime();
	}
	
	public static void main(String args[]) {
		Example example = new Example();
		example.startPeriodicalTask();
		example.startTimingTask();
	}
}
