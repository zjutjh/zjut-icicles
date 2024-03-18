package edu.zjut.lmq;

public class Point {
	private double lng;
	private double lat;
	private long timestamp;

	public Point() {
	}

	public Point(double lng, double lat, long timestamp) {
		this.lng = lng;
		this.lat = lat;
		this.timestamp = timestamp;
	}

	public double getLng() {
		return lng;
	}

	public void setLng(double lng) {
		this.lng = lng;
	}

	public double getLat() {
		return lat;
	}

	public void setLat(double lat) {
		this.lat = lat;
	}

	public long getTimestamp() {
		return timestamp;
	}

	public void setTimestamp(long timestamp) {
		this.timestamp = timestamp;
	}
}
