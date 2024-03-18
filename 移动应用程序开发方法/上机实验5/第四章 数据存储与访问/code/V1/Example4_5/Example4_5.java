package edu.zjut.lmq;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

public class Example4_5 {
	public List<List<Point>> readPointsFromFile(String filePath)
			throws IOException {
		List<List<Point>> routes = new ArrayList<List<Point>>();
		BufferedReader reader = new BufferedReader(new InputStreamReader(
				new FileInputStream(filePath), "UTF-8"));
		String line = null;
		while ((line = reader.readLine()) != null) {
			List<Point> points = new ArrayList<Point>();
			String values1[] = line.split(" ");
			for (int i = 0; i < values1.length; i++) {
				String pointValue = values1[i];
				String values2[] = pointValue.split(",");
				String strLng = values2[0];
				String strLat = values2[1];
				String strTime = values2[2];
				double lng = Double.parseDouble(strLng);
				double lat = Double.parseDouble(strLat);
				long timestamp = Long.parseLong(strTime);
				Point point = new Point(lng, lat, timestamp);
				points.add(point);
			}
			routes.add(points);
		}
		reader.close();
		return routes;
	}

	public void printRoutes(List<List<Point>> routes) {
		for (int i = 0; i < routes.size(); i++) {
			System.out.println("路径" + (i + 1));
			List<Point> points = routes.get(i);
			for (Point point : points) {
				System.out.println("经度：" + point.getLng() + ", 纬度："
						+ point.getLat() + ", 采样时间：" + point.getTimestamp());
			}
		}
	}

	public void writeRoutesToKml(List<List<Point>> routes, String filePath)
			throws IOException {
		BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(
				new FileOutputStream(filePath), "UTF-8"));
		writer.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		writer.write("<kml xmlns=\"http://earth.google.com/kml/2.2\">\n");
		writer.write("<Document>\n");
		writer.write("<name>Trajectories</name>\n\n");
		int count = 1;
		for (List<Point> points : routes) {
			writer.write("<Placemark>\n");
			writer.write("<name>路径" + count + "</name>\n");
			writer.write("<Style id=\"yellowLine\">\n");
			writer.write("<LineStyle>\n");
			writer.write("<color>7f00ffff</color>\n");
			writer.write("<width>4</width>\n");
			writer.write("</LineStyle>\n");
			writer.write("</Style>\n");
			writer.write("<LineString>\n");
			writer.write("<extrude>0</extrude>\n");
			writer.write("<tessellate>1</tessellate>\n");
			writer.write("<altitudeMode>clampToGround</altitudeMode>\n");
			writer.write("<coordinates>\n");
			for (Point point : points) {
				writer.write(point.getLng() + ", " + point.getLat() + ", 0\n");
			}
			writer.write("</coordinates>\n");
			writer.write("</LineString>\n");
			writer.write("</Placemark>\n\n");
			count++;
		}
		writer.write("</Document>\n");
		writer.write("</kml>");
		writer.close();
	}

	public static void main(String args[]) {
		try {
			Example4_5 example = new Example4_5();
			List<List<Point>> routes = example.readPointsFromFile("route.txt");
			example.printRoutes(routes);
			example.writeRoutesToKml(routes, "route.kml");
		} catch (IOException ex) {
			ex.printStackTrace();
		}
	}
}
