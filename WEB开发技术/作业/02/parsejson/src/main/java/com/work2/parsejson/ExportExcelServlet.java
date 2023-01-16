package com.work2.parsejson;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Scanner;

import com.alibaba.fastjson.JSON;
import com.alibaba.fastjson.JSONArray;
import com.alibaba.fastjson.JSONObject;

import jakarta.servlet.ServletException;
import jakarta.servlet.annotation.WebServlet;
import jakarta.servlet.http.HttpServlet;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;

@WebServlet("/getExcel")
public class ExportExcelServlet extends HttpServlet {
  private String message = null;
  private String code = "200404";

  public void init() {
    this.message = null;
    this.code = "200404";
  }

  static String extractPostRequestBody(HttpServletRequest request) throws IOException {
    if ("POST".equalsIgnoreCase(request.getMethod())) {
      Scanner s = new Scanner(request.getInputStream(), "UTF-8").useDelimiter("\\A");
      return s.hasNext() ? s.next() : "";
    }
    return "";
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse res) throws ServletException, IOException {

    try {

      String s = extractPostRequestBody(req);
      JSONArray reqArray = JSON.parseArray(s);
      System.out.println(reqArray);

      res.setHeader("Content-Encoding", "gb2312");
      res.setHeader("Content-Disposition", "attachment; filename="
          + java.net.URLEncoder.encode("score.xls", "UTF-8"));
      res.setContentType("application/vnd.ms-excel;charset=gb2312");

      PrintWriter out = res.getWriter();
      out.println("序号\t学号\t姓名\t课程名\t成绩");
      for (int i = 0; i < reqArray.size(); i++) {
        JSONObject item = reqArray.getJSONObject(i);
        out.print(String.valueOf(i + 1) + "\t");
        out.print(item.getString("stuid") + "\t");
        out.print(item.getString("name") + "\t");
        out.print(item.getString("courseName") + "\t");
        out.print(item.getString("score") + "\n");
      }

    } catch (Exception err) {
      err.printStackTrace();
      this.message = err.getMessage();
    }

  }
}
