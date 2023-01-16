package com.work2.parsejson;

import java.io.IOException;
import java.io.InputStream;
import java.io.PrintWriter;
import java.rmi.ServerException;

import com.alibaba.fastjson.JSON;
import com.alibaba.fastjson.JSONArray;
import com.alibaba.fastjson.JSONObject;

import jakarta.servlet.annotation.MultipartConfig;
import jakarta.servlet.annotation.WebServlet;
import jakarta.servlet.http.HttpServlet;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.servlet.http.Part;

@WebServlet("/parsejson.do")
@MultipartConfig(location = "/Users/j10c/playground/tmp", fileSizeThreshold = 1024)
public class ParseJsonServlet extends HttpServlet {
  private String message = null;
  private JSONArray data = null;
  private String code = "200404";

  @Override
  public void init() {
    this.message = null;
    this.code = "200404";
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse res) throws ServerException, IOException {
    this.init();
    try {
      Part p = req.getPart("json");
      if (p.getSize() > 1024 * 1024) {
        p.delete();
      }

      InputStream is = p.getInputStream();
      byte[] bytes = new byte[is.available()];
      is.read(bytes);
      String str = new String(bytes);
      this.code = "1";
      this.data = JSON.parseArray(str);

    } catch (Exception err) {
      err.printStackTrace();
      this.code = "200500";
      this.message = err.getMessage();
    }

    JSONObject resBody = new JSONObject();
    resBody.put("msg", this.message);
    resBody.put("code", this.code);
    resBody.put("data", this.data);

    res.setContentType("application/json;charset=utf-8");
    PrintWriter out = res.getWriter();
    out.println(resBody);

  }

}
