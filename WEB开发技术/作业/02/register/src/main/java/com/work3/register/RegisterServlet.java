package com.work3.register;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

import com.alibaba.fastjson.JSONObject;

import jakarta.servlet.*;
import jakarta.servlet.http.*;

public class RegisterServlet extends HttpServlet {
  static final String fileBasePath = "/Users/j10c/playground";
  private String message = null;
  private String code = "200404";

  @Override
  public void init() {
  }

  public void checkForm(String items[]) throws Exception {
    for (String data : items) {
      if (data == null) {
        this.code = "200501";
        throw new Exception("表单格式不正确!");
      }
    }

    FileReader fr = new FileReader(fileBasePath + "/userinfo.txt");
    BufferedReader br = new BufferedReader(fr);
    try {
      String item;
      while (br.ready()) {
        item = br.readLine();
        if (item.split("\\|")[0].equals(items[0])) {
          br.close();
          fr.close();
          this.code = "200502";
          throw new Exception("你的用户名已被注册，请返回重新注册");
        }
      }
    } finally {
      br.close();
      fr.close();
    }
  }

  private static String byte2Hex(byte[] bytes) {
    StringBuffer stringBuffer = new StringBuffer();
    String temp = null;
    for (int i = 0; i < bytes.length; i++) {
      temp = Integer.toHexString(bytes[i] & 0xFF);
      if (temp.length() == 1) {
        stringBuffer.append("0");
      }
      stringBuffer.append(temp);
    }
    return stringBuffer.toString();
  }

  public static String sha256(String s) {
    String encode = "";
    try {
      MessageDigest m = MessageDigest.getInstance("sha256");
      byte[] hash = m.digest(s.getBytes("UTF-8"));
      encode = byte2Hex(hash);
    } catch (NoSuchAlgorithmException err) {
      err.printStackTrace();
    } catch (UnsupportedEncodingException err) {
      err.printStackTrace();
    }
    return encode;
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse res) throws ServletException, IOException {
    res.setContentType("text/html;charset=utf-8");
    try {
      String name = req.getParameter("name");
      String password = req.getParameter("password");
      String username = req.getParameter("username");
      String sex = req.getParameter("sex");
      String mail = req.getParameter("mail");
      String phone = req.getParameter("phone");

      String[] args = { name, password, username, sex, mail, phone };

      this.checkForm(args);

      FileWriter fr = new FileWriter(RegisterServlet.fileBasePath + "/userinfo.txt", true);
      BufferedWriter bw = new BufferedWriter(fr);

      bw.append(name + "|" + sha256(password) + "|" + username + "|" + sex + "|" + mail + "|" + phone);
      bw.newLine();
      bw.flush();
      bw.close();
      fr.close();
      this.code = "1";
      this.message = "注册成功";

    } catch (Exception err) {
      this.message = err.getMessage();
    }

    JSONObject resBody = new JSONObject();
    resBody.put("code", this.code);
    resBody.put("msg", this.message);

    res.setContentType("application/json;charset=utf-8");
    PrintWriter out = res.getWriter();
    out.println(resBody.toJSONString());
  }

}
