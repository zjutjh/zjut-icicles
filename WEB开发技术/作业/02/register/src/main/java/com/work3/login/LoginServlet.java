package com.work3.login;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.rmi.ServerException;

import com.alibaba.fastjson.JSONObject;
import com.alibaba.fastjson.annotation.JSONField;
import com.work3.register.RegisterServlet;

import jakarta.servlet.http.HttpServlet;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;

class LoginResponse {
  @JSONField(name = "msg")
  private String message = "";

  @JSONField(name = "code")
  private String code = "200404";

  public LoginResponse(String message, String code) {
    this.message = message;
    this.code = code;
    System.out.println(message + " " + code);
  }

}

public class LoginServlet extends HttpServlet {
  static final String fileBasePath = "/Users/j10c/playground";
  @JSONField(name = "msg")
  private String message = null;

  @JSONField(name = "code")
  private String code = "200404";

  @Override
  public void init() {
    this.message = null;
    this.code = "200404";
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse res) throws ServerException, IOException {
    String username = req.getParameter("username");
    String password = req.getParameter("password");

    FileReader fr = new FileReader(fileBasePath + "/userinfo.txt");
    BufferedReader br = new BufferedReader(fr);
    this.init();

    try {
      while (br.ready()) {
        String[] items = br.readLine().split("\\|");
        System.out.println(username + " " + items[0]);
        if (username.equals(items[0])) {
          if (RegisterServlet.sha256(password).equals(items[1])) {
            System.out.println("success");
            this.message = "登录成功";
            this.code = "1";
            break;
          } else {
            this.message = "用户名或密码错误，请重新登录";
            this.code = "200501";
            break;
          }
        }
      }
      if (message == null) {
        this.message = "用户名或密码错误，请重新登录";
        this.code = "200501";
      }

    } catch (Exception err) {
      this.message = err.getMessage();
      this.code = "200504";
    } finally {
      br.close();
      fr.close();
    }

    JSONObject resBody = new JSONObject();
    resBody.put("code", this.code);
    resBody.put("msg", this.message);

    res.setContentType("application/json;charset=utf-8");
    PrintWriter out = res.getWriter();
    out.println(resBody.toJSONString());
  }
}
