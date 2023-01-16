package com.work1.triangle;

import java.io.IOException;
import java.io.PrintWriter;

import jakarta.servlet.*;
import jakarta.servlet.annotation.WebServlet;
import jakarta.servlet.http.*;

class NumberValidException extends Exception {
  @Override
  public String getMessage() {
    return "三条边长无法构成三角形";
  }
}

@WebServlet("/computeTriangleArea.do")
public class TriangleServlet extends HttpServlet {
  private double area = 0;
  private String message = "undefined";

  public void init() {
    this.area = 0;
    this.message = "undefined";
  }

  public void checkNumberValid(double a, double b, double c) throws NumberValidException {
    if (Math.abs(a + b - c) < 0.00001 || Math.abs(a + c - b) < 0.00001 || Math.abs(b + c - a) < 0.00001 || a + b < c
        || b + c < a || a + c < b)
      throw new NumberValidException();
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse res)
      throws ServletException, IOException {

    try {
      double a = Double.parseDouble(req.getParameter("a"));
      double b = Double.parseDouble(req.getParameter("b"));
      double c = Double.parseDouble(req.getParameter("c"));
      if (a <= 0 || b <= 0 || c <= 0)
        throw new NumberFormatException();

      this.checkNumberValid(a, b, c);
      double semiLength = (a + b + c) / 2;
      this.area = Math.sqrt(semiLength * (semiLength - a) * (semiLength - b) * (semiLength - c));
      this.message = "<h1>三角形面积=" + String.valueOf(Math.floor(this.area * 100) / 100) + "</h>";

    } catch (NumberFormatException err) {
      this.message = "<h1>输入的边长有误！</h1>";
    } catch (NumberValidException err) {
      this.message = "<h1>" + err.getMessage() + "</h1>";
    } catch (Exception err) {
      this.message = err.getMessage();
    } finally {
      res.setContentType("text/html;charset=utf-8");
      PrintWriter out = res.getWriter();
      out.print(this.message);
    }

  }

  public void destory() {

  }
}
