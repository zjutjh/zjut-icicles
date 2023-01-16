package com.work4.studentinfo;

import java.io.IOException;
import java.io.PrintWriter;

import jakarta.servlet.RequestDispatcher;
import jakarta.servlet.ServletContext;
import jakarta.servlet.ServletException;
import jakarta.servlet.annotation.WebServlet;
import jakarta.servlet.http.HttpServlet;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;

@WebServlet("/postStudentInfo")
public class StudentServlet extends HttpServlet {
  private String message = "";
  private String code = "200500";
  private Student student;

  public void init() {
    this.message = "";
    this.code = "200500";
    this.student = new Student();
  }

  @Override
  public void doPost(HttpServletRequest req, HttpServletResponse res) throws ServletException, IOException {
    this.init();
    this.student.setName(req.getParameter("name"));
    this.student.setMajor(req.getParameter("major"));
    this.student.setStuid(req.getParameter("stuid"));

    ServletContext context = this.getServletContext();
    context.setAttribute("student", this.student);

    try {
      RequestDispatcher rd = req.getRequestDispatcher("/displayStudent.jsp");
      rd.forward(req, res);

    } catch (Exception err) {
      this.message = err.getMessage();
    }

    PrintWriter out = res.getWriter();
    out.println(this.message);
    out.println(this.code);
  }

}
