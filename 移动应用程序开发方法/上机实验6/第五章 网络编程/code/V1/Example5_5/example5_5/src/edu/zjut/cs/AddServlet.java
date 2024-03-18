package edu.zjut.cs;

import java.io.IOException;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

public class AddServlet extends HttpServlet {
	protected void doGet(HttpServletRequest req, HttpServletResponse resp)
			throws ServletException, IOException {
		StudentDAO sDAO = new StudentDAO();
		String sId = req.getParameter("id");
		String name = req.getParameter("sName");
		String major = req.getParameter("major");
		String sCredit = req.getParameter("credit");
		int id = Integer.parseInt(sId);
		double credit = Double.parseDouble(sCredit);
		Student student = new Student();
		student.setId(id);
		student.setName(name);
		student.setMajor(major);
		student.setCredit(credit);
		sDAO.addStudent(student);
	}

	@Override
	protected void doPost(HttpServletRequest req, HttpServletResponse resp)
			throws ServletException, IOException {
		doGet(req, resp);
	}
}
