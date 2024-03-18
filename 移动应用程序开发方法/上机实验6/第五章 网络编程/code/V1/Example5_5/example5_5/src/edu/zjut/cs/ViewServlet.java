package edu.zjut.cs;

import java.io.IOException;
import java.io.PrintWriter;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

public class ViewServlet extends HttpServlet {
	protected void doGet(HttpServletRequest req, HttpServletResponse resp)
			throws ServletException, IOException {
		StudentDAO sDAO = new StudentDAO();
		String sId = req.getParameter("id");
		int id = Integer.parseInt(sId);
		Student student = sDAO.getStudentById(id);
		String studentString = this.createStudentString(student);
		resp.setCharacterEncoding("UTF-8");
		resp.setContentType("text/html;charset=utf-8");
		PrintWriter writer = resp.getWriter();
		writer.write(studentString);
		writer.flush();
		writer.close();
	}

	@Override
	protected void doPost(HttpServletRequest req, HttpServletResponse resp)
			throws ServletException, IOException {
		doGet(req, resp);
	}

	private String createStudentString(Student student) {
		StringBuffer buffer = new StringBuffer();
		buffer.append(student.getId()).append(",");
		buffer.append(student.getName()).append(",");
		buffer.append(student.getMajor()).append(",");
		buffer.append(student.getCredit());
		return buffer.toString();
	}
}
