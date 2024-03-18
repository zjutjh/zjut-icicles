package edu.zjut.cs;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

public class ListServlet extends HttpServlet {
	protected void doGet(HttpServletRequest req, HttpServletResponse resp)
			throws ServletException, IOException {
		StudentDAO sDAO = new StudentDAO();
		List<Student> students = sDAO.getAllStudents();
		String studentString = this.createStudentsString(students);
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

	private String createStudentsString(List<Student> students) {
		StringBuffer buffer = new StringBuffer();
		for (Student student : students) {
			buffer.append(student.getId()).append(",");
			buffer.append(student.getName()).append(",");
			buffer.append(student.getMajor()).append(",");
			buffer.append(student.getCredit()).append("\n");
		}
		return buffer.toString();
	}
}
