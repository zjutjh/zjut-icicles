package edu.zjut.cs;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.List;

public class StudentDAO {
	private static final String DRIVER = "com.mysql.jdbc.Driver";
	private static final String URL = "jdbc:mysql://localhost:3306/mydb";
	private static final String ACCOUNT = "root";
	private static final String PASSWORD = "12345";

	public List<Student> getAllStudents() {
		try {
			Connection conn = this.getConn();
			Statement stmt = conn.createStatement();
			String sql = "SELECT * FROM student";
			ResultSet rs = stmt.executeQuery(sql);
			List<Student> students = new ArrayList<Student>();
			while (rs.next()) {
				int id = rs.getInt("id");
				String name = rs.getString("name");
				String major = rs.getString("major");
				double credit = rs.getDouble("credit");
				Student student = new Student();
				student.setId(id);
				student.setName(name);
				student.setMajor(major);
				student.setCredit(credit);
				students.add(student);
			}
			conn.close();
			return students;
		} catch (Exception ex) {
			ex.printStackTrace();
			return null;
		}
	}

	public void addStudent(Student student) {
		try {
			Connection conn = this.getConn();
			Statement stmt = conn.createStatement();
			String sql = "INSERT INTO student (id,name,major,credit) VALUES ("
					+ student.getId() + ",'" + student.getName() + "','"
					+ student.getMajor() + "'," + student.getCredit() + ")";
			stmt.executeUpdate(sql);
			conn.close();
		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}

	public Student getStudentById(int id) {
		try {
			Connection conn = this.getConn();
			Statement stmt = conn.createStatement();
			String sql = "SELECT * FROM student WHERE id=" + id;
			ResultSet rs = stmt.executeQuery(sql);
			Student student = null;
			while (rs.next()) {
				String name = rs.getString("name");
				String major = rs.getString("major");
				double credit = rs.getDouble("credit");
				student = new Student();
				student.setId(id);
				student.setName(name);
				student.setMajor(major);
				student.setCredit(credit);
				break;
			}
			conn.close();
			return student;
		} catch (Exception ex) {
			ex.printStackTrace();
			return null;
		}
	}

	private Connection getConn() {
		try {
			Class.forName(DRIVER);
			Connection conn = DriverManager.getConnection(URL, ACCOUNT,
					PASSWORD);
			return conn;
		} catch (Exception ex) {
			ex.printStackTrace();
			return null;
		}
	}

	public static void main(String args[]) {
		StudentDAO sDAO = new StudentDAO();
		/*
		 * List<Student> students = sDAO.getAllStudents(); for (Student student :
		 * students) { System.out.println(student.getName()); }
		 */
		Student student = new Student();
		student.setId(5);
		student.setName("Tom");
		student.setMajor("CS");
		student.setCredit(6.0);
		sDAO.addStudent(student);
	}
}
