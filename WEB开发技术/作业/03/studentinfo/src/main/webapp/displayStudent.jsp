<%@ page import="com.work4.studentinfo.Student" %>
<html>
  <body>
    <h1>Student Info</h1>

    <div>
      <jsp:useBean id="student" class="com.work4.studentinfo.Student" />
      <jsp:setProperty name="student" property="stuid" />
      <jsp:setProperty name="student" property="name" />
      <jsp:setProperty name="student" property="major" />

      <table border="true">
        <tr>
          <td>stuid</td>
          <td>name</td>
          <td>major</td>
        </tr>
        <tr>
          <td>
            <jsp:getProperty name="student" property="stuid" />
          </td>
          <td>
            <jsp:getProperty name="student" property="name" />
          </td>
          <td>
            <jsp:getProperty name="student" property="major" />
          </td>
        </tr>
      </table>
    </div>
  </body>
</html>
