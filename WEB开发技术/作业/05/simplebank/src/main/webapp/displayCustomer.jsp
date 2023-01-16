<%@ page isELIgnored ="false" language="java"%> <%@ page contentType="text/html;
charset=utf-8" %> <%@ page import="com.model.Customer"%>
<html>
  <body>
    <jsp:useBean
      id="customer"
      class="com.model.Customer"
      scope="session"
    ></jsp:useBean>
    <h1>Your Info</h1>
    <table>
      <tr>
        <td>Email</td>
        <td>Name</td>
        <td>Phone</td>
      </tr>
      <tr>
        <td>${customer.getEmail()}</td>
        <td>${customer.getCustName()}</td>
        <td>${customer.getPhone()}</td>
      </tr>
    </table>
  </body>
</html>
