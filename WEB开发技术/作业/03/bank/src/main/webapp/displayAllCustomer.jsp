<%@ page import="com.model.Customer" %>
<%@ page import="java.util.ArrayList" %>
<html>
  <body>
    <% 
      ArrayList<Customer> customers = (ArrayList<Customer>) session.getAttribute("customers");
    %>
    <h1>All customers Info</h1>
    <table>
      <tr>
        <td>Email</td>
        <td>Name</td>
        <td>Phone</td>
      </tr>
      <% for(Customer item: customers) { %>
      <tr>
        <td><%= item.getEmail() %></td>
        <td><%= item.getCustName() %></td>
        <td><%= item.getPhone() %></td>
      </tr>
      <% } %>
    </table>
  </body>
</html>
