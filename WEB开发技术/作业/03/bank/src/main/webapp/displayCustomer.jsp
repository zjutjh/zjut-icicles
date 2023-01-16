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
        <td><jsp:getProperty name="customer" property="email" /></td>
        <td><jsp:getProperty name="customer" property="custName" /></td>
        <td><jsp:getProperty name="customer" property="phone" /></td>
      </tr>
    </table>
  </body>
</html>
