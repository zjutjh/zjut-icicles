<html>
  <body>
    <% String email = (String) session.getAttribute("email"); %>
    <h1>Welcome <%= email %></h1>
  </body>
</html>
