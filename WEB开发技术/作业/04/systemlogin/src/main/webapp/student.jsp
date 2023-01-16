<%@ page contentType="text/html; charset=utf-8"%>
<!DOCTYPE html>
<jsp:useBean id="user" class="com.model.User" scope="session"> </jsp:useBean>
<html lang="zh-CN">
  <head>
    <meta charset="UTF-8" />
    <meta
      name="viewport"
      content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0,minimal-ui:ios"
    />
    <meta http-equiv="X-UA-Compatible" content="ie=edge" />
    <title>Welcome</title>
  </head>

  <body>
    <% if (user == null) {
    request.getRequestDispatcher("login.jsp").forward(request, response);} %>
    <h1><jsp:getProperty name="user" property="name" />同学，欢迎您！</h1>
  </body>
</html>
