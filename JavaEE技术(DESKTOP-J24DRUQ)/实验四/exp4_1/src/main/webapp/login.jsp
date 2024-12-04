<%@ page language="java" contentType="text/html; charset=UTF-8"
         pageEncoding="UTF-8" %>
<!DOCTYPE html>
<html>
<head>
    <title>Login Page</title>
    <style>
        body {
            text-align: center;
            padding: 40px;
        }

        form {
            display: inline-block;
            text-align: left;
            border: 1px solid #ccc;
            padding: 20px;
            border-radius: 8px;
            background-color: #f5f5f5;
        }

        h2 {
            color: #333;
        }
    </style>
</head>
<body>
<h1>User Login</h1>
<form action="login" method="post">
    请输入用户名：<input name="username" type="text"><br>
    请输入密码：<input name="password" type="password"><br>
    <input type="submit" value="登录">
</form>
</body>
</html>
