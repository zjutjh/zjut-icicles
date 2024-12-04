<%@ page language="java" contentType="text/html; charset=GBK" pageEncoding="GBK"%>

<!DOCTYPE html>
<html>
<head>
    <title>User Registration</title>
</head>
<body>
<h2>User Registration</h2>
<form action="RegisterServlet" method="post">
    <label for="username">Username:</label>
    <input type="text" id="username" name="username"><br><br>

    <label for="password">Password:</label>
    <input type="password" id="password" name="password"><br><br>

    <label for="usertype">User Type:</label>
    <select id="usertype" name="usertype">
        <option value="1">∆’Õ®”√ªß</option>
    </select><br><br>

    <input type="submit" value="Register">
</form>
</body>
</html>
