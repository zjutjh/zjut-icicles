<%@ page language="java" contentType="text/html; charset=GBK" pageEncoding="GBK"%>
<!DOCTYPE html PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN" "http://www.w3.org/TR/html4/loose.dtd">
<html>
<head>
  <meta http-equiv="Content-Type" content="text/html; charset=GBK">
  <title>用户登录页面</title>
  <script>
    function validateForm() {
      var username = document.getElementById("username").value;
      var password = document.getElementById("password").value;

      if (username === "" || username.length > 6) {
        alert("用户名不能为空且不能超过 6 位");
        return false;
      }

      if (password === "" || password.length > 6) {
        alert("密码不能为空且不能超过 6 位");
        return false;
      }

      return true;
    }
  </script>
</head>
<body>
<form action="login" method="post" onsubmit="return validateForm()">
  <table>
    <tr>
      <td>请输入用户名：</td>
      <td><input id="username" name="username" type="text"></td>
    </tr>
    <tr>
      <td>请输入密码：</td>
      <td><input id="password" name="password" type="password"></td>
    </tr>
    <tr>
      <td colspan="2" align="center">
        <input type="submit" value="登录">
      </td>
    </tr>
  </table>
</form>
</body>
</html>
