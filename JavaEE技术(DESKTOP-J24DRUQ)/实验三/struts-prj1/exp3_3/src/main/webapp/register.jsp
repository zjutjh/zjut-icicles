<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8" %>
<%@ taglib prefix="s" uri="/struts-tags" %>
<html>
<head>
    <s:head theme="xhtml"/>
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
<h2>注册</h2>
<s:form action="register" method="post">
    <s:textfield name="loginUser.username" label="请输入用户名"/>
    <s:password name="loginUser.password" label="请输入密码"/>
    <s:password name="loginUser.Repassword" label="确认密码"/>
    <s:textfield name="loginUser.realName" label="真实姓名"/>
    <s:radio name="loginUser.sex" list="#{1 : '男', 0 : '女'}" label="请选择性别"/>
    <s:textfield name="loginUser.birthday" label="请输入生日(yyyy-MM-dd)">
        <s:param name="value">
            <s:date name="loginUser.birthday" format="yyyy-MM-dd"/>
        </s:param>
    </s:textfield>
    <s:textfield name="loginUser.address" label="请输入联系地址"/>
    <s:textfield name="loginUser.phoneNumber" label="请输入联系电话"/>
    <s:textfield name="loginUser.email" label="请输入电子邮箱"/>
    <!-- 其他字段根据需要添加 -->

    <s:submit value="注册" method="register"/>
    <s:reset value="重置"/>
</s:form>
</body>
</html>
