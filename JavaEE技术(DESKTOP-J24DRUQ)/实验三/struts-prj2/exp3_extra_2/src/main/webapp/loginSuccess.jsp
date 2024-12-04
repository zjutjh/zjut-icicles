<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ taglib prefix="s" uri="/struts-tags" %>

<html>
<head>
    <title>Login Success</title>
    <style>
        body {
            text-align: center;
            padding: 40px;
        }

        h2 {
            color: #000000; /* Bootstrap's info color for emphasis */
        }

        p {
            color: #000000;
            font-weight: bold;
        }
    </style>
</head>
<body>


<h2>Login Success</h2>
<s:actionmessage/>
<p>登录成功！</p>
<a href="./allItems">查看所有商品信息</a>

</body>
</html>
