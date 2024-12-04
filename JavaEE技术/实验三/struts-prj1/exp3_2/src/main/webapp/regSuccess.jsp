<%@ page language="java" contentType="text/html; charset=UTF-8" pageEncoding="UTF-8"%>
<%@ taglib prefix="s" uri="/struts-tags" %>
<html>
<head>
  <s:head theme="xhtml"/>
  <style>
    body {
      text-align: center;
      padding: 40px;
    }

    h2 {
      color: #333;
    }
  </style>
</head>
<body>
<h2>注册成功</h2>
<!-- 数据标签 property -->
<s:property value="loginUser.username"/>

<!-- 控制标签 if/else -->
<s:if test='loginUser.sex=="1"'>
  <s:text name="先生，"/>
</s:if>
<s:else>
  <s:text name="女士，"/>
</s:else>

您注册成功了！

<!-- 数据标签 set -->
<s:set var="user" value="loginUser" scope="session"/>
</body>
</html>
