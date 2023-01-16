- `<% %>` Java 代码片段（记得每行加`;`）
- `<%! %>` 声明（记得加`;`）
- `<% page %>`  指令 
- `<%= %>` 表达式
- `<jsp:action />` 动作
- `%{}` EL表达式
- `<%-- --%>`注释

## Jsp Snippet

```jsp
<%@ page contentType="text/html; charset=utf-8"%>
<html>
  <head>
    <meta charset="UTF-8" />
    <title>Welcome</title>
  </head>

  <body>
  </body>

</html>
```

## 使用 JavaBean

```jsp
<jsp:useBean id="customer" class="com.model.Customer" scope="session"></jsp:useBean>
<!-- 记得声明 JavaBean 类，但是不需要引入 -->

<em>${customer.getEmail()}<em>

<em><jsp:getProperty name="customer" property="email" /></em>
<!-- 注意是单标签 -->

<em><%= customer.getEmail() %></em>
```

## 代码片段中读取`session`的内容

```jsp
<%@ page import="java.util.ArrayList" %>
<!-- 代码片段中使用的类，必须要 import -->
<!-- String 就不用加了 -->

<% ArrayList<Customer> customers = (ArrayList<Customer>) session.getAttribute("customers");
%>
```

## 代码片段中的`for`

```jsp
<!-- forEach -->
<% for(Customer item: customers) { %>
<tr>
  <td><%= item.getEmail() %></td>
  <td><%= item.getCustName() %></td>
  <td><%= item.getPhone() %></td>
</tr>
<% } %>
```

## 声明语句的作用域

> 每次访问页面，显示的数字累加

```jsp
<%! int a = 1; %>
<% a++; %>

<em>${ a }<em>
```

- 声明语句仅仅在Servlet初始化时执行一次，被保存在内存中
- 代码片段每次访问页面会执行一次

## 代码片段和声明语句的顺序

```jsp
<% a++; %>
<%! int a = 1; %>
```
不能过编译

## 转发请求

#todo

`<jsp:forward />`