# JavaWeb开卷纸

## 0. 这张纸怎么用

- 前半部分：按章节抄核心知识点
- 中间部分：高频固定 API / JSP 标签 / DAO 填空
- 最后部分：大题模板，考场直接套

这次**不考选择题**，所以不需要专门留“高频选择题”区。  
但“程序识别意思 / 找错”仍然要做，所以保留一个很小的“识别关键词”区就够了。

---

## 1. 第1章：Web / Maven / MVC 最小够用

### 1.1 URL / URI

- `URL` 是 `URI` 的子集

### 1.2 Maven

- 核心文件：`pom.xml`
- 三坐标：
  - `groupId`
  - `artifactId`
  - `version`
- Web 项目打包：`war`
- 常见目录：
  - `src/main/java`
  - `src/main/resources`
  - `src/main/webapp`
  - `src/test/java`

### 1.3 MVC

- `Model = JavaBean / DAO`
- `View = JSP`
- `Controller = Servlet`

一句话：

`Servlet 收请求，JavaBean/DAO 处理数据，JSP 负责显示。`

---

## 2. 第2章：Servlet

### 2.1 生命周期

- `init() -> service() -> doGet()/doPost() -> destroy()`

### 2.2 超链接 / 表单

- `<a href="...">` 默认走 `doGet()`
- 表单常用：
  - `GET`
  - `POST`

### 2.3 取参数

```java
String sno = request.getParameter("sno");
String name = request.getParameter("sname");
```

### 2.4 request 域存数据

```java
request.setAttribute("student", student);
Object obj = request.getAttribute("student");
```

### 2.5 请求转发

```java
RequestDispatcher dispatcher = request.getRequestDispatcher("/jsp/showInfo.jsp");
dispatcher.forward(request, response);
```

- 服务器内部转发
- 一次请求
- 地址栏不变
- `request` 数据保留

### 2.6 重定向

```java
response.sendRedirect("xxx");
```

- 浏览器重新请求
- 两次请求
- 地址栏变化
- `request` 数据丢失
- `session` 数据可保留

### 2.7 `sendRedirect` 路径题

当前请求：

`http://www.zjut.com/myapp/cool/bar.do`

- `sendRedirect("/foo/stuff.html")`
  - `http://www.zjut.com/foo/stuff.html`

- `sendRedirect("foo/stuff.html")`
  - `http://www.zjut.com/myapp/cool/foo/stuff.html`

### 2.8 输出流

- 输出文本：
  - `PrintWriter out = response.getWriter();`
- 输出二进制 / Word / 下载：
  - `OutputStream out = response.getOutputStream();`

### 2.9 ServletConfig / ServletContext / HttpSession

- `ServletConfig`：当前 Servlet 的配置
- `ServletContext`：整个 Web 应用共享
- `HttpSession`：单个用户会话

### 2.10 Session 常用方法

```java
HttpSession session = request.getSession(true);
session.setAttribute("key", value);
Object obj = session.getAttribute("key");
session.removeAttribute("key");
```

- `getAttribute()` 返回 `Object`
- 可能 `null`
- 同名 `setAttribute()` 覆盖旧值，不抛异常

---

## 3. 第3章：JSP / JavaBean / MVC

### 3.1 JSP 生命周期

- 可重写：
  - `jspInit()`
  - `jspDestroy()`
- 不可重写：
  - `_jspService()`

### 3.2 JSP 动作

请求转发：

```jsp
<jsp:forward page="view.jsp" />
```

### 3.3 include

- 静态包含：

```jsp
<%@ include file="xxx.jsp" %>
```

- 动态包含：

```jsp
<jsp:include page="xxx.jsp" />
```

### 3.4 JavaBean 规范

- 类必须 `public`
- 一般有无参构造
- 属性一般 `private`
- 每个属性配 `getXxx()` / `setXxx()`
- `boolean` 属性可用 `isXxx()`

### 3.5 `useBean` / `getProperty`

```jsp
<jsp:useBean id="student" type="com.model.Student" scope="request"/>
<jsp:getProperty name="student" property="sno" />
<jsp:getProperty name="student" property="name" />
```

理解：

- `useBean`：从作用域中取出 Bean
- `getProperty`：本质调用 getter
  - `property="sno"` -> `getSno()`
  - `property="name"` -> `getName()`

### 3.6 MVC 一般步骤

1. 定义 `JavaBean`
2. `Servlet` 处理请求
3. 给 `JavaBean` 赋值
4. 存入作用域对象
5. 转发到 `JSP`
6. `JSP` 取数据展示

---

## 4. 第4章：EL / JSTL

### 4.1 EL

```jsp
${student}
${student.name}
${requestScope.submitSource}
${cookie.userName.value}
```

### 4.2 JSTL 高频空

```jsp
<c:forEach var="movie" items="${movieList}" varStatus="foo">
```

```jsp
<c:if test="${userPref == 'safety'}">
```

```jsp
<c:set var="userLevel" scope="session" value="foo" />
```

```jsp
<c:choose>
    <c:when test="${userPref == 'performance'}"></c:when>
    <c:otherwise></c:otherwise>
</c:choose>
```

---

## 5. 第5章：JDBC / DataSource / DAO

### 5.1 JDBC 基本对象

- `Connection`
- `PreparedStatement`
- `ResultSet`

### 5.2 驱动加载

```java
Class.forName("com.mysql.jdbc.Driver");
```

- 异常：
  - `ClassNotFoundException`

### 5.3 DAO 填空骨架

```java
Connection conn = dataSource.getConnection();
PreparedStatement pstmt = conn.prepareStatement(sql);
ResultSet rst = pstmt.executeQuery();
Customer customer = new Customer();
custList.add(customer);
```

### 5.4 JDBC vs DataSource

- 传统 JDBC：
  - 每次创建连接
  - 每次关闭连接
  - 耗时，效率低

- DataSource：
  - 连接池复用连接
  - 减少创建连接开销
  - 提高效率

---

## 6. 第6章：Session / Cookie

### 6.1 Session

- 服务器端保存
- 会话级数据
- 常存用户登录状态、购物车

### 6.2 Cookie

发送：

```java
Cookie c = new Cookie("username", "admin");
c.setMaxAge(60 * 60 * 24 * 7);
response.addCookie(c);
```

读取：

```java
Cookie[] cookies = request.getCookies();
if (cookies != null) {
    for (Cookie cookie : cookies) {
        if ("username".equals(cookie.getName())) {
            String value = cookie.getValue();
        }
    }
}
```

### 6.3 Session vs Cookie

- `Session`：服务器端
- `Cookie`：客户端
- 常用 Cookie 保存 `sessionId`，服务器据此找到 Session

---

## 7. 第7章：Filter / Listener

### 7.1 Filter 作用

- 编码处理
- 登录校验
- 权限控制
- 审计

### 7.2 Filter 顺序题

直接访问：

`/admin/index.jsp`

若有：

- `Filter1 -> /admin/* + FORWARD`
- `Filter3 -> /admin/*`
- `Filter4 -> /*`

答案：

`Filter3, Filter4`

### 7.3 Listener 日志题

```java
request.setAttribute("a", "b");
request.setAttribute("a", "c");
request.removeAttribute("a");
```

日志：

`A: a->b  P: a->b  M: a->c`

---

## 8. 程序识别 / 找错关键词

只留最小够用：

- 有 `doGet/doPost`、`request/response`、`@WebServlet`：`Servlet`
- 有 `<%@ page %>`、`${}`、`<jsp:...>`、`<c:...>`：`JSP`
- `request.getParameter(...)`：接收参数
- `request.setAttribute(...)`：给 JSP 传数据
- `forward(...)`：服务器内部转发
- `sendRedirect(...)`：浏览器重定向
- `request.getCookies()`：读 Cookie
- `request.getSession()`：取 Session
- `Connection/PreparedStatement/ResultSet`：`DAO/JDBC`
- `doFilter(...)`：`Filter`
- `attributeAdded/sessionCreated/contextInitialized`：`Listener`

常见错句：

- `request.getCookie(...)`：错，应该 `request.getCookies()`
- `jsp:forward file="..."`：错，应该 `page="..."`
- `jsp:makeBean`：常考错，标准写法常用 `jsp:useBean`
- `_jspService()` 可重写：错
- `setAttribute()` 同名会抛异常：错

---

## 9. 大题模板区

### 9.1 简答题模板

#### JavaBean

- `public` 类
- 无参构造
- 属性 `private`
- `getXxx()/setXxx()`
- 封装数据，供 Servlet/JSP 使用

#### `forward()` vs `sendRedirect()`

- `forward`：一次请求，地址栏不变，`request` 可保留
- `redirect`：两次请求，地址栏变化，`request` 丢失，`session` 可保留

#### MVC

- `Model = JavaBean / DAO`
- `View = JSP`
- `Controller = Servlet`
- 步骤：
  - 接收请求
  - 处理数据
  - 存入作用域
  - 转发到 JSP
  - JSP 展示

#### JDBC vs DataSource

- JDBC：频繁创建/关闭连接，效率低
- DataSource：连接池复用连接，效率高

### 9.2 程序设计题模板

#### `jsp/input.jsp`

```jsp
<%@ page language="java" contentType="text/html; charset=UTF-8"
    pageEncoding="UTF-8"%>
<form action="../inputHandleServlet.do" method="post">
   学号：<input type="text" name="sno" size="15" /><br>
   姓名：<input type="text" name="sname" size="15"/><br>
   <input type="submit" value="登录" />
   <input type="reset" value="取消" />
</form>
```

#### `com.model.Student`

```java
package com.model;

public class Student {
    private String sno;
    private String name;

    public Student() {}

    public Student(String sno, String name) {
        this.sno = sno;
        this.name = name;
    }

    public String getSno() { return sno; }
    public void setSno(String sno) { this.sno = sno; }
    public String getName() { return name; }
    public void setName(String name) { this.name = name; }
}
```

#### `InputHandleServlet`

```java
package com.controller;

import java.io.IOException;
import javax.servlet.RequestDispatcher;
import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import com.model.Student;

@WebServlet("/inputHandleServlet.do")
public class InputHandleServlet extends HttpServlet {
    protected void doGet(HttpServletRequest request, HttpServletResponse response)
            throws ServletException, IOException {
        doPost(request, response);
    }

    protected void doPost(HttpServletRequest request, HttpServletResponse response)
            throws ServletException, IOException {
        request.setCharacterEncoding("UTF-8");
        String sno = request.getParameter("sno");
        String name = request.getParameter("sname");
        if (sno == null) sno = "";
        if (name == null) name = "";
        Student student = new Student(sno, name);
        request.setAttribute("student", student);
        RequestDispatcher dispatcher = request.getRequestDispatcher("/jsp/showInfo.jsp");
        dispatcher.forward(request, response);
    }
}
```

#### `jsp/showInfo.jsp`

```jsp
<%@ page language="java" contentType="text/html; charset=UTF-8"
    pageEncoding="UTF-8"%>
<jsp:useBean id="student" type="com.model.Student" scope="request"/>
学号：<jsp:getProperty name="student" property="sno" /> <br><br>
姓名：<jsp:getProperty name="student" property="name" /> <br><br>
<a href='jsp/input.jsp'>返回输入页面</a>
```

---

## 10. 最后 10 句

1. `Servlet 收请求，JSP 负责显示，JavaBean/DAO 负责数据。`
2. `init -> service -> doGet/doPost -> destroy`
3. `forward` 一次请求，`redirect` 两次请求
4. `forward` 能带 `request`，`redirect` 不能
5. `HttpSession.getAttribute()` 返回 `Object`
6. `Cookie` 用 `response.addCookie()` 发，用 `request.getCookies()` 取
7. `JSP` 可重写 `jspInit/jspDestroy`，不可重写 `_jspService`
8. `JavaBean = 私有属性 + 无参构造 + getter/setter`
9. `DataSource` 用连接池，比传统 JDBC 高效
10. `程序设计题就是 MVC 小综合题`
