# JavaWeb考前速成

## 1. 这次考试真正考什么

按 [Web课程总结..pdf](/D:/[AUTHOR]/Documents/material/JavaWeb/Web课程总结..pdf) 和 [Web应用开发综合练习题.txt](/D:/[AUTHOR]/Documents/material/JavaWeb/Web应用开发综合练习题.txt) 看，范围明显偏基础，重点不是前端，而是下面这些：

- Web基础：HTTP 请求/响应、URL 和 URI、常见状态码、表单
- MVC：Model / View / Controller 分工
- Servlet：生命周期、`doGet/doPost`、请求处理、转发/重定向
- 会话管理：`HttpSession`、Cookie
- JSP：生命周期、脚本元素、隐式对象、`jsp:forward`、`jsp:useBean`
- EL / JSTL：尤其是 `c:if`、`c:forEach`、`c:choose`
- JDBC / DataSource / DAO
- Servlet 高级应用：过滤器、监听器、线程安全
- Maven：老师点名“看一看”，通常考最基础的目录结构、`pom.xml`、`war`

不太像会考：

- 大量前端细节
- 很深的框架内容
- 特别新的 Jakarta 生态细节

## 2. 最值得背的三块“大题模板”

老师说“大题就这么考”，从练习题看，最像固定模板的是这三类：

### 模板 A：简答题

高频就是这 4 个：

- JavaBean 规范
- `forward()` 和 `sendRedirect()` 区别
- MVC 一般步骤
- 传统 JDBC 和 DataSource 对比

这 4 个几乎可以原样抄到开卷纸。

### 模板 B：程序分析 / 填空题

高频就是这 4 个：

- URL 重定向结果怎么写
- JSTL 标签属性怎么补
- 自定义标签 `HelloTag + TLD + JSP` 怎么补
- DAO 查询代码怎么补

### 模板 C：程序设计题

固定套路：

- `input.jsp` 表单
- `Student` JavaBean
- `InputHandleServlet`
- `showInfo.jsp`

这题本质就是 MVC 小综合题。

## 3. 速成顺序

如果你时间紧，建议按这个顺序学：

1. Web基础 + MVC
2. Servlet
3. JSP + EL + JSTL
4. Session + Cookie
5. JDBC + DataSource + DAO
6. Filter + Listener
7. Maven 最基础

原因很简单：

- 选择题会散着考基础概念
- 简答和大题几乎都绕不开 MVC / Servlet / JSP / DAO
- Maven 不一定出大题，但容易出常识题

## 4. 你现在先记住的核心句子

### 4.1 URL / URI

- URI 是统一资源标识符，范围更大
- URL 是 URI 的子集，表示资源位置
- 考试里常把 URL 理解成“能定位资源的地址”

### 4.2 HTTP 状态码

- `200`：成功
- `404`：资源不存在
- `500`：服务器内部错误
- `401`：未授权

### 4.3 MVC

- Model：数据和业务逻辑，常用 JavaBean / DAO
- View：显示页面，常用 JSP
- Controller：接收请求、调度处理，常用 Servlet

一句话：

`Servlet 收请求，JavaBean/DAO 处理数据，JSP 负责显示。`

## 5. Servlet 必会

### 5.1 生命周期

顺序：

`init() -> service() -> doGet()/doPost() -> destroy()`

要点：

- 容器先创建 Servlet 对象
- 初始化时调用 `init()`
- 每次请求先到 `service()`
- `service()` 再分发给 `doGet()` 或 `doPost()`
- 销毁前调用 `destroy()`

### 5.2 `doGet` 和 `doPost`

- 超链接访问一般走 `GET`
- 表单可配 `GET` 或 `POST`
- `<a href="...">` 点进去默认看 `doGet()`

### 5.3 请求域存数据

```java
request.setAttribute("student", student);
Object obj = request.getAttribute("student");
```

要记住：

- `request` 域只在一次请求里有效
- 转发后还能用
- 重定向后不能用

### 5.4 转发和重定向

`forward()`：

- 服务器内部跳转
- 浏览器地址栏通常不变
- 还是同一个请求
- `request` 中的数据可以带过去

`sendRedirect()`：

- 告诉浏览器重新发新请求
- 地址栏会变
- 是两个请求
- `request` 中的数据带不过去
- `session` 里的数据还能用

### 5.5 `sendRedirect` 路径题

若当前请求：

`http://www.zjut.com/myapp/cool/bar.do`

那么：

- `sendRedirect("/foo/stuff.html")`
  - 结果：`http://www.zjut.com/foo/stuff.html`
- `sendRedirect("foo/stuff.html")`
  - 结果：`http://www.zjut.com/myapp/cool/foo/stuff.html`

## 6. Session 和 Cookie 必会

### 6.1 Session

常见写法：

```java
HttpSession session = request.getSession(true);
session.setAttribute("key", value);
Object obj = session.getAttribute("key");
session.removeAttribute("key");
```

要点：

- `getAttribute()` 返回 `Object`
- 可能返回 `null`
- 同名属性再次 `setAttribute()` 会覆盖旧值，不会报错

### 6.2 Cookie

发送 Cookie：

```java
Cookie c = new Cookie("username", "admin");
c.setMaxAge(60 * 60 * 24 * 7);
response.addCookie(c);
```

读取 Cookie：

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

## 7. JSP / EL / JSTL 必会

### 7.1 JSP 生命周期

- 能重写：`jspInit()`、`jspDestroy()`
- 不能手动重写：`_jspService()`

所以题目里常选：

`jspInit() 和 jspDestroy() 都可以覆盖，但 _jspService() 不可以。`

### 7.2 JSP 常考标准动作

请求转发：

```jsp
<jsp:forward page="view.jsp" />
```

创建请求域 Bean：

```jsp
<jsp:useBean id="mybean" class="com.example.MyBean" scope="request" />
```

取 Bean 属性：

```jsp
<jsp:getProperty name="student" property="sno" />
```

### 7.3 JavaBean 规范

- 类必须是 `public`
- 一般要有无参构造
- 属性通常 `private`
- 用 `getXxx()` / `setXxx()` 访问属性
- `boolean` 属性可用 `isXxx()`

### 7.4 EL 你只要会这些

- 取作用域变量：`${student}`
- 取属性：`${student.name}`
- 取 Cookie：`${cookie.userName.value}`

### 7.5 JSTL 核心标签

`forEach`：

```jsp
<c:forEach var="movie" items="${movieList}" varStatus="foo">
    ${movie}
</c:forEach>
```

`if`：

```jsp
<c:if test="${userPref == 'safety'}">
```

`set`：

```jsp
<c:set var="userLevel" scope="session" value="foo" />
```

`choose`：

```jsp
<c:choose>
    <c:when test="${userPref == 'performance'}">
    </c:when>
    <c:otherwise>
    </c:otherwise>
</c:choose>
```

## 8. JDBC / DataSource / DAO 必会

### 8.1 传统 JDBC 缺点

- 每次都要创建连接
- 用完还要关闭连接
- 很耗时间
- 并发多时效率低

### 8.2 DataSource 优点

- 连接池复用连接
- 降低创建连接开销
- 提高响应速度
- Web 开发更常用

### 8.3 DAO 题常见补空

```java
Connection conn = dataSource.getConnection();
PreparedStatement pstmt = conn.prepareStatement(sql);
ResultSet rst = pstmt.executeQuery();
Customer customer = new Customer();
custList.add(customer);
```

### 8.4 DAO 模式一句话

`DAO 负责访问数据库，JavaBean 负责装数据，Servlet 负责调 DAO，JSP 负责显示。`

## 9. Filter / Listener 高频点

### 9.1 过滤器

作用：

- 拦截请求和响应
- 编码处理
- 登录校验
- 权限控制
- 压缩、加密、审计

顺序题要点：

- 直接访问 `/admin/index.jsp`
- 只会匹配这次请求本身的过滤器
- 配了 `FORWARD` 的过滤器，对“直接浏览器请求”通常不生效

练习题那道选择题答案是：

`Filter3, Filter4`

### 9.2 监听器

请求属性变化那道题要记住：

```java
request.setAttribute("a", "b");
request.setAttribute("a", "c");
request.removeAttribute("a");
```

日志：

`A: a->b  P: a->b  M: a->c`

## 10. Maven 最基础

### 10.1 你只要先会这几个词

- `pom.xml`：Maven 项目核心配置文件
- `groupId`：组织名
- `artifactId`：项目名
- `version`：版本号
- `packaging`：打包方式，Web 项目常见 `war`

### 10.2 常见目录

```text
src/main/java      Java 源码
src/main/resources 配置资源
src/main/webapp    Web 资源（JSP、HTML、WEB-INF）
src/test/java      测试代码
pom.xml            项目配置
```

### 10.3 Web 项目最小印象

```xml
<packaging>war</packaging>
```

### 10.4 常用命令

- `mvn clean`
- `mvn compile`
- `mvn test`
- `mvn package`

一句话：

`考试里 Maven 大概率考“它是干嘛的、pom.xml 是什么、Web 项目为什么是 war、目录结构怎么分”。`

## 11. 程序设计题固定写法

### 11.1 `input.jsp`

- 一个 form
- 两个文本框：`sno`、`sname`
- 提交按钮、重置按钮
- `action="../inputHandleServlet.do"`
- `method="post"`

### 11.2 `Student`

- `private String sno;`
- `private String name;`
- 无参构造
- 有参构造
- getter / setter

### 11.3 `InputHandleServlet`

必写步骤：

- `request.setCharacterEncoding("UTF-8");`
- `request.getParameter("sno")`
- `request.getParameter("sname")`
- `new Student(sno, name)`
- `request.setAttribute("student", student)`
- `request.getRequestDispatcher("/jsp/showInfo.jsp")`
- `dispatcher.forward(request, response)`

### 11.4 `showInfo.jsp`

核心写法：

```jsp
<jsp:useBean id="student" type="com.model.Student" scope="request"/>
学号：<jsp:getProperty name="student" property="sno" />
姓名：<jsp:getProperty name="student" property="name" />
<a href='jsp/input.jsp'>返回输入页面</a>
```

## 12. 临考策略

- 选择题先秒 HTTP 状态码、Servlet 生命周期、JSP 动作、Cookie/Session 方法
- 简答题直接背模板句，不临场现编
- 程序分析题先认模式：JSTL / 自定义标签 / DAO
- 程序设计题按 MVC 四件套去写：`JSP表单 -> Bean -> Servlet -> JSP显示`
- 开卷纸优先抄“定义 + 区别 + 固定代码模板”

## 13. 你现在最该背的 10 句

1. `Servlet 收请求，JSP 负责显示，JavaBean/DAO 负责数据。`
2. `init -> service -> doGet/doPost -> destroy`
3. `forward 是一次请求，redirect 是两次请求`
4. `forward 能带 request 数据，redirect 不能`
5. `HttpSession.getAttribute() 返回 Object，可能为 null`
6. `Cookie 用 response.addCookie() 发，用 request.getCookies() 取`
7. `JSP 可重写 jspInit 和 jspDestroy，不能重写 _jspService`
8. `JavaBean 要有无参构造、private 属性、getter/setter`
9. `DataSource 用连接池，比传统 JDBC 更高效`
10. `Maven Web 项目通常打成 war，核心文件是 pom.xml`

## 14. 自测 8 题

1. `404`、`500`、`401` 分别是什么意思？
2. 点一个 `<a href="...">` 超链接，Servlet 默认进 `doGet` 还是 `doPost``？`
3. `forward()` 和 `sendRedirect()` 最大区别是什么？
4. `request` 里的属性，重定向后还能拿到吗？
5. JSP 里哪个标准动作可以转发页面？
6. JavaBean 为什么一般要有无参构造？
7. DataSource 比传统 JDBC 好在哪？
8. Maven 的 `pom.xml` 是干什么的？

