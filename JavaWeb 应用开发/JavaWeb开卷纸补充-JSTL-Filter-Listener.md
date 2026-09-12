# JavaWeb 开卷纸补充：JSTL / Filter / Listener

## JSTL 高频填空
- `c:forEach` 常考 `varStatus`
- `c:if` 常考 `test`
- `c:set` 常考 `value`
- `c:choose` 里常考 `when / test / otherwise`

```jsp
<c:set var="passLine" value="60" />
<c:if test="${score >= passLine}"></c:if>
<c:forEach items="${studentList}" var="student" varStatus="s"></c:forEach>
<c:choose>
    <c:when test="${score >= 90}"></c:when>
    <c:otherwise></c:otherwise>
</c:choose>
```

## Filter 一眼识别

```java
public void doFilter(ServletRequest request, ServletResponse response, FilterChain chain)
        throws IOException, ServletException {
    // 前置处理
    chain.doFilter(request, response);
    // 后置处理
}
```

- 真正放行靠 `chain.doFilter(...)`
- 不写这句，请求到不了目标资源
- 常见作用：编码处理、登录校验、权限控制、审计
- `<dispatcher>FORWARD</dispatcher>` 只拦截转发
- 直接请求主要看 `REQUEST`

过滤器顺序题牢记：
- 若是直接访问 `/admin/index.jsp`
- `FORWARD` 类型过滤器不参与
- 所以只看 URL 匹配且 dispatcher 适用的过滤器

## Listener 一眼识别
- `contextInitialized()`：应用启动
- `sessionCreated()`：Session 创建
- `attributeAdded()`：加属性
- `attributeReplaced()`：改属性
- `attributeRemoved()`：删属性

```java
request.setAttribute("a", "b");
request.setAttribute("a", "c");
request.removeAttribute("a");
```

对应日志：
`A: a->b  P: a->b  M: a->c`

## 结合 Demo 记忆
- `jstlDemo.do`：看 `c:set / c:if / c:forEach / c:choose`
- `filterDemo.do`：看 `REQUEST` 过滤器和 `FORWARD` 过滤器谁生效
- `listenerDemo.do`：看 request/session 属性变化如何被 Listener 记录
