## JSP

```jsp
<jsp:useBean id="customer" class="com.model.Customer" scope="session"></jsp:useBean>
```

## Servlet

```java
HttpSession session = req.getSession();

session.setAttribute("customer", this.customer);
```