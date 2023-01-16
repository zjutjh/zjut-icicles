```java
HttpSession session = request.getSession();
(Object)session.getAttribute("name");
session.setAttribute("name", value);

session.invalidate();
```

## 把类存在 Session
```java
public class Class implements Serializable {
  // JavaBean
}
```
