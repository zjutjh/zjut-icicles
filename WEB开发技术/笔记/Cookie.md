## 读取 Cookie

```java
Cookie[] cookies = request.getCookies();

for (Cookie cookie: cookies) {
  String vaue = cookie.getValue();
  String name = cookie.getName();
  
}
```

## 发送 Cookie

```java
Cookie cookie = new Cookie("name", value);
cookie.setMaxAge(30); // 秒

cookie.setDomin();
```