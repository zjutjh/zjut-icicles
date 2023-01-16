```java
RequestDispatcher rd = req.getRequestDispatcher("/displayCustomer.jsp");
// 或者一个接口名

rd.forword(req, res);
```