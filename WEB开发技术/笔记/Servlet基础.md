## Servlet Snippet

```java
package com.model;

import java.io.*;
import jakarta.servlet.*;
import jakarta.servlet.http.*;
import jakarta.servlet.annotation.WebServlet;

@WebServlet("/CustomerServlet.do")
public class CustomerServlet extends HttpServlet {
	  public void doPost(HttpServletRequest req, HttpServletResponse res) throws ServletException, IOException {
      Printer out = res.getWriter();
	  }
}
```

#todo

## HttpServletResponse

## HttpServletRequest

`getParameter("name")` 获取 POST 请求 Param
