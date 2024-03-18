<%@ page pageEncoding="UTF-8" %>
<%@ page contentType="text/html;charset=UTF-8" %>
<%
  String query = request.getParameter("query");
  if (query.equals("query1")) {
    out.println("查询关键词1对应的内容");
  }
  else if (query.equals("query2")) {
    out.println("查询关键词2对应的内容");
  }
  else if (query.equals("query3")) {
    out.println("查询关键词3对应的内容");
  }
  else {
    out.println("查询不到对应的内容");
  }
%>