
## template

```java
res.setHeader("Content-Encoding", "gb2312");
res.setHeader("Content-Disposition", "attachment; filename="
          + java.net.URLEncoder.encode("score.xls", "UTF-8"));
res.setContentType("application/vnd.ms-excel;charset=gb2312");

PrintWriter out = res.getWriter();
out.println("序号\t学号\t姓名\t课程名\t成绩");
```