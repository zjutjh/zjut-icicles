## 文件路径

```java
// absolute path
String filePath = "/Users/j10c/projects/java/bank/target/bank/WEB-INF" + "/customerinfo.txt";

// relative path
String filePath = this.getServletContext().getRealPath("") + '/WEB-INF/customerinfo.txt';
// 注意 getRealPath("") 获取的路径

File file = new File(filePath);
```

## 写入

```cpp
FileWriter fw = new FileWriter(file, true);
// 第二个参数为 append , 表示追加内容
BufferedWriter bw = new BufferedWriter(fw);

bw.write("hello world");
bw.newLine(); // 换行，方便下一次写入
bw.flush();

bw.close();
fw.close();
```

## 读取

```cpp
FileReader fr = new FileReader(file)
BufferedReader br = new BufferedReader(fr);

while (br.ready()) {
  String item = br.readLine();
  // do something...
}

br.close();
fr.close();
```

## 下载二进制

```cpp
public void handleDownlaod(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    OutputStream out = resp.getOutputStream();
    String filePath = "/WEB-INF/files";
    // File file = new File(filePath + "/servlet.pdf");
    resp.setHeader("Content-Disposition", "attachment;filename=" + "servlet.pdf");
    resp.setContentType("application/pdf");

    InputStream fis = new FileInputStream(this.getServletContext().getRealPath("") + filePath + "/servlet.pdf");

    byte[] buffer = new byte[1024];
    int len = 0;
    while ((len = fis.read(buffer)) > 0) {
      out.write(buffer, 0, len);
    }
    out.flush();
    fis.close();
    out.close();
  }
```