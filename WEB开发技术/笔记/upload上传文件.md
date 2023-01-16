#todo

```html
<form action="fileUpload.do" method="post" enctype="multipart/form-data">
  单文件<input type="file" name="filename">
  多文件<input type="file" name="filename" multiple>
</form>
```

## 存入文件

```java
if (p.getSize() > 1024 * 1024) {
  p.delete();
  message = "文件太大";
} else {
  path = path + "/" + username;
  File f = new File(path);
  if (!f.exists()) {
    f.mkdirs();
  }
  String fname = getFileName(p);
  p.write(path + "/" + fname);
  message = "文件上传成功";
}
```

## 读取文件

```java
if (p.getSize() <= 1024 * 1024) {
  InputStream is = p.getInputStream();
  byte[] bytes = new byte[is.available()];
  is.read(bytes);
  String str = new String(bytes);
}
```