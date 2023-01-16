<!DOCTYPE html>
<html lang="zh-CN">
  <head>
    <meta charset="UTF-8" />
    <meta
      name="viewport"
      content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=0,minimal-ui:ios"
    />
    <meta http-equiv="X-UA-Compatible" content="ie=edge" />
    <title>login</title>
  </head>

  <body>
    <form action="LoginServlet.do" method="post">
      <div>
        <label for="username">Username: </label>
        <input id="username" name="username" />
      </div>

      <div>
        <label for="password">Password: </label>
        <input id="password" name="password" />
      </div>

      <button type="submit">Submit</button>
    </form>
  </body>
</html>
