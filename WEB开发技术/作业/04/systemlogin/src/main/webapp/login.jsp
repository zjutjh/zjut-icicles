<html>
  <head></head>
  <body>
    <h1>Login</h1>
    <form action="LoginActionServlet.do" method="post">
      <div>
        <label for="identity">Identity: </label>
        <select name="identity" id="identity">
          <option value="student">Student</option>
          <option value="teacher">Teacher</option>
        </select>
      </div>

      <div>
        <label for="username">Username: </label>
        <input id="username" name="username" required />
      </div>

      <div>
        <label for="password">Password: </label>
        <input id="password" name="password" required />
      </div>

      <button type="submit">Submit</button>
    </form>
  </body>
</html>
