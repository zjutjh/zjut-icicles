# 教务系统前端 (Flask)

这是一个使用 Flask 搭建的简单前端，用于与 `university_db_setup.sql` 创建的数据库进行交互。

## 功能

-   一个全局搜索框，可以搜索学生、课程、教师和公告。
-   动态显示搜索结果，无需刷新页面。
-   展示热门搜索词。
-   自动记录用户的搜索历史。

## 运行步骤

### 1. 安装依赖

请确保您已安装 Python 3。然后通过 pip 安装所需的库：

```bash
pip install -r requirements.txt
```

### 2. 配置数据库连接

打开 `src/database.py` 文件，找到以下配置部分：

```python
DB_USER = "your_username" # TODO: 请替换为您的用户名
DB_PASS = "your_password" # TODO: 请替换为您的密码
```

请将 `your_username` 和 `your_password` 替换为您自己的 GaussDB / PostgreSQL 数据库的用户名和密码。您也可以根据需要修改主机（`DB_HOST`）、端口（`DB_PORT`）和数据库名（`DB_NAME`）。

### 3. 启动应用

在 `src` 目录下，运行以下命令来启动 Flask Web 服务器：

```bash
python app.py
```

服务器启动后，您会看到类似以下的输出：

```
 * Serving Flask app 'app'
 * Debug mode: on
WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
 * Running on http://127.0.0.1:5001
Press CTRL+C to quit
```

### 4. 访问系统

打开您的浏览器，访问 `http://127.0.0.1:5001`。您应该能看到教务系统的搜索主页。

## 文件结构

-   `app.py`: Flask 应用主文件，包含路由和 API 端点。
-   `database.py`: 负责所有与数据库的交互。
-   `requirements.txt`: Python 依赖列表。
-   `templates/`: 存放 HTML 模板。
    -   `index.html`: 系统主页面。
-   `static/`: 存放静态文件。
    -   `css/style.css`: 页面的样式表。
    -   `js/main.js`: 处理前端交互的 JavaScript 文件。 