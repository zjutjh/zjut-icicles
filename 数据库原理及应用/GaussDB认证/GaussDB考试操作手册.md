# GaussDB考试操作手册

## 环境配置

### 基本信息
- **GaussDB 内网IP**: `ip_gauss`
- **弹性云服务器ECS IP**: `ip_ecs`  
- **开通GaussDB时设置的密码**: `password`

> ⚠️ **重要提醒**：上述三个内容需要根据实际情况填写，不要直接使用这些占位符！

## 0. 初始化操作

### 连接步骤
1. 按照提示步骤下单华为云GaussDB云服务器后，查看并记录上述两个IP地址
2. 打开xfce命令行
3. 连接ECS服务器：
   ```bash
   ssh root@ip_ecs
   ```
   > 💡 **注意**：将`ip_ecs`替换为实际的ECS IP地址
4. 提示输入yes/no时，输入 `yes`
   > ⚠️ **新手提醒**：必须输入完整的`yes`，不能只输入`y`
5. 输入密码：`Huawei@1234`（密码不显示，直接输入即可）
   > 💡 **注意**：输入密码时屏幕不会显示任何字符，这是正常的，直接输入完按回车即可
6. 连接成功后连接GaussDB：
   ```bash
   gsql -d postgres -U root -W password -p 8000 -h ip_gauss
   ```
   > 💡 **注意**：将`password`和`ip_gauss`替换为实际的密码和GaussDB内网IP

### 预期输出
```
gsql ((openGauss 2.0.0 build 78689da9) compiled at 2021-03-31 21:04:03 commit 0 last mr  )
Non-SSL connection (SSL connection is recommended when requiring high-security)
Type "help" for help.

postgres=#
```

## 题目1：用户和数据库创建

### 任务内容
1. 创建名为`db_dev`的用户，密码设置成`Huawei123!@`，并授予sysadmin的权限
2. 创建名为`finance`的数据库，编码格式采用默认的UTF-8，数据库的所有者为`db_dev`用户
3. 使用`db_dev`用户通过gsql客户端，将ECS弹性云服务器中`/root`目录下的`create_object.sql`文件导入`finance`数据库中
4. 数据导入完成后，使用`db_dev`用户通过gsql客户端登录`finance`数据库

### 操作步骤

#### 1. 创建用户并授权
```sql
CREATE USER db_dev WITH PASSWORD 'Huawei123!@';
ALTER USER db_dev SYSADMIN;
```
> ⚠️ **注意**：每条SQL语句后面要加分号`;`，执行完一条再执行下一条

#### 2. 创建数据库
```sql
CREATE DATABASE finance OWNER db_dev ENCODING 'UTF8';
```

### 预期输出
```sql
postgres=# CREATE USER db_dev WITH PASSWORD 'Huawei123!@';
CREATE ROLE
postgres=# ALTER USER db_dev SYSADMIN;
ALTER ROLE
postgres=# CREATE DATABASE finance OWNER db_dev ENCODING 'UTF8';
CREATE DATABASE
```

#### 3. 导入SQL文件
退出GaussDB：
```sql
\q
```
> 💡 **注意**：`\q`是退出命令，不需要分号
> ⚠️ **重要提醒**：必须先退出GaussDB才能执行下面的导入命令！

执行导入命令：
```bash
gsql -d finance -U db_dev -W 'Huawei123!@' -p 8000 -h ip_gauss -f /root/create_object.sql
```
> ⚠️ **重要**：这是一整行命令，不要换行！将`ip_gauss`替换为实际IP地址

#### 4. 登录finance数据库
```bash
gsql -d finance -U db_dev -W 'Huawei123!@' -p 8000 -h ip_gauss
```
> 💡 **提示**：随后复制题目给出的代码，执行即可

### 预期输出
```
gsql ((openGauss 2.0.0 build 78689da9) compiled at 2025-05-31 21:04:03 commit 0 last mr  )
Non-SSL connection (SSL connection is recommended when requiring high-security)
Type "help" for help.

finance=>
```
## 题目2：修改应用的数据库配置，并Java编译运行

### 任务内容
在`/root/db-dev-cert/src/expt/db/finance/expt/db/finance/resources`目录下，修改`config-db.properties`文件中数据库连接的信息（主节点内网IP地址），千万不要使用备节点的IP地址否则后续会出现权限不足的情况。

### 基本命令
- `cd` - 打开目录
- `vim` - 打开文件，按`i`进入输入模式
- 按`esc`后输入`:wq`保存文件，输入`:q!`不保存退出

### 操作步骤


#### 1. 退出GaussDB
```sql
\q
```
> ⚠️ **重要提醒**：必须先退出GaussDB才能执行Java编译命令！

#### 2. 修改配置文件
进入指定目录：
```bash
cd /root/db-dev-cert/src/expt/db/finance/expt/db/finance/resources
```

修改配置文件：
```bash
vim config-db.properties
```
随后将`192.168.x.x`替换为主节点内网IP地址`ip_gauss`，保存退出。

#### 3. 编译Java文件
进入指定目录：
```bash
cd /root/db-dev-cert/src/expt/db/finance/
```
> 💡 **注意**：此处如果目录不对以题目为准（这里目录不需要输入，复制命令即可）

编译Java文件：
```bash
javac -classpath ../../../ -d . *.java
```
> 💡 **注意**：此处如果命令不对以题目为准（这里不需要输入，复制命令即可）

#### 4. 运行程序
```bash
java -p /root/db-dev-cert/libs/opengauss-jdbc-2.0.0.jar expt.db.finance.launch
```
> 💡 **注意**：此处如果命令不对以题目为准（这里不需要输入，复制命令即可）

> 📝 **补充说明**：后续内容根据题目操作，操作完成后按0退出

### 预期输出
```
银行管理系统
1)客户信息查询
2)客户信息添加
3)客户信息修改
4)客户信息删除
5)客户开卡
6)客户销卡
7)账户信息查询
8)账户信息添加
9)账户信息修改
10)账户信息删除
11)查询客户卡余额
12)存取款业务
0)退出系统
=> 请输入需要进行的操作编号:
```

## 题目3：系统表查询

### 任务内容
使用系统表关联查询finance模式下所有表的模式名、表名和表的行数，并对表的行数进行排序，输出列的名称为：`schema_name`、`table_name`和`table_rows`

### 答案
```sql
SELECT
    schemaname AS schema_name,
    relname AS table_name,
    n_live_tup AS table_rows
FROM
    pg_stat_user_tables
WHERE
    schemaname = 'finance'
ORDER BY
    table_rows ASC;
```

或者单行格式：
```sql
SELECT schemaname AS schema_name, relname AS table_name, n_live_tup AS table_rows FROM pg_stat_user_tables WHERE schemaname = 'finance' ORDER BY table_rows ASC;
```
> 💡 **提示**：换行可替换为空格，两种格式都可以使用

### 预期输出(以题目为准)
```
 schema_name | table_name  | table_rows 
-------------+-------------+------------
 finance     | card_asset  |          3
 finance     | bank_card   |          5
 finance     | client      |          15
 finance     | account     |          25
(4 rows)
```


## 题目4：查询客户卡余额功能

### 任务内容
基于原有的系统，进行新模块"存/取款"的开发。在主菜单页面已添加"11)查询客户卡余额"选项，功能为输入客户ID为1，对客户的所有卡的余额进行查询。

### 操作步骤

#### 1. 修改CardAsset.java文件
```bash
cd /root/db-dev-cert/src/expt/db/finance/dao
vim CardAsset.java
```
> 💡 **注意**：使用vim命令打开文件，注意添加.java后缀

#### 2. 编辑queryCardAssetList方法
按`i`进入输入模式
> ⚠️ **vim操作提醒**：
> - 按`i`进入编辑模式
> - 使用键盘上下左右键移动光标

找到`queryCardAssetList`函数，应当如下：
```java
public ResultSet queryCardAssetList(int clientId) {
    PreparedStatement pstat = null;
    ResultSet rs = null;
    try {
        pstat = conn.prepareStatement("");  // 注意这一行
        pstat.setInt(1, clientId);
        rs = pstat.executeQuery();
    } catch (SQLException ex) {
        System.err.println("SQLException information");
        while (ex != null) {
            System.err.println("Error msg: " + ex.getMessage());
            ex = ex.getNextException();
        }
    }
    return rs;
}
```

在函数`pstat = conn.prepareStatement("");`这一行，在引号中编辑SQL语句：
```java
pstat = conn.prepareStatement("SELECT bc.b_number AS bank_card_number, bc.b_type AS bank_card_type, ca.card_money AS account_balance, ca.moneytype AS money_type FROM finance.bank_card bc JOIN finance.card_asset ca ON bc.b_number = ca.card_num WHERE bc.b_client_id = ?");
```

#### 3. 保存并编译运行
按`esc`键，输入`:wq`保存更改内容
> ⚠️ **vim保存提醒**：
> - 按`esc`退出编辑模式
> - 输入`:wq`保存并退出
> - 输入`:q!`不保存强制退出

```bash
cd /root/db-dev-cert/src/expt/db/finance/
javac -classpath ../../../ -d . *.java
java -p /root/db-dev-cert/libs/opengauss-jdbc-2.0.0.jar expt.db.finance.launch
```

### 预期输出
```
银行管理系统
1)客户信息查询
2)客户信息添加
3)客户信息修改
4)客户信息删除
5)客户开卡
6)客户销卡
7)账户信息查询
8)账户信息添加
9)账户信息修改
10)账户信息删除
11)查询客户卡余额
12)存取款业务
0)退出系统
=> 请输入需要进行的操作编号:11
==> 请输入需要查询信息的客户的ID:1
+
拥有的所有卡
bank_card_number  bank_card_type  account_balance  money_type
6222021302020000001   信用卡         1100.00         人民币
=> 请输入需要进行的操作编号:0
```
> ⚠️ **重要提醒**：完成题目4测试后，输入0退出Java程序，然后可以继续下一题！

## 题目5：客户开卡功能

### 任务内容
在主菜单页面输入"5"进入"客户开卡"模块，为客户ID为"1"的客户完成开卡，卡号为`6222021302020000111`。需要在完成开卡后，自动添加一条新开卡的存/取款信息，默认金额为"0"，默认币种为"人民币"。

### 操作步骤

#### 1. 修改insertCardAsset方法
```bash
cd /root/db-dev-cert/src/expt/db/finance/dao
vim CardAsset.java
```

按`i`进入输入模式，使用键盘上下左右键移动光标找到`insertCardAsset`函数，修改对应行：
```java
pstat = conn.prepareStatement("INSERT INTO finance.card_asset (card_num, card_money, moneytype) VALUES (?, ?, ?)");
```

按`esc`键，输入`:wq`保存更改

#### 2. 编译运行程序
```bash
cd /root/db-dev-cert/src/expt/db/finance/
javac -classpath ../../../ -d . *.java
java -p /root/db-dev-cert/libs/opengauss-jdbc-2.0.0.jar expt.db.finance.launch
```

### 预期输出（开卡操作）
```
=> 请输入需要进行的操作编号:5
==> 请输入需要开卡的客户的ID:1
==> 请输入银行卡号:6222021302020000111
开卡成功！
=> 请输入需要进行的操作编号:0
```
> ⚠️ **重要提醒**：完成开卡操作后，输入0退出Java程序，然后进行验证！

#### 3. 验证操作
> ⚠️ **重要提醒**：验证前需要先退出Java程序，然后重新连接GaussDB！

这里需要验证是否成功，登录GaussDB：
```bash
gsql -d finance -U db_dev -W 'Huawei123!@' -p 8000 -h ip_gauss
```

查询验证：
```sql
SELECT * FROM finance.card_asset WHERE card_num = '6222021302020000111';
\q
```
> 📝 **验证标准**：增加新数据为成功
> 💡 **提示**：查询完成后使用`\q`退出GaussDB

### 预期输出（验证结果）
```sql
finance=> SELECT * FROM finance.card_asset WHERE card_num = '6222021302020000111';
      card_num       | card_money | moneytype 
---------------------+------------+-----------
 6222021302020000111 |       0.00 | 人民币
(1 row)
finance=> \q
```

## 题目6：客户销卡功能

### 任务内容
在主菜单页面输入"6"进入"客户销卡"模块，完成销卡，卡号为`6222021302020000111`。需要在完成销卡后，自动删除此卡的存/取款信息。

### 操作步骤

#### 1. 修改deleteCardAsset方法
```bash
cd /root/db-dev-cert/src/expt/db/finance/dao
vim CardAsset.java
```

按`i`进入输入模式，使用键盘上下左右键移动光标找到`deleteCardAsset`函数，修改对应行：
```java
pstat = conn.prepareStatement("DELETE FROM finance.card_asset WHERE card_num = ?");
```

按`esc`键，输入`:wq`保存更改

#### 2. 编译运行程序
```bash
cd /root/db-dev-cert/src/expt/db/finance/
javac -classpath ../../../ -d . *.java
java -p /root/db-dev-cert/libs/opengauss-jdbc-2.0.0.jar expt.db.finance.launch
```

### 预期输出（销卡操作）
```
=> 请输入需要进行的操作编号:6
==> 请输入需要查询信息的客户的ID:1
==> 请输入需要销卡的银行卡号:6222021302020000111
销卡成功！
=> 请输入需要进行的操作编号:0
```
> ⚠️ **重要提醒**：完成销卡操作后，输入0退出Java程序，然后进行验证！

#### 3. 验证操作
> ⚠️ **重要提醒**：验证前需要先退出Java程序，然后重新连接GaussDB！

这里需要验证是否成功，登录GaussDB：
```bash
gsql -d finance -U db_dev -W 'Huawei123!@' -p 8000 -h ip_gauss
```

查询验证（应该没有数据返回）：
```sql
SELECT * FROM finance.card_asset WHERE card_num = '6222021302020000111';
\q
```
> 📝 **验证标准**：刚才数据被删除即为成功
> 💡 **提示**：查询完成后使用`\q`退出GaussDB

### 预期输出（验证结果）
```sql
finance=> SELECT * FROM finance.card_asset WHERE card_num = '6222021302020000111';
 card_num | card_money | moneytype 
----------+------------+-----------
(0 rows)
finance=> \q
```

## 题目7：存取款业务功能

### 任务内容
使用"11)查询客户卡余额"和"12)存取款业务"选项，通过卡号进行存/取款操作，使账户`6222021302020000001`的余额最终为`1000.00`人民币。

### 操作步骤

#### 1. 修改doDepositAndWithdrawal方法
```bash
cd /root/db-dev-cert/src/expt/db/finance/dao
vim CardAsset.java
```

按`i`进入输入模式，使用键盘上下左右键移动光标找到`doDepositAndWithdrawal`函数，修改对应行：
```java
pstat = conn.prepareStatement("UPDATE finance.card_asset SET card_money = card_money + ? WHERE card_num = ?");
```

按`esc`键，输入`:wq`保存更改

#### 2. 编译运行程序
```bash
cd /root/db-dev-cert/src/expt/db/finance/
javac -classpath ../../../ -d . *.java
java -p /root/db-dev-cert/libs/opengauss-jdbc-2.0.0.jar expt.db.finance.launch
```

#### 3. 操作流程
1. 选择"11"查询客户卡余额，输入客户ID为1
2. 选择"12"进行存取款业务
3. 按2取款100（根据当前余额计算需要存取的金额，使最终余额为1000.00）
4. 再次选择"11"验证余额是否为1000.00

> 💡 **操作提示**：
> - 具体对应业务流程代码在`/root/db-dev-cert/src/expt/db/finance/Service.java`文件下的`doDepositAndWithdrawal`方法中
> - 得分点：通过程序操作使账户6222021302020000001的余额最终为1000.00人民币

### 预期输出（完整操作流程）
```
=> 请输入需要进行的操作编号:11
==> 请输入需要查询信息的客户的ID:1
+
拥有的所有卡
bank_card_number  bank_card_type  account_balance  money_type
6222021302020000001   信用卡         1100.00         人民币
=> 请输入需要进行的操作编号:12
==> 请输入银行卡号:6222021302020000001
==> 请选择业务类型(1-存款 2-取款):2
==> 请输入金额:100
取款成功！
=> 请输入需要进行的操作编号:11
==> 请输入需要查询信息的客户的ID:1
+
拥有的所有卡
bank_card_number  bank_card_type  account_balance  money_type
6222021302020000001   信用卡         1000.00         人民币
=> 请输入需要进行的操作编号:0
```

## 重要注意事项

### 🔧 技术操作注意事项
1. **IP地址和密码替换**：所有`ip_gauss`、`ip_ecs`、`password`需要根据实际环境进行替换
2. **文件路径**：如果文件路径与题目不符，以题目为准
3. **编译和运行命令**：如果命令与题目不符，以题目为准
4. **SQL语句格式**：可以换行书写，也可以写成单行格式

### 📝 vim编辑器操作提醒
- **进入编辑模式**：按`i`键
- **退出编辑模式**：按`esc`键
- **保存并退出**：`:wq`
- **不保存退出**：`:q!`
- **查找内容**：`/关键词`然后按回车

### ⚠️ 新手常见错误
1. **密码输入**：输入密码时屏幕不显示字符是正常的
2. **SQL分号**：每条SQL语句后面要加分号`;`
3. **命令完整性**：长命令不要换行，要保持在一行
4. **大小写敏感**：Linux命令和SQL关键词要注意大小写
5. **文件保存**：修改代码后一定要保存文件
6. **验证步骤**：每个题目完成后都要按要求验证结果

### 🎯 考试技巧
1. **按顺序操作**：严格按照题目顺序执行，不要跳步骤
2. **仔细检查**：每次修改代码后都要仔细检查语法
3. **及时验证**：完成每个功能后立即验证结果
4. **保持冷静**：遇到错误不要慌张，检查命令和代码是否正确

### 🚪 退出操作提醒
- **退出GaussDB**：使用`\q`命令（不需要分号）
- **退出Java程序**：在主菜单输入`0`
- **退出vim编辑器**：按`esc`后输入`:wq`保存退出，或`:q!`不保存退出