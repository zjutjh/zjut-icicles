为简化数据库db_zjut的管理，老师的信息和学生的信息由校级教务教学部门来设置，课程信息和选课信息由院级教务教学部门来设置，
教师可以设置学生成绩，学生查看课程成绩。以系统管理员zjutuser的身份登陆数据库db_zjut，分别建立如下用户和角色。
用户包括校级教务教学主管UeduSL，院级教务教学部门主管CeduSJ、CeduSA，教师TeaY、TeaW和学生StuG、StuL。
角色包括UeduManagerRole、CeduManagerRole和TeacherRole、StudentRole。
验证权限分配之前，请备份好数据库，针对不同用户所具有的权限，
分别以系统管理员zjutuser身份或以上用户身份登陆到数据库db_zjut中（用gsql终端工具登陆更方便切换用户，具体命令可参考安装手册中的命令），
设计相应的SQL 语句，进行操作加以验证，并记录操作结果。 

-- 初始化数据库环境

（1）
--若数据库Db_Uni存在的话将删除
DROP DATABASE IF EXISTS db_zjut;
--采用字符集UTF-8创建名为db_zjut的数据库，并设置zjutuser为其属主
CREATE DATABASE db_zjut ENCODING = 'UTF-8'  OWNER zjutuser;

（2）
--查看当前模式搜索路径
SHOW SEARCH_PATH;
--若数据库db_zjut中存在模式zjutuser的话将删除
DROP SCHEMA IF EXISTS zjutuser;
--在数据库db_zjut中创建名为zjutuser的模式，并授权给用户zjutuser所有。
CREATE SCHEMA zjutuser AUTHORIZATION zjutuser;
-- 设置当前会话的搜索路径为zjutuser 模式、Puhlic 模式，随后创建的基本表就会自动创建zjutuser 模式下。
SET SEARCH_PATH TO zjutuser , Public;
--再次查看当前模式搜索路径
SHOW SEARCH_PATH;

（3）
--在db_zjut数据库的zjutuser模式中创建4个基本表。
DROP TABLE IF EXISTS Liyh_Reports;
DROP TABLE IF EXISTS Liyh_Students;
CREATE TABLE Liyh_Students			/*列级完整性约束条件*/
	(lyh_Sno VARCHAR(6),		/*lyh_Sno为主键*/
	 lyh_Sname VARCHAR(20) NOT NULL,	/*lyh_Sname不能为空值*/
	 lyh_Semail VARCHAR(50),
	 lyh_Scredit DECIMAL( 5,1 ),
	 lyh_Sroom VARCHAR(5),
   CONSTRAINT PK_Stu PRIMARY KEY(lyh_Sno),
   CONSTRAINT CK_Student_Scredit CHECK(lyh_Scredit>=0)
   );
DROP TABLE IF EXISTS Liyh_Teachers;
CREATE TABLE Liyh_Teachers			/*列级完整性约束条件*/
	(lyh_Tno VARCHAR(6),		/*lyh_Tno为主键*/
	 lyh_Tname VARCHAR(20) NOT NULL,	/*lyh_Tname不能为空值*/
	 lyh_Temail VARCHAR(50),
	 lyh_Tsalary DECIMAL( 5,1 ),
   CONSTRAINT PK_Tea PRIMARY KEY(lyh_Tno)
   );

DROP TABLE IF EXISTS Liyh_Courses;
CREATE TABLE Liyh_Courses			/*列级完整性约束条件*/
	(lyh_Cno VARCHAR(6),		/*lyh_Cno为主键*/
	 lyh_Cname VARCHAR(20) NOT NULL,		/*lyh_Cname不能为空值*/
	 lyh_Ccredit DECIMAL( 5,1 ),
   CONSTRAINT PK_Cou PRIMARY KEY(lyh_Cno)
  ); 
DROP TABLE IF EXISTS Liyh_Reports;
CREATE TABLE Liyh_Reports			/*列级完整性约束条件*/
	(lyh_Sno VARCHAR(6) ,
	 lyh_Tno VARCHAR(6) ,
	 lyh_Cno VARCHAR(6) ,		
	 lyh_Score DECIMAL( 5,1 ),
	 CONSTRAINT PK_Rep PRIMARY KEY(lyh_Sno, lyh_Tno, lyh_Cno),/*lyh_Sno,lyh_Tno,lyh_Cno为主键*/
	 CONSTRAINT FK_Stu_Rep FOREIGN KEY(lyh_Sno) REFERENCES Liyh_Students,
	 CONSTRAINT FK_Tea_Rep FOREIGN KEY(lyh_Tno) REFERENCES Liyh_Teachers,
   CONSTRAINT FK_Cou_Rep FOREIGN KEY(lyh_Cno) REFERENCES Liyh_Courses
   );

-- 首先以系统管理员zjutuser登录:
-- gsql -d db_zjut -U zjutuser -W <密码> -p <端口号>
--（4）
--更改表Students：增加属性Ssex(类型是VARCHAR，长度为3)
ALTER TABLE IF EXISTS  Liyh_Students  ADD COLUMN lyh_Ssex  VARCHAR(3);

--取消Scredit"大于等于0"约束
ALTER TABLE IF EXISTS Liyh_Students DROP CONSTRAINT  CK_Student_Scredit;

--修改表Students中的属性Sname的数据类型改成长度为30。
ALTER TABLE IF EXISTS  Liyh_Students  MODIFY  lyh_Sname  VARCHAR(30);

--修改表Teachers中的属性Tname的数据类型改成长度为30。
ALTER TABLE IF EXISTS  Liyh_Teachers  MODIFY  lyh_Tname  VARCHAR(30);

--修改表Courses中的属性Cname的数据类型改成长度为30。
ALTER TABLE IF EXISTS  Liyh_Courses   MODIFY  lyh_Cname  VARCHAR(30);

--(5)	
--删除表Liyh_Students的一个属性lyh_Sroom。
ALTER TABLE IF EXISTS  Liyh_Students  DROP COLUMN  lyh_Sroom;

--(6)	
--删除表Liyh_Reports，然后重建表Liyh_Reports。
DROP TABLE IF EXISTS Liyh_Reports;
CREATE TABLE Liyh_Reports			/*列级完整性约束条件*/
	(lyh_Sno VARCHAR(6) NOT NULL,
	 lyh_Tno VARCHAR(6) NOT NULL,
	 lyh_Cno VARCHAR(6) NOT NULL,		/*lyh_Sno,lyh_Tno,lyh_Cno为主键*/
	 lyh_Score DECIMAL( 5,1 ),
	 PRIMARY KEY(lyh_Sno, lyh_Tno, lyh_Cno),
	 CONSTRAINT FK_Stu_Rep FOREIGN KEY(lyh_Sno) REFERENCES Liyh_Students,
	 CONSTRAINT FK_Tea_Rep FOREIGN KEY(lyh_Tno) REFERENCES Liyh_Teachers,
         CONSTRAINT FK_Cou_Rep FOREIGN KEY(lyh_Cno) REFERENCES Liyh_Courses
        )
INSERT INTO Liyh_Students
VALUES ('S01','王建平','WJP@zjut.edu.cn', 23.1,'男'),
        ('S02','刘华','LH@zjut.edu.cn', 24.6,'女'),
        ('S03','范林军','FLJ@zjut.edu.cn', 16.6,'女'),
        ('S04','李伟','LW@zjut.edu.cn', 15.8,'男'),
        ('S26','黄河','HUanghe@zjut.edu.cn', 13.4,'男'),
        ('S52','长江','Changjiang@zjut.edu.cn', 12.4,'男');

--向Teachers表如下数据
INSERT INTO Liyh_Teachers 
VALUES ('T01','刘涛','LT@zjut.edu.cn', 4300),
        ('T02','吴碧艳','WBY@zjut.edu.cn', 2500),
        ('T03','张莹','ZY@zjut.edu.cn', 3000),
        ('T04','张宁雅','ZNY@zjut.edu.cn', 5500),
        ('T05','叶帅','YS@zjut.edu.cn', 3800),
        ('T06','杨光美','YGM@zjut.edu.cn', 3500),
        ('T07','程潜','CQ@zjut.edu.cn', 5000);

--向Courses表如下数据
INSERT INTO Liyh_Courses 
  VALUES ('C01','C++',4),
        ('C02','UML',4),
        ('C03','JAVA',3),
        ('C04','算法分析与设计',3),
        ('C05','数据库原理及应用',3),
        ('C06','数据结构与算法',4),
        ('C07','计算机组成原理',4),
        ('C08','英语',6),
        ('C09','数字生活',2),
        ('C10','音乐鉴赏',2),
        ('C11','体育1',2);
--向Reports表如下数据
INSERT INTO Liyh_Reports 
VALUES ('S01','T01', 'C01',83),
      ('S01','T03', 'C03',85),
      ('S02','T01', 'C01',75),
      ('S02','T02', 'C02',45),
      ('S02','T03', 'C03',NULL),
      ('S02','T04', 'C04',NULL),
      ('S02','T05', 'C05',70),
      ('S02','T04', 'C06',83),
      ('S02','T05', 'C07',90),
      ('S02','T01', 'C08',83),
      ('S02','T02', 'C09',77),
      ('S02','T07', 'C10',83),
      ('S02','T06', 'C11',88),
      ('S03','T01', 'C08',63),
      ('S03','T02', 'C02',93),
      ('S03','T01', 'C01',78),
      ('S04','T06', 'C06',89),
      ('S04','T05', 'C05',93),
      ('S26','T07', 'C10',45),
      ('S26','T04', 'C04',86),
      ('S52','T07', 'C10',91),
      ('S52','T06', 'C11',90),
      ('S52','T05', 'C05',NULL),
      ('S52','T01', 'C08',64),
      ('S52','T02', 'C09',81);
(1)	 创建用户
/* 注意: 
CREATE USER 语句不是SQL标准，因此不同的RDBMS 语法和内容相差甚远,这里用的是openGauss创建用户语句,同时查阅资料如何将系统权限授权给角色或用户，参考GRANT语法解释。
通过CREATE USER创建的用户，默认具有LOGIN权限；
通过CREATE USER创建用户的同时系统会在执行该命令的数据库中，为该用户创建一个同名的SCHEMA；其他数据库中，则不自动创建同名的SCHEMA；用户可使用CREATE SCHEMA命令，分别在其他数据库中，为该用户创建同名SCHEMA。
系统管理员在普通用户同名schema下创建的对象，所有者为schema的同名用户（非系统管理员）。
*/
--①创建校院两级教务教学主管用户，要求具有创建用户或角色的权利。
CREATE USER "UeduSL" WITH CREATEROLE PASSWORD 'Bigdata@123';
CREATE USER "CeduSJ" WITH CREATEROLE PASSWORD 'Bigdata@123';
CREATE USER "CeduSA" WITH CREATEROLE PASSWORD 'Bigdata@123';
--②创建教师和学生用户。
CREATE USER "TeaY"  IDENTIFIED BY  'Bigdata@123';
CREATE USER "TeaW"  IDENTIFIED BY  'Bigdata@123';
CREATE USER "StuG"  IDENTIFIED BY  'Bigdata@123';
CREATE USER "StuL"  IDENTIFIED BY  'Bigdata@123';
(2)	创建角色并分配权限
--①分别创建校院两级教务教学管理角色UeduManagerRole和CeduManagerRole、教师角色TeacherRole和学生角色StudentRole。
CREATE ROLE "UeduManagerRole" WITH CREATEROLE PASSWORD 'Bigdata@123';
CREATE ROLE "CeduManagerRole" WITH CREATEROLE PASSWORD 'Bigdata@123';
CREATE ROLE "TeacherRole" WITH  PASSWORD 'Bigdata@123';
CREATE ROLE "StudentRole" WITH  PASSWORD 'Bigdata@123'; 
--②把数据库db_zjut的模式zjutuser的使用权限授予所有角色与部分用户。
GRANT USAGE ON SCHEMA zjutuser to "UeduManagerRole" WITH GRANT OPTION;
GRANT USAGE ON SCHEMA zjutuser to "CeduManagerRole" WITH GRANT OPTION;
GRANT USAGE ON SCHEMA zjutuser to "TeacherRole" WITH GRANT OPTION;
GRANT USAGE ON SCHEMA zjutuser to "StudentRole" WITH GRANT OPTION;
GRANT USAGE ON SCHEMA zjutuser to "UeduSL","CeduSJ","CeduSA","TeaY","TeaW","StuG","StuL" WITH GRANT OPTION;
--③把Liyh_Teachers、Liyh_Students、Liyh_Courses和Liyh_Reports的相应权限分别授权给不同的角色
GRANT ALL ON TABLE Liyh_Teachers, Liyh_Students to "UeduManagerRole" WITH GRANT OPTION;
GRANT ALL ON TABLE Liyh_Courses, Liyh_Reports to "CeduManagerRole" WITH GRANT OPTION;
GRANT SELECT, UPDATE ON TABLE Liyh_Reports to "TeacherRole" WITH GRANT OPTION;
GRANT SELECT ON TABLE Liyh_Reports to "StudentRole" WITH GRANT OPTION;
(3)	为用户分配角色及权限
--①为用户按角色分配并授予权限
GRANT "UeduManagerRole" TO "UeduSL" WITH ADMIN OPTION;
GRANT "CeduManagerRole" TO "CeduSJ" WITH ADMIN OPTION;
GRANT "TeacherRole" TO "TeaY" WITH ADMIN OPTION;
GRANT "StudentRole" TO "StuG" WITH ADMIN OPTION;
--②按用户单独赋予权限
GRANT SELECT ON TABLE Liyh_Teachers, Liyh_Students, Liyh_Courses, Liyh_Reports to "CeduSA","TeaW", "StuL" WITH GRANT OPTION;

(4)	分别以UeduSL、CeduSJ、TeaY和StuG的身份登陆数据库db_zjut，验证按角色授予的权限，
用SQL语言查询zjutuser.Liyh_Teachers、zjutuser.Liyh_Students、zjutuser.Liyh_Courses和zjutuser.Liyh_Reports表，查询结果如何？并分析原因。
-- 答案分析:

-- 以UeduSL身份登录验证:
gsql -d db_zjut -U UeduSL -W 'Bigdata@123' -p 26000
-- UeduSL拥有UeduManagerRole角色，该角色对Liyh_Teachers和Liyh_Students表有ALL权限
SELECT * FROM zjutuser.Liyh_Teachers;  -- 成功(拥有权限)
SELECT * FROM zjutuser.Liyh_Students;  -- 成功(拥有权限)
SELECT * FROM zjutuser.Liyh_Courses;   -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Reports;   -- 失败(角色无此权限)

-- 以CeduSJ身份登录验证:
gsql -d db_zjut -U CeduSJ -W 'Bigdata@123' -p 26000
-- CeduSJ拥有CeduManagerRole角色，该角色对Liyh_Courses和Liyh_Reports表有ALL权限
SELECT * FROM zjutuser.Liyh_Teachers;  -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Students;  -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Courses;   -- 成功(拥有权限)
SELECT * FROM zjutuser.Liyh_Reports;   -- 成功(拥有权限)

-- 以TeaY身份登录验证:
gsql -d db_zjut -U TeaY -W 'Bigdata@123' -p 26000
-- TeaY拥有TeacherRole角色，该角色对Liyh_Reports表有SELECT,UPDATE权限
SELECT * FROM zjutuser.Liyh_Teachers;  -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Students;  -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Courses;   -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Reports;   -- 成功(拥有权限)

-- 以StuG身份登录验证:
gsql -d db_zjut -U StuG -W 'Bigdata@123' -p 26000
-- StuG拥有StudentRole角色，该角色对Liyh_Reports表仅有SELECT权限
SELECT * FROM zjutuser.Liyh_Teachers;  -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Students;  -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Courses;   -- 失败(角色无此权限)
SELECT * FROM zjutuser.Liyh_Reports;   -- 成功(拥有权限)

(5)	用系统管理员zjutuser授予用户"CeduSA"对表zjutuser.Liyh_Students插入和更新的权限，但不授予删除权限，
并且授予用户"CeduSA"传播这两个权限的权利。以"CeduSA"的身份登陆，用SQL语言插入和更新Liyh_Students表，结果如何？（注意更新操作的授权）

-- 以系统管理员zjutuser身份执行授权:
gsql -d db_zjut -U zjutuser -W 'Bigdata@123' -p 26000
GRANT INSERT, UPDATE ON TABLE zjutuser.Liyh_Students TO "CeduSA" WITH GRANT OPTION;

-- 以CeduSA身份登录验证:
gsql -d db_zjut -U CeduSA -W 'Bigdata@123' -p 26000
-- 执行插入操作:
INSERT INTO zjutuser.Liyh_Students (lyh_Sno, lyh_Sname, lyh_Semail, lyh_Scredit, lyh_Ssex) 
VALUES ('S_test', '测试生', 'test@example.com', 20.0, '男');
-- 结果: 成功。因为zjutuser已授予CeduSA对Liyh_Students表的INSERT权限。

-- 执行更新操作:
UPDATE zjutuser.Liyh_Students 
SET lyh_Scredit = 21.0 
WHERE lyh_Sno = 'S_test';
-- 结果: 成功。因为zjutuser已授予CeduSA对Liyh_Students表的UPDATE权限。
-- CeduSA同时获得了传播这两个权限的权利(WITH GRANT OPTION)。

(6)	用系统管理员zjutuser授予允许用户"TeaW"在表Liyh_Reports中插入元组，更新lyh_Score列，可以查询除了lyh_Sno以外的所有列。
以"TeaW"的身份登陆，用SQL语言插入更新并查询Liyh_Reports表，结果如何？（注意更新操作的授权）

-- 注意：原始步骤(3)ii已授予TeaW对Liyh_Reports的SELECT权限。
-- 此处的"查询除了lyh_Sno以外的所有列"需要通过先撤销全表SELECT权限，再单独授予各列权限实现。

-- 以系统管理员zjutuser身份执行授权:
gsql -d db_zjut -U zjutuser -W 'Bigdata@123' -p 26000

-- 首先撤销TeaW对Liyh_Reports表的所有SELECT权限
REVOKE SELECT ON TABLE zjutuser.Liyh_Reports FROM "TeaW";

-- 然后单独授予TeaW对Liyh_Reports表中除lyh_Sno外所有列的SELECT权限
GRANT SELECT (lyh_Tno, lyh_Cno, lyh_Score) ON TABLE zjutuser.Liyh_Reports TO "TeaW";

-- 授予TeaW对Liyh_Reports表的INSERT权限和对lyh_Score列的UPDATE权限
GRANT INSERT ON TABLE zjutuser.Liyh_Reports TO "TeaW";
GRANT UPDATE (lyh_Score) ON TABLE zjutuser.Liyh_Reports TO "TeaW";

-- 以TeaW身份登录验证:
gsql -d db_zjut -U TeaW -W 'Bigdata@123' -p 26000

-- 执行插入操作:
INSERT INTO zjutuser.Liyh_Reports (lyh_Sno, lyh_Tno, lyh_Cno, lyh_Score) 
VALUES ('S01', 'T02', 'C02', 90.0);
-- 结果: 成功。因为zjutuser已授予TeaW对Liyh_Reports表的INSERT权限。

-- 执行查询操作 (只能查询除lyh_Sno外的列):
SELECT lyh_Tno, lyh_Cno, lyh_Score FROM zjutuser.Liyh_Reports;
-- 结果: 成功。因为zjutuser已授予TeaW对Liyh_Reports表除lyh_Sno外所有列的SELECT权限。

-- 尝试查询包含lyh_Sno的列:
SELECT * FROM zjutuser.Liyh_Reports;
-- 结果: 失败。因为TeaW没有对lyh_Sno列的SELECT权限。

-- 执行更新操作:
UPDATE zjutuser.Liyh_Reports 
SET lyh_Score = lyh_Score + 0.5 
WHERE lyh_Tno = 'T01' AND lyh_Cno = 'C01';
-- 结果: 成功。因为zjutuser已授予TeaW对Liyh_Reports表lyh_Score列的UPDATE权限。
-- 注意：由于无法在WHERE子句中使用lyh_Sno，所以必须使用其他列作为条件。

(7)	用户"CeduSA"授予用户"TeaW"对表Liyh_Students插入和更新的权限，并且授予用户"TeaW"传播插入和更新操作的权利。
分别以"CeduSA"和"TeaW"的身份登陆，用SQL语言验证以上授权操作，结果如何？

-- 以CeduSA身份登录(CeduSA在问题5中获得了Liyh_Students的INSERT,UPDATE WITH GRANT OPTION权限):
gsql -d db_zjut -U CeduSA -W 'Bigdata@123' -p 26000
GRANT INSERT, UPDATE ON TABLE zjutuser.Liyh_Students TO "TeaW" WITH GRANT OPTION;
-- 结果: 成功。因为CeduSA拥有带GRANT OPTION的相应权限。

-- 以TeaW身份登录验证:
gsql -d db_zjut -U TeaW -W 'Bigdata@123' -p 26000
-- 验证插入权限:
INSERT INTO zjutuser.Liyh_Students (lyh_Sno, lyh_Sname, lyh_Semail, lyh_Scredit, lyh_Ssex) 
VALUES ('S_t2', '测试生W', 'teaw@example.com', 21.0, '女');
-- 结果: 成功。表明TeaW获得了INSERT权限。

-- 验证更新权限:
UPDATE zjutuser.Liyh_Students 
SET lyh_Scredit = 22.0 
WHERE lyh_Sno = 'S_t2';
-- 结果: 成功。表明TeaW获得了UPDATE权限。

-- 验证传播权限:
GRANT INSERT ON TABLE zjutuser.Liyh_Students TO "StuL";
-- 结果: 成功。表明TeaW获得了WITH GRANT OPTION权限。

(8)	收回用户"CeduSA"对表Liyh_Courses查询权限的授权。分别以"CeduSA"和"TeaW"的身份登陆，用SQL语言查询Liyh_Courses表，查询结果如何？

-- 以系统管理员zjutuser身份执行收回操作:
gsql -d db_zjut -U zjutuser -W 'Bigdata@123' -p 26000
REVOKE SELECT ON TABLE zjutuser.Liyh_Courses FROM "CeduSA";

-- 以CeduSA身份登录验证:
gsql -d db_zjut -U CeduSA -W 'Bigdata@123' -p 26000
SELECT * FROM zjutuser.Liyh_Courses;
-- 结果: 失败(权限不足)。
-- 原因: zjutuser收回了之前在步骤(3)ii中直接授予CeduSA的SELECT权限。
-- CeduSA没有通过其他角色(如CeduManagerRole)获得该权限。

-- 以TeaW身份登录验证:
gsql -d db_zjut -U TeaW -W 'Bigdata@123' -p 26000
SELECT * FROM zjutuser.Liyh_Courses;
-- 结果: 成功。
-- 原因: 对CeduSA的权限收回不影响TeaW的权限。
-- TeaW在步骤(3)ii中被zjutuser直接授予了对Liyh_Courses表的SELECT权限，该权限未被收回。

(9)	由上面（6）和（7）的授权，再由用户"TeaW"对用户"StuL"授予表Liyh_Students插入和更新的权限，并且授予用户"StuL"传播插入和更新操作的权力。
这时候，如果由"StuL"对"CeduSA"授予表Liyh_Students的插入和更新权限是否能得到成功？如果能够成功，那么如果由用户"TeaW"取消"StuL"的权限，
对"CeduSA"会有什么影响？如果再由zjutuser取消"CeduSA"的权限，对"TeaW"有什么影响？

-- TeaW授权给StuL:
-- 以TeaW身份登录(TeaW在问题7中从CeduSA获得了Liyh_Students的INSERT,UPDATE WITH GRANT OPTION权限):
gsql -d db_zjut -U TeaW -W 'Bigdata@123' -p 26000
GRANT INSERT, UPDATE ON TABLE zjutuser.Liyh_Students TO "StuL" WITH GRANT OPTION;
-- 结果: 成功。

-- StuL授权给CeduSA:
-- 以StuL身份登录:
gsql -d db_zjut -U StuL -W 'Bigdata@123' -p 26000
GRANT INSERT, UPDATE ON TABLE zjutuser.Liyh_Students TO "CeduSA";
-- 结果: 成功。StuL拥有带GRANT OPTION的权限，可以将其授予CeduSA。
-- 尽管CeduSA已有此权限(来自zjutuser)，授权本身可以成功。

-- TeaW取消StuL的权限:
-- 以TeaW身份登录:
gsql -d db_zjut -U TeaW -W 'Bigdata@123' -p 26000

REVOKE INSERT, UPDATE ON TABLE zjutuser.Liyh_Students FROM "StuL" CASCADE;

-- 结果: 成功。
-- 对CeduSA的影响: StuL授予CeduSA的权限会被级联撤销(因为StuL失去了授权基础)。
-- 但是，CeduSA仍然拥有zjutuser在步骤(5)中授予的INSERT,UPDATE权限。
-- 所以CeduSA的最终权限状态不受影响。

-- zjutuser取消CeduSA的权限:
-- 以zjutuser身份登录:
gsql -d db_zjut -U zjutuser -W 'Bigdata@123' -p 26000
REVOKE INSERT, UPDATE ON TABLE zjutuser.Liyh_Students FROM "CeduSA" CASCADE;
-- 结果: 成功。
-- 对TeaW的影响: TeaW的INSERT,UPDATE权限最初是由CeduSA授予的。
-- 当CeduSA被zjutuser撤销这些权限时，依赖于CeduSA授权的TeaW的权限也会被级联撤销。
-- 因此，TeaW将失去对Liyh_Students表的INSERT和UPDATE权限。
-- 进而，如果之前TeaW授权给了其他用户(如StuL)且未撤销，这些用户的权限也会被级联撤销。
