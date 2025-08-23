（1）
--若数据库Db_Uni存在的话将删除
DROP DATABASE IF EXISTS Db_Uni;
--采用字符集UTF-8创建名为Db_Uni 的数据库，并设置zjutuser为其属主
CREATE DATABASE Db_Uni ENCODING = 'UTF-8'  OWNER zjutuser;

（2）
--查看当前模式搜索路径
SHOW SEARCH_PATH;
--若数据库Db_Uni中存在模式zjutuser的话将删除
DROP SCHEMA IF EXISTS zjutuser;
--在数据库Db_Uni中创建名为zjutuser的模式，并授权给用户zjutuser所有。
CREATE SCHEMA zjutuser AUTHORIZATION zjutuser;
-- 设置当前会话的搜索路径为zjutuser 模式、Puhlic 模式，随后创建的基本表就会自动创建zjutuser 模式下。
SET SEARCH_PATH TO zjutuser , Public;
--再次查看当前模式搜索路径
SHOW SEARCH_PATH;

（3）
--在Db_Uni 数据库的zjutuser模式中创建4 个基本表。
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

/*或：
DROP TABLE IF EXISTS Reports;
DROP TABLE IF EXISTS Students;
CREATE TABLE Students			/*列级完整性约束条件*/
	(Sno VARCHAR(6) PRIMARY KEY,		/*Sno为主键*/
	 Sname VARCHAR(20) NOT NULL,		/*Sname不能为空值*/
	 Semail VARCHAR(50),
	 Scredit DECIMAL( 5,1 ),
	 Sroom VARCHAR(5),
         CONSTRAINT CK_Student_Scredit CHECK(Scredit>=0)
       );
DROP TABLE IF EXISTS Teachers;
CREATE TABLE Teachers			/*列级完整性约束条件*/
	(Tno VARCHAR(6) PRIMARY KEY,		/*Tno为主键*/
	 Tname VARCHAR(20) NOT NULL,		/*Tname不能为空值*/
	 Temail VARCHAR(50),
	 Tsalary DECIMAL( 5,1 )
       );

DROP TABLE IF EXISTS Courses;
CREATE TABLE Courses			/*列级完整性约束条件*/
	(Cno VARCHAR(6) PRIMARY KEY,		/*Cno为主键*/
	 Cname VARCHAR(20) NOT NULL,		/*Cname不能为空值*/
	 Ccredit DECIMAL( 5,1 )
  ); 

DROP TABLE IF EXISTS Reports;
CREATE TABLE Reports			/*列级完整性约束条件*/
	(Sno VARCHAR(6) NOT NULL,
	 Tno VARCHAR(6) NOT NULL,
	 Cno VARCHAR(6) NOT NULL,		/*Sno,Tno,Cno为主键*/
	 Score DECIMAL( 5,1 ),
	 PRIMARY KEY(Sno, Tno, Cno),
	 CONSTRAINT FK_Stu_Rep FOREIGN KEY(Sno) REFERENCES Students,
	 CONSTRAINT FK_Tea_Rep FOREIGN KEY(Tno) REFERENCES Teachers,
         CONSTRAINT FK_Cou_Rep FOREIGN KEY(Cno) REFERENCES Courses
        );
*/
（4）
--更改表Students：增加属性Ssex(类型是VARCHAR，长度为3)
ALTER TABLE IF EXISTS  Liyh_Students  ADD COLUMN lyh_Ssex  VARCHAR(3);

--取消Scredit“大于等于0”约束
ALTER TABLE IF EXISTS Liyh_Students DROP CONSTRAINT  CK_Student_Scredit;

--修改表Students中的属性Sname的数据类型改成长度为30。
ALTER TABLE IF EXISTS  Liyh_Students  MODIFY  lyh_Sname  VARCHAR(30);

--修改表Teachers中的属性Tname的数据类型改成长度为30。
ALTER TABLE IF EXISTS  Liyh_Teachers  MODIFY  lyh_Tname  VARCHAR(30);

--修改表Courses中的属性Cname的数据类型改成长度为30。
ALTER TABLE IF EXISTS  Liyh_Courses   MODIFY  lyh_Cname  VARCHAR(30);

(5)	
--删除表Liyh_Students的一个属性lyh_Sroom。
ALTER TABLE IF EXISTS  Liyh_Students  DROP COLUMN  lyh_Sroom;

(6)	
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


(7)	
--为Liyh_Courses表创建按lyh_Cno降序排列的索引。
CREATE INDEX IF NOT EXISTS idx_Cno_desc ON Liyh_Courses(lyh_Cno DESC);

(8)	
--为Liyh_Students表创建按lyh_Sno升序排列的索引。
CREATE INDEX IF NOT EXISTS idx_Sno_asc ON Liyh_Students(lyh_Sno ASC);

(9)	
--创建表Liyh_Students的按lyh_Sname升序排列的唯一性索引。
CREATE UNIQUE INDEX IF NOT EXISTS idx_Sname_asc ON Liyh_Students(lyh_Sname ASC);

(10)	
--删除Liyh_Students表lyh_Sno的升序索引，删除Liyh_Courses表lyh_Cno的降序索引。
DROP INDEX IF EXISTS idx_Sno_asc;
DROP INDEX IF EXISTS idx_Cno_desc;




