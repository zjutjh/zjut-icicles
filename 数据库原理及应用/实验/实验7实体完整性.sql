-- 用DAS输入如下命令，运行并观察和记录结果。
-- （1）	实体完整性约束定义及在符合约束条件下的插入与更新操作。
-- ①列级实体完整性约束
CREATE TABLE Liyh_Stu_Union1
(lyh_Sno CHAR(8) NOT NULL UNIQUE,
			lyh_Sname CHAR(8),
			lyh_Ssex CHAR(3),
			lyh_Sage INT,
			lyh_Sdept CHAR(20)
			);
DROP TABLE Liyh_Stu_Union1;

CREATE TABLE Liyh_Stu_Union2
(lyh_Sno CHAR(8) CONSTRAINT PK_Stu_Union2 PRIMARY KEY,
			lyh_Sname CHAR(8),
			lyh_Ssex CHAR(3),
			lyh_Sage INT,
			lyh_Sdept CHAR(20)
			);
DROP TABLE Liyh_Stu_Union2;

--   当Sname不唯一时，也可以作为候选键，在Sno和Sname选择一个作为主键。
CREATE TABLE Liyh_Stu_Union3
(lyh_Sno CHAR(8) CONSTRAINT PK_Stu_Union3 PRIMARY KEY,
			lyh_Sname CHAR(8) UNIQUE,
			lyh_Ssex CHAR(3),
			lyh_Sage INT,
			lyh_Sdept CHAR(20)
			);
DROP TABLE Liyh_Stu_Union3;

-- ②表级实体完整性约束
-- 建表后定义完整性约束
CREATE TABLE Liyh_Stu_Union3(lyh_Sno CHAR(8) NOT NULL UNIQUE,
			lyh_Sname CHAR(8),
			lyh_Ssex CHAR(3),
			lyh_Sage INT,
			lyh_Sdept CHAR(20));
ALTER TABLE Liyh_Stu_Union3 ADD CONSTRAINT PK_Stu_Union3 PRIMARY KEY(lyh_Sno);
DROP TABLE Liyh_Stu_Union3;

-- 建表时创建
CREATE TABLE Liyh_Stu_Union(lyh_Sno CHAR(8) NOT NULL UNIQUE,
			lyh_Sname CHAR(8),
			lyh_Ssex CHAR(3),
			lyh_Sage INT,
			lyh_Sdept CHAR(20),
			CONSTRAINT PK_Stu_Union PRIMARY KEY(lyh_Sno));

-- 当主键为多个属性组成时
CREATE TABLE Liyh_Report1		
	(lyh_Sno VARCHAR(6) ,
	 lyh_Tno VARCHAR(6) ,
	 lyh_Cno VARCHAR(6) ,		
	 lyh_Score DECIMAL( 5,1 ),
	 CONSTRAINT PK_Rept1 PRIMARY KEY(lyh_Sno, lyh_Tno, lyh_Cno)
	 );
DROP TABLE Liyh_Report1;

CREATE TABLE Liyh_Report2		
	(lyh_Sno VARCHAR(6) ,
	 lyh_Tno VARCHAR(6) ,
	 lyh_Cno VARCHAR(6) ,		
	 lyh_Score DECIMAL( 5,1 )
	 ); 
ALTER TABLE Liyh_Report2 ADD CONSTRAINT PK_Rept2 PRIMARY KEY(lyh_Sno, lyh_Tno, lyh_Cno);
ALTER TABLE Liyh_Report2 DROP CONSTRAINT PK_Rept2;/*删除实体完整性约束*/
DROP TABLE Liyh_Report2;

-- ③符合约束条件的插入与更新
INSERT INTO Liyh_Stu_Union VALUES('S07','王兵','M',23,'CS');
UPDATE Liyh_Stu_Union SET lyh_Sno='  ' WHERE lyh_Sdept='CS';
UPDATE Liyh_Stu_Union SET lyh_Sno='S02' WHERE lyh_Sname='王兵';
SELECT * FROM Liyh_Stu_Union;

-- （2）	违反实体完整性的插入操作。
INSERT INTO Liyh_Stu_Union VALUES('S02','黄山','M',23,'CS');

-- （3）	违反实体完整性的更新操作。
UPDATE Liyh_Stu_Union SET lyh_Sno=NULL WHERE lyh_Sno='S02';

-- （4）事务的处理，包括事务的建立、处理以及出错时的事务回滚。
-- ①	正常事务的处理。
START TRANSACTION; 
SET LOCAL TRANSACTION ISOLATION LEVEL READ COMMITTED READ WRITE;
INSERT INTO Liyh_Stu_Union VALUES('S09','李永','M',25,'EE');
INSERT INTO Liyh_Stu_Union VALUES('S03','黄浩','F',25,'EE');
INSERT INTO Liyh_Stu_Union VALUES('S05','汪浩','F',26,'EE');
SELECT * FROM Liyh_Stu_Union;
COMMIT;

-- ②	事务的处理中出错时的事务回滚。
START TRANSACTION; 
SET LOCAL TRANSACTION ISOLATION LEVEL READ COMMITTED READ WRITE;
INSERT INTO Liyh_Stu_Union VALUES('S07','李宁','M',25,'EE');
SELECT * FROM Liyh_Stu_Union;
INSERT INTO Liyh_Stu_Union VALUES('S09','王琦','F',22,'CS');
SELECT * FROM Liyh_Stu_Union;
COMMIT;
ROLLBACK;
-- ③	检验事务回滚。
SELECT * FROM Liyh_Stu_Union;

-- ④删除表。
DROP TABLE Liyh_Stu_Union;

-- （5）通过建立Scholarship表，插入数据，观察当与现有的数据环境不符时，建立实体完整性和参照完整性的情况。
-- ①	建立Scholarship表，插入数据并显示结果。
CREATE TABLE Liyh_Scholarship 
(lyh_M_ID VARCHAR(10),
			lyh_Stu_id CHAR(8),
			lyh_R_Money INT);

INSERT INTO Liyh_Scholarship VALUES('M01','S07',5000);
INSERT INTO Liyh_Scholarship VALUES('M01','S08',8000);
SELECT * FROM Liyh_Scholarship;

-- ②	修改表的实体完整性，观察执行情况。
ALTER TABLE Liyh_Scholarship ADD CONSTRAINT PK_Scholarship PRIMARY KEY(lyh_M_ID);

-- ③	修改表的参照完整性，观察执行情况。
ALTER TABLE Liyh_Scholarship ADD
CONSTRAINT FK_Scholarship FOREIGN KEY(lyh_Stu_id) REFERENCES Liyh_Studentsbk(lyh_Sno);

-- ④删除表。
DROP TABLE Liyh_Scholarship;
