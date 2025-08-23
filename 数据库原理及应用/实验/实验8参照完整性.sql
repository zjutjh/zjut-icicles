-- (1)	建立表Course，参考实验7建立表Stu_Union，并分别在表Liyh_Course和Liyh_Stu_Union插入数据。
-- ①建立表Liyh_Course并插入数据
CREATE TABLE Liyh_Course(
	lyh_Cno CHAR(4) NOT NULL UNIQUE,
	lyh_Cname VARCHAR(50) NOT NULL,
	lyh_Cpoints INT,
	CONSTRAINT PK_Course PRIMARY KEY(lyh_Cno));

INSERT INTO Liyh_Course Values('C01','ComputerNetworks',2);	
INSERT INTO Liyh_Course Values('C02','ArtificialIntelligence',3);
INSERT INTO Liyh_Course Values('C03','Database',3);
INSERT INTO Liyh_Course Values('C04','OS',3);

-- ②参考实验7建立表Liyh_Stu_Union，在表Liyh_Stu_Union中插入数据
CREATE TABLE Liyh_Stu_Union(lyh_Sno CHAR(8) NOT NULL UNIQUE,
lyh_Sname CHAR(8),
lyh_Ssex CHAR(3),
lyh_Sage INT,
lyh_Sdept CHAR(20),
CONSTRAINT PK_Stu_Union PRIMARY KEY(lyh_Sno));

INSERT INTO Liyh_Stu_Union VALUES('S09','李永','M',25,'EE');
INSERT INTO Liyh_Stu_Union VALUES('S03','黄浩','F',25,'EE');
INSERT INTO Liyh_Stu_Union VALUES('S05','汪浩','F',26,'EE');
INSERT INTO Liyh_Stu_Union VALUES('S02','蒋欣','F',26,'EE');
INSERT INTO Liyh_Stu_Union Values('S01','李用','M',24,'FF');
INSERT INTO Liyh_Stu_Union VALUES('S07','李宁','F',26,'CS');
SELECT * FROM Liyh_Stu_Union;
-- (2)	建立表SC，令lyh_Sno和lyh_Cno分别为参照Liyh_Stu_Union表以及Liyh_Course表，作为外键，设定为级联删除，并令(lyh_Sno,lyh_Cno)为其主键。在不违反参照完整性的前提下，插入数据。
CREATE Table Liyh_SC(
lyh_Sno CHAR(8),
lyh_Cno CHAR(4),
lyh_Scredit INT,
CONSTRAINT PK_SC PRIMARY KEY(lyh_Sno,lyh_Cno),
CONSTRAINT FK_SC_Sno FOREIGN KEY(lyh_Sno) REFERENCES Liyh_Stu_Union (lyh_Sno) ON DELETE CASCADE,
CONSTRAINT FK_SC_Cno FOREIGN KEY(lyh_Cno) REFERENCES Liyh_Course (lyh_Cno) ON DELETE CASCADE
);

INSERT INTO Liyh_SC VALUES('S02','C01',2);
INSERT INTO Liyh_SC VALUES ('S02','C02',2);
INSERT INTO Liyh_SC VALUES ('S01','C01',2);
INSERT INTO Liyh_SC VALUES ('S01','C02',2);
INSERT INTO Liyh_SC VALUES ('S03','C03',2);
INSERT INTO Liyh_SC VALUES ('S03','C04',3);
SELECT * FROM Liyh_SC;

-- (3)	违反参照完整性的插入数据
INSERT INTO Liyh_SC VALUES('S99','C99',2);
-- (4)	在Liyh_Stu_Union中删除数据，演示并分析级联删除。
DELETE FROM Liyh_Stu_Union WHERE lyh_Sno='S01';
SELECT * FROM Liyh_SC;
-- (5)	在Liyh_Course中删除数据，演示并分析级联删除。
DELETE FROM Liyh_Course WHERE lyh_Cno='C02';
SELECT * FROM Liyh_SC;
-- (6)	为了演示多重级联删除，建立Liyh_Stu_Card表，令lyh_Card_id为其主键，令lyh_Sno为外键，参考Liyh_Stu_Union表，并插入数据。
CREATE TABLE Liyh_Stu_Card(
lyh_Card_id CHAR(14),
lyh_Sno CHAR(8),
lyh_Remained_money DECIMAL(10,2),
Constraint PK_Stu_Card PRIMARY KEY(lyh_Card_id),
Constraint FK_Stu_Card_Sno FOREIGN KEY(lyh_Sno) REFERENCES Liyh_Stu_Union(lyh_Sno) ON DELETE CASCADE);

INSERT INTO Liyh_Stu_Card VALUES('05212567','S03',400.25);
INSERT INTO Liyh_Stu_Card VALUES('05212222','S09',600.50);

SELECT * FROM Liyh_Stu_Card;
-- (7)	为了演示多重级联删除，建立Liyh_ICBC_Card表，令lyh_Bank_id为其主键，令lyh_Stu_card_id为外键，参考Liyh_Stu_Card表，并插入数据。
CREATE TABLE Liyh_ICBC_Card(
	lyh_Bank_id CHAR(20),
	lyh_Stu_card_id CHAR(14),
	lyh_Restored_money DECIMAL(10,2),
	constraint PK_ICBC_Card PRIMARY KEY(lyh_Bank_id),
    constraint FK_ICBC_Card_Stu_id FOREIGN KEY(lyh_Stu_card_id) REFERENCES Liyh_Stu_Card(lyh_Card_id) ON DELETE CASCADE
);

INSERT INTO Liyh_ICBC_Card VALUES('9558844022312','05212567',15000.1);
INSERT INTO Liyh_ICBC_Card VALUES('9558844023645','05212222',50000.3);

SELECT * FROM Liyh_ICBC_Card;
-- (8)	通过删除Liyh_Stu_Union表中的一条记录，演示4个表的多重级联删除。
DELETE FROM Liyh_Stu_Union WHERE lyh_Sno='S03';
SELECT * FROM Liyh_Stu_Card;
SELECT * FROM Liyh_ICBC_Card;
SELECT * FROM Liyh_Stu_Union;
SELECT * FROM Liyh_SC;
-- (9)	演示事务中进行多重级联删除失败的处理。
-- ①为演示多种级联删除情况，首先输入数据
INSERT INTO Liyh_Stu_Union VALUES('S03','黄浩','F',25,'EE');
INSERT INTO Liyh_SC VALUES ('S03','C03',2);
INSERT INTO Liyh_SC VALUES ('S03','C04',3);
INSERT INTO Liyh_Stu_Card VALUES('05212567','S03',400.25);
INSERT INTO Liyh_ICBC_Card VALUES('9558844022312','05212567',15000.1);
SELECT * FROM Liyh_Stu_Card;
SELECT * FROM Liyh_ICBC_Card;
SELECT * FROM Liyh_Stu_Union;
SELECT * FROM Liyh_SC;

-- ②修改Liyh_ICBC_Card表的外键属性，使其变为On delete No action。
ALTER TABLE Liyh_ICBC_Card
DROP CONSTRAINT FK_ICBC_Card_Stu_id;
ALTER TABLE Liyh_ICBC_Card
ADD CONSTRAINT FK_ICBC_Card_Stu_id FOREIGN KEY (lyh_Stu_card_id)
REFERENCES Liyh_Stu_Card(lyh_Card_id) ON DELETE NO ACTION;
-- ③演示事务中通过删除Liyh_Stu_Union表中的一条记录，多重级联删除失败，整个事务回滚到事务的初始状态
DELETE FROM Liyh_Stu_Union WHERE lyh_Sno='S03';
SELECT * FROM Liyh_Stu_Card;
SELECT * FROM Liyh_ICBC_Card;
SELECT * FROM Liyh_Stu_Union;
SELECT * FROM Liyh_SC;

-- 输入如下SQL语句：
START  TRANSACTION;
DELETE FROM Liyh_Stu_Card WHERE lyh_Card_id='05212222';
SELECT * FROM Liyh_Stu_Card;
SELECT * FROM Liyh_ICBC_Card;
Commit;


-- 输入如下SQL语句：
SELECT * FROM Liyh_Stu_Card;
SELECT * FROM Liyh_ICBC_Card;

-- (10)	演示互参考问题及其解决方法。要建立教师授课和课程指定教师听课关系的两张表，规定一个教师可以授多门课，但是每个课程只能指定一个教师去听课，所以要为两张表建立相互之间的参照关系。
-- ①在新建SQL终端窗口中输入如下SQL语句：
CREATE TABLE Liyh_Listen_course(
lyh_Tno CHAR(6),
lyh_Tname VARCHAR(20),
lyh_Cno CHAR(4),
CONSTRAINT PK_listen_course PRIMARY KEY(lyh_Tno),
CONSTRAINT FK_listen_course FOREIGN KEY(lyh_Cno)
REFERENCES Liyh_Teach_course(lyh_Cno)
);
CREATE TABLE Liyh_Teach_course(
lyh_Cno CHAR(4),
lyh_Cname VARCHAR(30),
lyh_Tno CHAR(6),
CONSTRAINT PK_Teach_course PRIMARY KEY(lyh_Cno),
CONSTRAINT FK_Teach_course FOREIGN KEY(lyh_Tno)
REFERENCES Liyh_Listen_course(lyh_Tno)
);
-- ②在新建SQL终端窗口中输入如下SQL语句：
CREATE TABLE Liyh_Listen_course(
lyh_Tno CHAR(6),
lyh_Tname VARCHAR(20),
lyh_Cno CHAR(4),
CONSTRAINT PK_listen_course PRIMARY KEY(lyh_Tno)
);
-- ③ 在新建SQL终端窗口中输入如下SQL语句：
CREATE TABLE Liyh_Teach_course(
lyh_Cno CHAR(4),
lyh_Cname VARCHAR(30),
lyh_Tno CHAR(6),
CONSTRAINT PK_Teach_course PRIMARY KEY(lyh_Cno),
CONSTRAINT FK_Teach_course FOREIGN KEY(lyh_Tno)
REFERENCES Liyh_Listen_course(lyh_Tno)
);

ALTER TABLE Liyh_Listen_course 
ADD CONSTRAINT FK_listen_course FOREIGN KEY(lyh_Cno)
REFERENCES Liyh_Teach_course(lyh_Cno);

