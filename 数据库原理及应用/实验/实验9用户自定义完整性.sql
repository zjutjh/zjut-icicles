-- 用DAS运行并观察结果。
-- (1) 创建Teacher表，并自定义2个约束U1以及U2，其中U1规定Tname字段唯一，U2规定Tage (级别)字段的上限是28。
-- 仿照Teacher表，创建表Worker表，并自定义2个约束U3以及U4，其中U3规定Wname字段唯一，U4规定Wag (级别)字段的上限是28。
-- ①创建Teacher表
CREATE TABLE Liyh_Teacher
(
    lyh_Tno CHAR(5),
    lyh_Tname CHAR(8) CONSTRAINT U1 UNIQUE,
    lyh_Tsex CHAR(3),
    lyh_Tage INT CONSTRAINT U2 CHECK (lyh_Tage<=28),
    lyh_Tdept CHAR(20),
    CONSTRAINT PK_Teacher PRIMARY KEY(lyh_Tno)
);

-- ②仿照Teacher表，创建表Worker 
CREATE TABLE Liyh_Worker
(
    lyh_Wno CHAR(5),
    lyh_Wname CHAR(8) CONSTRAINT U3 UNIQUE,
    lyh_Wsex CHAR(3),
    lyh_Wage INT CONSTRAINT U4 CHECK (lyh_Wage<=28),
    lyh_Wdept CHAR(20),
    CONSTRAINT PK_Worker PRIMARY KEY(lyh_Wno)
);

-- (2) 在Worker, Teacher表中插入一条合法记录。
-- ①在Teacher表中插入一条合法记录
INSERT INTO Liyh_Teacher (lyh_Tno, lyh_Tname, lyh_Tsex, lyh_Tage, lyh_Tdept) 
VALUES ('T01', '李用', 'M', 14, '后勤部');
SELECT * FROM Liyh_Teacher;

-- ②仿照Teacher表，在Worker表中插入一条合法记录。
INSERT INTO Liyh_Worker (lyh_Wno, lyh_Wname, lyh_Wsex, lyh_Wage, lyh_Wdept) 
VALUES ('W01', '张三', 'M', 20, '保卫部');
SELECT * FROM Liyh_Worker;

-- (3) 演示插入违反U2、U4约束的例子，U2、U4分别规定元组的Wage,Tage属性的值必须<=28。
-- ①在Teacher表中插入一条违反U2约束的记录。
INSERT INTO Liyh_Teacher (lyh_Tno, lyh_Tname, lyh_Tsex, lyh_Tage, lyh_Tdept) 
VALUES ('T02', '王勇', 'M', 38, '后勤部');
SELECT * FROM Liyh_Teacher;

-- ②仿照Teacher表，在Worker表中插入一条违反U4约束的记录。
INSERT INTO Liyh_Worker (lyh_Wno, lyh_Wname, lyh_Wsex, lyh_Wage, lyh_Wdept) 
VALUES ('W02', '李四', 'M', 35, '保卫部');
SELECT * FROM Liyh_Worker;

-- (4) 去除U2和U4约束。输入如下SQL语句
-- ①在Teacher表中去除U2约束。
ALTER TABLE Liyh_Teacher DROP CONSTRAINT IF EXISTS U2;

-- ②仿照Teacher表，在Worker表中去除U4约束。
ALTER TABLE Liyh_Worker DROP CONSTRAINT IF EXISTS U4;

-- (5) 去除U2和U4约束，重新做（3） 
-- ①在Teacher表中插入记录（原本违反U2约束）
INSERT INTO Liyh_Teacher (lyh_Tno, lyh_Tname, lyh_Tsex, lyh_Tage, lyh_Tdept) 
VALUES ('T03', '刘勇', 'M', 38, '后勤部');
SELECT * FROM Liyh_Teacher;

-- ②在Worker表中插入记录（原本违反U4约束）
INSERT INTO Liyh_Worker (lyh_Wno, lyh_Wname, lyh_Wsex, lyh_Wage, lyh_Wdept) 
VALUES ('W03', '王五', 'M', 35, '保卫部');
SELECT * FROM Liyh_Worker; 
