
(1)	使用SQL语句向Students表中插入元组(Sno：S78; Sname：李迪; Semail:LD@zjut.edu.cn; Scredit：0;Ssex：男)。
INSERT INTO Liyh_Students (lyh_Sno, lyh_Sname, lyh_Semail, lyh_Scredit, lyh_Ssex)
VALUES ('S78', '李迪', 'LD@zjut.edu.cn', 0, '男');

(2)	对每个课程，求学生的选课人数和学生的平均成绩，并把结果存入数据库。使用SELECT INTO 和INSERT INTO 两种方法实现。
-- 方法一：使用 SELECT INTO 创建并插入数据到新表
-- 首先创建一个新表来存储结果
DROP TABLE IF EXISTS Liyh_CourseStats;
SELECT r.lyh_Cno, c.lyh_Cname, 
       COUNT(r.lyh_Sno) AS StudentCount, 
       AVG(r.lyh_Score) AS AverageScore
INTO Liyh_CourseStats
FROM Liyh_Reports r
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
GROUP BY r.lyh_Cno, c.lyh_Cname;

-- 方法二：使用 INSERT INTO 插入数据到已存在的表
-- 首先创建表结构
DROP TABLE IF EXISTS Liyh_CourseStatistics;
CREATE TABLE Liyh_CourseStatistics (
    lyh_Cno VARCHAR(6),
    lyh_Cname VARCHAR(30),
    StudentCount INT,
    AverageScore DECIMAL(5,2)
);

-- 然后插入数据
INSERT INTO Liyh_CourseStatistics (lyh_Cno, lyh_Cname, StudentCount, AverageScore)
SELECT r.lyh_Cno, c.lyh_Cname, 
       COUNT(r.lyh_Sno) AS StudentCount, 
       AVG(r.lyh_Score) AS AverageScore
FROM Liyh_Reports r
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
GROUP BY r.lyh_Cno, c.lyh_Cname;

(3)	在Students表中使用SQL语句将姓名为李迪的学生的学号改为S70。
UPDATE Liyh_Students
SET lyh_Sno = 'S70'
WHERE lyh_Sname = '李迪';

(4)	在Teachers表中使用SQL语句将所有教师的工资加500元。
UPDATE Liyh_Teachers
SET lyh_Tsalary = lyh_Tsalary + 500;

(5)	将姓名为刘华的学生的课程"数据库原理及其应用"的成绩加上6分。
UPDATE Liyh_Reports
SET lyh_Score = lyh_Score + 6
WHERE lyh_Sno = (SELECT lyh_Sno FROM Liyh_Students WHERE lyh_Sname = '刘华')
  AND lyh_Cno = (SELECT lyh_Cno FROM Liyh_Courses WHERE lyh_Cname = '数据库原理及应用');

(6)	在Students表中使用SQL语句删除姓名为李迪的学生信息。
DELETE FROM Liyh_Students
WHERE lyh_Sname = '李迪';

(7)	删除所有选修课程JAVA的选修课记录。
DELETE FROM Liyh_Reports
WHERE lyh_Cno = (SELECT lyh_Cno FROM Liyh_Courses WHERE lyh_Cname = 'JAVA');

(8)	对Courses表做删去学分<=4的元组操作，讨论该操作所受到的约束。

DELETE FROM Liyh_Courses
WHERE lyh_Ccredit <= 4;
/*
讨论：
这个删除操作会受到参照完整性约束的限制。在Liyh_Reports表中，lyh_Cno是一个外键，引用了Liyh_Courses表的主键。
如果我们尝试删除Liyh_Courses表中学分<=4的课程，而这些课程在Liyh_Reports表中被引用，那么删除操作会失败，除非：

设置了级联删除（CASCADE DELETE），这会同时删除Liyh_Reports表中相关的记录
先删除Liyh_Reports表中引用这些课程的记录
 将Liyh_Reports表中的外键约束暂时禁用

从数据来看，大部分课程的学分<=4，删除这些课程将会移除大部分课程数据，这可能不是预期的操作。
更安全的做法是先检查哪些课程被引用了:
*/
SELECT c.lyh_Cno, c.lyh_Cname, c.lyh_Ccredit, COUNT(r.lyh_Sno) AS ReferencedCount
FROM Liyh_Courses c
LEFT JOIN Liyh_Reports r ON c.lyh_Cno = r.lyh_Cno
WHERE c.lyh_Ccredit <= 4
GROUP BY c.lyh_Cno, c.lyh_Cname, c.lyh_Ccredit;

-- 如果真的需要删除这些课程，需要先处理外键约束：

-- 方案1：先删除引用记录
DELETE FROM Liyh_Reports
WHERE lyh_Cno IN (SELECT lyh_Cno FROM Liyh_Courses WHERE lyh_Ccredit <= 4);

-- 然后删除课程
DELETE FROM Liyh_Courses
WHERE lyh_Ccredit <= 4;

-- 方案2：如果数据库支持，可以使用级联删除来一次性完成操作
ALTER TABLE Liyh_Reports
DROP CONSTRAINT FK_Cou_Rep;

ALTER TABLE Liyh_Reports
ADD CONSTRAINT FK_Cou_Rep FOREIGN KEY(lyh_Cno) 
REFERENCES Liyh_Courses(lyh_Cno) ON DELETE CASCADE;

DELETE FROM Liyh_Courses
WHERE lyh_Ccredit <= 4;

