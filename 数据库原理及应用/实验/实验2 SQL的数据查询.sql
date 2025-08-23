-- 以db_zjut数据库为例，该数据库中有四张如实验1，其中Score是每门课的考试成绩，
-- Scredit是学生所有考试合格课程所获得的积分总数，Ccredit每门课程的学分数。
-- 在数据库中，存在这样的联系：学生可以选择课程，一个课程对应一个教师。在表Reports中保存学生的选课记录和考试成绩。
-- 请先通过终端输入执行SQL语句完成如下符合条件的元组插入后，再对数据库进行有关的查询操作： 
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

（1）	查询性别为"男"的所有学生的名称并按学号升序排列。
SELECT lyh_Sno, lyh_Sname
FROM Liyh_Students
WHERE lyh_Ssex = '男'
ORDER BY lyh_Sno ASC;

（2）	查询学生的选课成绩合格的课程成绩，并把成绩换算为积分。积分的计算公式为：[1+(考试成绩-60)*0.1]*Ccredit。考试成绩>=60 否则=0
SELECT r.lyh_Sno, r.lyh_Cno, r.lyh_Score, 
       CASE 
           WHEN r.lyh_Score >= 60 THEN (1 + (r.lyh_Score - 60) * 0.1) * c.lyh_Ccredit
           ELSE 0
       END AS Credit
FROM Liyh_Reports r
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
WHERE r.lyh_Score IS NOT NULL;

（3）	查询学分是3或4的课程的名称。
SELECT lyh_Cname
FROM Liyh_Courses
WHERE lyh_Ccredit IN (3, 4);

（4）	查询所有课程名称中含有"算法"的课程编号。
SELECT lyh_Cno
FROM Liyh_Courses
WHERE lyh_Cname LIKE '%算法%';

（5）	查询所有选课记录的课程号（不重复显示）。
SELECT DISTINCT lyh_Cno
FROM Liyh_Reports;

（6）	统计所有老师的平均工资。
SELECT AVG(lyh_Tsalary) AS AverageSalary
FROM Liyh_Teachers;

（7）	查询所有教师的编号及选修其课程的学生的平均成绩，按平均成绩降序排列。
SELECT r.lyh_Tno, t.lyh_Tname, AVG(r.lyh_Score) AS AverageScore
FROM Liyh_Reports r
JOIN Liyh_Teachers t ON r.lyh_Tno = t.lyh_Tno
WHERE r.lyh_Score IS NOT NULL
GROUP BY r.lyh_Tno, t.lyh_Tname
ORDER BY AverageScore DESC;

（8）	统计各个课程的选课人数和平均成绩。
SELECT r.lyh_Cno, c.lyh_Cname, COUNT(r.lyh_Sno) AS StudentCount, AVG(r.lyh_Score) AS AverageScore
FROM Liyh_Reports r
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
GROUP BY r.lyh_Cno, c.lyh_Cname;

（9）	查询至少选修了三门课程的学生编号和姓名。
SELECT r.lyh_Sno, s.lyh_Sname
FROM Liyh_Reports r
JOIN Liyh_Students s ON r.lyh_Sno = s.lyh_Sno
GROUP BY r.lyh_Sno, s.lyh_Sname
HAVING COUNT(DISTINCT r.lyh_Cno) >= 3;

（10）	查询编号S26的学生所选的全部课程的课程名和成绩。
SELECT c.lyh_Cname, r.lyh_Score
FROM Liyh_Reports r
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
WHERE r.lyh_Sno = 'S26';

（11）	查询所有选修了"数据库原理及应用"课程的学生编号和姓名。
SELECT r.lyh_Sno, s.lyh_Sname
FROM Liyh_Reports r
JOIN Liyh_Students s ON r.lyh_Sno = s.lyh_Sno
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
WHERE c.lyh_Cname = '数据库原理及应用';

（12）	求出选修了同一个课程的学生对。
SELECT r1.lyh_Sno AS Student1, r2.lyh_Sno AS Student2, r1.lyh_Cno
FROM Liyh_Reports r1
JOIN Liyh_Reports r2 ON r1.lyh_Cno = r2.lyh_Cno AND r1.lyh_Sno < r2.lyh_Sno;

（13）	求出至少被两名学生选修的课程编号。
SELECT lyh_Cno
FROM Liyh_Reports
GROUP BY lyh_Cno
HAVING COUNT(DISTINCT lyh_Sno) >= 2;

（14）	查询选修了编号S26的学生所选的某个课程的学生编号。
-- 查询S26学生选修的每门课程及其同班同学
SELECT r1.lyh_Cno AS 课程编号, 
       c.lyh_Cname AS 课程名称,
       r1.lyh_Sno AS 班级同学学号,
       s.lyh_Sname AS 班级同学姓名
FROM Liyh_Reports r1
JOIN Liyh_Courses c ON r1.lyh_Cno = c.lyh_Cno
JOIN Liyh_Students s ON r1.lyh_Sno = s.lyh_Sno
WHERE r1.lyh_Cno IN (
    SELECT r2.lyh_Cno
    FROM Liyh_Reports r2
    WHERE r2.lyh_Sno = 'S26'
) 
ORDER BY r1.lyh_Cno, r1.lyh_Sno;

（15）	查询学生的基本信息及选修课程编号和成绩。
SELECT s.*, r.lyh_Cno, r.lyh_Score
FROM Liyh_Students s
LEFT JOIN Liyh_Reports r ON s.lyh_Sno = r.lyh_Sno;

（16）	查询学号S52的学生的姓名和选修的课程名称及成绩。
SELECT s.lyh_Sname, c.lyh_Cname, r.lyh_Score
FROM Liyh_Students s
JOIN Liyh_Reports r ON s.lyh_Sno = r.lyh_Sno
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
WHERE s.lyh_Sno = 'S52';

（17）	查询和学号S52的学生同性别的所有学生资料。
SELECT s2.*
FROM Liyh_Students s1, Liyh_Students s2
WHERE s1.lyh_Sno = 'S52' AND s1.lyh_Ssex = s2.lyh_Ssex;

（18）	查询所有选课的学生的详细信息。
SELECT DISTINCT s.*
FROM Liyh_Students s
JOIN Liyh_Reports r ON s.lyh_Sno = r.lyh_Sno;

（19）	查询没有学生选的课程的编号和名称。
SELECT c.lyh_Cno, c.lyh_Cname
FROM Liyh_Courses c
WHERE c.lyh_Cno NOT IN (
    SELECT DISTINCT lyh_Cno
    FROM Liyh_Reports
);

（20）	查询选修了课程名为C++的学生学号和姓名。
SELECT r.lyh_Sno, s.lyh_Sname
FROM Liyh_Reports r
JOIN Liyh_Students s ON r.lyh_Sno = s.lyh_Sno
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
WHERE c.lyh_Cname = 'C++';

（21）	找出选修课程UML或者课程C++的学生学号和姓名。
SELECT DISTINCT r.lyh_Sno, s.lyh_Sname
FROM Liyh_Reports r
JOIN Liyh_Students s ON r.lyh_Sno = s.lyh_Sno
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
WHERE c.lyh_Cname IN ('UML', 'C++');

（22）	找出和课程UML或课程C++的学分一样课程名称。
SELECT DISTINCT c2.lyh_Cname
FROM Liyh_Courses c1
JOIN Liyh_Courses c2 ON c1.lyh_Ccredit = c2.lyh_Ccredit
WHERE c1.lyh_Cname IN ('UML', 'C++') AND c2.lyh_Cname NOT IN ('UML', 'C++');

（23）	查询所有选修编号C01的课程的学生的姓名。
SELECT s.lyh_Sname
FROM Liyh_Students s
JOIN Liyh_Reports r ON s.lyh_Sno = r.lyh_Sno
WHERE r.lyh_Cno = 'C01';

（24）	查询选修了所有课程的学生姓名。
SELECT s.lyh_Sname
FROM Liyh_Students s
WHERE NOT EXISTS (
    SELECT *
    FROM Liyh_Courses c
    WHERE NOT EXISTS (
        SELECT *
        FROM Liyh_Reports r
        WHERE r.lyh_Sno = s.lyh_Sno AND r.lyh_Cno = c.lyh_Cno
    )
);

（25）	利用集合并运算，查询选修课程C++或选择课程JAVA的学生的编号、姓名和积分。
SELECT s.lyh_Sno, s.lyh_Sname, SUM(
    CASE 
        WHEN r.lyh_Score >= 60 THEN (1 + (r.lyh_Score - 60) * 0.1) * c.lyh_Ccredit
        ELSE 0
    END
) AS TotalCredit
FROM Liyh_Students s
JOIN Liyh_Reports r ON s.lyh_Sno = r.lyh_Sno
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
WHERE s.lyh_Sno IN (
    SELECT r1.lyh_Sno
    FROM Liyh_Reports r1
    JOIN Liyh_Courses c1 ON r1.lyh_Cno = c1.lyh_Cno
    WHERE c1.lyh_Cname = 'C++'
    UNION
    SELECT r2.lyh_Sno
    FROM Liyh_Reports r2
    JOIN Liyh_Courses c2 ON r2.lyh_Cno = c2.lyh_Cno
    WHERE c2.lyh_Cname = 'JAVA'
)
GROUP BY s.lyh_Sno, s.lyh_Sname;

（26）	实现集合交运算，查询既选修课程C++又选修课程JAVA的学生的编号、姓名和积分。
SELECT s.lyh_Sno, s.lyh_Sname, SUM(
    CASE 
        WHEN r.lyh_Score >= 60 THEN (1 + (r.lyh_Score - 60) * 0.1) * c.lyh_Ccredit
        ELSE 0
    END
) AS TotalCredit
FROM Liyh_Students s
JOIN Liyh_Reports r ON s.lyh_Sno = r.lyh_Sno
JOIN Liyh_Courses c ON r.lyh_Cno = c.lyh_Cno
WHERE s.lyh_Sno IN (
    SELECT r1.lyh_Sno
    FROM Liyh_Reports r1
    JOIN Liyh_Courses c1 ON r1.lyh_Cno = c1.lyh_Cno
    WHERE c1.lyh_Cname = 'C++'
    INTERSECT
    SELECT r2.lyh_Sno
    FROM Liyh_Reports r2
    JOIN Liyh_Courses c2 ON r2.lyh_Cno = c2.lyh_Cno
    WHERE c2.lyh_Cname = 'JAVA'
)
GROUP BY s.lyh_Sno, s.lyh_Sname;

（27）	实现集合减运算，查询选修课程C++而没有选修课程JAVA的学生的编号。
SELECT r1.lyh_Sno
FROM Liyh_Reports r1
JOIN Liyh_Courses c1 ON r1.lyh_Cno = c1.lyh_Cno
WHERE c1.lyh_Cname = 'C++'
EXCEPT
SELECT r2.lyh_Sno
FROM Liyh_Reports r2
JOIN Liyh_Courses c2 ON r2.lyh_Cno = c2.lyh_Cno
WHERE c2.lyh_Cname = 'JAVA';
