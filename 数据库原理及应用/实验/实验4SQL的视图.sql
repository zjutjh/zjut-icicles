
-- (1) 创建行列子集视图CS_View，给出选课成绩合格的学生的编号、教师编号、所选课程号和该课程成绩，并查询该视图。
CREATE VIEW CS_View AS
SELECT lyh_Sno, lyh_Tno, lyh_Cno, lyh_Score
FROM Liyh_Reportsbk
WHERE lyh_Score >= 60;

-- 查询视图 CS_View
SELECT * FROM CS_View;


-- (2) 创建基于多个基本表的视图SCT_View，该视图由学生姓名、该生所选课程名及讲授该课程的教师姓名构成，并查询该视图。从视图SCT_View中查询所有选修课程“数据库原理及应用”的学生姓名。
CREATE VIEW SCT_View AS
SELECT s.lyh_Sname, c.lyh_Cname, t.lyh_Tname
FROM Liyh_Studentsbk s
JOIN Liyh_Reportsbk r ON s.lyh_Sno = r.lyh_Sno
JOIN Liyh_Coursesbk c ON r.lyh_Cno = c.lyh_Cno
JOIN Liyh_Teachersbk t ON r.lyh_Tno = t.lyh_Tno;

-- 查询视图 SCT_View
SELECT * FROM SCT_View;

-- 从视图 SCT_View 中查询所有选修课程“数据库原理及应用”的学生姓名
SELECT lyh_Sname 
FROM SCT_View 
WHERE lyh_Cname = '数据库原理及应用';


-- (3) 创建带表达式的视图EXP_View，由学生姓名及所选课程名和所有课程成绩都比原来多5分这几个属性组成，并查询该视图。
CREATE VIEW EXP_View AS
SELECT s.lyh_Sname, c.lyh_Cname, r.lyh_Score + 5 AS ModifiedScore
FROM Liyh_Studentsbk s
JOIN Liyh_Reportsbk r ON s.lyh_Sno = r.lyh_Sno
JOIN Liyh_Coursesbk c ON r.lyh_Cno = c.lyh_Cno
WHERE r.lyh_Score IS NOT NULL; -- 只处理有成绩的记录

-- 查询视图 EXP_View
SELECT * FROM EXP_View;


-- (4) 创建分组视图Group_View，将学生的学号及他的平均成绩定义为一个视图，并查询该视图。
CREATE VIEW Group_View AS
SELECT lyh_Sno, AVG(lyh_Score) AS AverageScore
FROM Liyh_Reportsbk
WHERE lyh_Score IS NOT NULL -- 只计算非空成绩的平均分
GROUP BY lyh_Sno;

-- 查询视图 Group_View
SELECT * FROM Group_View;


-- (5) 创建一个基于视图的视图，基于（1）中建立的视图，定义一个包括学生编号、学生所选课程数目和平均成绩的视图VV_View，并查询该视图。
CREATE VIEW VV_View AS
SELECT lyh_Sno, COUNT(lyh_Cno) AS CourseCount, AVG(lyh_Score) AS AverageScore
FROM CS_View -- 基于第(1)步创建的视图
GROUP BY lyh_Sno;

-- 查询视图 VV_View
SELECT * FROM VV_View;


-- (6) 创建全量物化视图MV_R_View，给出Reports中选课成绩合格的学生选修课程的元组数，
--并查询视图MV_R_View。向基表Reports插入数据('S52','T05','C07',93)，
--并查询视图MV_R_View。对全量物化视图做增量刷新，将会出现什么现象？为什么？
--向基表Reports插入数据('S52','T05','C03',83)，对全量物化视图MV_R_View做全量刷新，
--并查询视图MV_R_View。思考全量物化视图、增量物化视图与普通视图的区别。


-- 创建物化视图
CREATE MATERIALIZED VIEW MV_R_View AS
SELECT COUNT(*) AS QualifiedReportCount
FROM Liyh_Reportsbk
WHERE lyh_Score >= 60;

-- 查询物化视图 (初始状态)
SELECT * FROM MV_R_View;

-- 向基表 Liyh_Reportsbk 插入数据
INSERT INTO Liyh_Reportsbk VALUES ('S52','T05','C07',93);

-- 再次查询物化视图 (数据未变，因为未刷新)
SELECT * FROM MV_R_View;

-- 尝试增量刷新 (基于COUNT(*)通常不支持简单增量刷新)
-- REFRESH MATERIALIZED VIEW CONCURRENTLY MV_R_View; -- 尝试并发增量刷新
-- 现象与原因：如果数据库系统不支持基于此聚合的增量刷新，该命令会失败或不执行。即使支持，也可能需要特定设置（如唯一索引）。普通增量刷新 REFRESH MATERIALIZED VIEW MV_R_View; 会锁表。

-- 向基表 Liyh_Reportsbk 插入另一条数据
INSERT INTO Liyh_Reportsbk VALUES ('S52','T05','C03',83);

-- 执行全量刷新
REFRESH MATERIALIZED VIEW MV_R_View;

-- 再次查询物化视图 (数据已更新)
SELECT * FROM MV_R_View;

-- 思考：
-- 普通视图：存储查询定义，每次查询实时执行，数据最新，不占额外存储。
-- 物化视图：存储查询结果，查询速度快，数据非实时（需刷新），占用存储空间。
-- 全量刷新：重新计算整个物化视图的结果。
-- 增量刷新：只计算基表变化部分对物化视图结果的影响（若支持），效率可能更高。


-- (8) 删除视图SCT_View、视图EXP_View和视图Group_View，删除物化视图MV_R_View。
-- 删除VV_View也很重要，因为它依赖于CS_View
DROP VIEW IF EXISTS SCT_View;
DROP VIEW IF EXISTS EXP_View;
DROP VIEW IF EXISTS Group_View;
--DROP VIEW IF EXISTS VV_View; -- 需要先删除依赖CS_View的视图
DROP MATERIALIZED VIEW IF EXISTS MV_R_View;


-- (9) 操作SQL命令DROP VIEW CS_View;将出现什么现象？为什么？请给出删除视图CS_View的方法。
DROP VIEW IF EXISTS CS_View;
-- 现象：如果第(5)步创建的视图VV_View没有在第(8)步被删除，那么此操作会失败，并报错提示存在依赖对象(VV_View)。
-- 原因：数据库不允许直接删除被其他对象（如另一个视图）依赖的对象，以保证数据一致性。
-- 方法：
-- 方法一：先删除依赖它的视图VV_View，再删除CS_View（如第(8)步所示）。
-- DROP VIEW IF EXISTS VV_View;
-- DROP VIEW IF EXISTS CS_View;
-- 方法二：使用级联删除（CASCADE），同时删除CS_View及其所有依赖对象。
-- DROP VIEW IF EXISTS CS_View CASCADE;


-- (10) 创建一个带WITH CHECK OPTION子句的视图CS_View_opt，给出选课成绩合格的学生的编号、教师编号、所选课程号和该课程成绩，并查询该视图。通过CS_View_opt视图，插入元组（'S52','T05','C07',51）到视图CS_View_opt中。分析对插入操作有什么影响。通过视图CS_View_opt，把所有课程编号为C01的课程的成绩都减去5分。这个操作数据库是否会正确执行，为什么？如果加上5分（原来95分以上的不变）呢？在视图CS_View_opt删除编号S03学生的记录，会产生什么结果？ 
CREATE VIEW CS_View_opt AS
SELECT lyh_Sno, lyh_Tno, lyh_Cno, lyh_Score
FROM Liyh_Reportsbk
WHERE lyh_Score >= 60
WITH CHECK OPTION;

-- 查询视图 CS_View_opt
SELECT * FROM CS_View_opt;

-- 尝试通过视图插入元组 ('S52','T05','C07',51)
-- INSERT INTO CS_View_opt (lyh_Sno, lyh_Tno, lyh_Cno, lyh_Score) VALUES ('S52','T05','C07',51);
-- 分析：此插入操作将失败。
-- 因为 `WITH CHECK OPTION` 要求所有通过该视图进行的 `INSERT` 或 `UPDATE` 操作
-- 所产生的行必须满足视图的 `WHERE` 条件 (`lyh_Score >= 60`)。由于插入的成绩是51，
-- 不满足条件，操作被阻止。

-- 尝试通过视图将C01课程成绩减去5分
-- UPDATE CS_View_opt SET lyh_Score = lyh_Score - 5 WHERE lyh_Cno = 'C01';
-- 分析：此更新操作可能会失败。如果存在C01课程的记录其原始成绩在60到64分之间，
-- 减去5分后将不再满足 `lyh_Score >= 60` 的条件。
--`WITH CHECK OPTION` 会阻止这类使得记录不再满足视图条件的更新。
-- 但所有C01的成绩减5后仍然>=60，更新成功。

-- 尝试通过视图将成绩加上5分（原来95分以上的不变，假设满分100）
-- UPDATE CS_View_opt SET lyh_Score = LEAST(lyh_Score + 5, 100) WHERE lyh_Score <= 95 AND lyh_Cno = 'C01';
-- 或 UPDATE CS_View_opt SET lyh_Score = lyh_Score + 5 WHERE lyh_Score <= 95 AND lyh_Cno = 'C01'    ;
-- 分析：此更新操作将成功执行。因为所有通过视图可见的记录（`lyh_Score >= 60`），增加5分后其成绩仍然大于等于60，满足视图的 `WHERE` 条件。`WITH CHECK OPTION` 允许这样的更新。

-- 尝试通过视图删除编号S03学生的记录
-- DELETE FROM CS_View_opt WHERE lyh_Sno = 'S03';
-- 分析：此删除操作将成功执行。它会删除基表 `Liyh_Reportsbk` 中所有满足 `lyh_Sno = 'S03'` 并且 `lyh_Score >= 60`（即视图可见）的记录。`WITH CHECK OPTION` 对 `DELETE` 操作没有限制作用。
