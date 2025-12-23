--(1)	查询所有选课记录的成绩并将它换算为五分制（满分为5分，合格为3分，即公式：round（成绩/100*5）），注意,创建表时允许Score取NULL值。
-- 查询所有选课记录并将成绩换算为五分制
SELECT lyh_Sno AS 学号, lyh_Cno AS 课程号, lyh_Score AS 原始成绩,
       CASE 
           WHEN lyh_Score IS NULL THEN NULL  -- NULL值保持NULL
           ELSE ROUND(lyh_Score/100.0*5, 0)  -- 将成绩换算为五分制
       END AS 五分制成绩,
       CASE 
           WHEN lyh_Score IS NULL THEN '未参加考试'
           WHEN lyh_Score >= 60 THEN '合格'
           ELSE '不合格'
       END AS 成绩状态
FROM Liyh_Reportsbk;

--(2)	通过查询选修编号C07的课程的学生的人数，其中成绩合格的学生人数，不合格的人数，讨论NULL值的特殊含义。
-- 查询C07课程的学生人数、合格人数、不合格人数和NULL值人数
SELECT 
    COUNT(lyh_Sno) AS 选课总人数,
    SUM(CASE WHEN lyh_Score >= 60 THEN 1 ELSE 0 END) AS 合格人数,
    SUM(CASE WHEN lyh_Score < 60 AND lyh_Score IS NOT NULL THEN 1 ELSE 0 END) AS 不合格人数,
    SUM(CASE WHEN lyh_Score IS NULL THEN 1 ELSE 0 END) AS 未参加考试人数
FROM Liyh_Reportsbk
WHERE lyh_Cno = 'C07';

-- NULL值的特殊含义讨论：
-- 1. NULL表示"未知"或"缺失"的值，而不是0或空字符串
-- 2. NULL与任何值比较（包括NULL本身）结果都是UNKNOWN，不是TRUE也不是FALSE
-- 3. 聚合函数如COUNT(*)会计算所有行，而COUNT(列名)会忽略NULL值
-- 4. NULL在表示"未参加考试"或"成绩未录入"等情况时非常有用

--(3)	通过实验检验在使用ORDER BY进行排序时(升序，降序)，取NULL的项是否出现在结果中？如果有，在什么位置？
-- 使用升序排序，观察NULL值的位置
SELECT lyh_Sno, lyh_Cno, lyh_Score
FROM Liyh_Reportsbk
ORDER BY lyh_Score ASC;

-- 使用降序排序，观察NULL值的位置
SELECT lyh_Sno, lyh_Cno, lyh_Score
FROM Liyh_Reportsbk
ORDER BY lyh_Score DESC;

-- 结论：NULL值在排序中确实会出现在结果中
-- 在PostgreSQL中，默认情况下：
-- - 升序(ASC)排序时，NULL值出现在最后
-- - 降序(DESC)排序时，NULL值出现在最前面
-- 可以使用NULLS FIRST或NULLS LAST显式指定NULL值的位置

--(4)	在上面的查询的过程中如果加上保留字DISTINCT会有什么效果呢？
-- 使用DISTINCT关键字，去除重复值
SELECT DISTINCT lyh_Score
FROM Liyh_Reportsbk
ORDER BY lyh_Score ASC;

-- 结论：
-- 1. DISTINCT对NULL的处理：多个NULL值被视为相同值，只保留一个
-- 2. DISTINCT会去除结果集中的重复行，但NULL值仍然保留在排序的位置
-- 3. NULL值在DISTINCT操作后仍然保持其在排序中的特殊位置

--(5)	通过实验说明使用分组GROUP BY对取值为NULL的项的处理。
-- 按成绩分组计数
SELECT lyh_Score, COUNT(*) AS 人数
FROM Liyh_Reportsbk
GROUP BY lyh_Score
ORDER BY lyh_Score ASC;

-- 结论：
-- 1. GROUP BY将NULL视为一个独立的组
-- 2. 所有NULL值被归为同一组
-- 3. NULL组在结果中会显示，并且遵循ORDER BY的排序规则

--(6)	结合分组，使用集合函数求每个同学的平均分、总的选课记录、最高成绩、最低成绩和总成绩。
-- 计算每个学生的成绩统计信息
SELECT 
    lyh_Sno AS 学号,
    COUNT(*) AS 选课总数,
    COUNT(lyh_Score) AS 有成绩课程数,  -- 排除NULL值
    ROUND(AVG(lyh_Score), 1) AS 平均分,
    MAX(lyh_Score) AS 最高分,
    MIN(lyh_Score) AS 最低分,
    SUM(lyh_Score) AS 总分
FROM Liyh_Reportsbk
GROUP BY lyh_Sno
ORDER BY lyh_Sno;

-- 注意：
-- 1. 聚合函数AVG、MAX、MIN、SUM自动忽略NULL值
-- 2. COUNT(列名)会忽略NULL值，而COUNT(*)计算所有行
-- 3. 如果分组中所有值都为NULL，则聚合函数结果为NULL

--(7)	查询成绩小于0的选课记录，统计总数、平均分、最大值和最小值。
-- 查询成绩小于0的记录及统计值
SELECT COUNT(*) AS 负分记录数, 
       AVG(lyh_Score) AS 平均分, 
       MAX(lyh_Score) AS 最高分, 
       MIN(lyh_Score) AS 最低分
FROM Liyh_Reportsbk
WHERE lyh_Score < 0;

-- 注：预期结果可能为空集(空表)，因为正常情况下成绩不应有负数
-- 空集对聚合函数的影响：
-- 1. COUNT(*) 返回0
-- 2. 其他聚合函数(AVG, MAX, MIN)在空集上返回NULL

--(8)	采用嵌套查询的方式，利用比较运算符和谓词ALL的结合来查询表Courses中最少的学分。假设数据库中只有一个记录的时候，使用前面的方法会得到什么结果，为什么？
-- 使用ALL谓词查询最少学分
SELECT lyh_Cno, lyh_Cname, lyh_Ccredit
FROM Liyh_Coursesbk
WHERE lyh_Ccredit <= ALL(
    SELECT lyh_Ccredit 
    FROM Liyh_Coursesbk
);

-- 假设只有一条记录的情况分析：
-- 如果数据库中只有一条记录，上述查询仍然能正确返回该记录
-- 原因：
-- 1. 当内部查询只返回一个值时，"<=ALL"等价于"<="简单比较
-- 2. 任何值都小于等于自身，所以唯一的记录会被选中
-- 3. ALL谓词要求比较满足所有内部查询结果，当只有一个结果时仍然有效

--(9)	创建一个学生表S（No，Sno，Sname），教师表T（No，Tno，Tname）作为实验用的表。其中，No分别是这两个表的主键，其他键允许为空。
-- 创建实验用学生表S
DROP TABLE IF EXISTS Liyh_S;
CREATE TABLE Liyh_S (
    lyh_No VARCHAR(10) PRIMARY KEY,
    lyh_Sno VARCHAR(10),
    lyh_Sname VARCHAR(20)
);

-- 创建实验用教师表T
DROP TABLE IF EXISTS Liyh_T;
CREATE TABLE Liyh_T (
    lyh_No VARCHAR(10) PRIMARY KEY,
    lyh_Tno VARCHAR(10),
    lyh_Tname VARCHAR(20)
);

--(10)	向S插入元组('n1'，'S01'，'李迪')、('n2'，'S02'，'李岚)、('n3'，'S05'，NULL)、('n4'，'S04'，'关红')；
-- 向学生表S插入数据
INSERT INTO Liyh_S (lyh_No, lyh_Sno, lyh_Sname) VALUES
('n1', 'S01', '李迪'),
('n2', 'S02', '李岚'),
('n3', 'S05', NULL),
('n4', 'S04', '关红');

--(11)	向T插入元组('n1'，'T09'，'李迪')、('n2'，'T08'，'李兰')、('n3'，'T01'，NULL)、('n4'，'T02'，NULL)。
-- 向教师表T插入数据
INSERT INTO Liyh_T (lyh_No, lyh_Tno, lyh_Tname) VALUES
('n1', 'T09', '李迪'),
('n2', 'T08', '李兰'),
('n3', 'T01', NULL),
('n4', 'T02', NULL);

--(12)	对这两个表作对姓名的等值连接运算，找出既是老师又是学生的人员的学生编号和教师编号。
-- 方法1：使用等值连接，找出既是老师又是学生的人（不考虑NULL值）
SELECT s.lyh_Sno AS 学生编号, t.lyh_Tno AS 教师编号, s.lyh_Sname AS 姓名
FROM Liyh_S s
JOIN Liyh_T t ON s.lyh_Sname = t.lyh_Tname
WHERE s.lyh_Sname IS NOT NULL AND t.lyh_Tname IS NOT NULL;

-- 方法2：使用IS NOT DISTINCT FROM运算符（可以处理NULL值的相等比较）
SELECT s.lyh_Sno AS 学生编号, t.lyh_Tno AS 教师编号, s.lyh_Sname AS 姓名
FROM Liyh_S s, Liyh_T t
WHERE s.lyh_Sname IS NOT DISTINCT FROM t.lyh_Tname;

-- 特别说明：
-- 1. 普通等值比较(=)不能匹配NULL值
-- 2. 'NULL = NULL'的结果是UNKNOWN，而不是TRUE
-- 3. 如果要考虑NULL值匹配，需要使用IS NOT DISTINCT FROM运算符或额外的NULL值检查
-- 4. 这个例子展示了NULL值在连接操作中的复杂性
