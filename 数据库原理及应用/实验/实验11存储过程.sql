-- 用DAS运行并观察结果。
-- 11.2 实验内容（用DAS完成如下内容）
-- （1）创建一个无参数的存储过程
-- ①	创建一个无参数的简单存储过程Liyh_proc_co，查询所有课程的信息。
-- ② 执行存储过程Liyh_proc_co。
-- （2）创建一个带输入参数的存储过程。
-- ①创建一个带输入参数的存储过程Liyh_proc_re，查询指定课程的学生成绩信息，其中输入参数用于接收课程号。
-- ② 执行存储过程Liyh_proc_re。
-- （3）创建一个带输入参数和输出参数的存储过程。
-- ①创建一个带有输入参数和输出参数的存储过程Liyh_proc_tname，查询学生选修该教师所教授的全部课程的成绩，返回该教师的姓名。其中输入参数用于接收教师的教师编号，输出参数用于返回该教师的姓名。
-- ② 执行存储过程Liyh_proc_tname。
-- （4）创建一个带参数游标的存储过程。
-- ①创建一个与（2）的功能一样的带参数游标的存储过程Liyh_proc_re_cur，查询指定课程的学生成绩信息，其中输入参数用于接收课程号。
-- ② 执行存储过程Liyh_proc_re_cur。
-- （5）创建一个嵌套调用的存储过程。
-- ①创建一个嵌套调用的存储过程Liyh_proc_cou_stu，输入课程名称，可以查询该课程选课学生的学号及成绩。首先创建一个带有输入和输出参数的存储过程Liyh_proc_cou，查询指定课程名称的课程编号，输入参数课程名称保存在Cou_name，输出参数保存在Cou_no中，然后根据Cou_no在Liyh_Reports表中查询学生选课情况。
-- ② 执行存储过程Liyh_proc_cou_stu('数据库原理及应用')。
-- （6）创建一个改变数据库数据的存储过程。
-- ①创建一个嵌套调用的存储过程Liyh_proc_del_stu，该存储过程传入一个学分的参数，当学分达到这个值及以上的学生就作为已经完成了学业，可以删除该学生的记录。
-- ②  执行存储过程Liyh_proc_del_stu并查看存储过程执行的结果。
-- （7）删除存储过程
-- 注意删除之前，要查看存储过程的嵌套关系，如果某个存储过程被另一个存储过程调用，则不能轻易被删除。
-- （8）查看存储过程
-- ①查看pg_proc的结构
-- ②查看存储过程的所有者、对象类型信息。提示：若信息过多时 可ctrl+c结束显示。
-- ③查看存储过程的详细定义

-- 11.3 实验步骤
-- （1）创建一个无参数的存储过程
-- ①	创建一个无参数的简单存储过程Liyh_proc_co，查询所有课程的信息。
CREATE OR REPLACE PROCEDURE Liyh_proc_co () 
AS
DECLARE 
    lyh_stu_Cno  VARCHAR(6);
    lyh_stu_Cname  VARCHAR(20);
    lyh_cou_Credit  DECIMAL(5,1);
    CURSOR  C  IS  SELECT Liyh_Courses.lyh_Cno, Liyh_Courses.lyh_Cname, Liyh_Courses.lyh_Ccredit FROM Liyh_Courses;  
BEGIN 
    OPEN C; 
    LOOP 
        FETCH C INTO lyh_stu_Cno, lyh_stu_Cname, lyh_cou_Credit; 
        EXIT WHEN C%NOTFOUND;  
        RAISE info 'Cno: %, Cname: %, Credit: %', lyh_stu_Cno, lyh_stu_Cname, lyh_cou_Credit;
    END LOOP; 
    CLOSE C; 
END;  
/

-- ②  执行存储过程Liyh_proc_co。
CALL Liyh_proc_co();

-- （2）创建一个带输入参数的存储过程。
-- ①创建一个带输入参数的存储过程Liyh_proc_re，查询指定课程的学生成绩信息，其中输入参数用于接收课程号。
CREATE OR REPLACE PROCEDURE Liyh_proc_re(lyh_inCno IN VARCHAR(6)) 
AS 
DECLARE
    lyh_stu_Sno  VARCHAR(6);
    lyh_stu_Sname  VARCHAR(20);
    lyh_cou_name  VARCHAR(20);
    lyh_cou_Score   DECIMAL(5,1);
    CURSOR  C  IS  SELECT Liyh_Students.lyh_Sno, Liyh_Students.lyh_Sname, Liyh_Courses.lyh_Cname, Liyh_Reports.lyh_Score  
                   FROM Liyh_Students, Liyh_Courses, Liyh_Reports
                   WHERE Liyh_Students.lyh_Sno=Liyh_Reports.lyh_Sno AND Liyh_Reports.lyh_Cno=Liyh_Courses.lyh_Cno AND Liyh_Courses.lyh_Cno=lyh_inCno;
BEGIN 
    OPEN C; 
    LOOP 
        FETCH C INTO lyh_stu_Sno, lyh_stu_Sname, lyh_cou_name, lyh_cou_Score; 
        EXIT WHEN C%NOTFOUND;  
        RAISE info 'Sno: %, Sname: %, Cname: %, Score: %', lyh_stu_Sno, lyh_stu_Sname, lyh_cou_name, lyh_cou_Score;
    END LOOP; 
    CLOSE C; 
END;  
/

-- ②  执行存储过程Liyh_proc_re。
CALL Liyh_proc_re('C01');

-- （3）创建一个带输入参数和输出参数的存储过程。
-- ①创建一个带有输入参数和输出参数的存储过程Liyh_proc_tname，查询学生选修该教师所教授的全部课程的成绩，返回该教师的姓名。其中输入参数用于接收教师的教师编号，输出参数用于返回该教师的姓名。
CREATE OR REPLACE PROCEDURE Liyh_proc_tname(lyh_Ttno IN VARCHAR(6), lyh_Tea_name OUT VARCHAR(20)) 
AS
DECLARE
    lyh_Stu_no VARCHAR(6);
    lyh_Tea_no VARCHAR(6);
    lyh_Cou_no VARCHAR(6);		
    lyh_Cou_Score DECIMAL(5,1);
    CURSOR C IS SELECT lyh_Sno, lyh_Tno, lyh_Cno, lyh_Score FROM Liyh_Reports WHERE Liyh_Reports.lyh_Tno=lyh_Ttno;
BEGIN
    SELECT lyh_Tname INTO lyh_Tea_name FROM Liyh_Teachers WHERE lyh_Tno=lyh_Ttno;
    OPEN C; 
    LOOP 
        FETCH C INTO lyh_Stu_no, lyh_Tea_no, lyh_Cou_no, lyh_Cou_Score;
        EXIT WHEN C%NOTFOUND;  
        RAISE info 'Sno: %, Tno: %, Cno: %, Score: %', lyh_Stu_no, lyh_Tea_no, lyh_Cou_no, lyh_Cou_Score;   
    END LOOP; 
    CLOSE C;  
END;
/

-- ②  执行存储过程Liyh_proc_tname。
CALL Liyh_proc_tname('T01', NULL);

-- （4）创建一个带参数游标的存储过程。
-- ①创建一个与（2）的功能一样的带参数游标的存储过程Liyh_proc_re_cur，查询指定课程的学生成绩信息，其中输入参数用于接收课程号。
CREATE OR REPLACE PROCEDURE Liyh_proc_re_cur(lyh_inCno IN VARCHAR(6)) 
AS 
DECLARE
    lyh_stu_Sno  VARCHAR(6);
    lyh_stu_Sname  VARCHAR(20);
    lyh_cou_name  VARCHAR(20);
    lyh_cou_Score   DECIMAL(5,1);
    CURSOR  C(lyh_Cur_Cno VARCHAR(6))  IS  SELECT Liyh_Students.lyh_Sno, Liyh_Students.lyh_Sname, Liyh_Courses.lyh_Cname, Liyh_Reports.lyh_Score  
                                           FROM Liyh_Students, Liyh_Courses, Liyh_Reports
                                           WHERE Liyh_Students.lyh_Sno=Liyh_Reports.lyh_Sno AND Liyh_Reports.lyh_Cno=Liyh_Courses.lyh_Cno AND Liyh_Courses.lyh_Cno=lyh_Cur_Cno;
BEGIN 
    OPEN C(lyh_inCno); 
    LOOP 
        FETCH C INTO lyh_stu_Sno, lyh_stu_Sname, lyh_cou_name, lyh_cou_Score; 
        EXIT WHEN C%NOTFOUND;  
        RAISE info 'Sno: %, Sname: %, Cname: %, Score: %', lyh_stu_Sno, lyh_stu_Sname, lyh_cou_name, lyh_cou_Score;
    END LOOP; 
    CLOSE C; 
END;  
/

-- ②  执行存储过程Liyh_proc_re_cur。
CALL Liyh_proc_re_cur('C01');

-- （5）创建一个嵌套调用的存储过程。
-- ①创建一个嵌套调用的存储过程Liyh_proc_cou_stu，输入课程名称，可以查询该课程选课学生的学号及成绩。首先创建一个带有输入和输出参数的存储过程Liyh_proc_cou，查询指定课程名称的课程编号，输入参数课程名称保存在Cou_name，输出参数保存在Cou_no中，然后根据Cou_no在Liyh_Reports表中查询学生选课情况。
CREATE OR REPLACE PROCEDURE Liyh_proc_cou(lyh_Cou_name IN VARCHAR(20), lyh_Cou_no OUT VARCHAR(6)) 
AS
DECLARE
BEGIN
    SELECT lyh_Cno INTO lyh_Cou_no FROM Liyh_Courses WHERE lyh_Cname = lyh_Cou_name ORDER BY 1 LIMIT 1;
END;
/

CREATE OR REPLACE PROCEDURE Liyh_proc_cou_stu(lyh_Cou_name IN VARCHAR(20))
AS
DECLARE
    lyh_outCou_no VARCHAR(6);
    lyh_Stu_no VARCHAR(6);
    lyh_Tea_no VARCHAR(6);
    lyh_Cou_no VARCHAR(6);		
    lyh_Cou_Score DECIMAL(5,1);
    CURSOR C(lyh_Cur_Cno VARCHAR(6)) IS SELECT lyh_Sno, lyh_Tno, lyh_Cno, lyh_Score FROM Liyh_Reports WHERE Liyh_Reports.lyh_Cno=lyh_Cur_Cno;
BEGIN
    CALL Liyh_proc_cou(lyh_Cou_name, lyh_outCou_no);
    OPEN C(lyh_outCou_no); 
    LOOP 
        FETCH C INTO lyh_Stu_no, lyh_Tea_no, lyh_Cou_no, lyh_Cou_Score;
        EXIT WHEN C%NOTFOUND;  
        RAISE info 'Sno: %, Tno: %, Cno: %, Score: %', lyh_Stu_no, lyh_Tea_no, lyh_Cou_no, lyh_Cou_Score;   
    END LOOP; 
    CLOSE C;  
END;
/

-- ②  执行存储过程Liyh_proc_cou_stu。
CALL Liyh_proc_cou_stu('数据库原理及应用');

-- （6）创建一个改变数据库数据的存储过程。
-- ①创建一个嵌套调用的存储过程Liyh_proc_del_stu，该存储过程传入一个学分的参数，当学分达到这个值及以上的学生就作为已经完成了学业，可以删除该学生的记录。
CREATE OR REPLACE PROCEDURE Liyh_proc_del_stu(lyh_min_credit DECIMAL(5,1))
AS
DECLARE
    lyh_stu_sno  VARCHAR(6);
    CURSOR  C(lyh_cur_credit DECIMAL(5,1))  IS SELECT lyh_Sno FROM Liyh_Students WHERE lyh_Scredit >= lyh_cur_credit;   
BEGIN 
    OPEN C(lyh_min_credit); 
    LOOP 
        FETCH C INTO lyh_stu_sno; 
        EXIT WHEN C%NOTFOUND;  
        DELETE FROM Liyh_Reports WHERE lyh_Sno=lyh_stu_sno;
        DELETE FROM Liyh_Students WHERE lyh_Sno = lyh_stu_sno;
    END LOOP; 
    CLOSE C; 
END;  
/

-- ②  执行存储过程Liyh_proc_del_stu并查看存储过程执行的结果。
CALL Liyh_proc_del_stu(24);
SELECT * FROM Liyh_Students;
SELECT * FROM Liyh_Reports;

-- （7）删除存储过程
-- 删除存储过程，例如删除Liyh_proc_co存储过程。
-- 注意删除之前，要查看存储过程的嵌套关系，如果某个存储过程被另一个存储过程调用，则不能轻易被删除，比如（5）。
DROP PROCEDURE Liyh_proc_co;

-- （8）查看存储过程
-- ①查看pg_proc的结构
\d pg_proc
-- 上述命令为psql命令，在DAS中无法使用，以下为DAS命令
SELECT column_name, data_type, is_nullable 
FROM information_schema.columns 
WHERE table_name = 'pg_proc' 
ORDER BY ordinal_position;

-- ②查看嵌套调用的存储过程Liyh_proc_cou_stu的所有者、对象类型信息。提示：若信息过多时 可ctrl+c结束显示。
SELECT proname, proowner, prokind FROM pg_proc WHERE proname='liyh_proc_cou_stu';

-- ③查看存储过程的详细定义
SELECT proname, prosrc FROM pg_proc WHERE proname='liyh_proc_cou_stu';
