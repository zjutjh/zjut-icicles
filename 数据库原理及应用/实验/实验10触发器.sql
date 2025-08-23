-- 用DAS运行并观察结果。
-- 10.2 实验内容（用实验9的Teacher表完成）
-- (1) 为Teacher表建立触发器tri1，当插入或使更新表中的数据时，保证所操作的记录的Tage值大于0。
-- (2) 为Teacher表建立触发器tri2，禁止删除编号为T01的班主任。
-- (3) Teacher表中的人员的编号是唯一且不可更变的，创建触发器tri3实现更新中编号的不可改变性。
-- (4) 演示违反tri1触发器的约束的插入操作。
-- (5) 演示违反tri1触发器的约束的更新操作。
-- (6) 演示违反tri2触发器的约束的删除操作。
-- (7) 演示违反tri3触发器的约束的更新操作。

-- 10.3 实验步骤(用DAS)
-- (1) 仍然使用自定义完整性实验中的Teacher表。为此表建立触发器tri1，当插入或使更新表中的数据时，保证所操作的记录的Tage值大于0。
-- 输入如下SQL语句
CREATE OR REPLACE FUNCTION lyh_tri1_func() RETURNS TRIGGER AS
$$
DECLARE
BEGIN
    IF NEW.lyh_Tage > 0 THEN
        RAISE NOTICE '职工年龄大于0的整数! 操作成功！';
        RETURN NEW;
    ELSE
        RAISE EXCEPTION '职工年龄必须是大于0的整数! 操作失败！';
    END IF; 
END
$$ LANGUAGE PLPGSQL;

CREATE TRIGGER lyh_tri1
AFTER INSERT OR UPDATE ON Liyh_Teacher
FOR EACH ROW
EXECUTE PROCEDURE lyh_tri1_func();
-- 提示：在开发者指南中搜索raise，可找到其语法，RAISE NOTICE显示调试信息，RAISE EXCEPTION抛出一个正常终止当前事务的异常。

-- (2) 为Teacher表建立触发器tri2，禁止删除编号为T01的班主任。
-- 在OpenGauss的控制台中输入如下SQL语句
CREATE OR REPLACE FUNCTION lyh_tri2_func() RETURNS TRIGGER AS
$$
DECLARE
BEGIN
    IF OLD.lyh_Tno = 'T01' THEN
        RAISE EXCEPTION '此人是班主任! 删除操作失败!';
    ELSE
        RETURN OLD; 
    END IF; 
END
$$ LANGUAGE PLPGSQL;

CREATE TRIGGER lyh_tri2
BEFORE DELETE ON Liyh_Teacher
FOR EACH ROW
EXECUTE PROCEDURE lyh_tri2_func();
    
-- (3) Teacher表中的人员的编号是唯一且不可更变的，创建触发器tri3实现更新中编号的不可改变性。
-- 在OpenGauss的控制台中输入如下SQL语句
CREATE OR REPLACE FUNCTION lyh_tri3_func() RETURNS TRIGGER AS
$$
DECLARE
BEGIN
    RAISE EXCEPTION '职工编号不能修改！';
END
$$ LANGUAGE PLPGSQL;

CREATE TRIGGER lyh_tri3
BEFORE UPDATE OF lyh_Tno ON Liyh_Teacher
FOR EACH ROW
EXECUTE PROCEDURE lyh_tri3_func();
     
-- (4) 演示违反tri1触发器的约束的插入操作。
-- 尝试插入年龄为负数的记录（应该失败）
INSERT INTO Liyh_Teacher VALUES('T05', '李宏', 'F', -8, '开发部');
-- 然后插入一条合法记录（应该成功）
INSERT INTO Liyh_Teacher VALUES('T05', '李宏', 'F', 8, '开发部');
SELECT * FROM Liyh_Teacher;

-- (5) 演示违反tri1触发器的约束的更新操作。
-- 尝试将年龄更新为负数（应该失败）
UPDATE Liyh_Teacher SET lyh_Tage = -7 WHERE lyh_Tno = 'T01';
SELECT * FROM Liyh_Teacher;

-- (6) 演示违反tri2触发器的约束的删除操作。
-- 尝试删除班主任（应该失败）
DELETE FROM Liyh_Teacher WHERE lyh_Tno = 'T01';


-- (7) 演示违反tri3触发器的约束的更新操作。
-- 尝试修改职工编号（应该失败）
UPDATE Liyh_Teacher SET lyh_Tno = 'T07' WHERE lyh_Tno = 'T01';
-- 尝试修改其他字段（应该成功）
UPDATE Liyh_Teacher SET lyh_Tsex = 'F' WHERE lyh_Tno = 'T05';
SELECT * FROM Liyh_Teacher;