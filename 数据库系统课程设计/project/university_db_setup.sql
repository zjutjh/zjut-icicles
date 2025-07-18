-- =============================================================================
-- Script: 教务管理系统数据库构建脚本
-- Author: Gemini AI Assistant
-- Date: 2025-07-05
-- Description: 基于《数据库课程设计报告.md》的详细设计，
--              构建一个完整的教务管理系统数据库。
--              - 表名/视图名前缀: Liyh_
--              - 字段名前缀: lyh_
--              - 兼容 GaussDB / PostgreSQL
-- =============================================================================

-- =============================================================================
-- 0. 数据库和模式设置 (可选，手动执行)
-- =============================================================================
/*
CREATE DATABASE liyh_university_db;
\c liyh_university_db;

CREATE SCHEMA liyh_schema;
SET search_path TO liyh_schema, public;
*/

-- =============================================================================
-- 1. 表结构创建 (CREATE TABLE)
-- =============================================================================

-- -----------------------------------------------------
-- 基础表 (无外键依赖或依赖最少)
-- -----------------------------------------------------

-- 院系表
CREATE TABLE Liyh_Department (
    lyh_dept_id VARCHAR(4) PRIMARY KEY,
    lyh_dept_name VARCHAR(30) NOT NULL UNIQUE,
    lyh_dept_code VARCHAR(10) UNIQUE,
    lyh_description TEXT,
    lyh_dean_id VARCHAR(8), -- 稍后添加外键约束
    lyh_established_date DATE,
    lyh_phone VARCHAR(20),
    lyh_office_location VARCHAR(50)
);
COMMENT ON TABLE Liyh_Department IS '院系信息表';

-- 建筑物表
CREATE TABLE Liyh_Building (
    lyh_building_id VARCHAR(6) PRIMARY KEY,
    lyh_building_name VARCHAR(30) NOT NULL UNIQUE,
    lyh_building_code VARCHAR(10),
    lyh_address VARCHAR(100),
    lyh_floors INT CHECK (lyh_floors > 0),
    lyh_built_year INT
);
COMMENT ON TABLE Liyh_Building IS '校园建筑物信息表';

-- 时间段表
CREATE TABLE Liyh_TimeSlot (
    lyh_time_slot_id VARCHAR(10) PRIMARY KEY,
    lyh_day_of_week INT NOT NULL CHECK (lyh_day_of_week BETWEEN 1 AND 7), -- 1=周一, 7=周日
    lyh_start_time TIME NOT NULL,
    lyh_end_time TIME NOT NULL,
    lyh_period_desc VARCHAR(20)
);
COMMENT ON TABLE Liyh_TimeSlot IS '标准化的上课时间安排表';

-- 成绩等级表
CREATE TABLE Liyh_GradeScale (
    lyh_grade_id SERIAL PRIMARY KEY,
    lyh_letter_grade VARCHAR(2) NOT NULL UNIQUE,
    lyh_min_score DECIMAL(5, 2) NOT NULL,
    lyh_max_score DECIMAL(5, 2) NOT NULL,
    lyh_grade_point DECIMAL(3, 2) NOT NULL CHECK (lyh_grade_point >= 0),
    lyh_is_passing BOOLEAN NOT NULL DEFAULT TRUE
);
COMMENT ON TABLE Liyh_GradeScale IS '成绩评级标准表';

-- 学期表
CREATE TABLE Liyh_Semester (
    lyh_semester_id VARCHAR(12) PRIMARY KEY,
    lyh_academic_year INT NOT NULL,
    lyh_semester VARCHAR(10) NOT NULL CHECK (lyh_semester IN ('Spring', 'Fall', 'Summer')),
    lyh_start_date DATE NOT NULL,
    lyh_end_date DATE NOT NULL,
    lyh_enrollment_start TIMESTAMP,
    lyh_enrollment_end TIMESTAMP,
    lyh_is_current BOOLEAN DEFAULT FALSE
);
COMMENT ON TABLE Liyh_Semester IS '教学周期时间安排表';

-- -----------------------------------------------------
-- 核心实体表
-- -----------------------------------------------------

-- 专业表
CREATE TABLE Liyh_Major (
    lyh_major_id VARCHAR(6) PRIMARY KEY,
    lyh_major_name VARCHAR(30) NOT NULL,
    lyh_major_code VARCHAR(10) UNIQUE,
    lyh_dept_id VARCHAR(4) NOT NULL,
    lyh_duration INT DEFAULT 4,
    lyh_degree_type VARCHAR(20),
    lyh_coordinator_id VARCHAR(8), -- 稍后添加外键
    lyh_required_credits INT NOT NULL CHECK (lyh_required_credits > 0),
    lyh_description TEXT,
    FOREIGN KEY (lyh_dept_id) REFERENCES Liyh_Department(lyh_dept_id)
);
COMMENT ON TABLE Liyh_Major IS '学科专业信息表';

-- 用户账户表
CREATE TABLE Liyh_User (
    lyh_user_id VARCHAR(12) PRIMARY KEY, -- 统一长度以容纳学号和工号
    lyh_username VARCHAR(20) NOT NULL UNIQUE,
    lyh_password_hash VARCHAR(64) NOT NULL,
    lyh_role VARCHAR(10) NOT NULL CHECK (lyh_role IN ('student', 'instructor', 'admin')),
    lyh_created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    lyh_last_login_at TIMESTAMP,
    lyh_status VARCHAR(10) NOT NULL DEFAULT 'active' CHECK (lyh_status IN ('active', 'disabled', 'locked'))
);
COMMENT ON TABLE Liyh_User IS '系统用户的登录凭证和基本信息表';


-- 学生信息表
CREATE TABLE Liyh_Student (
    lyh_student_id VARCHAR(12) PRIMARY KEY,
    lyh_student_name VARCHAR(20) NOT NULL,
    lyh_gender VARCHAR(2) CHECK (lyh_gender IN ('男', '女')),
    lyh_birth_date DATE,
    lyh_id_card VARCHAR(18) UNIQUE,
    lyh_phone VARCHAR(11),
    lyh_email VARCHAR(50) UNIQUE,
    lyh_address VARCHAR(100),
    lyh_major_id VARCHAR(6) NOT NULL,
    lyh_admission_year INT,
    lyh_grade INT,
    lyh_duration INT,
    lyh_total_credits DECIMAL(5, 2) DEFAULT 0.00,
    lyh_current_gpa DECIMAL(3, 2) DEFAULT 0.00,
    lyh_academic_status VARCHAR(10) DEFAULT '在读' CHECK (lyh_academic_status IN ('在读', '休学', '退学', '毕业')),
    FOREIGN KEY (lyh_student_id) REFERENCES Liyh_User(lyh_user_id),
    FOREIGN KEY (lyh_major_id) REFERENCES Liyh_Major(lyh_major_id)
);
COMMENT ON TABLE Liyh_Student IS '学生的详细个人信息表';

-- 教师信息表
CREATE TABLE Liyh_Instructor (
    lyh_instructor_id VARCHAR(8) PRIMARY KEY,
    lyh_instructor_name VARCHAR(20) NOT NULL,
    lyh_gender VARCHAR(2) CHECK (lyh_gender IN ('男', '女')),
    lyh_title VARCHAR(20) CHECK (lyh_title IN ('助教', '讲师', '副教授', '教授')),
    lyh_education VARCHAR(20) CHECK (lyh_education IN ('学士', '硕士', '博士')),
    lyh_dept_id VARCHAR(4) NOT NULL,
    lyh_office_location VARCHAR(50),
    lyh_phone VARCHAR(11),
    lyh_email VARCHAR(50) UNIQUE,
    lyh_hire_date DATE,
    lyh_salary DECIMAL(10, 2),
    lyh_research_area VARCHAR(100),
    FOREIGN KEY (lyh_instructor_id) REFERENCES Liyh_User(lyh_user_id),
    FOREIGN KEY (lyh_dept_id) REFERENCES Liyh_Department(lyh_dept_id)
);
COMMENT ON TABLE Liyh_Instructor IS '教师的基本信息和职业信息表';

-- 补全 Liyh_Department 和 Liyh_Major 的外键约束
ALTER TABLE Liyh_Department ADD CONSTRAINT fk_dean FOREIGN KEY (lyh_dean_id) REFERENCES Liyh_Instructor(lyh_instructor_id);
ALTER TABLE Liyh_Major ADD CONSTRAINT fk_coordinator FOREIGN KEY (lyh_coordinator_id) REFERENCES Liyh_Instructor(lyh_instructor_id);

-- 管理员表
CREATE TABLE Liyh_Admin (
    lyh_admin_id VARCHAR(8) PRIMARY KEY,
    lyh_admin_name VARCHAR(20) NOT NULL,
    lyh_department VARCHAR(30),
    lyh_permission_level INT CHECK (lyh_permission_level BETWEEN 1 AND 9),
    lyh_contact_info VARCHAR(50),
    FOREIGN KEY (lyh_admin_id) REFERENCES Liyh_User(lyh_user_id)
);
COMMENT ON TABLE Liyh_Admin IS '教务管理员的基本信息表';

-- 课程表
CREATE TABLE Liyh_Course (
    lyh_course_id VARCHAR(8) PRIMARY KEY,
    lyh_course_title VARCHAR(50) NOT NULL,
    lyh_description TEXT,
    lyh_credits DECIMAL(3, 1) NOT NULL CHECK (lyh_credits > 0),
    lyh_type VARCHAR(10) NOT NULL CHECK (lyh_type IN ('必修', '专业选修', '通识选修')),
    lyh_dept_id VARCHAR(4) NOT NULL,
    lyh_difficulty_level INT CHECK (lyh_difficulty_level BETWEEN 1 AND 5),
    lyh_is_active BOOLEAN DEFAULT TRUE,
    FOREIGN KEY (lyh_dept_id) REFERENCES Liyh_Department(lyh_dept_id)
);
COMMENT ON TABLE Liyh_Course IS '课程的基本信息和属性表';

-- 教室表
CREATE TABLE Liyh_Classroom (
    lyh_classroom_id VARCHAR(10) PRIMARY KEY,
    lyh_building_id VARCHAR(6) NOT NULL,
    lyh_room_number VARCHAR(10) NOT NULL,
    lyh_capacity INT NOT NULL CHECK (lyh_capacity > 0),
    lyh_room_type VARCHAR(15) CHECK (lyh_room_type IN ('普通教室', '多媒体', '实验室')),
    lyh_equipment VARCHAR(100),
    lyh_is_available BOOLEAN DEFAULT TRUE,
    FOREIGN KEY (lyh_building_id) REFERENCES Liyh_Building(lyh_building_id)
);
COMMENT ON TABLE Liyh_Classroom IS '具体的教学场所信息表';


-- -----------------------------------------------------
-- 关系/连接表
-- -----------------------------------------------------

-- 先修关系表
CREATE TABLE Liyh_Prerequisite (
    lyh_prereq_id SERIAL PRIMARY KEY,
    lyh_course_id VARCHAR(8) NOT NULL,
    lyh_prereq_course_id VARCHAR(8) NOT NULL,
    lyh_requirement_type VARCHAR(10) DEFAULT '必须' CHECK (lyh_requirement_type IN ('必须', '建议')),
    FOREIGN KEY (lyh_course_id) REFERENCES Liyh_Course(lyh_course_id),
    FOREIGN KEY (lyh_prereq_course_id) REFERENCES Liyh_Course(lyh_course_id),
    UNIQUE(lyh_course_id, lyh_prereq_course_id)
);
COMMENT ON TABLE Liyh_Prerequisite IS '课程间的先修要求关系表';

-- 培养计划表
CREATE TABLE Liyh_Curriculum (
    lyh_curriculum_id SERIAL PRIMARY KEY,
    lyh_major_id VARCHAR(6) NOT NULL,
    lyh_course_id VARCHAR(8) NOT NULL,
    lyh_recommended_semester INT,
    lyh_course_nature VARCHAR(20),
    lyh_is_required BOOLEAN NOT NULL,
    FOREIGN KEY (lyh_major_id) REFERENCES Liyh_Major(lyh_major_id),
    FOREIGN KEY (lyh_course_id) REFERENCES Liyh_Course(lyh_course_id),
    UNIQUE(lyh_major_id, lyh_course_id)
);
COMMENT ON TABLE Liyh_Curriculum IS '专业的课程安排和要求表';

-- 教学班表
CREATE TABLE Liyh_Section (
    lyh_section_id SERIAL PRIMARY KEY,
    lyh_course_id VARCHAR(8) NOT NULL,
    lyh_semester_id VARCHAR(12) NOT NULL,
    lyh_section_number VARCHAR(10) NOT NULL,
    lyh_capacity INT NOT NULL CHECK (lyh_capacity > 0),
    lyh_enrolled_count INT DEFAULT 0 CHECK (lyh_enrolled_count >= 0),
    lyh_time_slot_id VARCHAR(10),
    lyh_classroom_id VARCHAR(10),
    lyh_notes VARCHAR(200),
    FOREIGN KEY (lyh_course_id) REFERENCES Liyh_Course(lyh_course_id),
    FOREIGN KEY (lyh_semester_id) REFERENCES Liyh_Semester(lyh_semester_id),
    FOREIGN KEY (lyh_time_slot_id) REFERENCES Liyh_TimeSlot(lyh_time_slot_id),
    FOREIGN KEY (lyh_classroom_id) REFERENCES Liyh_Classroom(lyh_classroom_id),
    UNIQUE(lyh_course_id, lyh_semester_id, lyh_section_number)
);
COMMENT ON TABLE Liyh_Section IS '课程在特定学期的开设情况（教学班）表';

-- 授课关系表
CREATE TABLE Liyh_Teaches (
    lyh_teach_id SERIAL PRIMARY KEY,
    lyh_instructor_id VARCHAR(8) NOT NULL,
    lyh_section_id INT NOT NULL,
    lyh_role VARCHAR(10) DEFAULT '主讲' CHECK (lyh_role IN ('主讲', '助教', '实验')),
    lyh_workload DECIMAL(3, 1),
    FOREIGN KEY (lyh_instructor_id) REFERENCES Liyh_Instructor(lyh_instructor_id),
    FOREIGN KEY (lyh_section_id) REFERENCES Liyh_Section(lyh_section_id),
    UNIQUE(lyh_instructor_id, lyh_section_id)
);
COMMENT ON TABLE Liyh_Teaches IS '教师与教学班的分配关系表';

-- 选课记录表
CREATE TABLE Liyh_Takes (
    lyh_take_id SERIAL PRIMARY KEY,
    lyh_student_id VARCHAR(12) NOT NULL,
    lyh_section_id INT NOT NULL,
    lyh_enroll_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    lyh_status VARCHAR(10) DEFAULT '正常' CHECK (lyh_status IN ('正常', '已退课', '重修')),
    lyh_numeric_grade DECIMAL(5, 2) CHECK (lyh_numeric_grade BETWEEN 0.00 AND 100.00),
    lyh_letter_grade VARCHAR(2),
    lyh_grade_point DECIMAL(3, 2),
    lyh_is_retake BOOLEAN DEFAULT FALSE,
    FOREIGN KEY (lyh_student_id) REFERENCES Liyh_Student(lyh_student_id),
    FOREIGN KEY (lyh_section_id) REFERENCES Liyh_Section(lyh_section_id),
    FOREIGN KEY (lyh_letter_grade) REFERENCES Liyh_GradeScale(lyh_letter_grade),
    UNIQUE(lyh_student_id, lyh_section_id)
);
COMMENT ON TABLE Liyh_Takes IS '学生选修教学班的完整信息及成绩表';

-- 教学评估表
CREATE TABLE Liyh_Evaluation (
    lyh_eval_id SERIAL PRIMARY KEY,
    lyh_student_id VARCHAR(12) NOT NULL,
    lyh_section_id INT NOT NULL,
    lyh_instructor_rating INT CHECK (lyh_instructor_rating BETWEEN 1 AND 5),
    lyh_course_rating INT CHECK (lyh_course_rating BETWEEN 1 AND 5),
    lyh_comments TEXT,
    lyh_eval_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    lyh_is_anonymous BOOLEAN DEFAULT TRUE,
    FOREIGN KEY (lyh_student_id) REFERENCES Liyh_Student(lyh_student_id),
    FOREIGN KEY (lyh_section_id) REFERENCES Liyh_Section(lyh_section_id),
    UNIQUE(lyh_student_id, lyh_section_id)
);
COMMENT ON TABLE Liyh_Evaluation IS '学生对课程和教师的评价表';

-- -----------------------------------------------------
-- 系统管理表
-- -----------------------------------------------------

-- 通知公告表
CREATE TABLE Liyh_Announcement (
    lyh_announce_id SERIAL PRIMARY KEY,
    lyh_title VARCHAR(100) NOT NULL,
    lyh_content TEXT NOT NULL,
    lyh_publisher_id VARCHAR(12) NOT NULL,
    lyh_publish_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    lyh_expiry_date DATE,
    lyh_target_audience VARCHAR(20) DEFAULT 'all' CHECK (lyh_target_audience IN ('all', 'student', 'instructor')),
    lyh_priority INT DEFAULT 1 CHECK (lyh_priority BETWEEN 1 AND 5),
    FOREIGN KEY (lyh_publisher_id) REFERENCES Liyh_User(lyh_user_id)
);
COMMENT ON TABLE Liyh_Announcement IS '系统内的信息发布表';

-- 操作日志表
CREATE TABLE Liyh_AuditLog (
    lyh_log_id SERIAL PRIMARY KEY,
    lyh_user_id VARCHAR(12),
    lyh_operation_type VARCHAR(20) NOT NULL,
    lyh_target_object VARCHAR(50),
    lyh_operation_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    lyh_ip_address VARCHAR(45), -- Supports IPv6
    lyh_result VARCHAR(10) CHECK (lyh_result IN ('SUCCESS', 'FAILED')),
    lyh_details TEXT,
    FOREIGN KEY (lyh_user_id) REFERENCES Liyh_User(lyh_user_id)
);
COMMENT ON TABLE Liyh_AuditLog IS '系统关键操作的审计信息表';


-- =============================================================================
-- 2. 索引创建 (CREATE INDEX)
-- =============================================================================
-- 为外键和常用查询字段创建索引以提升性能

-- User Management
CREATE INDEX IF NOT EXISTS idx_liyh_student_major_id ON Liyh_Student(lyh_major_id);
CREATE INDEX IF NOT EXISTS idx_liyh_student_academic_status ON Liyh_Student(lyh_academic_status);
CREATE INDEX IF NOT EXISTS idx_liyh_instructor_dept_id ON Liyh_Instructor(lyh_dept_id);

-- Course & Teaching
CREATE INDEX IF NOT EXISTS idx_liyh_course_dept_id ON Liyh_Course(lyh_dept_id);
CREATE INDEX IF NOT EXISTS idx_liyh_course_title ON Liyh_Course(lyh_course_title); -- For searching by name
CREATE INDEX IF NOT EXISTS idx_liyh_section_course_sem ON Liyh_Section(lyh_course_id, lyh_semester_id);
CREATE INDEX IF NOT EXISTS idx_liyh_section_classroom_timeslot ON Liyh_Section(lyh_classroom_id, lyh_time_slot_id);
CREATE INDEX IF NOT EXISTS idx_liyh_prereq_course_id ON Liyh_Prerequisite(lyh_course_id);
CREATE INDEX IF NOT EXISTS idx_liyh_curriculum_major_id ON Liyh_Curriculum(lyh_major_id);
CREATE INDEX IF NOT EXISTS idx_liyh_teaches_instructor_id ON Liyh_Teaches(lyh_instructor_id);
CREATE INDEX IF NOT EXISTS idx_liyh_teaches_section_id ON Liyh_Teaches(lyh_section_id);

-- Enrollment & Grades
CREATE INDEX IF NOT EXISTS idx_liyh_takes_section_id ON Liyh_Takes(lyh_section_id);
CREATE INDEX IF NOT EXISTS idx_liyh_takes_student_status ON Liyh_Takes(lyh_student_id, lyh_status);
CREATE INDEX IF NOT EXISTS idx_liyh_takes_grade ON Liyh_Takes(lyh_numeric_grade);

-- System Management
CREATE INDEX IF NOT EXISTS idx_liyh_auditlog_user_time ON Liyh_AuditLog(lyh_user_id, lyh_operation_time);
CREATE INDEX IF NOT EXISTS idx_liyh_announcement_time_target ON Liyh_Announcement(lyh_publish_time DESC, lyh_target_audience);
CREATE INDEX IF NOT EXISTS idx_liyh_evaluation_section_id ON Liyh_Evaluation(lyh_section_id);


-- =============================================================================
-- 3. 视图创建 (CREATE VIEW)
-- =============================================================================
-- 创建视图以简化常用复杂查询

-- 学生详细信息视图
CREATE OR REPLACE VIEW Liyh_v_StudentDetails AS
SELECT
    s.lyh_student_id,
    s.lyh_student_name,
    s.lyh_gender,
    s.lyh_email,
    s.lyh_admission_year,
    s.lyh_academic_status,
    m.lyh_major_name,
    d.lyh_dept_name
FROM
    Liyh_Student s
JOIN
    Liyh_Major m ON s.lyh_major_id = m.lyh_major_id
JOIN
    Liyh_Department d ON m.lyh_dept_id = d.lyh_dept_id;

-- 课程表视图 (教学班详细信息)
CREATE OR REPLACE VIEW Liyh_v_CourseSchedule AS
SELECT
    sec.lyh_section_id,
    c.lyh_course_id,
    c.lyh_course_title,
    c.lyh_credits,
    sem.lyh_semester_id,
    i.lyh_instructor_name,
    ts.lyh_period_desc,
    b.lyh_building_name,
    cr.lyh_room_number,
    sec.lyh_capacity,
    sec.lyh_enrolled_count
FROM
    Liyh_Section sec
JOIN Liyh_Course c ON sec.lyh_course_id = c.lyh_course_id
JOIN Liyh_Semester sem ON sec.lyh_semester_id = sem.lyh_semester_id
LEFT JOIN Liyh_Teaches t ON sec.lyh_section_id = t.lyh_section_id AND t.lyh_role = '主讲'
LEFT JOIN Liyh_Instructor i ON t.lyh_instructor_id = i.lyh_instructor_id
LEFT JOIN Liyh_TimeSlot ts ON sec.lyh_time_slot_id = ts.lyh_time_slot_id
LEFT JOIN Liyh_Classroom cr ON sec.lyh_classroom_id = cr.lyh_classroom_id
LEFT JOIN Liyh_Building b ON cr.lyh_building_id = b.lyh_building_id;

-- 学生成绩单视图
CREATE OR REPLACE VIEW Liyh_v_StudentGrades AS
SELECT
    t.lyh_student_id,
    s.lyh_student_name,
    c.lyh_course_id,
    c.lyh_course_title,
    c.lyh_credits,
    c.lyh_type AS lyh_course_type,
    sem.lyh_semester_id,
    t.lyh_numeric_grade,
    t.lyh_letter_grade,
    t.lyh_grade_point,
    gs.lyh_is_passing
FROM
    Liyh_Takes t
JOIN Liyh_Student s ON t.lyh_student_id = s.lyh_student_id
JOIN Liyh_Section sec ON t.lyh_section_id = sec.lyh_section_id
JOIN Liyh_Course c ON sec.lyh_course_id = c.lyh_course_id
JOIN Liyh_Semester sem ON sec.lyh_semester_id = sem.lyh_semester_id
LEFT JOIN Liyh_GradeScale gs ON t.lyh_letter_grade = gs.lyh_letter_grade
WHERE t.lyh_status = '正常';

-- 教师授课视图
CREATE OR REPLACE VIEW Liyh_v_FacultyCourses AS
SELECT
    i.lyh_instructor_id,
    i.lyh_instructor_name,
    c.lyh_course_id,
    c.lyh_course_title,
    sec.lyh_section_id,
    sem.lyh_semester_id
FROM
    Liyh_Teaches t
JOIN Liyh_Instructor i ON t.lyh_instructor_id = i.lyh_instructor_id
JOIN Liyh_Section sec ON t.lyh_section_id = sec.lyh_section_id
JOIN Liyh_Course c ON sec.lyh_course_id = c.lyh_course_id
JOIN Liyh_Semester sem ON sec.lyh_semester_id = sem.lyh_semester_id;


-- =============================================================================
-- 4. 触发器 (TRIGGERS)
-- =============================================================================
-- 创建触发器以实现自动化业务规则

-- 触发器1: 选课/退课时，自动更新教学班的已选人数
CREATE OR REPLACE FUNCTION Liyh_f_UpdateEnrolledCount()
RETURNS TRIGGER AS $$
BEGIN
    IF (TG_OP = 'INSERT') THEN
        UPDATE Liyh_Section
        SET lyh_enrolled_count = lyh_enrolled_count + 1
        WHERE lyh_section_id = NEW.lyh_section_id;
        RETURN NEW;
    ELSIF (TG_OP = 'DELETE') THEN
        UPDATE Liyh_Section
        SET lyh_enrolled_count = lyh_enrolled_count - 1
        WHERE lyh_section_id = OLD.lyh_section_id;
        RETURN OLD;
    ELSIF (TG_OP = 'UPDATE' AND NEW.lyh_status = '已退课' AND OLD.lyh_status = '正常') THEN
         UPDATE Liyh_Section
        SET lyh_enrolled_count = lyh_enrolled_count - 1
        WHERE lyh_section_id = OLD.lyh_section_id;
        RETURN NEW;
    END IF;
    RETURN NULL;
END;
$$ LANGUAGE plpgsql;

DROP TRIGGER IF EXISTS Liyh_trg_UpdateEnrolledCount ON Liyh_Takes;
CREATE TRIGGER Liyh_trg_UpdateEnrolledCount
AFTER INSERT OR DELETE OR UPDATE ON Liyh_Takes
FOR EACH ROW EXECUTE PROCEDURE Liyh_f_UpdateEnrolledCount();
COMMENT ON TRIGGER Liyh_trg_UpdateEnrolledCount ON Liyh_Takes IS '当学生选课或退课时，自动增减Liyh_Section表中的已选人数';

-- 触发器2: 录入或修改成绩时，自动在审计日志中记录
CREATE OR REPLACE FUNCTION Liyh_f_LogGradeChange()
RETURNS TRIGGER AS $$
DECLARE
    log_details TEXT;
BEGIN
    IF (TG_OP = 'UPDATE' AND OLD.lyh_numeric_grade IS DISTINCT FROM NEW.lyh_numeric_grade) THEN
        log_details := '学生ID: ' || OLD.lyh_student_id ||
                       ', 教学班ID: ' || OLD.lyh_section_id ||
                       ', 成绩从 ' || COALESCE(OLD.lyh_numeric_grade::TEXT, 'NULL') ||
                       ' 修改为 ' || COALESCE(NEW.lyh_numeric_grade::TEXT, 'NULL');

        INSERT INTO Liyh_AuditLog (lyh_user_id, lyh_operation_type, lyh_target_object, lyh_result, lyh_details)
        VALUES (current_setting('my.app_user'), 'UPDATE_GRADE', 'Liyh_Takes:' || OLD.lyh_take_id, 'SUCCESS', log_details);
    END IF;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

DROP TRIGGER IF EXISTS Liyh_trg_LogGradeChange ON Liyh_Takes;
CREATE TRIGGER Liyh_trg_LogGradeChange
AFTER UPDATE ON Liyh_Takes
FOR EACH ROW EXECUTE PROCEDURE Liyh_f_LogGradeChange();
COMMENT ON TRIGGER Liyh_trg_LogGradeChange ON Liyh_Takes IS '当成绩被修改时，自动在审计日志表中创建一条记录';

-- 触发器3: 录入成绩后，如果及格，自动更新学生总学分
CREATE OR REPLACE FUNCTION Liyh_f_UpdateTotalCreditsAfterGrade()
RETURNS TRIGGER AS $$
DECLARE
    v_credits DECIMAL(3,1);
    v_is_passing BOOLEAN;
BEGIN
    -- 仅当成绩从无到有，并且状态正常时才计算
    IF (TG_OP = 'UPDATE' AND OLD.lyh_numeric_grade IS NULL AND NEW.lyh_numeric_grade IS NOT NULL AND NEW.lyh_status = '正常') THEN
        -- 获取课程学分
        SELECT c.lyh_credits INTO v_credits
        FROM Liyh_Section s
        JOIN Liyh_Course c ON s.lyh_course_id = c.lyh_course_id
        WHERE s.lyh_section_id = NEW.lyh_section_id;

        -- 判断是否及格
        SELECT gs.lyh_is_passing INTO v_is_passing
        FROM Liyh_GradeScale gs
        WHERE NEW.lyh_numeric_grade BETWEEN gs.lyh_min_score AND gs.lyh_max_score
        LIMIT 1;

        IF v_is_passing THEN
            UPDATE Liyh_Student
            SET lyh_total_credits = lyh_total_credits + v_credits
            WHERE lyh_student_id = NEW.lyh_student_id;
        END IF;
    END IF;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

DROP TRIGGER IF EXISTS Liyh_trg_UpdateTotalCreditsAfterGrade ON Liyh_Takes;
CREATE TRIGGER Liyh_trg_UpdateTotalCreditsAfterGrade
AFTER UPDATE OF lyh_numeric_grade ON Liyh_Takes
FOR EACH ROW EXECUTE PROCEDURE Liyh_f_UpdateTotalCreditsAfterGrade();
COMMENT ON TRIGGER Liyh_trg_UpdateTotalCreditsAfterGrade ON Liyh_Takes IS '录入及格成绩后，自动为学生增加学分';

-- 触发器4: 录入成绩后，自动更新学生的GPA
CREATE OR REPLACE FUNCTION Liyh_f_UpdateStudentGPAAfterGrade()
RETURNS TRIGGER AS $$
DECLARE
    v_total_points DECIMAL(10, 2) := 0;
    v_total_credits DECIMAL(5, 2) := 0;
    v_gpa DECIMAL(3, 2) := 0;
BEGIN
    -- 当成绩被录入或更新时，重新计算该学生的GPA
    IF (NEW.lyh_numeric_grade IS NOT NULL AND NEW.lyh_status = '正常') THEN
        -- 计算学生的总绩点和总学分
        SELECT
            SUM(c.lyh_credits * t.lyh_grade_point),
            SUM(c.lyh_credits)
        INTO
            v_total_points, v_total_credits
        FROM Liyh_Takes t
        JOIN Liyh_Section s ON t.lyh_section_id = s.lyh_section_id
        JOIN Liyh_Course c ON s.lyh_course_id = c.lyh_course_id
        WHERE t.lyh_student_id = NEW.lyh_student_id
          AND t.lyh_grade_point IS NOT NULL
          AND t.lyh_status = '正常';

        -- 计算GPA
        IF v_total_credits > 0 THEN
            v_gpa = v_total_points / v_total_credits;
        END IF;

        -- 更新学生的GPA
        UPDATE Liyh_Student
        SET lyh_current_gpa = v_gpa
        WHERE lyh_student_id = NEW.lyh_student_id;
    END IF;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

DROP TRIGGER IF EXISTS Liyh_trg_UpdateStudentGPAAfterGrade ON Liyh_Takes;
CREATE TRIGGER Liyh_trg_UpdateStudentGPAAfterGrade
AFTER INSERT OR UPDATE OF lyh_numeric_grade ON Liyh_Takes
FOR EACH ROW EXECUTE PROCEDURE Liyh_f_UpdateStudentGPAAfterGrade();
COMMENT ON TRIGGER Liyh_trg_UpdateStudentGPAAfterGrade ON Liyh_Takes IS '录入或更新成绩后，自动重新计算学生的GPA';


-- =============================================================================
-- 5. 存储过程 (STORED PROCEDURES)
-- =============================================================================
-- 创建存储过程以封装复杂业务逻辑
DROP PROCEDURE IF EXISTS Liyh_sp_EnrollStudent(VARCHAR(12), INT, INOUT TEXT);



-- 存储过程1: 学生选课
CREATE OR REPLACE PROCEDURE Liyh_sp_EnrollStudent(
    p_student_id VARCHAR(12),
    p_section_id INT,
    INOUT status_message TEXT
)
AS
DECLARE
    v_capacity INT;
    v_enrolled INT;
    v_section_status TEXT;
    v_student_status TEXT;
    v_time_slot_id VARCHAR(10);
    v_has_conflict INT;
    v_prereq_course_id VARCHAR(8);
    v_prereq_passed INT;
    v_semester_id VARCHAR(12);
BEGIN
    -- 1. 检查教学班和学生是否存在且状态正常
    SELECT lyh_capacity, lyh_enrolled_count, sem.lyh_is_current, sec.lyh_time_slot_id, sec.lyh_semester_id
    INTO v_capacity, v_enrolled, v_section_status, v_time_slot_id, v_semester_id
    FROM Liyh_Section sec JOIN Liyh_Semester sem ON sec.lyh_semester_id = sem.lyh_semester_id
    WHERE sec.lyh_section_id = p_section_id;

    IF NOT FOUND THEN
        status_message := '错误：教学班不存在。';
        RETURN;
    END IF;

    SELECT lyh_academic_status INTO v_student_status FROM Liyh_Student WHERE lyh_student_id = p_student_id;
    IF v_student_status <> '在读' THEN
        status_message := '错误：学生学籍状态异常，无法选课。';
        RETURN;
    END IF;

    -- 2. 检查课程容量
    IF v_enrolled >= v_capacity THEN
        status_message := '错误：课程容量已满。';
        RETURN;
    END IF;

    -- 3. 检查时间冲突
    SELECT COUNT(*) INTO v_has_conflict
    FROM Liyh_Takes t
    JOIN Liyh_Section s ON t.lyh_section_id = s.lyh_section_id
    WHERE t.lyh_student_id = p_student_id
      AND s.lyh_time_slot_id = v_time_slot_id
      AND s.lyh_semester_id = v_semester_id  -- 同一学期内的时间冲突
      AND t.lyh_status = '正常';

    IF v_has_conflict > 0 THEN
        status_message := '错误：上课时间冲突。';
        RETURN;
    END IF;

    -- 4. 检查先修课程要求（修正版：必须及格才算完成先修）
    FOR v_prereq_course_id IN (
        SELECT p.lyh_prereq_course_id
        FROM Liyh_Prerequisite p
        JOIN Liyh_Section s ON p.lyh_course_id = s.lyh_course_id
        WHERE s.lyh_section_id = p_section_id AND p.lyh_requirement_type = '必须'
    )
    LOOP
        SELECT COUNT(*) INTO v_prereq_passed
        FROM Liyh_Takes t
        JOIN Liyh_Section s ON t.lyh_section_id = s.lyh_section_id
        JOIN Liyh_Course c ON s.lyh_course_id = c.lyh_course_id
        LEFT JOIN Liyh_GradeScale gs ON t.lyh_letter_grade = gs.lyh_letter_grade
        WHERE t.lyh_student_id = p_student_id
          AND c.lyh_course_id = v_prereq_course_id
          AND t.lyh_status = '正常'
          AND COALESCE(gs.lyh_is_passing, FALSE) = TRUE;  -- 必须及格

        IF v_prereq_passed = 0 THEN
            status_message := '错误：未完成先修课程 ' || v_prereq_course_id || ' 或成绩不及格。';
            RETURN;
        END IF;
    END LOOP;

    -- 5. 执行选课
    BEGIN
        INSERT INTO Liyh_Takes (lyh_student_id, lyh_section_id)
        VALUES (p_student_id, p_section_id);
        status_message := '选课成功！';
    EXCEPTION
        WHEN unique_violation THEN
            status_message := '错误：您已选修此课程。';
    END;
END;
/
--COMMENT ON PROCEDURE Liyh_sp_EnrollStudent IS '学生选课（修正版：严格检查先修课程及格要求）';

-- 存储过程2: 计算并更新学生GPA
DROP PROCEDURE IF EXISTS Liyh_sp_CalculateStudentGPA(VARCHAR(12));
CREATE OR REPLACE PROCEDURE Liyh_sp_CalculateStudentGPA(p_student_id VARCHAR(12))
AS
DECLARE
    v_total_points DECIMAL(10, 2) := 0;
    v_total_credits DECIMAL(5, 2) := 0;
    v_gpa DECIMAL(3, 2) := 0;
BEGIN
    SELECT
        SUM(c.lyh_credits * t.lyh_grade_point),
        SUM(c.lyh_credits)
    INTO
        v_total_points, v_total_credits
    FROM Liyh_Takes t
    JOIN Liyh_Section s ON t.lyh_section_id = s.lyh_section_id
    JOIN Liyh_Course c ON s.lyh_course_id = c.lyh_course_id
    WHERE t.lyh_student_id = p_student_id
      AND t.lyh_grade_point IS NOT NULL
      AND t.lyh_status = '正常';

    IF v_total_credits > 0 THEN
        v_gpa = v_total_points / v_total_credits;
    END IF;

    UPDATE Liyh_Student
    SET lyh_current_gpa = v_gpa
    WHERE lyh_student_id = p_student_id;
END;
/

-- 存储过程3: 获取学生课表
DROP PROCEDURE IF EXISTS Liyh_sp_GetStudentTimetable(VARCHAR(12), VARCHAR(12));
CREATE OR REPLACE FUNCTION Liyh_sp_GetStudentTimetable(p_student_id VARCHAR(12), p_semester_id VARCHAR(12))
RETURNS TABLE (
    course_title VARCHAR,
    instructor_name VARCHAR,
    period_desc VARCHAR,
    building_name VARCHAR,
    room_number VARCHAR
) AS $$
BEGIN
    RETURN QUERY
    SELECT
        vs.lyh_course_title,
        vs.lyh_instructor_name,
        vs.lyh_period_desc,
        vs.lyh_building_name,
        vs.lyh_room_number
    FROM Liyh_v_CourseSchedule vs
    JOIN Liyh_Takes t ON vs.lyh_section_id = t.lyh_section_id
    WHERE t.lyh_student_id = p_student_id
      AND vs.lyh_semester_id = p_semester_id
      AND t.lyh_status = '正常';
END;
$$ LANGUAGE plpgsql;
/
-- 存储过程4: 学生成绩按学年统计
DROP PROCEDURE IF EXISTS Liyh_sp_GetStudentYearlyGrades(VARCHAR(12), INT);
CREATE OR REPLACE FUNCTION Liyh_sp_GetStudentYearlyGrades(
    p_student_id VARCHAR(12),
    p_academic_year INT
)
RETURNS TABLE (
    semester VARCHAR,
    course_title VARCHAR,
    credits DECIMAL,
    numeric_grade DECIMAL,
    letter_grade VARCHAR,
    grade_point DECIMAL
) AS $$
BEGIN
    RETURN QUERY
    SELECT
        sem.lyh_semester,
        c.lyh_course_title,
        c.lyh_credits,
        t.lyh_numeric_grade,
        t.lyh_letter_grade,
        t.lyh_grade_point
    FROM Liyh_Takes t
    JOIN Liyh_Section sec ON t.lyh_section_id = sec.lyh_section_id
    JOIN Liyh_Course c ON sec.lyh_course_id = c.lyh_course_id
    JOIN Liyh_Semester sem ON sec.lyh_semester_id = sem.lyh_semester_id
    WHERE t.lyh_student_id = p_student_id
      AND sem.lyh_academic_year = p_academic_year
      AND t.lyh_status = '正常'
    ORDER BY sem.lyh_semester, c.lyh_course_title;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_GetStudentYearlyGrades IS '查询学生指定学年的所有成绩';
/
-- 存储过程5: 学生成绩名次排定
DROP PROCEDURE IF EXISTS Liyh_sp_GetCourseRanking(INT);
CREATE OR REPLACE FUNCTION Liyh_sp_GetCourseRanking(
    p_section_id INT
)
RETURNS TABLE (
    rank INT,
    student_id VARCHAR,
    student_name VARCHAR,
    numeric_grade DECIMAL,
    letter_grade VARCHAR
) AS $$
BEGIN
    RETURN QUERY
    SELECT
        RANK() OVER (ORDER BY t.lyh_numeric_grade DESC NULLS LAST)::INT as rank,
        t.lyh_student_id,
        s.lyh_student_name,
        t.lyh_numeric_grade,
        t.lyh_letter_grade
    FROM Liyh_Takes t
    JOIN Liyh_Student s ON t.lyh_student_id = s.lyh_student_id
    WHERE t.lyh_section_id = p_section_id
      AND t.lyh_status = '正常'
    ORDER BY rank;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_GetCourseRanking IS '获取指定教学班的成绩排名';
/
-- 存储过程6: 班级课程开设查询
DROP PROCEDURE IF EXISTS Liyh_sp_GetMajorCourses(VARCHAR(6), VARCHAR(12));
CREATE OR REPLACE FUNCTION Liyh_sp_GetMajorCourses(
    p_major_id VARCHAR(6),
    p_semester_id VARCHAR(12)
)
RETURNS TABLE (
    course_id VARCHAR,
    course_title VARCHAR,
    credits DECIMAL,
    course_nature VARCHAR,
    section_count BIGINT,
    total_capacity INT
) AS $$
BEGIN
    RETURN QUERY
    SELECT
        c.lyh_course_id,
        c.lyh_course_title,
        c.lyh_credits,
        cur.lyh_course_nature,
        COUNT(DISTINCT sec.lyh_section_id) as section_count,
        SUM(sec.lyh_capacity)::INT as total_capacity
    FROM Liyh_Curriculum cur
    JOIN Liyh_Course c ON cur.lyh_course_id = c.lyh_course_id
    LEFT JOIN Liyh_Section sec ON c.lyh_course_id = sec.lyh_course_id 
        AND sec.lyh_semester_id = p_semester_id
    WHERE cur.lyh_major_id = p_major_id
    GROUP BY c.lyh_course_id, c.lyh_course_title, c.lyh_credits, cur.lyh_course_nature
    ORDER BY cur.lyh_course_nature, c.lyh_course_title;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_GetMajorCourses IS '查询指定专业在某学期的课程开设情况';
 
-- 存储过程7: 按地区统计学生数
DROP PROCEDURE IF EXISTS Liyh_sp_CountStudentsByRegion();
CREATE OR REPLACE FUNCTION Liyh_sp_CountStudentsByRegion()
RETURNS TABLE (
    region VARCHAR,
    student_count BIGINT,
    percentage DECIMAL
) AS $$
DECLARE
    v_total_students INT;
BEGIN
    -- 获取学生总数
    SELECT COUNT(*) INTO v_total_students FROM Liyh_Student WHERE lyh_academic_status = '在读';
    
    RETURN QUERY
    SELECT
        (COALESCE(
            CASE 
                WHEN lyh_address LIKE '%省%' THEN SUBSTRING(lyh_address FROM 1 FOR POSITION('省' IN lyh_address))
                WHEN lyh_address LIKE '%市%' THEN SUBSTRING(lyh_address FROM 1 FOR POSITION('市' IN lyh_address))
                ELSE '其他地区'
            END,
            '未知地区'
        ))::VARCHAR as region,
        COUNT(*) as student_count,
        ROUND(COUNT(*) * 100.0 / v_total_students, 2) as percentage
    FROM Liyh_Student
    WHERE lyh_academic_status = '在读'
    GROUP BY region
    ORDER BY student_count DESC;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_CountStudentsByRegion IS '统计各地区学生数量及占比';


-- 额外视图1: 学生GPA排名视图
CREATE OR REPLACE VIEW Liyh_v_StudentGPARanking AS
SELECT
    s.lyh_student_id,
    s.lyh_student_name,
    m.lyh_major_name,
    s.lyh_grade as grade_level,
    s.lyh_current_gpa,
    s.lyh_total_credits,
    RANK() OVER (PARTITION BY s.lyh_major_id, s.lyh_grade ORDER BY s.lyh_current_gpa DESC) as major_rank,
    RANK() OVER (PARTITION BY s.lyh_grade ORDER BY s.lyh_current_gpa DESC) as grade_rank,
    RANK() OVER (ORDER BY s.lyh_current_gpa DESC) as overall_rank
FROM Liyh_Student s
JOIN Liyh_Major m ON s.lyh_major_id = m.lyh_major_id
WHERE s.lyh_academic_status = '在读' AND s.lyh_current_gpa > 0;
COMMENT ON VIEW Liyh_v_StudentGPARanking IS '学生GPA排名视图（专业内、年级内、全校）';

-- 额外视图2: 课程平均成绩统计视图
CREATE OR REPLACE VIEW Liyh_v_CourseStatistics AS
SELECT
    c.lyh_course_id,
    c.lyh_course_title,
    sem.lyh_semester_id,
    COUNT(DISTINCT t.lyh_student_id) as student_count,
    ROUND(AVG(t.lyh_numeric_grade), 2) as avg_grade,
    ROUND(STDDEV(t.lyh_numeric_grade), 2) as std_deviation,
    MAX(t.lyh_numeric_grade) as max_grade,
    MIN(t.lyh_numeric_grade) as min_grade,
    SUM(CASE WHEN gs.lyh_is_passing THEN 1 ELSE 0 END) as pass_count,
    ROUND(SUM(CASE WHEN gs.lyh_is_passing THEN 1 ELSE 0 END) * 100.0 / COUNT(*), 2) as pass_rate
FROM Liyh_Takes t
JOIN Liyh_Section sec ON t.lyh_section_id = sec.lyh_section_id
JOIN Liyh_Course c ON sec.lyh_course_id = c.lyh_course_id
JOIN Liyh_Semester sem ON sec.lyh_semester_id = sem.lyh_semester_id
LEFT JOIN Liyh_GradeScale gs ON t.lyh_letter_grade = gs.lyh_letter_grade
WHERE t.lyh_status = '正常' AND t.lyh_numeric_grade IS NOT NULL
GROUP BY c.lyh_course_id, c.lyh_course_title, sem.lyh_semester_id;
COMMENT ON VIEW Liyh_v_CourseStatistics IS '课程成绩统计视图（平均分、及格率等）';

-- 额外视图3: 教师任课查询视图
CREATE OR REPLACE VIEW Liyh_v_InstructorWorkload AS
SELECT
    i.lyh_instructor_id,
    i.lyh_instructor_name,
    i.lyh_title,
    sem.lyh_semester_id,
    COUNT(DISTINCT t.lyh_section_id) as section_count,
    SUM(t.lyh_workload) as total_workload,
    STRING_AGG(DISTINCT c.lyh_course_title, ', ' ORDER BY c.lyh_course_title) as courses_taught
FROM Liyh_Instructor i
JOIN Liyh_Teaches t ON i.lyh_instructor_id = t.lyh_instructor_id
JOIN Liyh_Section sec ON t.lyh_section_id = sec.lyh_section_id
JOIN Liyh_Course c ON sec.lyh_course_id = c.lyh_course_id
JOIN Liyh_Semester sem ON sec.lyh_semester_id = sem.lyh_semester_id
WHERE t.lyh_role = '主讲'
GROUP BY i.lyh_instructor_id, i.lyh_instructor_name, i.lyh_title, sem.lyh_semester_id;
COMMENT ON VIEW Liyh_v_InstructorWorkload IS '教师工作量统计视图';


-- =============================================================================
-- 6. 示例数据插入 (INSERT INTO)
-- =============================================================================
-- 插入示例数据用于测试和开发

-- 基础数据
INSERT INTO Liyh_Department (lyh_dept_id, lyh_dept_name, lyh_dept_code) VALUES
('CS01', '计算机学院', 'CS'),
('BS01', '商学院', 'BUS');

INSERT INTO Liyh_Building (lyh_building_id, lyh_building_name, lyh_floors) VALUES
('BLD01', '教一楼', 5),
('BLD02', '实验楼A', 10);

INSERT INTO Liyh_Classroom (lyh_classroom_id, lyh_building_id, lyh_room_number, lyh_capacity, lyh_room_type) VALUES
('CLS001', 'BLD01', '101', 120, '多媒体'),
('CLS002', 'BLD01', '203', 80, '普通教室'),
('CLS003', 'BLD02', '301', 60, '实验室');

INSERT INTO Liyh_TimeSlot (lyh_time_slot_id, lyh_day_of_week, lyh_start_time, lyh_end_time, lyh_period_desc) VALUES
('MON-12', 1, '08:00:00', '09:40:00', '周一第1-2节'),
('WED-34', 3, '10:00:00', '11:40:00', '周三第3-4节'),
('FRI-56', 5, '14:00:00', '15:40:00', '周五第5-6节');

INSERT INTO Liyh_GradeScale (lyh_letter_grade, lyh_min_score, lyh_max_score, lyh_grade_point, lyh_is_passing) VALUES
('A+', 95.00, 100.00, 4.0, TRUE),
('A', 90.00, 94.99, 4.0, TRUE),
('B+', 85.00, 89.99, 3.5, TRUE),
('B', 80.00, 84.99, 3.0, TRUE),
('C', 70.00, 79.99, 2.0, TRUE),
('D', 60.00, 69.99, 1.0, TRUE),
('F', 0.00, 59.99, 0.0, FALSE);

INSERT INTO Liyh_Semester (lyh_semester_id, lyh_academic_year, lyh_semester, lyh_start_date, lyh_end_date, lyh_is_current) VALUES
('2024-Fall', 2024, 'Fall', '2024-09-01', '2025-01-15', TRUE),
('2024-Spring', 2024, 'Spring', '2024-02-20', '2024-07-05', FALSE);

-- 用户、教师、学生、专业
INSERT INTO Liyh_User (lyh_user_id, lyh_username, lyh_password_hash, lyh_role) VALUES
('T001', 'liyh_teacher', 'hash_placeholder', 'instructor'),
('2023001', 'zhang_student', 'hash_placeholder', 'student'),
('2023002', 'wang_student', 'hash_placeholder', 'student'),
('A001', 'admin_user', 'hash_placeholder', 'admin');

INSERT INTO Liyh_Instructor (lyh_instructor_id, lyh_instructor_name, lyh_title, lyh_dept_id, lyh_email) VALUES
('T001', '李老师', '教授', 'CS01', 'teacher.li@example.com');

UPDATE Liyh_Department SET lyh_dean_id = 'T001' WHERE lyh_dept_id = 'CS01';

INSERT INTO Liyh_Major (lyh_major_id, lyh_major_name, lyh_dept_id, lyh_coordinator_id, lyh_required_credits) VALUES
('CS0101', '计算机科学与技术', 'CS01', 'T001', 160),
('BS0101', '市场营销', 'BS01', NULL, 150);

INSERT INTO Liyh_Student (lyh_student_id, lyh_student_name, lyh_major_id, lyh_admission_year, lyh_grade) VALUES
('2023001', '张三', 'CS0101', 2023, 1),
('2023002', '王五', 'CS0101', 2023, 1);

-- 更新学生地址信息
UPDATE Liyh_Student SET lyh_address = '浙江省杭州市西湖区' WHERE lyh_student_id = '2023001';
UPDATE Liyh_Student SET lyh_address = '江苏省南京市玄武区' WHERE lyh_student_id = '2023002';

-- 添加更多学生数据以支持统计功能
INSERT INTO Liyh_User (lyh_user_id, lyh_username, lyh_password_hash, lyh_role) VALUES
('2023003', 'li_student', 'hash_placeholder', 'student'),
('2023004', 'zhao_student', 'hash_placeholder', 'student'),
('2023005', 'chen_student', 'hash_placeholder', 'student');

INSERT INTO Liyh_Student (lyh_student_id, lyh_student_name, lyh_major_id, lyh_admission_year, lyh_grade, lyh_address) VALUES
('2023003', '李四', 'CS0101', 2023, 1, '浙江省宁波市海曙区'),
('2023004', '赵六', 'BS0101', 2023, 1, '上海市浦东新区'),
('2023005', '陈七', 'CS0101', 2023, 1, '浙江省杭州市拱墅区');

-- 添加更多教师
INSERT INTO Liyh_User (lyh_user_id, lyh_username, lyh_password_hash, lyh_role) VALUES
('T002', 'wang_teacher', 'hash_placeholder', 'instructor');

INSERT INTO Liyh_Instructor (lyh_instructor_id, lyh_instructor_name, lyh_title, lyh_dept_id, lyh_email) VALUES
('T002', '王老师', '副教授', 'CS01', 'teacher.wang@example.com');

-- 课程和教学班
INSERT INTO Liyh_Course (lyh_course_id, lyh_course_title, lyh_credits, lyh_type, lyh_dept_id) VALUES
('CS101', '数据库系统', 4.0, '必修', 'CS01'),
('CS102', '数据结构', 3.0, '必修', 'CS01'),
('BUS101', '市场营销学', 3.0, '必修', 'BS01');

INSERT INTO Liyh_Prerequisite(lyh_course_id, lyh_prereq_course_id) VALUES ('CS101', 'CS102');

INSERT INTO Liyh_Section (lyh_course_id, lyh_semester_id, lyh_section_number, lyh_capacity, lyh_time_slot_id, lyh_classroom_id) VALUES
('CS101', '2024-Fall', '01', 100, 'MON-12', 'CLS001'),
('CS102', '2024-Fall', '01', 100, 'WED-34', 'CLS001'),
('CS101', '2024-Fall', '02', 50, 'FRI-56', 'CLS002');

INSERT INTO Liyh_Teaches (lyh_instructor_id, lyh_section_id) VALUES
('T001', 1),
('T001', 2),
('T001', 3);

-- 选课和成绩
INSERT INTO Liyh_Takes (lyh_student_id, lyh_section_id) VALUES
('2023001', 2); -- 张三 选 数据结构
INSERT INTO Liyh_Takes (lyh_student_id, lyh_section_id) VALUES
('2023002', 2); -- 王五 选 数据结构

-- 录入成绩
-- 假设教务员'A001'在操作
SET my.app_user = 'A001';
UPDATE Liyh_Takes
SET lyh_numeric_grade = 92.5, lyh_letter_grade = 'A'
WHERE lyh_student_id = '2023001' AND lyh_section_id = 2;

UPDATE Liyh_Takes
SET lyh_numeric_grade = 58, lyh_letter_grade = 'F'
WHERE lyh_student_id = '2023002' AND lyh_section_id = 2;


-- =============================================================================
-- 7. 查询与统计 (SELECT)
-- =============================================================================
-- 演示如何使用数据库进行查询和统计

-- 1. 查询学生'张三'(ID: 2023001)的详细信息 (使用视图)
SELECT * FROM Liyh_v_StudentDetails WHERE lyh_student_id = '2023001';

-- 2. 查询2024-Fall学期的所有课程安排 (使用视图)
SELECT * FROM Liyh_v_CourseSchedule WHERE lyh_semester_id = '2024-Fall';

-- 3. 查询学生'张三'(ID: 2023001)的成绩单 (使用视图)
SELECT * FROM Liyh_v_StudentGrades WHERE lyh_student_id = '2023001';

-- 4. 学生 '张三' 尝试选修 '数据库系统' (CS101) 的 01 班 (section_id=1) (使用存储过程)
-- 预期失败，因为先修课程'数据结构'成绩刚录入，但此存储过程版本未计算GPA和学分
DO $$
DECLARE
    msg TEXT;
BEGIN
    PERFORM Liyh_sp_EnrollStudent('2023001', 1, msg);
    RAISE NOTICE '%', msg;
END $$;
-- 预期成功，因为已修完数据结构，虽然不及格但这里只判断了是否修过
DO $$
DECLARE
    msg TEXT;
BEGIN
    PERFORM Liyh_sp_EnrollStudent('2023002', 1, msg);
    RAISE NOTICE '%', msg;
END $$;


-- 5. 更新学生'张三'的GPA和总学分 (使用存储过程)
-- 首先更新绩点
UPDATE Liyh_Takes t SET lyh_grade_point = gs.lyh_grade_point
FROM Liyh_GradeScale gs WHERE t.lyh_letter_grade = gs.lyh_letter_grade AND t.lyh_student_id = '2023001';
-- 调用存储过程
CALL Liyh_sp_CalculateStudentGPA('2023001');
-- 查看结果
SELECT lyh_current_gpa, lyh_total_credits FROM Liyh_Student WHERE lyh_student_id = '2023001';

-- 6. 获取学生'张三'在2024-Fall的课表
SELECT * FROM Liyh_sp_GetStudentTimetable('2023001', '2024-Fall');

-- 7. 统计'数据结构'(course_id=CS102)在2024-Fall学期的平均分
SELECT
    c.lyh_course_title,
    AVG(t.lyh_numeric_grade) as average_grade
FROM Liyh_Takes t
JOIN Liyh_Section s ON t.lyh_section_id = s.lyh_section_id
JOIN Liyh_Course c ON s.lyh_course_id = c.lyh_course_id
WHERE s.lyh_course_id = 'CS102' AND s.lyh_semester_id = '2024-Fall'
GROUP BY c.lyh_course_title;

-- 8. 查看成绩修改日志
SELECT * FROM Liyh_AuditLog WHERE lyh_operation_type = 'UPDATE_GRADE';

-- 9. 学生成绩按学年统计 (使用新增存储过程)
SELECT * FROM Liyh_sp_GetStudentYearlyGrades('2023001', 2024);

-- 10. 查看某教学班的成绩排名 (使用新增存储过程)
SELECT * FROM Liyh_sp_GetCourseRanking(2); -- 数据结构课程的排名

-- 11. 查询专业课程开设情况 (使用新增存储过程)
SELECT * FROM Liyh_sp_GetMajorCourses('CS0101', '2024-Fall');

-- 12. 按地区统计学生数 (使用新增存储过程)
SELECT * FROM Liyh_sp_CountStudentsByRegion();

-- 13. 查看学生GPA排名 (使用新增视图)
SELECT * FROM Liyh_v_StudentGPARanking WHERE grade_level = 1;

-- 14. 查看课程统计信息 (使用新增视图)
SELECT * FROM Liyh_v_CourseStatistics WHERE lyh_semester_id = '2024-Fall';

-- 15. 查看教师工作量 (使用新增视图)
SELECT * FROM Liyh_v_InstructorWorkload WHERE lyh_semester_id = '2024-Fall';

-- 16. 测试修正后的选课逻辑
-- 王五(2023002)成绩不及格，应该无法选修数据库系统
DO $$
DECLARE
    msg TEXT;
BEGIN
    PERFORM Liyh_sp_EnrollStudent('2023002', 1, msg);
    RAISE NOTICE '选课结果: %', msg;
END $$;

-- 17. 为更多学生添加选课和成绩记录
INSERT INTO Liyh_Takes (lyh_student_id, lyh_section_id) VALUES
('2023003', 2), -- 李四选数据结构
('2023004', 2), -- 赵六选数据结构  
('2023005', 2); -- 陈七选数据结构

SET my.app_user = 'A001'; -- 假设是管理员 'A001' 在操作

-- 录入更多成绩
UPDATE Liyh_Takes SET lyh_numeric_grade = 88, lyh_letter_grade = 'B+' WHERE lyh_student_id = '2023003' AND lyh_section_id = 2;
UPDATE Liyh_Takes SET lyh_numeric_grade = 76, lyh_letter_grade = 'C' WHERE lyh_student_id = '2023004' AND lyh_section_id = 2;
UPDATE Liyh_Takes SET lyh_numeric_grade = 95, lyh_letter_grade = 'A+' WHERE lyh_student_id = '2023005' AND lyh_section_id = 2;

-- 更新所有学生的绩点
UPDATE Liyh_Takes t SET lyh_grade_point = gs.lyh_grade_point
FROM Liyh_GradeScale gs WHERE t.lyh_letter_grade = gs.lyh_letter_grade;

-- 计算所有学生的GPA
CALL Liyh_sp_CalculateStudentGPA('2023001');
CALL Liyh_sp_CalculateStudentGPA('2023002');
CALL Liyh_sp_CalculateStudentGPA('2023003');
CALL Liyh_sp_CalculateStudentGPA('2023004');
CALL Liyh_sp_CalculateStudentGPA('2023005');

-- 18. 再次查看更新后的排名和统计
SELECT * FROM Liyh_sp_GetCourseRanking(2); -- 数据结构课程的最新排名
SELECT * FROM Liyh_v_StudentGPARanking ORDER BY overall_rank; -- 全校GPA排名
SELECT * FROM Liyh_v_CourseStatistics; -- 课程统计信息

-- ======================= END OF SCRIPT ======================= 

-- =============================================================================
-- 8. 模糊搜索功能扩展
-- =============================================================================
-- 实现各种模糊搜索功能，提升系统的用户体验

-- -----------------------------------------------------
-- 8.1 学生姓名模糊搜索
-- -----------------------------------------------------
-- 支持：部分姓名匹配
CREATE OR REPLACE FUNCTION Liyh_sp_SearchStudentsByName(
    p_search_term VARCHAR(50),
    p_limit INT DEFAULT 50
)
RETURNS TABLE (
    student_id VARCHAR,
    student_name VARCHAR,
    gender VARCHAR,
    major_name VARCHAR,
    grade INT,
    academic_status VARCHAR,
    match_type VARCHAR
) AS $$
BEGIN
    RETURN QUERY
    WITH search_results AS (
        -- 完全匹配
        SELECT 
            s.lyh_student_id as student_id,
            s.lyh_student_name as student_name,
            s.lyh_gender as gender,
            m.lyh_major_name as major_name,
            s.lyh_grade as grade,
            s.lyh_academic_status as academic_status,
            '完全匹配'::VARCHAR as match_type,
            1 as priority
        FROM Liyh_Student s
        JOIN Liyh_Major m ON s.lyh_major_id = m.lyh_major_id
        WHERE LOWER(s.lyh_student_name) = LOWER(p_search_term)
        
        UNION ALL
        
        -- 部分匹配（包含搜索词）
        SELECT 
            s.lyh_student_id as student_id,
            s.lyh_student_name as student_name,
            s.lyh_gender as gender,
            m.lyh_major_name as major_name,
            s.lyh_grade as grade,
            s.lyh_academic_status as academic_status,
            '部分匹配'::VARCHAR as match_type,
            2 as priority
        FROM Liyh_Student s
        JOIN Liyh_Major m ON s.lyh_major_id = m.lyh_major_id
        WHERE LOWER(s.lyh_student_name) LIKE '%' || LOWER(p_search_term) || '%'
          AND LOWER(s.lyh_student_name) != LOWER(p_search_term)
    )
    SELECT DISTINCT ON (search_results.student_id)
        search_results.student_id,
        search_results.student_name,
        search_results.gender,
        search_results.major_name,
        search_results.grade,
        search_results.academic_status,
        search_results.match_type
    FROM search_results
    ORDER BY search_results.student_id, search_results.priority
    LIMIT p_limit;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_SearchStudentsByName IS '学生姓名模糊搜索，支持完全匹配和部分匹配';

-- -----------------------------------------------------
-- 8.2 课程名称智能搜索
-- -----------------------------------------------------
CREATE OR REPLACE FUNCTION Liyh_sp_SearchCoursesByKeyword(
    p_keyword VARCHAR(100),
    p_dept_id VARCHAR(4) DEFAULT NULL,
    p_course_type VARCHAR(10) DEFAULT NULL,
    p_limit INT DEFAULT 50
)
RETURNS TABLE (
    course_id VARCHAR,
    course_title VARCHAR,
    credits DECIMAL,
    type VARCHAR,
    dept_name VARCHAR,
    description TEXT,
    match_score FLOAT
) AS $$
BEGIN
    RETURN QUERY
    WITH course_matches AS (
        SELECT 
            c.lyh_course_id as course_id,
            c.lyh_course_title as course_title,
            c.lyh_credits as credits,
            c.lyh_type as type,
            d.lyh_dept_name as dept_name,
            c.lyh_description as description,
            -- 计算匹配分数
            CASE 
                WHEN LOWER(c.lyh_course_title) = LOWER(p_keyword) THEN 1.0::FLOAT
                WHEN LOWER(c.lyh_course_title) LIKE LOWER(p_keyword) || '%' THEN 0.8::FLOAT
                WHEN LOWER(c.lyh_course_title) LIKE '%' || LOWER(p_keyword) || '%' THEN 0.6::FLOAT
                WHEN c.lyh_description IS NOT NULL AND LOWER(c.lyh_description) LIKE '%' || LOWER(p_keyword) || '%' THEN 0.4::FLOAT
                ELSE 0.0::FLOAT -- 移除了 similarity
            END as match_score
        FROM Liyh_Course c
        JOIN Liyh_Department d ON c.lyh_dept_id = d.lyh_dept_id
        WHERE c.lyh_is_active = TRUE
          AND (p_dept_id IS NULL OR c.lyh_dept_id = p_dept_id)
          AND (p_course_type IS NULL OR c.lyh_type = p_course_type)
          AND (
              LOWER(c.lyh_course_title) LIKE '%' || LOWER(p_keyword) || '%'
              OR (c.lyh_description IS NOT NULL AND LOWER(c.lyh_description) LIKE '%' || LOWER(p_keyword) || '%')
          )
    )
    SELECT 
        cm.course_id,
        cm.course_title,
        cm.credits,
        cm.type,
        cm.dept_name,
        cm.description,
        cm.match_score
    FROM course_matches cm
    WHERE cm.match_score > 0
    ORDER BY cm.match_score DESC, cm.course_title
    LIMIT p_limit;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_SearchCoursesByKeyword IS '课程智能搜索，支持按名称、描述、院系、类型等条件搜索';

-- -----------------------------------------------------
-- 8.3 教师多维度搜索
-- -----------------------------------------------------
CREATE OR REPLACE FUNCTION Liyh_sp_SearchInstructors(
    p_search_term VARCHAR(100) DEFAULT NULL,
    p_title VARCHAR(20) DEFAULT NULL,
    p_dept_id VARCHAR(4) DEFAULT NULL,
    p_research_area VARCHAR(100) DEFAULT NULL,
    p_limit INT DEFAULT 50
)
RETURNS TABLE (
    instructor_id VARCHAR,
    instructor_name VARCHAR,
    title VARCHAR,
    education VARCHAR,
    dept_name VARCHAR,
    research_area VARCHAR,
    email VARCHAR,
    office_location VARCHAR
) AS $$
BEGIN
    RETURN QUERY
    SELECT 
        i.lyh_instructor_id AS instructor_id,
        i.lyh_instructor_name AS instructor_name,
        i.lyh_title AS title,
        i.lyh_education AS education,
        d.lyh_dept_name AS dept_name,
        i.lyh_research_area AS research_area,
        i.lyh_email AS email,
        i.lyh_office_location AS office_location
    FROM Liyh_Instructor i
    JOIN Liyh_Department d ON i.lyh_dept_id = d.lyh_dept_id
    WHERE (
        -- 姓名搜索
        p_search_term IS NULL 
        OR LOWER(i.lyh_instructor_name) LIKE '%' || LOWER(p_search_term) || '%'
    )
    AND (p_title IS NULL OR i.lyh_title = p_title)
    AND (p_dept_id IS NULL OR i.lyh_dept_id = p_dept_id)
    AND (
        -- 研究方向搜索
        p_research_area IS NULL 
        OR i.lyh_research_area IS NULL
        OR LOWER(i.lyh_research_area) LIKE '%' || LOWER(p_research_area) || '%'
    )
    ORDER BY 
        CASE 
            WHEN p_search_term IS NOT NULL AND LOWER(i.lyh_instructor_name) = LOWER(p_search_term) THEN 0
            WHEN p_search_term IS NOT NULL AND LOWER(i.lyh_instructor_name) LIKE LOWER(p_search_term) || '%' THEN 1
            WHEN p_search_term IS NOT NULL AND LOWER(i.lyh_instructor_name) LIKE '%' || LOWER(p_search_term) || '%' THEN 2
            ELSE 3
        END,
        i.lyh_instructor_name
    LIMIT p_limit;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_SearchInstructors IS '教师多维度搜索，支持按姓名、职称、院系、研究方向等条件搜索';

-- -----------------------------------------------------
-- 8.4 公告全文搜索
-- -----------------------------------------------------
CREATE OR REPLACE FUNCTION Liyh_sp_SearchAnnouncements(
    p_keyword VARCHAR(200),
    p_target_audience VARCHAR(20) DEFAULT NULL,
    p_date_from DATE DEFAULT NULL,
    p_date_to DATE DEFAULT NULL,
    p_limit INT DEFAULT 50
)
RETURNS TABLE (
    announce_id INT,
    title VARCHAR,
    content TEXT,
    publisher_name VARCHAR,
    publish_time TIMESTAMP,
    target_audience VARCHAR,
    priority INT,
    relevance_score FLOAT
) AS $$
BEGIN
    RETURN QUERY
    WITH announcement_scores AS (
        SELECT 
            a.lyh_announce_id as announce_id,
            a.lyh_title as title,
            a.lyh_content as content,
            u.lyh_username as publisher_name,
            a.lyh_publish_time as publish_time,
            a.lyh_target_audience as target_audience,
            a.lyh_priority as priority,
            -- 计算相关性分数
            (
                (CASE WHEN LOWER(a.lyh_title) LIKE '%' || LOWER(p_keyword) || '%' THEN 0.5 ELSE 0 END) +
                (CASE WHEN LOWER(a.lyh_content) LIKE '%' || LOWER(p_keyword) || '%' THEN 0.3 ELSE 0 END) +
                (CASE WHEN a.lyh_priority >= 3 THEN 0.2 ELSE 0 END)
            )::FLOAT as relevance_score
        FROM Liyh_Announcement a
        JOIN Liyh_User u ON a.lyh_publisher_id = u.lyh_user_id
        WHERE (
            LOWER(a.lyh_title) LIKE '%' || LOWER(p_keyword) || '%'
            OR LOWER(a.lyh_content) LIKE '%' || LOWER(p_keyword) || '%'
        )
        AND (p_target_audience IS NULL OR a.lyh_target_audience = p_target_audience)
        AND (p_date_from IS NULL OR a.lyh_publish_time >= p_date_from)
        AND (p_date_to IS NULL OR a.lyh_publish_time <= p_date_to)
        AND (a.lyh_expiry_date IS NULL OR a.lyh_expiry_date >= CURRENT_DATE)
    )
    SELECT 
        ans.announce_id,
        ans.title,
        ans.content,
        ans.publisher_name,
        ans.publish_time,
        ans.target_audience,
        ans.priority,
        ans.relevance_score
    FROM announcement_scores ans
    WHERE ans.relevance_score > 0
    ORDER BY ans.relevance_score DESC, ans.publish_time DESC
    LIMIT p_limit;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_SearchAnnouncements IS '公告全文搜索，支持按关键词、受众、时间范围等条件搜索';

-- -----------------------------------------------------
-- 8.5 综合搜索功能
-- -----------------------------------------------------
CREATE OR REPLACE FUNCTION Liyh_sp_GlobalSearch(
    p_search_term VARCHAR(200),
    p_search_type VARCHAR(20) DEFAULT 'all', -- 'all', 'student', 'course', 'instructor', 'announcement'
    p_limit INT DEFAULT 20
)
RETURNS TABLE (
    result_type VARCHAR,
    result_id VARCHAR,
    result_title VARCHAR,
    result_description TEXT,
    match_score FLOAT
) AS $$
BEGIN
    RETURN QUERY
    WITH all_results AS (
        -- 搜索学生
        SELECT 
            '学生'::VARCHAR as result_type,
            s.student_id as result_id,
            (s.student_name || ' (' || s.major_name || ')')::VARCHAR as result_title,
            '年级: ' || s.grade || ', 状态: ' || s.academic_status as result_description,
            0.8::FLOAT as match_score
        FROM Liyh_sp_SearchStudentsByName(p_search_term, p_limit) s
        WHERE p_search_type IN ('all', 'student')
        
        UNION ALL
        
        -- 搜索课程
        SELECT 
            '课程'::VARCHAR as result_type,
            c.course_id as result_id,
            (c.course_title || ' (' || c.credits || '学分)')::VARCHAR as result_title,
            COALESCE(c.description, c.type || ' - ' || c.dept_name) as result_description,
            c.match_score
        FROM Liyh_sp_SearchCoursesByKeyword(p_search_term, NULL, NULL, p_limit) c
        WHERE p_search_type IN ('all', 'course')
        
        UNION ALL
        
        -- 搜索教师
        SELECT 
            '教师'::VARCHAR as result_type,
            i.instructor_id as result_id,
            (i.instructor_name || ' (' || i.title || ')')::VARCHAR as result_title,
            i.dept_name || COALESCE(' - ' || i.research_area, '') as result_description,
            0.7::FLOAT as match_score
        FROM Liyh_sp_SearchInstructors(p_search_term, NULL, NULL, NULL, p_limit) i
        WHERE p_search_type IN ('all', 'instructor')
        
        UNION ALL
        
        -- 搜索公告
        SELECT 
            '公告'::VARCHAR as result_type,
            a.announce_id::VARCHAR as result_id,
            a.title as result_title,
            SUBSTRING(a.content FROM 1 FOR 200) || '...' as result_description,
            a.relevance_score as match_score
        FROM Liyh_sp_SearchAnnouncements(p_search_term, NULL, NULL, NULL, p_limit) a
        WHERE p_search_type IN ('all', 'announcement')
    )
    SELECT 
        ar.result_type,
        ar.result_id,
        ar.result_title,
        ar.result_description,
        ar.match_score
    FROM all_results ar
    ORDER BY ar.match_score DESC, ar.result_type, ar.result_title
    LIMIT p_limit;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_GlobalSearch IS '全局综合搜索，可搜索学生、课程、教师、公告等所有内容';

-- -----------------------------------------------------
-- 8.6 搜索历史记录
-- -----------------------------------------------------
-- 创建搜索历史表
CREATE TABLE IF NOT EXISTS Liyh_SearchHistory (
    lyh_search_id SERIAL PRIMARY KEY,
    lyh_user_id VARCHAR(12),
    lyh_search_term VARCHAR(200) NOT NULL,
    lyh_search_type VARCHAR(20),
    lyh_search_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    lyh_result_count INT,
    FOREIGN KEY (lyh_user_id) REFERENCES Liyh_User(lyh_user_id)
);
--COMMENT ON TABLE Liyh_SearchHistory IS '用户搜索历史记录表';

-- 创建索引以提升搜索性能
CREATE INDEX IF NOT EXISTS idx_liyh_search_history_user ON Liyh_SearchHistory(lyh_user_id, lyh_search_time DESC);
CREATE INDEX IF NOT EXISTS idx_liyh_search_history_term ON Liyh_SearchHistory(lyh_search_term);

-- 记录搜索历史的存储过程
CREATE OR REPLACE PROCEDURE Liyh_sp_LogSearchHistory(
    p_user_id VARCHAR(12),
    p_search_term VARCHAR(200),
    p_search_type VARCHAR(20),
    p_result_count INT
)
AS
BEGIN
    INSERT INTO Liyh_SearchHistory (lyh_user_id, lyh_search_term, lyh_search_type, lyh_result_count)
    VALUES (p_user_id, p_search_term, p_search_type, p_result_count);
END;
/

-- 获取热门搜索词
CREATE OR REPLACE FUNCTION Liyh_sp_GetPopularSearchTerms(
    p_days INT DEFAULT 7,
    p_limit INT DEFAULT 10
)
RETURNS TABLE (
    search_term VARCHAR,
    search_count BIGINT,
    avg_result_count FLOAT
) AS $$
BEGIN
    RETURN QUERY
    SELECT 
        lyh_search_term::VARCHAR,
        COUNT(*)::BIGINT as search_count,
        AVG(lyh_result_count)::FLOAT as avg_result_count
    FROM Liyh_SearchHistory
    WHERE lyh_search_time >= CURRENT_TIMESTAMP - INTERVAL '1 day' * p_days
    GROUP BY lyh_search_term
    ORDER BY search_count DESC
    LIMIT p_limit;
END;
$$ LANGUAGE plpgsql;
--COMMENT ON FUNCTION Liyh_sp_GetPopularSearchTerms IS '获取指定天数内的热门搜索词';

-- -----------------------------------------------------
-- 8.7 创建必要的扩展（如果数据库支持）
-- -----------------------------------------------------
-- 启用模糊匹配扩展（PostgreSQL/GaussDB）
-- 注意：需要数据库管理员权限
-- CREATE EXTENSION IF NOT EXISTS pg_trgm;
-- CREATE EXTENSION IF NOT EXISTS fuzzystrmatch;

-- =============================================================================
-- 9. 模糊搜索功能测试示例
-- =============================================================================
-- 演示如何使用模糊搜索功能

-- 9.1 测试学生姓名搜索
-- 搜索姓"张"的学生
SELECT * FROM Liyh_sp_SearchStudentsByName('张', 10);

-- 搜索名字包含"三"的学生
SELECT * FROM Liyh_sp_SearchStudentsByName('三', 10);

-- 9.2 测试课程搜索
-- 搜索包含"数据"的课程
SELECT * FROM Liyh_sp_SearchCoursesByKeyword('数据', NULL, NULL, 10);

-- 搜索计算机学院的必修课
SELECT * FROM Liyh_sp_SearchCoursesByKeyword('', 'CS01', '必修', 10);

-- 9.3 测试教师搜索
-- 搜索姓"李"的教师
SELECT * FROM Liyh_sp_SearchInstructors('李', NULL, NULL, NULL, 10);

-- 搜索所有教授
SELECT * FROM Liyh_sp_SearchInstructors(NULL, '教授', NULL, NULL, 10);

-- 9.4 测试公告搜索
-- 首先插入一些测试公告
INSERT INTO Liyh_Announcement (lyh_title, lyh_content, lyh_publisher_id, lyh_target_audience) VALUES
('2024年秋季学期选课通知', '各位同学请注意，2024年秋季学期选课将于下周一开始...', 'A001', 'student'),
('关于期末考试安排的通知', '期末考试将于第16周开始，请同学们做好准备...', 'A001', 'all'),
('教师培训通知', '本学期教师培训将于本周五下午2点在教一楼301举行...', 'A001', 'instructor');

-- 搜索包含"选课"的公告
SELECT * FROM Liyh_sp_SearchAnnouncements('选课', NULL, NULL, NULL, 10);

-- 搜索面向学生的公告
SELECT * FROM Liyh_sp_SearchAnnouncements('', 'student', NULL, NULL, 10);

-- 9.5 测试全局搜索
-- 搜索"数据"相关的所有内容
SELECT * FROM Liyh_sp_GlobalSearch('数据', 'all', 20);

-- 只搜索学生信息中的"王"
SELECT * FROM Liyh_sp_GlobalSearch('王', 'student', 10);

-- 9.6 记录搜索历史
CALL Liyh_sp_LogSearchHistory('2023001', '数据库', 'course', 3);
CALL Liyh_sp_LogSearchHistory('2023002', '李老师', 'instructor', 1);
CALL Liyh_sp_LogSearchHistory('2023001', '选课', 'announcement', 2);

-- 查看热门搜索词
SELECT * FROM Liyh_sp_GetPopularSearchTerms(7, 5);

-- =============================================================================
-- 10. 性能优化建议
-- =============================================================================
-- 模糊搜索性能优化建议

/*
性能优化建议：

1. 索引优化：
   - 为经常搜索的字段创建索引
   - 使用GIN索引支持全文搜索
   - 考虑使用部分索引减少索引大小

2. 查询优化：
   - 限制返回结果数量
   - 使用分页避免一次返回太多数据
   - 对于复杂搜索，考虑使用物化视图

3. 缓存策略：
   - 缓存热门搜索结果
   - 使用Redis等缓存中间件
   - 定期更新缓存数据

4. 数据库配置：
   - 调整shared_buffers和work_mem参数
   - 启用查询计划缓存
   - 定期执行VACUUM和ANALYZE

5. 应用层优化：
   - 实现搜索防抖，避免频繁查询
   - 使用异步搜索提升用户体验
   - 实现搜索建议和自动完成功能
*/

-- ======================= END OF FUZZY SEARCH IMPLEMENTATION ======================= 