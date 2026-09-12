CREATE TABLE IF NOT EXISTS sys_user (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    username VARCHAR(64) NOT NULL UNIQUE,
    real_name VARCHAR(64) NOT NULL,
    password_sm3 VARCHAR(128) NOT NULL,
    mobile VARCHAR(32),
    email VARCHAR(128),
    team_id BIGINT,
    member_id BIGINT,
    status VARCHAR(32) NOT NULL,
    must_change_password BOOLEAN NOT NULL DEFAULT TRUE,
    password_expire_at TIMESTAMP NULL,
    last_login_at TIMESTAMP NULL,
    created_at TIMESTAMP NOT NULL,
    updated_at TIMESTAMP NOT NULL
);

CREATE TABLE IF NOT EXISTS sys_role (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    role_code VARCHAR(64) NOT NULL UNIQUE,
    role_name VARCHAR(64) NOT NULL,
    description VARCHAR(255)
);

CREATE TABLE IF NOT EXISTS sys_user_role (
    user_id BIGINT NOT NULL,
    role_id BIGINT NOT NULL,
    PRIMARY KEY (user_id, role_id),
    CONSTRAINT fk_sys_user_role_user FOREIGN KEY (user_id) REFERENCES sys_user(id),
    CONSTRAINT fk_sys_user_role_role FOREIGN KEY (role_id) REFERENCES sys_role(id)
);

CREATE TABLE IF NOT EXISTS team (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    team_name VARCHAR(100) NOT NULL,
    department_name VARCHAR(100) NOT NULL,
    manager_user_id BIGINT,
    description VARCHAR(255),
    status VARCHAR(32) NOT NULL,
    created_at TIMESTAMP NOT NULL,
    updated_at TIMESTAMP NOT NULL,
    CONSTRAINT fk_team_manager FOREIGN KEY (manager_user_id) REFERENCES sys_user(id)
);

CREATE TABLE IF NOT EXISTS team_member (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    name VARCHAR(64) NOT NULL,
    team_id BIGINT,
    position VARCHAR(100),
    job_level VARCHAR(32),
    phone VARCHAR(32),
    email VARCHAR(128),
    status VARCHAR(32) NOT NULL,
    entry_date DATE,
    CONSTRAINT fk_team_member_team FOREIGN KEY (team_id) REFERENCES team(id)
);

CREATE TABLE IF NOT EXISTS project (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    project_name VARCHAR(120) NOT NULL,
    owner_member_id BIGINT,
    status VARCHAR(32) NOT NULL,
    priority VARCHAR(32) NOT NULL,
    start_date DATE NOT NULL,
    end_date DATE,
    milestone VARCHAR(255),
    description VARCHAR(500),
    CONSTRAINT fk_project_owner FOREIGN KEY (owner_member_id) REFERENCES team_member(id)
);

CREATE TABLE IF NOT EXISTS weekly_task (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    title VARCHAR(200) NOT NULL,
    project_id BIGINT,
    assignee_member_id BIGINT NOT NULL,
    priority VARCHAR(32) NOT NULL,
    deadline DATE,
    status VARCHAR(32) NOT NULL,
    progress_rate INT NOT NULL DEFAULT 0,
    week_no VARCHAR(32) NOT NULL,
    plan_start_date DATE,
    plan_end_date DATE,
    issue_desc VARCHAR(500),
    CONSTRAINT fk_weekly_task_project FOREIGN KEY (project_id) REFERENCES project(id),
    CONSTRAINT fk_weekly_task_assignee FOREIGN KEY (assignee_member_id) REFERENCES team_member(id)
);

CREATE TABLE IF NOT EXISTS task_progress (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    task_id BIGINT NOT NULL,
    progress_rate INT NOT NULL,
    issue_desc VARCHAR(500),
    comment VARCHAR(500) NOT NULL,
    update_time TIMESTAMP NOT NULL,
    CONSTRAINT fk_task_progress_task FOREIGN KEY (task_id) REFERENCES weekly_task(id)
);

CREATE TABLE IF NOT EXISTS weekly_report (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    member_id BIGINT NOT NULL,
    week_no VARCHAR(32) NOT NULL,
    summary VARCHAR(1000) NOT NULL,
    next_plan VARCHAR(1000) NOT NULL,
    manager_comment VARCHAR(1000),
    status VARCHAR(32) NOT NULL,
    created_at TIMESTAMP NOT NULL,
    updated_at TIMESTAMP NOT NULL,
    CONSTRAINT fk_weekly_report_member FOREIGN KEY (member_id) REFERENCES team_member(id)
);

CREATE TABLE IF NOT EXISTS monthly_assessment (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    member_id BIGINT NOT NULL,
    assess_month VARCHAR(16) NOT NULL,
    dept_manager_score DOUBLE,
    tech_director_score DOUBLE,
    general_manager_score DOUBLE,
    final_score DOUBLE,
    rating_level VARCHAR(8),
    status VARCHAR(32) NOT NULL,
    comment VARCHAR(500),
    created_at TIMESTAMP NOT NULL,
    updated_at TIMESTAMP NOT NULL,
    CONSTRAINT fk_monthly_assessment_member FOREIGN KEY (member_id) REFERENCES team_member(id)
);

CREATE TABLE IF NOT EXISTS assessment_score_detail (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    assessment_id BIGINT NOT NULL,
    scorer_user_id BIGINT NOT NULL,
    scorer_role VARCHAR(64) NOT NULL,
    raw_score DOUBLE NOT NULL,
    comment VARCHAR(500),
    score_time TIMESTAMP NOT NULL,
    CONSTRAINT fk_score_detail_assessment FOREIGN KEY (assessment_id) REFERENCES monthly_assessment(id),
    CONSTRAINT fk_score_detail_user FOREIGN KEY (scorer_user_id) REFERENCES sys_user(id)
);

CREATE TABLE IF NOT EXISTS audit_log (
    id BIGINT PRIMARY KEY AUTO_INCREMENT,
    operator_user_id BIGINT,
    action_type VARCHAR(64) NOT NULL,
    target_type VARCHAR(64) NOT NULL,
    target_id VARCHAR(64) NOT NULL,
    result VARCHAR(32) NOT NULL,
    action_time TIMESTAMP NOT NULL,
    detail VARCHAR(500),
    CONSTRAINT fk_audit_log_operator FOREIGN KEY (operator_user_id) REFERENCES sys_user(id)
);
