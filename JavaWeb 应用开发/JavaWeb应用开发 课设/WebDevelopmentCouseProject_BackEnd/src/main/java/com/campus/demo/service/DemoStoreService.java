package com.campus.demo.service;

import com.campus.demo.common.PageResult;
import com.campus.demo.dto.ChangePasswordRequest;
import com.campus.demo.dto.CreateMemberRequest;
import com.campus.demo.dto.CreateProjectRequest;
import com.campus.demo.dto.CreateTaskProgressRequest;
import com.campus.demo.dto.CreateTaskRequest;
import com.campus.demo.dto.CreateTeamRequest;
import com.campus.demo.dto.CreateUserRequest;
import com.campus.demo.dto.CreateWeeklyReportRequest;
import com.campus.demo.dto.FinalizeAssessmentRequest;
import com.campus.demo.dto.GenerateAssessmentRequest;
import com.campus.demo.dto.LoginRequest;
import com.campus.demo.dto.MemberStatusUpdateRequest;
import com.campus.demo.dto.ProjectStatusUpdateRequest;
import com.campus.demo.dto.RegisterUserRequest;
import com.campus.demo.dto.ReviewWeeklyReportRequest;
import com.campus.demo.dto.ScoreAssessmentRequest;
import com.campus.demo.dto.TaskStatusUpdateRequest;
import com.campus.demo.dto.UpdateMemberRequest;
import com.campus.demo.dto.UpdateProjectRequest;
import com.campus.demo.dto.UpdateTaskRequest;
import com.campus.demo.dto.UpdateTeamRequest;
import com.campus.demo.dto.UpdateUserRequest;
import com.campus.demo.dto.UpdateWeeklyReportRequest;
import com.campus.demo.dto.UserStatusUpdateRequest;
import com.campus.demo.entity.AssessmentScoreDetail;
import com.campus.demo.entity.AuditLog;
import com.campus.demo.entity.CurrentUser;
import com.campus.demo.entity.DashboardData;
import com.campus.demo.entity.GenerateAssessmentResult;
import com.campus.demo.entity.LoginResponseData;
import com.campus.demo.entity.Member;
import com.campus.demo.entity.MonthlyAssessment;
import com.campus.demo.entity.MonthlyStatistics;
import com.campus.demo.entity.Project;
import com.campus.demo.entity.QuarterlyStatistics;
import com.campus.demo.entity.Role;
import com.campus.demo.entity.SysUser;
import com.campus.demo.entity.Task;
import com.campus.demo.entity.TaskProgress;
import com.campus.demo.entity.Team;
import com.campus.demo.entity.WeeklyReport;
import com.campus.demo.entity.YearlyStatistics;
import com.campus.demo.enums.AssessmentStatus;
import com.campus.demo.enums.AuditResult;
import com.campus.demo.enums.MemberStatus;
import com.campus.demo.enums.PriorityLevel;
import com.campus.demo.enums.ProjectStatus;
import com.campus.demo.enums.RoleCode;
import com.campus.demo.enums.TaskStatus;
import com.campus.demo.enums.TeamStatus;
import com.campus.demo.enums.UserStatus;
import com.campus.demo.enums.WeeklyReportStatus;
import com.campus.demo.util.MaskingUtils;
import com.campus.demo.util.PasswordCodec;
import jakarta.annotation.PostConstruct;
import org.springframework.jdbc.core.JdbcTemplate;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.sql.ResultSet;
import java.sql.SQLException;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.YearMonth;
import java.time.format.DateTimeFormatter;
import java.time.temporal.WeekFields;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;

@Service
public class DemoStoreService {

    private static final DateTimeFormatter MONTH_FORMATTER = DateTimeFormatter.ofPattern("yyyy-MM");
    private static final long DEMO_OPERATOR_ID = 3L;
    private static final int TOKEN_EXPIRE_SECONDS = 1800;

    private final JdbcTemplate jdbcTemplate;
    private final Map<String, Long> tokenSessions = new ConcurrentHashMap<>();
    private final Map<RoleCode, List<String>> permissionMap = new EnumMap<>(RoleCode.class);

    public DemoStoreService(JdbcTemplate jdbcTemplate) {
        this.jdbcTemplate = jdbcTemplate;
        seedPermissions();
    }

    @PostConstruct
    public void init() {
        ensureBaseRolesInitialized();
        seedDemoDataIfNeeded();
    }

    @Transactional
    public LoginResponseData login(LoginRequest request) {
        SysUser user = findUserByUsername(request.getUsername());
        if (user == null) {
            writeAudit(null, "LOGIN", "auth", request.getUsername(), AuditResult.FAIL, "username not found");
            throw new IllegalArgumentException("Invalid username or password");
        }

        if (user.getStatus() == UserStatus.LOCKED) {
            writeAudit(user.getId(), "LOGIN", "auth", user.getUsername(), AuditResult.FAIL, "user locked");
            throw new IllegalArgumentException("User is locked");
        }
        if (user.getStatus() == UserStatus.DISABLED) {
            writeAudit(user.getId(), "LOGIN", "auth", user.getUsername(), AuditResult.FAIL, "user disabled");
            throw new IllegalArgumentException("User is disabled");
        }
        if (!PasswordCodec.matches(request.getPassword(), user.getPasswordHash())) {
            writeAudit(user.getId(), "LOGIN", "auth", user.getUsername(), AuditResult.FAIL, "invalid password");
            throw new IllegalArgumentException("Invalid username or password");
        }

        LocalDateTime now = LocalDateTime.now();
        jdbcTemplate.update(
                "UPDATE sys_user SET last_login_at = ?, updated_at = ? WHERE id = ?",
                now,
                now,
                user.getId()
        );

        user.setLastLoginAt(now);
        user.setUpdatedAt(now);

        String token = UUID.randomUUID().toString().replace("-", "");
        tokenSessions.put(token, user.getId());
        writeAudit(user.getId(), "LOGIN", "auth", user.getUsername(), AuditResult.SUCCESS, "login success");
        return new LoginResponseData(token, "Bearer", TOKEN_EXPIRE_SECONDS, buildCurrentUser(refreshUser(user)));
    }

    public Boolean logout(String authorizationHeader) {
        String token = extractBearerToken(authorizationHeader);
        if (token != null) {
            Long userId = tokenSessions.remove(token);
            if (userId != null) {
                writeAudit(userId, "LOGOUT", "auth", String.valueOf(userId), AuditResult.SUCCESS, "logout success");
            }
        }
        return Boolean.TRUE;
    }

    public CurrentUser currentUser(String authorizationHeader) {
        return buildCurrentUser(resolveCurrentUser(authorizationHeader));
    }

    @Transactional
    public Boolean changePassword(String authorizationHeader, ChangePasswordRequest request) {
        SysUser user = resolveCurrentUser(authorizationHeader);
        if (!PasswordCodec.matches(request.getOldPassword(), user.getPasswordHash())) {
            throw new IllegalArgumentException("Old password is incorrect");
        }
        if (!Objects.equals(request.getNewPassword(), request.getConfirmPassword())) {
            throw new IllegalArgumentException("New passwords do not match");
        }
        validatePassword(request.getNewPassword());

        LocalDateTime now = LocalDateTime.now();
        jdbcTemplate.update(
                "UPDATE sys_user SET password_sm3 = ?, must_change_password = ?, password_expire_at = ?, updated_at = ? WHERE id = ?",
                PasswordCodec.encode(request.getNewPassword()),
                false,
                now.plusDays(90),
                now,
                user.getId()
        );
        writeAudit(user.getId(), "CHANGE_PASSWORD", "sys_user", String.valueOf(user.getId()), AuditResult.SUCCESS, "change password");
        return Boolean.TRUE;
    }

    public List<Role> listRoles() {
        return jdbcTemplate.query(
                "SELECT role_code, role_name, description FROM sys_role ORDER BY id",
                (rs, rowNum) -> new Role(
                        RoleCode.valueOf(rs.getString("role_code")),
                        rs.getString("role_name"),
                        rs.getString("description")
                )
        );
    }

    public PageResult<SysUser> listUsers(String keyword, RoleCode roleCode, UserStatus status, Integer pageNo, Integer pageSize) {
        StringBuilder sql = new StringBuilder("SELECT * FROM sys_user WHERE 1=1");
        List<Object> args = new ArrayList<>();
        if (keyword != null && !keyword.isBlank()) {
            String like = "%" + keyword.trim() + "%";
            sql.append(" AND (username LIKE ? OR real_name LIKE ?)");
            args.add(like);
            args.add(like);
        }
        if (status != null) {
            sql.append(" AND status = ?");
            args.add(status.name());
        }
        sql.append(" ORDER BY id");

        List<SysUser> users = jdbcTemplate.query(sql.toString(), (rs, rowNum) -> mapSysUser(rs), args.toArray());
        List<SysUser> refreshed = users.stream()
                .peek(this::loadRolesIntoUser)
                .map(this::refreshUser)
                .filter(item -> roleCode == null || item.getRoleCodes().contains(roleCode))
                .toList();
        return buildPage(refreshed, pageNo, pageSize);
    }

    public SysUser getUser(Long userId) {
        return refreshUser(requireUser(userId));
    }

    @Transactional
    public SysUser createUser(CreateUserRequest request) {
        ensureUsernameUnique(request.getUsername());
        validatePassword(request.getPassword());
        validateTeamAndMemberBinding(request.getTeamId(), request.getMemberId());

        LocalDateTime now = LocalDateTime.now();
        jdbcTemplate.update(
                "INSERT INTO sys_user (username, real_name, password_sm3, mobile, email, team_id, member_id, status, must_change_password, password_expire_at, last_login_at, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                request.getUsername(),
                request.getRealName(),
                PasswordCodec.encode(request.getPassword()),
                request.getMobile(),
                request.getEmail(),
                request.getTeamId(),
                request.getMemberId(),
                UserStatus.ENABLED.name(),
                Boolean.TRUE.equals(request.getMustChangePassword()),
                now.plusDays(90),
                null,
                now,
                now
        );

        SysUser created = requireUserByUsername(request.getUsername());
        replaceUserRoles(created.getId(), request.getRoleCodes());
        writeAudit(currentOperatorId(), "CREATE_USER", "sys_user", String.valueOf(created.getId()), AuditResult.SUCCESS, "create user " + created.getUsername());
        return getUser(created.getId());
    }

    @Transactional
    public SysUser registerUser(RegisterUserRequest request) {
        ensureUsernameUnique(request.getUsername());
        validatePassword(request.getPassword());
        validateTeamAndMemberBinding(request.getTeamId(), request.getMemberId());

        List<RoleCode> roleCodes = request.getRoleCodes() == null || request.getRoleCodes().isEmpty()
                ? List.of(RoleCode.TEAM_MEMBER)
                : request.getRoleCodes();

        LocalDateTime now = LocalDateTime.now();
        jdbcTemplate.update(
                "INSERT INTO sys_user (username, real_name, password_sm3, mobile, email, team_id, member_id, status, must_change_password, password_expire_at, last_login_at, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                request.getUsername(),
                request.getRealName(),
                PasswordCodec.encode(request.getPassword()),
                request.getMobile(),
                request.getEmail(),
                request.getTeamId(),
                request.getMemberId(),
                UserStatus.ENABLED.name(),
                Boolean.TRUE.equals(request.getMustChangePassword()),
                now.plusDays(90),
                null,
                now,
                now
        );

        SysUser created = requireUserByUsername(request.getUsername());
        replaceUserRoles(created.getId(), roleCodes);
        writeAudit(null, "REGISTER_USER", "sys_user", String.valueOf(created.getId()), AuditResult.SUCCESS, "register user " + created.getUsername());
        return getUser(created.getId());
    }

    @Transactional
    public SysUser updateUser(Long userId, UpdateUserRequest request) {
        requireUser(userId);
        validateTeamAndMemberBinding(request.getTeamId(), request.getMemberId());

        jdbcTemplate.update(
                "UPDATE sys_user SET real_name = ?, mobile = ?, email = ?, team_id = ?, member_id = ?, updated_at = ? WHERE id = ?",
                request.getRealName(),
                request.getMobile(),
                request.getEmail(),
                request.getTeamId(),
                request.getMemberId(),
                LocalDateTime.now(),
                userId
        );
        replaceUserRoles(userId, request.getRoleCodes());
        SysUser updated = getUser(userId);
        writeAudit(currentOperatorId(), "UPDATE_USER", "sys_user", String.valueOf(userId), AuditResult.SUCCESS, "update user " + updated.getUsername());
        return updated;
    }

    @Transactional
    public SysUser updateUserStatus(Long userId, UserStatusUpdateRequest request) {
        requireUser(userId);
        jdbcTemplate.update(
                "UPDATE sys_user SET status = ?, updated_at = ? WHERE id = ?",
                request.getStatus().name(),
                LocalDateTime.now(),
                userId
        );
        writeAudit(currentOperatorId(), "UPDATE_USER_STATUS", "sys_user", String.valueOf(userId), AuditResult.SUCCESS, request.getReason());
        return getUser(userId);
    }

    public PageResult<Team> listTeams(String keyword, TeamStatus status, Integer pageNo, Integer pageSize) {
        StringBuilder sql = new StringBuilder("SELECT * FROM team WHERE 1=1");
        List<Object> args = new ArrayList<>();
        if (keyword != null && !keyword.isBlank()) {
            String like = "%" + keyword.trim() + "%";
            sql.append(" AND (team_name LIKE ? OR department_name LIKE ?)");
            args.add(like);
            args.add(like);
        }
        if (status != null) {
            sql.append(" AND status = ?");
            args.add(status.name());
        }
        sql.append(" ORDER BY id");

        List<Team> teams = jdbcTemplate.query(sql.toString(), (rs, rowNum) -> mapTeam(rs), args.toArray())
                .stream()
                .map(this::refreshTeam)
                .toList();
        return buildPage(teams, pageNo, pageSize);
    }

    public Team getTeam(Long teamId) {
        return refreshTeam(requireTeam(teamId));
    }

    @Transactional
    public Team createTeam(CreateTeamRequest request) {
        validateManagerUser(request.getManagerId());
        LocalDateTime now = LocalDateTime.now();
        TeamStatus finalStatus = request.getStatus() == null ? TeamStatus.ACTIVE : request.getStatus();
        jdbcTemplate.update(
                "INSERT INTO team (team_name, department_name, manager_user_id, description, status, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?)",
                request.getTeamName(),
                request.getDepartmentName(),
                request.getManagerId(),
                request.getDescription(),
                finalStatus.name(),
                now,
                now
        );
        Long teamId = findLastInsertedId("team");
        writeAudit(currentOperatorId(), "CREATE_TEAM", "team", String.valueOf(teamId), AuditResult.SUCCESS, "create team " + request.getTeamName());
        return getTeam(teamId);
    }

    @Transactional
    public Team updateTeam(Long teamId, UpdateTeamRequest request) {
        requireTeam(teamId);
        validateManagerUser(request.getManagerId());
        jdbcTemplate.update(
                "UPDATE team SET team_name = ?, department_name = ?, manager_user_id = ?, description = ?, status = ?, updated_at = ? WHERE id = ?",
                request.getTeamName(),
                request.getDepartmentName(),
                request.getManagerId(),
                request.getDescription(),
                request.getStatus().name(),
                LocalDateTime.now(),
                teamId
        );
        writeAudit(currentOperatorId(), "UPDATE_TEAM", "team", String.valueOf(teamId), AuditResult.SUCCESS, "update team " + request.getTeamName());
        return getTeam(teamId);
    }

    public PageResult<Member> listMembers(String keyword, Long teamId, MemberStatus status, Integer pageNo, Integer pageSize) {
        StringBuilder sql = new StringBuilder("SELECT tm.*, t.team_name FROM team_member tm LEFT JOIN team t ON t.id = tm.team_id WHERE 1=1");
        List<Object> args = new ArrayList<>();
        if (keyword != null && !keyword.isBlank()) {
            String like = "%" + keyword.trim() + "%";
            sql.append(" AND (tm.name LIKE ? OR tm.position LIKE ?)");
            args.add(like);
            args.add(like);
        }
        if (teamId != null) {
            sql.append(" AND tm.team_id = ?");
            args.add(teamId);
        }
        if (status != null) {
            sql.append(" AND tm.status = ?");
            args.add(status.name());
        }
        sql.append(" ORDER BY tm.id");

        List<Member> members = jdbcTemplate.query(sql.toString(), (rs, rowNum) -> mapMember(rs), args.toArray())
                .stream()
                .map(this::refreshMember)
                .toList();
        return buildPage(members, pageNo, pageSize);
    }

    public Member getMember(Long memberId) {
        return refreshMember(requireMember(memberId));
    }

    @Transactional
    public Member createMember(CreateMemberRequest request) {
        if (request.getTeamId() != null) {
            requireTeam(request.getTeamId());
        }
        jdbcTemplate.update(
                "INSERT INTO team_member (name, team_id, position, job_level, phone, email, status, entry_date) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                request.getName(),
                request.getTeamId(),
                request.getPosition(),
                request.getJobLevel(),
                request.getPhone(),
                request.getEmail(),
                MemberStatus.ONBOARDING.name(),
                request.getEntryDate()
        );
        Long memberId = findLastInsertedId("team_member");
        writeAudit(currentOperatorId(), "CREATE_MEMBER", "team_member", String.valueOf(memberId), AuditResult.SUCCESS, "create member " + request.getName());
        return getMember(memberId);
    }

    @Transactional
    public Member updateMember(Long memberId, UpdateMemberRequest request) {
        requireMember(memberId);
        if (request.getTeamId() != null) {
            requireTeam(request.getTeamId());
        }
        jdbcTemplate.update(
                "UPDATE team_member SET name = ?, team_id = ?, position = ?, job_level = ?, phone = ?, email = ?, entry_date = ? WHERE id = ?",
                request.getName(),
                request.getTeamId(),
                request.getPosition(),
                request.getJobLevel(),
                request.getPhone(),
                request.getEmail(),
                request.getEntryDate(),
                memberId
        );
        writeAudit(currentOperatorId(), "UPDATE_MEMBER", "team_member", String.valueOf(memberId), AuditResult.SUCCESS, "update member " + request.getName());
        return getMember(memberId);
    }

    @Transactional
    public Member updateMemberStatus(Long memberId, MemberStatusUpdateRequest request) {
        requireMember(memberId);
        jdbcTemplate.update("UPDATE team_member SET status = ? WHERE id = ?", request.getStatus().name(), memberId);
        writeAudit(currentOperatorId(), "UPDATE_MEMBER_STATUS", "team_member", String.valueOf(memberId), AuditResult.SUCCESS, request.getReason());
        return getMember(memberId);
    }

    public PageResult<Project> listProjects(String keyword, ProjectStatus status, PriorityLevel priority, Integer pageNo, Integer pageSize) {
        StringBuilder sql = new StringBuilder("SELECT p.*, m.name AS owner_name FROM project p LEFT JOIN team_member m ON m.id = p.owner_member_id WHERE 1=1");
        List<Object> args = new ArrayList<>();
        if (keyword != null && !keyword.isBlank()) {
            String like = "%" + keyword.trim() + "%";
            sql.append(" AND (p.project_name LIKE ? OR p.milestone LIKE ?)");
            args.add(like);
            args.add(like);
        }
        if (status != null) {
            sql.append(" AND p.status = ?");
            args.add(status.name());
        }
        if (priority != null) {
            sql.append(" AND p.priority = ?");
            args.add(priority.name());
        }
        sql.append(" ORDER BY p.id");

        List<Project> projects = jdbcTemplate.query(sql.toString(), (rs, rowNum) -> mapProject(rs), args.toArray());
        return buildPage(projects, pageNo, pageSize);
    }

    public Project getProject(Long projectId) {
        return requireProject(projectId);
    }

    @Transactional
    public Project createProject(CreateProjectRequest request) {
        if (request.getOwnerId() != null) {
            requireMember(request.getOwnerId());
        }
        jdbcTemplate.update(
                "INSERT INTO project (project_name, owner_member_id, status, priority, start_date, end_date, milestone, description) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                request.getProjectName(),
                request.getOwnerId(),
                request.getStatus().name(),
                request.getPriority().name(),
                request.getStartDate(),
                request.getEndDate(),
                request.getMilestone(),
                request.getDescription()
        );
        Long projectId = findLastInsertedId("project");
        writeAudit(currentOperatorId(), "CREATE_PROJECT", "project", String.valueOf(projectId), AuditResult.SUCCESS, "create project " + request.getProjectName());
        return getProject(projectId);
    }

    @Transactional
    public Project updateProject(Long projectId, UpdateProjectRequest request) {
        Project existing = requireProject(projectId);
        if (request.getOwnerId() != null) {
            requireMember(request.getOwnerId());
        }
        jdbcTemplate.update(
                "UPDATE project SET project_name = ?, owner_member_id = ?, priority = ?, start_date = ?, end_date = ?, milestone = ?, description = ? WHERE id = ?",
                request.getProjectName(),
                request.getOwnerId(),
                request.getPriority().name(),
                request.getStartDate(),
                request.getEndDate(),
                request.getMilestone(),
                request.getDescription(),
                projectId
        );
        writeAudit(currentOperatorId(), "UPDATE_PROJECT", "project", String.valueOf(projectId), AuditResult.SUCCESS, "update project " + existing.getProjectName());
        return getProject(projectId);
    }

    @Transactional
    public Project updateProjectStatus(Long projectId, ProjectStatusUpdateRequest request) {
        requireProject(projectId);
        jdbcTemplate.update("UPDATE project SET status = ? WHERE id = ?", request.getStatus().name(), projectId);
        writeAudit(currentOperatorId(), "UPDATE_PROJECT_STATUS", "project", String.valueOf(projectId), AuditResult.SUCCESS, request.getComment());
        return getProject(projectId);
    }

    public PageResult<Task> listTasks(String weekNo, Long assigneeId, Long projectId, TaskStatus status, PriorityLevel priority, Integer pageNo, Integer pageSize) {
        StringBuilder sql = new StringBuilder(
                "SELECT t.*, p.project_name, m.name AS assignee_name " +
                        "FROM weekly_task t " +
                        "LEFT JOIN project p ON p.id = t.project_id " +
                        "LEFT JOIN team_member m ON m.id = t.assignee_member_id WHERE 1=1"
        );
        List<Object> args = new ArrayList<>();
        if (weekNo != null && !weekNo.isBlank()) {
            sql.append(" AND t.week_no = ?");
            args.add(weekNo);
        }
        if (assigneeId != null) {
            sql.append(" AND t.assignee_member_id = ?");
            args.add(assigneeId);
        }
        if (projectId != null) {
            sql.append(" AND t.project_id = ?");
            args.add(projectId);
        }
        if (status != null) {
            sql.append(" AND t.status = ?");
            args.add(status.name());
        }
        if (priority != null) {
            sql.append(" AND t.priority = ?");
            args.add(priority.name());
        }
        sql.append(" ORDER BY t.id");

        List<Task> tasks = jdbcTemplate.query(sql.toString(), (rs, rowNum) -> mapTask(rs), args.toArray());
        return buildPage(tasks, pageNo, pageSize);
    }

    public Task getTask(Long taskId) {
        return requireTask(taskId);
    }

    @Transactional
    public Task createTask(CreateTaskRequest request) {
        requireMember(request.getAssigneeId());
        if (request.getProjectId() != null) {
            requireProject(request.getProjectId());
        }

        jdbcTemplate.update(
                "INSERT INTO weekly_task (title, project_id, assignee_member_id, priority, deadline, status, progress_rate, week_no, plan_start_date, plan_end_date, issue_desc) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                request.getTitle(),
                request.getProjectId(),
                request.getAssigneeId(),
                request.getPriority().name(),
                request.getDeadline(),
                TaskStatus.TODO.name(),
                0,
                request.getWeekNo(),
                request.getPlanStartDate(),
                request.getPlanEndDate(),
                request.getIssueDesc()
        );
        Long taskId = findLastInsertedId("weekly_task");
        writeAudit(currentOperatorId(), "CREATE_TASK", "weekly_task", String.valueOf(taskId), AuditResult.SUCCESS, "create task " + request.getTitle());
        return getTask(taskId);
    }

    @Transactional
    public Task updateTask(Long taskId, UpdateTaskRequest request) {
        requireTask(taskId);
        requireMember(request.getAssigneeId());
        if (request.getProjectId() != null) {
            requireProject(request.getProjectId());
        }

        jdbcTemplate.update(
                "UPDATE weekly_task SET title = ?, project_id = ?, assignee_member_id = ?, priority = ?, deadline = ?, plan_start_date = ?, plan_end_date = ?, issue_desc = ? WHERE id = ?",
                request.getTitle(),
                request.getProjectId(),
                request.getAssigneeId(),
                request.getPriority().name(),
                request.getDeadline(),
                request.getPlanStartDate(),
                request.getPlanEndDate(),
                request.getIssueDesc(),
                taskId
        );
        writeAudit(currentOperatorId(), "UPDATE_TASK", "weekly_task", String.valueOf(taskId), AuditResult.SUCCESS, "update task " + request.getTitle());
        return getTask(taskId);
    }

    @Transactional
    public Task updateTaskStatus(Long taskId, TaskStatusUpdateRequest request) {
        Task task = requireTask(taskId);
        Integer progressRate = request.getProgressRate() == null ? task.getProgressRate() : request.getProgressRate();
        String issueDesc = request.getIssueDesc() == null ? task.getIssueDesc() : request.getIssueDesc();
        if (request.getStatus() == TaskStatus.DONE) {
            progressRate = 100;
        }
        jdbcTemplate.update(
                "UPDATE weekly_task SET status = ?, progress_rate = ?, issue_desc = ? WHERE id = ?",
                request.getStatus().name(),
                progressRate,
                issueDesc,
                taskId
        );
        writeAudit(currentOperatorId(), "UPDATE_TASK_STATUS", "weekly_task", String.valueOf(taskId), AuditResult.SUCCESS, "update task status");
        return getTask(taskId);
    }

    public List<TaskProgress> listTaskProgress(Long taskId) {
        requireTask(taskId);
        return jdbcTemplate.query(
                "SELECT * FROM task_progress WHERE task_id = ? ORDER BY id",
                (rs, rowNum) -> mapTaskProgress(rs),
                taskId
        );
    }

    @Transactional
    public TaskProgress addTaskProgress(Long taskId, CreateTaskProgressRequest request) {
        Task task = requireTask(taskId);
        LocalDateTime now = LocalDateTime.now();
        jdbcTemplate.update(
                "INSERT INTO task_progress (task_id, progress_rate, issue_desc, comment, update_time) VALUES (?, ?, ?, ?, ?)",
                taskId,
                request.getProgressRate(),
                request.getIssueDesc(),
                request.getComment(),
                now
        );

        TaskStatus nextStatus = task.getStatus();
        if (Objects.equals(request.getProgressRate(), 100)) {
            nextStatus = TaskStatus.DONE;
        } else if (task.getStatus() == TaskStatus.TODO) {
            nextStatus = TaskStatus.IN_PROGRESS;
        }

        jdbcTemplate.update(
                "UPDATE weekly_task SET progress_rate = ?, issue_desc = ?, status = ? WHERE id = ?",
                request.getProgressRate(),
                request.getIssueDesc(),
                nextStatus.name(),
                taskId
        );

        Long progressId = findLastInsertedId("task_progress");
        writeAudit(currentOperatorId(), "ADD_TASK_PROGRESS", "task_progress", String.valueOf(progressId), AuditResult.SUCCESS, "add task progress");
        return listTaskProgress(taskId).stream()
                .filter(item -> Objects.equals(item.getId(), progressId))
                .findFirst()
                .orElseThrow(() -> new IllegalStateException("Task progress creation failed"));
    }

    public PageResult<WeeklyReport> listReports(String weekNo, Long memberId, WeeklyReportStatus status, Integer pageNo, Integer pageSize) {
        StringBuilder sql = new StringBuilder(
                "SELECT r.*, m.name AS member_name FROM weekly_report r LEFT JOIN team_member m ON m.id = r.member_id WHERE 1=1"
        );
        List<Object> args = new ArrayList<>();
        if (weekNo != null && !weekNo.isBlank()) {
            sql.append(" AND r.week_no = ?");
            args.add(weekNo);
        }
        if (memberId != null) {
            sql.append(" AND r.member_id = ?");
            args.add(memberId);
        }
        if (status != null) {
            sql.append(" AND r.status = ?");
            args.add(status.name());
        }
        sql.append(" ORDER BY r.id");

        List<WeeklyReport> reports = jdbcTemplate.query(sql.toString(), (rs, rowNum) -> mapWeeklyReport(rs), args.toArray());
        return buildPage(reports, pageNo, pageSize);
    }

    public WeeklyReport getReport(Long reportId) {
        return requireReport(reportId);
    }

    @Transactional
    public WeeklyReport createReport(CreateWeeklyReportRequest request) {
        requireMember(request.getMemberId());
        LocalDateTime now = LocalDateTime.now();
        jdbcTemplate.update(
                "INSERT INTO weekly_report (member_id, week_no, summary, next_plan, manager_comment, status, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                request.getMemberId(),
                request.getWeekNo(),
                request.getSummary(),
                request.getNextPlan(),
                null,
                WeeklyReportStatus.SUBMITTED.name(),
                now,
                now
        );
        Long reportId = findLastInsertedId("weekly_report");
        writeAudit(currentOperatorId(), "CREATE_WEEKLY_REPORT", "weekly_report", String.valueOf(reportId), AuditResult.SUCCESS, "create weekly report");
        return getReport(reportId);
    }

    @Transactional
    public WeeklyReport updateReport(Long reportId, UpdateWeeklyReportRequest request) {
        requireReport(reportId);
        jdbcTemplate.update(
                "UPDATE weekly_report SET summary = ?, next_plan = ?, updated_at = ? WHERE id = ?",
                request.getSummary(),
                request.getNextPlan(),
                LocalDateTime.now(),
                reportId
        );
        writeAudit(currentOperatorId(), "UPDATE_WEEKLY_REPORT", "weekly_report", String.valueOf(reportId), AuditResult.SUCCESS, "update weekly report");
        return getReport(reportId);
    }

    @Transactional
    public WeeklyReport reviewReport(Long reportId, ReviewWeeklyReportRequest request) {
        requireReport(reportId);
        String reviewStatus = request.getStatus().trim().toUpperCase(Locale.ROOT);
        if (!"REVIEWED".equals(reviewStatus) && !"RETURNED".equals(reviewStatus)) {
            throw new IllegalArgumentException("Status must be REVIEWED or RETURNED");
        }

        jdbcTemplate.update(
                "UPDATE weekly_report SET status = ?, manager_comment = ?, updated_at = ? WHERE id = ?",
                reviewStatus,
                request.getManagerComment(),
                LocalDateTime.now(),
                reportId
        );
        writeAudit(currentOperatorId(), "REVIEW_WEEKLY_REPORT", "weekly_report", String.valueOf(reportId), AuditResult.SUCCESS, request.getManagerComment());
        return getReport(reportId);
    }

    @Transactional
    public GenerateAssessmentResult generateAssessments(GenerateAssessmentRequest request) {
        List<Member> targetMembers = listMembers(null, request.getTeamId(), null, 1, Integer.MAX_VALUE).getRecords().stream()
                .filter(member -> member.getStatus() != MemberStatus.LEFT)
                .toList();

        int generated = 0;
        int skipped = 0;
        for (Member member : targetMembers) {
            MonthlyAssessment existing = findAssessmentByMemberAndMonth(member.getId(), request.getAssessMonth());
            if (existing != null && !Boolean.TRUE.equals(request.getOverwriteExisting())) {
                skipped++;
                continue;
            }
            if (existing != null) {
                jdbcTemplate.update("DELETE FROM assessment_score_detail WHERE assessment_id = ?", existing.getId());
                jdbcTemplate.update("DELETE FROM monthly_assessment WHERE id = ?", existing.getId());
            }

            LocalDateTime now = LocalDateTime.now();
            jdbcTemplate.update(
                    "INSERT INTO monthly_assessment (member_id, assess_month, dept_manager_score, tech_director_score, general_manager_score, final_score, rating_level, status, comment, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    member.getId(),
                    request.getAssessMonth(),
                    null,
                    null,
                    null,
                    null,
                    null,
                    AssessmentStatus.DRAFT.name(),
                    null,
                    now,
                    now
            );
            generated++;
        }

        writeAudit(currentOperatorId(), "GENERATE_ASSESSMENT", "monthly_assessment", request.getAssessMonth(), AuditResult.SUCCESS, "generate assessments");
        return new GenerateAssessmentResult(request.getAssessMonth(), generated, skipped);
    }

    public PageResult<MonthlyAssessment> listAssessments(String assessMonth, Long teamId, Long memberId, AssessmentStatus status, Integer pageNo, Integer pageSize) {
        StringBuilder sql = new StringBuilder(
                "SELECT ma.*, m.name AS member_name, m.team_id, t.team_name " +
                        "FROM monthly_assessment ma " +
                        "LEFT JOIN team_member m ON m.id = ma.member_id " +
                        "LEFT JOIN team t ON t.id = m.team_id WHERE 1=1"
        );
        List<Object> args = new ArrayList<>();
        if (assessMonth != null && !assessMonth.isBlank()) {
            sql.append(" AND ma.assess_month = ?");
            args.add(assessMonth);
        }
        if (teamId != null) {
            sql.append(" AND m.team_id = ?");
            args.add(teamId);
        }
        if (memberId != null) {
            sql.append(" AND ma.member_id = ?");
            args.add(memberId);
        }
        if (status != null) {
            sql.append(" AND ma.status = ?");
            args.add(status.name());
        }
        sql.append(" ORDER BY ma.id");

        List<MonthlyAssessment> assessments = jdbcTemplate.query(sql.toString(), (rs, rowNum) -> mapMonthlyAssessment(rs), args.toArray());
        assessments.forEach(item -> item.setScoreDetails(loadScoreDetails(item.getId())));
        return buildPage(assessments, pageNo, pageSize);
    }

    public MonthlyAssessment getAssessment(Long assessmentId) {
        MonthlyAssessment assessment = requireAssessment(assessmentId);
        assessment.setScoreDetails(loadScoreDetails(assessmentId));
        return assessment;
    }

    @Transactional
    public MonthlyAssessment scoreAssessment(Long assessmentId, ScoreAssessmentRequest request, String authorizationHeader) {
        getAssessment(assessmentId);
        SysUser scorer = resolveCurrentUser(authorizationHeader);
        if (!scorer.getRoleCodes().contains(request.getScorerRole())) {
            throw new IllegalArgumentException("Current user does not have the requested scoring role");
        }

        jdbcTemplate.update(
                "DELETE FROM assessment_score_detail WHERE assessment_id = ? AND scorer_role = ?",
                assessmentId,
                request.getScorerRole().name()
        );
        jdbcTemplate.update(
                "INSERT INTO assessment_score_detail (assessment_id, scorer_user_id, scorer_role, raw_score, comment, score_time) VALUES (?, ?, ?, ?, ?, ?)",
                assessmentId,
                scorer.getId(),
                request.getScorerRole().name(),
                request.getRawScore(),
                request.getComment(),
                LocalDateTime.now()
        );

        List<AssessmentScoreDetail> details = loadScoreDetails(assessmentId);
        Double deptScore = scoreOf(details, RoleCode.DEPT_MANAGER);
        Double techScore = scoreOf(details, RoleCode.TECH_DIRECTOR);
        Double gmScore = scoreOf(details, RoleCode.GENERAL_MANAGER);

        Double finalScore = null;
        String ratingLevel = null;
        AssessmentStatus nextStatus;
        if (deptScore != null && techScore != null && gmScore != null) {
            finalScore = roundToTwo(deptScore * 0.4 + techScore * 0.3 + gmScore * 0.3);
            ratingLevel = toRating(finalScore);
            nextStatus = AssessmentStatus.PENDING_FINALIZE;
        } else {
            nextStatus = AssessmentStatus.SCORING;
        }

        jdbcTemplate.update(
                "UPDATE monthly_assessment SET dept_manager_score = ?, tech_director_score = ?, general_manager_score = ?, final_score = ?, rating_level = ?, status = ?, updated_at = ? WHERE id = ?",
                deptScore,
                techScore,
                gmScore,
                finalScore,
                ratingLevel,
                nextStatus.name(),
                LocalDateTime.now(),
                assessmentId
        );

        writeAudit(scorer.getId(), "SCORE_ASSESSMENT", "monthly_assessment", String.valueOf(assessmentId), AuditResult.SUCCESS, request.getComment());
        return getAssessment(assessmentId);
    }

    @Transactional
    public MonthlyAssessment finalizeAssessment(Long assessmentId, FinalizeAssessmentRequest request, String authorizationHeader) {
        SysUser operator = resolveCurrentUser(authorizationHeader);
        if (!hasAnyRole(operator, RoleCode.DEPT_MANAGER, RoleCode.SYSTEM_ADMIN)) {
            throw new IllegalArgumentException("Current user cannot finalize this assessment");
        }

        MonthlyAssessment assessment = getAssessment(assessmentId);
        if (assessment.getFinalScore() == null) {
            throw new IllegalArgumentException("Assessment is not fully scored yet");
        }

        jdbcTemplate.update(
                "UPDATE monthly_assessment SET status = ?, comment = ?, updated_at = ? WHERE id = ?",
                AssessmentStatus.FINALIZED.name(),
                request.getComment(),
                LocalDateTime.now(),
                assessmentId
        );
        writeAudit(operator.getId(), "FINALIZE_ASSESSMENT", "monthly_assessment", String.valueOf(assessmentId), AuditResult.SUCCESS, request.getComment());
        return getAssessment(assessmentId);
    }

    public DashboardData getDashboardData() {
        String currentMonth = YearMonth.now().format(MONTH_FORMATTER);
        List<MonthlyAssessment> finalizedThisMonth = finalizedAssessments().stream()
                .filter(item -> currentMonth.equals(item.getAssessMonth()))
                .toList();

        double averageScore = averageScore(finalizedThisMonth);
        int pendingAssessments = (int) listAssessments(null, null, null, null, 1, Integer.MAX_VALUE).getRecords().stream()
                .filter(item -> item.getStatus() != AssessmentStatus.FINALIZED)
                .count();
        int overdueTasks = (int) listTasks(null, null, null, null, null, 1, Integer.MAX_VALUE).getRecords().stream()
                .filter(item -> item.getDeadline() != null && item.getDeadline().isBefore(LocalDate.now()))
                .filter(item -> item.getStatus() != TaskStatus.DONE && item.getStatus() != TaskStatus.CANCELLED)
                .count();
        int reportsPendingReview = listReports(null, null, WeeklyReportStatus.SUBMITTED, 1, Integer.MAX_VALUE).getRecords().size();
        int activeProjects = (int) listProjects(null, null, null, 1, Integer.MAX_VALUE).getRecords().stream()
                .filter(item -> item.getStatus() == ProjectStatus.IN_PROGRESS || item.getStatus() == ProjectStatus.PLANNING)
                .count();

        return new DashboardData(roundToTwo(averageScore), pendingAssessments, overdueTasks, reportsPendingReview, activeProjects);
    }

    public MonthlyStatistics getMonthlyStatistics(String month, Long teamId) {
        String targetMonth = month == null ? YearMonth.now().format(MONTH_FORMATTER) : month;
        List<MonthlyAssessment> filtered = finalizedAssessments().stream()
                .filter(item -> targetMonth.equals(item.getAssessMonth()))
                .filter(item -> teamId == null || Objects.equals(item.getTeamId(), teamId))
                .toList();

        MonthlyStatistics statistics = new MonthlyStatistics();
        statistics.setMonth(targetMonth);
        statistics.setAverageScore(roundToTwo(averageScore(filtered)));
        statistics.setHighestScore(filtered.stream().map(MonthlyAssessment::getFinalScore).filter(Objects::nonNull).max(Double::compareTo).orElse(0.0));
        statistics.setLowestScore(filtered.stream().map(MonthlyAssessment::getFinalScore).filter(Objects::nonNull).min(Double::compareTo).orElse(0.0));
        statistics.setAssessedCount(filtered.size());
        statistics.setTeamComparison(buildTeamComparison(filtered));
        return statistics;
    }

    public QuarterlyStatistics getQuarterlyStatistics(String quarter, Long teamId) {
        String targetQuarter = quarter == null ? currentQuarter() : quarter;
        List<String> months = monthsForQuarter(targetQuarter);
        List<MonthlyAssessment> filtered = finalizedAssessments().stream()
                .filter(item -> months.contains(item.getAssessMonth()))
                .filter(item -> teamId == null || Objects.equals(item.getTeamId(), teamId))
                .toList();

        QuarterlyStatistics statistics = new QuarterlyStatistics();
        statistics.setQuarter(targetQuarter);
        statistics.setAverageScore(roundToTwo(averageScore(filtered)));
        statistics.setAssessedCount(filtered.size());
        statistics.setTrend(buildAverageScoreTrend(months, filtered));
        return statistics;
    }

    public YearlyStatistics getYearlyStatistics(String year, Long teamId) {
        String targetYear = year == null ? String.valueOf(LocalDate.now().getYear()) : year;
        List<String> months = buildMonthsOfYear(targetYear);
        List<MonthlyAssessment> filtered = finalizedAssessments().stream()
                .filter(item -> item.getAssessMonth().startsWith(targetYear + "-"))
                .filter(item -> teamId == null || Objects.equals(item.getTeamId(), teamId))
                .toList();

        YearlyStatistics statistics = new YearlyStatistics();
        statistics.setYear(targetYear);
        statistics.setAverageScore(roundToTwo(averageScore(filtered)));
        statistics.setAssessedCount(filtered.size());
        statistics.setMonthlyTrend(buildAverageScoreTrend(months, filtered));
        return statistics;
    }

    public PageResult<AuditLog> listAuditLogs(Long operatorId, String actionType, String targetType, AuditResult result, Integer pageNo, Integer pageSize) {
        StringBuilder sql = new StringBuilder(
                "SELECT al.*, su.real_name AS operator_name " +
                        "FROM audit_log al " +
                        "LEFT JOIN sys_user su ON su.id = al.operator_user_id WHERE 1=1"
        );
        List<Object> args = new ArrayList<>();
        if (operatorId != null) {
            sql.append(" AND al.operator_user_id = ?");
            args.add(operatorId);
        }
        if (actionType != null && !actionType.isBlank()) {
            sql.append(" AND al.action_type LIKE ?");
            args.add("%" + actionType.trim() + "%");
        }
        if (targetType != null && !targetType.isBlank()) {
            sql.append(" AND al.target_type LIKE ?");
            args.add("%" + targetType.trim() + "%");
        }
        if (result != null) {
            sql.append(" AND al.result = ?");
            args.add(result.name());
        }
        sql.append(" ORDER BY al.id DESC");

        List<AuditLog> logs = jdbcTemplate.query(sql.toString(), (rs, rowNum) -> mapAuditLog(rs), args.toArray());
        return buildPage(logs, pageNo, pageSize);
    }

    public AuditLog getAuditLog(Long logId) {
        AuditLog auditLog = findAuditLog(logId);
        if (auditLog == null) {
            throw new IllegalArgumentException("Audit log not found");
        }
        return auditLog;
    }

    private void seedPermissions() {
        permissionMap.put(RoleCode.SYSTEM_ADMIN, List.of("user:read", "user:write", "team:write", "member:write", "project:write", "audit:read"));
        permissionMap.put(RoleCode.DEPT_MANAGER, List.of("team:read", "task:assign", "report:review", "assessment:score"));
        permissionMap.put(RoleCode.TECH_DIRECTOR, List.of("project:read", "assessment:score", "statistics:read"));
        permissionMap.put(RoleCode.GENERAL_MANAGER, List.of("assessment:score", "assessment:finalize", "statistics:read"));
        permissionMap.put(RoleCode.AUDIT_ADMIN, List.of("audit:read"));
        permissionMap.put(RoleCode.TEAM_MEMBER, List.of("task:read", "task:update", "report:write"));
    }

    @Transactional
    protected void seedDemoDataIfNeeded() {
        Long userCount = jdbcTemplate.queryForObject("SELECT COUNT(*) FROM sys_user", Long.class);
        if (userCount != null && userCount > 0) {
            return;
        }

        LocalDateTime now = LocalDateTime.now();
        insertUser(1L, "manager01", "Manager Zhang", "[PHONE]", "[EMAIL]", 10L, null, false, UserStatus.ENABLED, now.minusDays(20), now.minusDays(1));
        insertUser(2L, "director01", "Director Li", "[PHONE]", "[EMAIL]", null, null, false, UserStatus.ENABLED, now.minusDays(20), now.minusDays(1));
        insertUser(3L, "admin01", "Admin Chen", "[PHONE]", "[EMAIL]", null, null, false, UserStatus.ENABLED, now.minusDays(20), now.minusDays(1));
        insertUser(4L, "gm01", "General Wang", "[PHONE]", "[EMAIL]", null, null, false, UserStatus.ENABLED, now.minusDays(20), now.minusDays(1));
        insertUser(5L, "audit01", "Audit Zhao", "[PHONE]", "[EMAIL]", null, null, false, UserStatus.ENABLED, now.minusDays(20), now.minusDays(1));
        insertUser(6L, "dev01", "Developer Wu", "[PHONE]", "[EMAIL]", 10L, 1001L, true, UserStatus.ENABLED, now.minusDays(20), now.minusDays(1));
        insertUser(7L, "qa01", "Tester Xu", "[PHONE]", "[EMAIL]", 11L, 1002L, false, UserStatus.ENABLED, now.minusDays(20), now.minusDays(1));
        insertUser(8L, "manager02", "Manager Zhou", "[PHONE]", "[EMAIL]", 11L, null, false, UserStatus.ENABLED, now.minusDays(20), now.minusDays(1));

        insertUserRole(1L, requireRoleId(RoleCode.DEPT_MANAGER));
        insertUserRole(2L, requireRoleId(RoleCode.TECH_DIRECTOR));
        insertUserRole(3L, requireRoleId(RoleCode.SYSTEM_ADMIN));
        insertUserRole(4L, requireRoleId(RoleCode.GENERAL_MANAGER));
        insertUserRole(5L, requireRoleId(RoleCode.AUDIT_ADMIN));
        insertUserRole(6L, requireRoleId(RoleCode.TEAM_MEMBER));
        insertUserRole(7L, requireRoleId(RoleCode.TEAM_MEMBER));
        insertUserRole(8L, requireRoleId(RoleCode.DEPT_MANAGER));

        jdbcTemplate.update(
                "INSERT INTO team (id, team_name, department_name, manager_user_id, description, status, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                10L, "Platform RnD", "Engineering Center", 1L, "Builds the backend platform and assessment module", TeamStatus.ACTIVE.name(), now.minusDays(30), now.minusDays(3)
        );
        jdbcTemplate.update(
                "INSERT INTO team (id, team_name, department_name, manager_user_id, description, status, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                11L, "QA Team", "Engineering Center", 8L, "Handles testing, quality and release support", TeamStatus.ACTIVE.name(), now.minusDays(30), now.minusDays(3)
        );

        jdbcTemplate.update(
                "INSERT INTO team_member (id, name, team_id, position, job_level, phone, email, status, entry_date) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                1001L, "Dev Wang", 10L, "Java Engineer", "P5", "[PHONE]", "[EMAIL]", MemberStatus.ACTIVE.name(), LocalDate.now().minusYears(1)
        );
        jdbcTemplate.update(
                "INSERT INTO team_member (id, name, team_id, position, job_level, phone, email, status, entry_date) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                1002L, "QA Zhao", 11L, "Test Engineer", "P4", "[PHONE]", "[EMAIL]", MemberStatus.ACTIVE.name(), LocalDate.now().minusMonths(10)
        );
        jdbcTemplate.update(
                "INSERT INTO team_member (id, name, team_id, position, job_level, phone, email, status, entry_date) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                1003L, "Backend Qian", 10L, "Backend Engineer", "P5", "[PHONE]", "[EMAIL]", MemberStatus.ACTIVE.name(), LocalDate.now().minusMonths(8)
        );
        jdbcTemplate.update(
                "INSERT INTO team_member (id, name, team_id, position, job_level, phone, email, status, entry_date) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                1004L, "Frontend Sun", 10L, "Frontend Engineer", "P5", "[PHONE]", "[EMAIL]", MemberStatus.ONBOARDING.name(), LocalDate.now().minusMonths(2)
        );

        jdbcTemplate.update(
                "INSERT INTO project (id, project_name, owner_member_id, status, priority, start_date, end_date, milestone, description) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                2001L, "Performance System V1.0", 1001L, ProjectStatus.IN_PROGRESS.name(), PriorityLevel.HIGH.name(), LocalDate.now().minusMonths(1), LocalDate.now().plusMonths(1), "Finish MVP", "Course design main project"
        );
        jdbcTemplate.update(
                "INSERT INTO project (id, project_name, owner_member_id, status, priority, start_date, end_date, milestone, description) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                2002L, "Testing Platform Upgrade", 1002L, ProjectStatus.PLANNING.name(), PriorityLevel.MEDIUM.name(), LocalDate.now().minusDays(15), LocalDate.now().plusMonths(2), "Improve automation chain", "Support daily QA work"
        );

        String weekNo = currentWeekNo();
        jdbcTemplate.update(
                "INSERT INTO weekly_task (id, title, project_id, assignee_member_id, priority, deadline, status, progress_rate, week_no, plan_start_date, plan_end_date, issue_desc) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                3001L, "Complete monthly assessment API integration", 2001L, 1001L, PriorityLevel.HIGH.name(), LocalDate.now().plusDays(2), TaskStatus.IN_PROGRESS.name(), 60, weekNo, LocalDate.now().minusDays(5), LocalDate.now().plusDays(2), null
        );
        jdbcTemplate.update(
                "INSERT INTO weekly_task (id, title, project_id, assignee_member_id, priority, deadline, status, progress_rate, week_no, plan_start_date, plan_end_date, issue_desc) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                3002L, "Add quarterly statistics endpoint", 2001L, 1003L, PriorityLevel.MEDIUM.name(), LocalDate.now().plusDays(5), TaskStatus.TODO.name(), 0, weekNo, LocalDate.now(), LocalDate.now().plusDays(5), null
        );
        jdbcTemplate.update(
                "INSERT INTO weekly_task (id, title, project_id, assignee_member_id, priority, deadline, status, progress_rate, week_no, plan_start_date, plan_end_date, issue_desc) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                3003L, "Prepare regression issue list", 2002L, 1002L, PriorityLevel.MEDIUM.name(), LocalDate.now().minusDays(1), TaskStatus.BLOCKED.name(), 35, weekNo, LocalDate.now().minusDays(4), LocalDate.now().plusDays(1), "integration environment unstable"
        );

        jdbcTemplate.update(
                "INSERT INTO task_progress (id, task_id, progress_rate, issue_desc, comment, update_time) VALUES (?, ?, ?, ?, ?, ?)",
                4001L, 3001L, 60, null, "Integrated APIs and fixed field mapping", now.minusDays(1)
        );
        jdbcTemplate.update(
                "INSERT INTO task_progress (id, task_id, progress_rate, issue_desc, comment, update_time) VALUES (?, ?, ?, ?, ?, ?)",
                4002L, 3003L, 35, "integration API timeout", "Regression checklist completed and waiting for environment recovery", now.minusHours(10)
        );

        jdbcTemplate.update(
                "INSERT INTO weekly_report (id, member_id, week_no, summary, next_plan, manager_comment, status, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                5001L, 1001L, weekNo, "Finished assessment integration and chart API design", "Complete quarterly summary and audit logs", "Good progress and ready for monthly scoring input", WeeklyReportStatus.REVIEWED.name(), now.minusDays(3), now.minusDays(2)
        );
        jdbcTemplate.update(
                "INSERT INTO weekly_report (id, member_id, week_no, summary, next_plan, manager_comment, status, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                5002L, 1002L, weekNo, "Prepared test plan and followed up integration blockers", "Complete regression and close blocking issues", null, WeeklyReportStatus.SUBMITTED.name(), now.minusDays(2), now.minusDays(2)
        );

        insertAssessmentWithScores(7001L, 1001L, previousMonth(), 86.0, 90.0, 88.0, "Stable delivery and solid API quality", true);
        insertAssessmentWithScores(7002L, 1002L, previousMonth(), 92.0, 91.0, 94.0, "Complete test coverage and timely support", true);
        insertAssessmentWithScores(7003L, 1003L, currentMonth(), 84.0, 87.0, 90.0, "Assessment finished for the month", true);
        insertAssessmentWithScores(7004L, 1004L, currentMonth(), null, null, null, null, false);

        writeAudit(DEMO_OPERATOR_ID, "SYSTEM_BOOTSTRAP", "system", "bootstrap", AuditResult.SUCCESS, "seed demo data");
    }

    private void ensureBaseRolesInitialized() {
        ensureRole(RoleCode.SYSTEM_ADMIN, "System Admin", "Manages users, roles, teams and base data");
        ensureRole(RoleCode.DEPT_MANAGER, "Department Manager", "Reviews reports and scores assessments");
        ensureRole(RoleCode.TECH_DIRECTOR, "Tech Director", "Supervises projects and technical scoring");
        ensureRole(RoleCode.GENERAL_MANAGER, "General Manager", "Final scoring and summary view");
        ensureRole(RoleCode.AUDIT_ADMIN, "Audit Admin", "Reviews audit logs and security records");
        ensureRole(RoleCode.TEAM_MEMBER, "Team Member", "Executes tasks and submits weekly reports");
    }

    private void ensureRole(RoleCode roleCode, String roleName, String description) {
        Long count = jdbcTemplate.queryForObject(
                "SELECT COUNT(*) FROM sys_role WHERE role_code = ?",
                Long.class,
                roleCode.name()
        );
        if (count != null && count > 0) {
            return;
        }
        jdbcTemplate.update(
                "INSERT INTO sys_role (role_code, role_name, description) VALUES (?, ?, ?)",
                roleCode.name(),
                roleName,
                description
        );
    }

    private void insertUser(Long id, String username, String realName, String mobile, String email, Long teamId, Long memberId,
                            boolean mustChangePassword, UserStatus status, LocalDateTime createdAt, LocalDateTime updatedAt) {
        jdbcTemplate.update(
                "INSERT INTO sys_user (id, username, real_name, password_sm3, mobile, email, team_id, member_id, status, must_change_password, password_expire_at, last_login_at, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                id,
                username,
                realName,
                PasswordCodec.encode("P@ssw0rd123"),
                mobile,
                email,
                teamId,
                memberId,
                status.name(),
                mustChangePassword,
                LocalDateTime.now().plusDays(90),
                LocalDateTime.now().minusDays(1),
                createdAt,
                updatedAt
        );
    }

    private void insertUserRole(Long userId, Long roleId) {
        jdbcTemplate.update("INSERT INTO sys_user_role (user_id, role_id) VALUES (?, ?)", userId, roleId);
    }

    private void insertAssessmentWithScores(Long id, Long memberId, String assessMonth, Double deptScore, Double techScore,
                                            Double gmScore, String comment, boolean finalized) {
        LocalDateTime now = LocalDateTime.now();
        Double finalScore = null;
        String ratingLevel = null;
        AssessmentStatus status = AssessmentStatus.DRAFT;

        if (deptScore != null || techScore != null || gmScore != null) {
            status = AssessmentStatus.SCORING;
        }
        if (deptScore != null && techScore != null && gmScore != null) {
            finalScore = roundToTwo(deptScore * 0.4 + techScore * 0.3 + gmScore * 0.3);
            ratingLevel = toRating(finalScore);
            status = finalized ? AssessmentStatus.FINALIZED : AssessmentStatus.PENDING_FINALIZE;
        }

        jdbcTemplate.update(
                "INSERT INTO monthly_assessment (id, member_id, assess_month, dept_manager_score, tech_director_score, general_manager_score, final_score, rating_level, status, comment, created_at, updated_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                id,
                memberId,
                assessMonth,
                deptScore,
                techScore,
                gmScore,
                finalScore,
                ratingLevel,
                status.name(),
                comment,
                now.minusDays(10),
                now.minusDays(1)
        );

        if (deptScore != null) {
            insertScoreDetail(id, 1L, RoleCode.DEPT_MANAGER, deptScore, "Department manager score");
        }
        if (techScore != null) {
            insertScoreDetail(id, 2L, RoleCode.TECH_DIRECTOR, techScore, "Tech director score");
        }
        if (gmScore != null) {
            insertScoreDetail(id, 4L, RoleCode.GENERAL_MANAGER, gmScore, "General manager score");
        }
    }

    private void insertScoreDetail(Long assessmentId, Long scorerUserId, RoleCode roleCode, Double score, String comment) {
        jdbcTemplate.update(
                "INSERT INTO assessment_score_detail (assessment_id, scorer_user_id, scorer_role, raw_score, comment, score_time) VALUES (?, ?, ?, ?, ?, ?)",
                assessmentId,
                scorerUserId,
                roleCode.name(),
                score,
                comment,
                LocalDateTime.now().minusDays(5)
        );
    }

    private SysUser refreshUser(SysUser user) {
        user.setMobileMasked(MaskingUtils.maskMobile(user.getMobile()));
        return user;
    }

    private Team refreshTeam(Team team) {
        if (team.getManagerId() != null) {
            SysUser manager = findUserById(team.getManagerId());
            team.setManagerName(manager == null ? null : manager.getRealName());
        } else {
            team.setManagerName(null);
        }
        Long memberCount = jdbcTemplate.queryForObject(
                "SELECT COUNT(*) FROM team_member WHERE team_id = ? AND status <> ?",
                Long.class,
                team.getId(),
                MemberStatus.LEFT.name()
        );
        team.setMemberCount(memberCount == null ? 0 : memberCount.intValue());
        return team;
    }

    private Member refreshMember(Member member) {
        member.setPhoneMasked(MaskingUtils.maskMobile(member.getPhone()));
        if (member.getTeamId() != null && (member.getTeamName() == null || member.getTeamName().isBlank())) {
            Team team = findTeamById(member.getTeamId());
            member.setTeamName(team == null ? null : team.getTeamName());
        }
        return member;
    }

    private CurrentUser buildCurrentUser(SysUser user) {
        CurrentUser currentUser = new CurrentUser();
        currentUser.setUserId(user.getId());
        currentUser.setUsername(user.getUsername());
        currentUser.setRealName(user.getRealName());
        currentUser.setRoleCodes(new ArrayList<>(user.getRoleCodes()));
        currentUser.setPermissions(user.getRoleCodes().stream()
                .flatMap(roleCode -> permissionMap.getOrDefault(roleCode, Collections.emptyList()).stream())
                .distinct()
                .toList());
        currentUser.setTeamId(user.getTeamId());
        if (user.getTeamId() != null) {
            Team team = findTeamById(user.getTeamId());
            currentUser.setTeamName(team == null ? null : team.getTeamName());
        }
        currentUser.setMustChangePassword(Boolean.TRUE.equals(user.getMustChangePassword()));
        currentUser.setPasswordExpired(user.getPasswordExpireAt() != null && user.getPasswordExpireAt().isBefore(LocalDateTime.now()));
        currentUser.setLastLoginAt(user.getLastLoginAt());
        return currentUser;
    }

    private SysUser resolveCurrentUser(String authorizationHeader) {
        String token = extractBearerToken(authorizationHeader);
        if (token == null || token.isBlank()) {
            throw new IllegalArgumentException("Missing Authorization header");
        }
        Long userId = tokenSessions.get(token);
        if (userId == null) {
            throw new IllegalArgumentException("Login session has expired");
        }
        return requireUser(userId);
    }

    private String extractBearerToken(String authorizationHeader) {
        if (authorizationHeader == null || authorizationHeader.isBlank()) {
            return null;
        }
        if (authorizationHeader.startsWith("Bearer ")) {
            return authorizationHeader.substring(7).trim();
        }
        return authorizationHeader.trim();
    }

    private SysUser requireUser(Long userId) {
        SysUser user = findUserById(userId);
        if (user == null) {
            throw new IllegalArgumentException("User not found");
        }
        return user;
    }

    private Team requireTeam(Long teamId) {
        Team team = findTeamById(teamId);
        if (team == null) {
            throw new IllegalArgumentException("Team not found");
        }
        return team;
    }

    private Member requireMember(Long memberId) {
        Member member = findMemberById(memberId);
        if (member == null) {
            throw new IllegalArgumentException("Member not found");
        }
        return refreshMember(member);
    }

    private Project requireProject(Long projectId) {
        Project project = findProjectById(projectId);
        if (project == null) {
            throw new IllegalArgumentException("Project not found");
        }
        return project;
    }

    private Task requireTask(Long taskId) {
        Task task = findTaskById(taskId);
        if (task == null) {
            throw new IllegalArgumentException("Task not found");
        }
        return task;
    }

    private WeeklyReport requireReport(Long reportId) {
        WeeklyReport report = findReportById(reportId);
        if (report == null) {
            throw new IllegalArgumentException("Weekly report not found");
        }
        return report;
    }

    private MonthlyAssessment requireAssessment(Long assessmentId) {
        MonthlyAssessment assessment = findAssessmentById(assessmentId);
        if (assessment == null) {
            throw new IllegalArgumentException("Assessment not found");
        }
        return assessment;
    }

    private void validatePassword(String password) {
        boolean valid = password != null
                && password.length() >= 8
                && password.matches(".*[A-Z].*")
                && password.matches(".*[a-z].*")
                && password.matches(".*\\d.*")
                && password.matches(".*[^A-Za-z0-9].*");
        if (!valid) {
            throw new IllegalArgumentException("Password must include upper, lower, digit and special characters");
        }
    }

    private void validateTeamAndMemberBinding(Long teamId, Long memberId) {
        if (teamId != null) {
            requireTeam(teamId);
        }
        if (memberId != null) {
            Member member = requireMember(memberId);
            if (teamId != null && !Objects.equals(member.getTeamId(), teamId)) {
                throw new IllegalArgumentException("memberId does not belong to teamId");
            }
        }
    }

    private void ensureUsernameUnique(String username) {
        Long count = jdbcTemplate.queryForObject("SELECT COUNT(*) FROM sys_user WHERE username = ?", Long.class, username);
        if (count != null && count > 0) {
            throw new IllegalArgumentException("Username already exists");
        }
    }

    private void validateManagerUser(Long managerId) {
        if (managerId == null) {
            return;
        }
        SysUser manager = requireUser(managerId);
        if (!manager.getRoleCodes().contains(RoleCode.DEPT_MANAGER)) {
            throw new IllegalArgumentException("managerId is not a department manager");
        }
    }

    private boolean hasAnyRole(SysUser user, RoleCode... roleCodes) {
        for (RoleCode roleCode : roleCodes) {
            if (user.getRoleCodes().contains(roleCode)) {
                return true;
            }
        }
        return false;
    }

    private <T> PageResult<T> buildPage(List<T> source, Integer pageNo, Integer pageSize) {
        int safePageNo = pageNo == null || pageNo < 1 ? 1 : pageNo;
        int safePageSize = pageSize == null || pageSize < 1 ? 10 : pageSize;
        long total = source.size();
        long pages = total == 0 ? 0 : (total + safePageSize - 1) / safePageSize;
        int fromIndex = Math.min((safePageNo - 1) * safePageSize, source.size());
        int toIndex = Math.min(fromIndex + safePageSize, source.size());
        return new PageResult<>(source.subList(fromIndex, toIndex), safePageNo, safePageSize, total, pages);
    }

    private void writeAudit(Long operatorId, String actionType, String targetType, String targetId, AuditResult result, String detail) {
        jdbcTemplate.update(
                "INSERT INTO audit_log (operator_user_id, action_type, target_type, target_id, result, action_time, detail) VALUES (?, ?, ?, ?, ?, ?, ?)",
                operatorId,
                actionType,
                targetType,
                targetId,
                result.name(),
                LocalDateTime.now(),
                detail
        );
    }

    private long currentOperatorId() {
        return DEMO_OPERATOR_ID;
    }

    private List<MonthlyAssessment> finalizedAssessments() {
        return listAssessments(null, null, null, AssessmentStatus.FINALIZED, 1, Integer.MAX_VALUE).getRecords();
    }

    private double averageScore(List<MonthlyAssessment> items) {
        return items.stream()
                .map(MonthlyAssessment::getFinalScore)
                .filter(Objects::nonNull)
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(0);
    }

    private List<Map<String, Object>> buildTeamComparison(List<MonthlyAssessment> items) {
        return items.stream()
                .filter(item -> item.getTeamName() != null)
                .collect(Collectors.groupingBy(MonthlyAssessment::getTeamName, LinkedHashMap::new, Collectors.toList()))
                .entrySet()
                .stream()
                .map(entry -> {
                    Map<String, Object> row = new LinkedHashMap<>();
                    row.put("teamName", entry.getKey());
                    row.put("averageScore", roundToTwo(averageScore(entry.getValue())));
                    return row;
                })
                .toList();
    }

    private List<Map<String, Object>> buildAverageScoreTrend(List<String> periods, List<MonthlyAssessment> source) {
        return periods.stream()
                .map(period -> {
                    Map<String, Object> point = new LinkedHashMap<>();
                    point.put("month", period);
                    point.put("averageScore", roundToTwo(averageScoreByMonth(period, source)));
                    return point;
                })
                .toList();
    }

    private double averageScoreByMonth(String month, List<MonthlyAssessment> source) {
        return averageScore(source.stream()
                .filter(item -> month.equals(item.getAssessMonth()))
                .toList());
    }

    private String currentQuarter() {
        int month = LocalDate.now().getMonthValue();
        int quarter = (month - 1) / 3 + 1;
        return LocalDate.now().getYear() + "-Q" + quarter;
    }

    private List<String> monthsForQuarter(String quarter) {
        String[] parts = quarter.split("-Q");
        int year = Integer.parseInt(parts[0]);
        int quarterNo = Integer.parseInt(parts[1]);
        int startMonth = (quarterNo - 1) * 3 + 1;
        List<String> months = new ArrayList<>();
        for (int offset = 0; offset < 3; offset++) {
            months.add(year + "-" + String.format("%02d", startMonth + offset));
        }
        return months;
    }

    private List<String> buildMonthsOfYear(String year) {
        List<String> months = new ArrayList<>();
        for (int month = 1; month <= 12; month++) {
            months.add(year + "-" + String.format("%02d", month));
        }
        return months;
    }

    private Double roundToTwo(double value) {
        return Math.round(value * 100.0) / 100.0;
    }

    private String toRating(Double finalScore) {
        if (finalScore == null) {
            return null;
        }
        if (finalScore >= 90) {
            return "A";
        }
        if (finalScore >= 80) {
            return "B";
        }
        if (finalScore >= 70) {
            return "C";
        }
        return "D";
    }

    private String currentWeekNo() {
        WeekFields weekFields = WeekFields.ISO;
        LocalDate today = LocalDate.now();
        return today.getYear() + "-W" + String.format("%02d", today.get(weekFields.weekOfWeekBasedYear()));
    }

    private String currentMonth() {
        return YearMonth.now().format(MONTH_FORMATTER);
    }

    private String previousMonth() {
        return YearMonth.now().minusMonths(1).format(MONTH_FORMATTER);
    }

    private Double scoreOf(List<AssessmentScoreDetail> details, RoleCode roleCode) {
        return details.stream()
                .filter(item -> item.getScorerRole() == roleCode)
                .map(AssessmentScoreDetail::getRawScore)
                .findFirst()
                .orElse(null);
    }

    private void replaceUserRoles(Long userId, List<RoleCode> roleCodes) {
        jdbcTemplate.update("DELETE FROM sys_user_role WHERE user_id = ?", userId);
        for (Long roleId : findRoleIdsByCodes(roleCodes)) {
            insertUserRole(userId, roleId);
        }
    }

    private List<Long> findRoleIdsByCodes(List<RoleCode> roleCodes) {
        List<Long> roleIds = new ArrayList<>();
        for (RoleCode roleCode : roleCodes) {
            roleIds.add(requireRoleId(roleCode));
        }
        return roleIds;
    }

    private Long requireRoleId(RoleCode roleCode) {
        Long roleId = jdbcTemplate.queryForObject("SELECT id FROM sys_role WHERE role_code = ?", Long.class, roleCode.name());
        if (roleId == null) {
            throw new IllegalArgumentException("Role not found: " + roleCode.name());
        }
        return roleId;
    }

    private void loadRolesIntoUser(SysUser user) {
        List<RoleCode> roleCodes = jdbcTemplate.query(
                "SELECT sr.role_code FROM sys_user_role sur JOIN sys_role sr ON sr.id = sur.role_id WHERE sur.user_id = ? ORDER BY sr.id",
                (rs, rowNum) -> RoleCode.valueOf(rs.getString("role_code")),
                user.getId()
        );
        user.setRoleCodes(new ArrayList<>(roleCodes));
    }

    private SysUser requireUserByUsername(String username) {
        SysUser user = queryOne("SELECT * FROM sys_user WHERE username = ?", this::mapSysUser, username);
        if (user == null) {
            throw new IllegalArgumentException("User not found");
        }
        return user;
    }

    private SysUser findUserByUsername(String username) {
        SysUser user = queryOne("SELECT * FROM sys_user WHERE username = ?", this::mapSysUser, username);
        if (user != null) {
            loadRolesIntoUser(user);
            refreshUser(user);
        }
        return user;
    }

    private SysUser findUserById(Long userId) {
        SysUser user = queryOne("SELECT * FROM sys_user WHERE id = ?", this::mapSysUser, userId);
        if (user != null) {
            loadRolesIntoUser(user);
            refreshUser(user);
        }
        return user;
    }

    private Team findTeamById(Long teamId) {
        Team team = queryOne("SELECT * FROM team WHERE id = ?", this::mapTeam, teamId);
        return team == null ? null : refreshTeam(team);
    }

    private Member findMemberById(Long memberId) {
        return queryOne(
                "SELECT tm.*, t.team_name FROM team_member tm LEFT JOIN team t ON t.id = tm.team_id WHERE tm.id = ?",
                this::mapMember,
                memberId
        );
    }

    private Project findProjectById(Long projectId) {
        return queryOne(
                "SELECT p.*, m.name AS owner_name FROM project p LEFT JOIN team_member m ON m.id = p.owner_member_id WHERE p.id = ?",
                this::mapProject,
                projectId
        );
    }

    private Task findTaskById(Long taskId) {
        return queryOne(
                "SELECT t.*, p.project_name, m.name AS assignee_name FROM weekly_task t LEFT JOIN project p ON p.id = t.project_id LEFT JOIN team_member m ON m.id = t.assignee_member_id WHERE t.id = ?",
                this::mapTask,
                taskId
        );
    }

    private WeeklyReport findReportById(Long reportId) {
        return queryOne(
                "SELECT r.*, m.name AS member_name FROM weekly_report r LEFT JOIN team_member m ON m.id = r.member_id WHERE r.id = ?",
                this::mapWeeklyReport,
                reportId
        );
    }

    private MonthlyAssessment findAssessmentById(Long assessmentId) {
        MonthlyAssessment assessment = queryOne(
                "SELECT ma.*, m.name AS member_name, m.team_id, t.team_name FROM monthly_assessment ma LEFT JOIN team_member m ON m.id = ma.member_id LEFT JOIN team t ON t.id = m.team_id WHERE ma.id = ?",
                this::mapMonthlyAssessment,
                assessmentId
        );
        if (assessment != null) {
            assessment.setScoreDetails(loadScoreDetails(assessmentId));
        }
        return assessment;
    }

    private MonthlyAssessment findAssessmentByMemberAndMonth(Long memberId, String assessMonth) {
        return queryOne(
                "SELECT ma.*, m.name AS member_name, m.team_id, t.team_name FROM monthly_assessment ma LEFT JOIN team_member m ON m.id = ma.member_id LEFT JOIN team t ON t.id = m.team_id WHERE ma.member_id = ? AND ma.assess_month = ?",
                this::mapMonthlyAssessment,
                memberId,
                assessMonth
        );
    }

    private List<AssessmentScoreDetail> loadScoreDetails(Long assessmentId) {
        return jdbcTemplate.query(
                "SELECT d.*, u.real_name AS scorer_name FROM assessment_score_detail d LEFT JOIN sys_user u ON u.id = d.scorer_user_id WHERE d.assessment_id = ? ORDER BY d.id",
                (rs, rowNum) -> mapAssessmentScoreDetail(rs),
                assessmentId
        );
    }

    private AuditLog findAuditLog(Long logId) {
        return queryOne(
                "SELECT al.*, su.real_name AS operator_name FROM audit_log al LEFT JOIN sys_user su ON su.id = al.operator_user_id WHERE al.id = ?",
                this::mapAuditLog,
                logId
        );
    }

    private Long findLastInsertedId(String tableName) {
        return jdbcTemplate.queryForObject("SELECT MAX(id) FROM " + tableName, Long.class);
    }

    private <T> T queryOne(String sql, ResultMapper<T> mapper, Object... args) {
        List<T> rows = jdbcTemplate.query(sql, (rs, rowNum) -> mapper.map(rs), args);
        return rows.isEmpty() ? null : rows.get(0);
    }

    private SysUser mapSysUser(ResultSet rs) throws SQLException {
        SysUser user = new SysUser();
        user.setId(rs.getLong("id"));
        user.setUsername(rs.getString("username"));
        user.setRealName(rs.getString("real_name"));
        user.setMobile(rs.getString("mobile"));
        user.setEmail(rs.getString("email"));
        user.setTeamId(getLong(rs, "team_id"));
        user.setMemberId(getLong(rs, "member_id"));
        user.setStatus(enumValue(UserStatus.class, rs.getString("status")));
        user.setPasswordHash(rs.getString("password_sm3"));
        user.setMustChangePassword(rs.getBoolean("must_change_password"));
        user.setPasswordExpireAt(rs.getObject("password_expire_at", LocalDateTime.class));
        user.setLastLoginAt(rs.getObject("last_login_at", LocalDateTime.class));
        user.setCreatedAt(rs.getObject("created_at", LocalDateTime.class));
        user.setUpdatedAt(rs.getObject("updated_at", LocalDateTime.class));
        user.setRoleCodes(new ArrayList<>());
        return user;
    }

    private Team mapTeam(ResultSet rs) throws SQLException {
        Team team = new Team();
        team.setId(rs.getLong("id"));
        team.setTeamName(rs.getString("team_name"));
        team.setDepartmentName(rs.getString("department_name"));
        team.setManagerId(getLong(rs, "manager_user_id"));
        team.setDescription(rs.getString("description"));
        team.setStatus(enumValue(TeamStatus.class, rs.getString("status")));
        team.setCreatedAt(rs.getObject("created_at", LocalDateTime.class));
        team.setUpdatedAt(rs.getObject("updated_at", LocalDateTime.class));
        return team;
    }

    private Member mapMember(ResultSet rs) throws SQLException {
        Member member = new Member();
        member.setId(rs.getLong("id"));
        member.setName(rs.getString("name"));
        member.setTeamId(getLong(rs, "team_id"));
        member.setTeamName(readOptionalString(rs, "team_name"));
        member.setPosition(rs.getString("position"));
        member.setJobLevel(rs.getString("job_level"));
        member.setPhone(rs.getString("phone"));
        member.setEmail(rs.getString("email"));
        member.setStatus(enumValue(MemberStatus.class, rs.getString("status")));
        member.setEntryDate(rs.getObject("entry_date", LocalDate.class));
        return member;
    }

    private Project mapProject(ResultSet rs) throws SQLException {
        Project project = new Project();
        project.setId(rs.getLong("id"));
        project.setProjectName(rs.getString("project_name"));
        project.setOwnerId(getLong(rs, "owner_member_id"));
        project.setOwnerName(readOptionalString(rs, "owner_name"));
        project.setStatus(enumValue(ProjectStatus.class, rs.getString("status")));
        project.setPriority(enumValue(PriorityLevel.class, rs.getString("priority")));
        project.setStartDate(rs.getObject("start_date", LocalDate.class));
        project.setEndDate(rs.getObject("end_date", LocalDate.class));
        project.setMilestone(rs.getString("milestone"));
        project.setDescription(rs.getString("description"));
        return project;
    }

    private Task mapTask(ResultSet rs) throws SQLException {
        Task task = new Task();
        task.setId(rs.getLong("id"));
        task.setTitle(rs.getString("title"));
        task.setProjectId(getLong(rs, "project_id"));
        task.setProjectName(readOptionalString(rs, "project_name"));
        task.setAssigneeId(rs.getLong("assignee_member_id"));
        task.setAssigneeName(readOptionalString(rs, "assignee_name"));
        task.setPriority(enumValue(PriorityLevel.class, rs.getString("priority")));
        task.setDeadline(rs.getObject("deadline", LocalDate.class));
        task.setStatus(enumValue(TaskStatus.class, rs.getString("status")));
        task.setProgressRate(rs.getInt("progress_rate"));
        task.setWeekNo(rs.getString("week_no"));
        task.setPlanStartDate(rs.getObject("plan_start_date", LocalDate.class));
        task.setPlanEndDate(rs.getObject("plan_end_date", LocalDate.class));
        task.setIssueDesc(rs.getString("issue_desc"));
        return task;
    }

    private TaskProgress mapTaskProgress(ResultSet rs) throws SQLException {
        TaskProgress progress = new TaskProgress();
        progress.setId(rs.getLong("id"));
        progress.setTaskId(rs.getLong("task_id"));
        progress.setProgressRate(rs.getInt("progress_rate"));
        progress.setIssueDesc(rs.getString("issue_desc"));
        progress.setComment(rs.getString("comment"));
        progress.setUpdateTime(rs.getObject("update_time", LocalDateTime.class));
        return progress;
    }

    private WeeklyReport mapWeeklyReport(ResultSet rs) throws SQLException {
        WeeklyReport report = new WeeklyReport();
        report.setId(rs.getLong("id"));
        report.setMemberId(rs.getLong("member_id"));
        report.setMemberName(readOptionalString(rs, "member_name"));
        report.setWeekNo(rs.getString("week_no"));
        report.setSummary(rs.getString("summary"));
        report.setNextPlan(rs.getString("next_plan"));
        report.setManagerComment(rs.getString("manager_comment"));
        report.setStatus(enumValue(WeeklyReportStatus.class, rs.getString("status")));
        report.setCreatedAt(rs.getObject("created_at", LocalDateTime.class));
        report.setUpdatedAt(rs.getObject("updated_at", LocalDateTime.class));
        return report;
    }

    private MonthlyAssessment mapMonthlyAssessment(ResultSet rs) throws SQLException {
        MonthlyAssessment assessment = new MonthlyAssessment();
        assessment.setId(rs.getLong("id"));
        assessment.setMemberId(rs.getLong("member_id"));
        assessment.setMemberName(readOptionalString(rs, "member_name"));
        assessment.setTeamId(readOptionalLong(rs, "team_id"));
        assessment.setTeamName(readOptionalString(rs, "team_name"));
        assessment.setAssessMonth(rs.getString("assess_month"));
        assessment.setDeptManagerScore(readOptionalDouble(rs, "dept_manager_score"));
        assessment.setTechDirectorScore(readOptionalDouble(rs, "tech_director_score"));
        assessment.setGeneralManagerScore(readOptionalDouble(rs, "general_manager_score"));
        assessment.setFinalScore(readOptionalDouble(rs, "final_score"));
        assessment.setRatingLevel(rs.getString("rating_level"));
        assessment.setStatus(enumValue(AssessmentStatus.class, rs.getString("status")));
        assessment.setComment(rs.getString("comment"));
        assessment.setCreatedAt(rs.getObject("created_at", LocalDateTime.class));
        assessment.setUpdatedAt(rs.getObject("updated_at", LocalDateTime.class));
        assessment.setScoreDetails(new ArrayList<>());
        return assessment;
    }

    private AssessmentScoreDetail mapAssessmentScoreDetail(ResultSet rs) throws SQLException {
        AssessmentScoreDetail detail = new AssessmentScoreDetail();
        detail.setId(rs.getLong("id"));
        detail.setAssessmentId(rs.getLong("assessment_id"));
        detail.setScorerUserId(rs.getLong("scorer_user_id"));
        detail.setScorerName(readOptionalString(rs, "scorer_name"));
        detail.setScorerRole(enumValue(RoleCode.class, rs.getString("scorer_role")));
        detail.setRawScore(readOptionalDouble(rs, "raw_score"));
        detail.setComment(rs.getString("comment"));
        detail.setScoreTime(rs.getObject("score_time", LocalDateTime.class));
        return detail;
    }

    private AuditLog mapAuditLog(ResultSet rs) throws SQLException {
        AuditLog auditLog = new AuditLog();
        auditLog.setId(rs.getLong("id"));
        auditLog.setOperatorId(readOptionalLong(rs, "operator_user_id"));
        String operatorName = readOptionalString(rs, "operator_name");
        auditLog.setOperatorName(operatorName == null || operatorName.isBlank() ? "system" : operatorName);
        auditLog.setActionType(rs.getString("action_type"));
        auditLog.setTargetType(rs.getString("target_type"));
        auditLog.setTargetId(rs.getString("target_id"));
        auditLog.setResult(enumValue(AuditResult.class, rs.getString("result")));
        auditLog.setActionTime(rs.getObject("action_time", LocalDateTime.class));
        auditLog.setDetail(rs.getString("detail"));
        return auditLog;
    }

    private Long getLong(ResultSet rs, String column) throws SQLException {
        long value = rs.getLong(column);
        return rs.wasNull() ? null : value;
    }

    private Long readOptionalLong(ResultSet rs, String column) {
        try {
            return getLong(rs, column);
        } catch (SQLException ex) {
            return null;
        }
    }

    private Double readOptionalDouble(ResultSet rs, String column) {
        try {
            double value = rs.getDouble(column);
            return rs.wasNull() ? null : value;
        } catch (SQLException ex) {
            return null;
        }
    }

    private String readOptionalString(ResultSet rs, String column) {
        try {
            rs.findColumn(column);
            return rs.getString(column);
        } catch (SQLException ex) {
            return null;
        }
    }

    private <E extends Enum<E>> E enumValue(Class<E> enumType, String value) {
        return value == null ? null : Enum.valueOf(enumType, value);
    }

    @FunctionalInterface
    private interface ResultMapper<T> {
        T map(ResultSet rs) throws SQLException;
    }
}
