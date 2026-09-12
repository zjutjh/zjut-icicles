export interface ApiResponseEnvelope<T> {
  code: number
  message?: string
  msg?: string
  data: T
}

export interface PageResult<T> {
  records: T[]
  pageNo: number
  pageSize: number
  total: number
  pages: number
}

export interface PageQuery {
  pageNo?: number
  pageSize?: number
}

export type RoleCode =
  | 'SYSTEM_ADMIN'
  | 'DEPT_MANAGER'
  | 'TECH_DIRECTOR'
  | 'GENERAL_MANAGER'
  | 'AUDIT_ADMIN'
  | 'TEAM_MEMBER'

export type UserStatus = 'ENABLED' | 'DISABLED' | 'LOCKED'
export type TeamStatus = 'ACTIVE' | 'INACTIVE'
export type MemberStatus = 'ONBOARDING' | 'ACTIVE' | 'SUSPENDED' | 'LEFT'
export type ProjectStatus = 'PLANNING' | 'IN_PROGRESS' | 'ON_HOLD' | 'COMPLETED' | 'CLOSED'
export type TaskStatus = 'TODO' | 'IN_PROGRESS' | 'BLOCKED' | 'DONE' | 'CANCELLED'
export type WeeklyReportStatus = 'DRAFT' | 'SUBMITTED' | 'REVIEWED' | 'RETURNED'
export type AssessmentStatus = 'DRAFT' | 'SCORING' | 'PENDING_FINALIZE' | 'FINALIZED'
export type PriorityLevel = 'LOW' | 'MEDIUM' | 'HIGH' | 'CRITICAL'
export type AuditResult = 'SUCCESS' | 'FAIL'

export interface Role {
  code: RoleCode
  name: string
  description?: string
}

export interface CurrentUser {
  userId: number
  username: string
  realName: string
  roleCodes: RoleCode[]
  permissions: string[]
  teamId?: number | null
  teamName?: string | null
  mustChangePassword?: boolean
  passwordExpired?: boolean
  lastLoginAt?: string | null
}

export interface LoginRequest {
  username: string
  password: string
}

export interface LoginResponseData {
  token: string
  tokenType: string
  expiresIn: number
  user: CurrentUser
}

export interface ChangePasswordRequest {
  oldPassword: string
  newPassword: string
}

export interface User {
  id: number
  username: string
  realName: string
  mobileMasked?: string | null
  email?: string | null
  roleCodes: RoleCode[]
  teamId?: number | null
  memberId?: number | null
  status: UserStatus
  passwordExpireAt?: string | null
  lastLoginAt?: string | null
  createdAt?: string
  updatedAt?: string
}

export interface UserListQuery extends PageQuery {
  keyword?: string
  roleCode?: RoleCode
  status?: UserStatus
}

export interface CreateUserRequest {
  username: string
  password: string
  realName: string
  mobile?: string
  email?: string
  roleCodes: RoleCode[]
  teamId?: number | null
  memberId?: number | null
  mustChangePassword?: boolean
}

export interface UpdateUserRequest {
  realName: string
  mobile?: string
  email?: string
  roleCodes: RoleCode[]
  teamId?: number | null
  memberId?: number | null
}

export interface UserStatusUpdateRequest {
  status: UserStatus
  reason?: string
}

export interface Team {
  id: number
  teamName: string
  departmentName: string
  managerId?: number | null
  managerName?: string | null
  description?: string | null
  status: TeamStatus
  memberCount?: number
  createdAt?: string
  updatedAt?: string
}

export interface TeamListQuery extends PageQuery {
  keyword?: string
  status?: TeamStatus
}

export interface CreateTeamRequest {
  teamName: string
  departmentName: string
  managerId?: number | null
  description?: string
}

export interface UpdateTeamRequest extends CreateTeamRequest {
  status: TeamStatus
}

export interface Member {
  id: number
  name: string
  teamId?: number | null
  teamName?: string | null
  position?: string | null
  jobLevel?: string | null
  phoneMasked?: string | null
  email?: string | null
  status: MemberStatus
  entryDate?: string | null
}

export interface MemberListQuery extends PageQuery {
  keyword?: string
  teamId?: number
  status?: MemberStatus
}

export interface CreateMemberRequest {
  name: string
  teamId?: number | null
  position?: string
  jobLevel?: string
  phone?: string
  email?: string
  entryDate?: string
}

export type UpdateMemberRequest = CreateMemberRequest

export interface MemberStatusUpdateRequest {
  status: MemberStatus
  reason?: string
}

export interface Project {
  id: number
  projectName: string
  ownerId?: number | null
  ownerName?: string | null
  status: ProjectStatus
  priority: PriorityLevel
  startDate: string
  endDate?: string | null
  milestone?: string | null
  description?: string | null
}

export interface ProjectListQuery extends PageQuery {
  keyword?: string
  status?: ProjectStatus
  priority?: PriorityLevel
}

export interface CreateProjectRequest {
  projectName: string
  ownerId?: number | null
  status: ProjectStatus
  priority: PriorityLevel
  startDate: string
  endDate?: string
  milestone?: string
  description?: string
}

export interface UpdateProjectRequest {
  projectName: string
  ownerId?: number | null
  priority: PriorityLevel
  startDate: string
  endDate?: string
  milestone?: string
  description?: string
}

export interface ProjectStatusUpdateRequest {
  status: ProjectStatus
  comment?: string
}

export interface Task {
  id: number
  title: string
  projectId?: number | null
  projectName?: string | null
  assigneeId: number
  assigneeName?: string | null
  priority: PriorityLevel
  deadline?: string | null
  status: TaskStatus
  progressRate?: number | null
  weekNo?: string | null
  planStartDate?: string | null
  planEndDate?: string | null
  issueDesc?: string | null
}

export interface TaskListQuery extends PageQuery {
  weekNo?: string
  assigneeId?: number
  projectId?: number
  status?: TaskStatus
  priority?: PriorityLevel
}

export interface CreateTaskRequest {
  title: string
  projectId?: number | null
  assigneeId: number
  priority: PriorityLevel
  deadline?: string
  weekNo: string
  planStartDate?: string
  planEndDate?: string
  issueDesc?: string
}

export interface UpdateTaskRequest {
  title: string
  projectId?: number | null
  assigneeId: number
  priority: PriorityLevel
  deadline?: string
  planStartDate?: string
  planEndDate?: string
  issueDesc?: string
}

export interface TaskStatusUpdateRequest {
  status: TaskStatus
  progressRate?: number | null
  issueDesc?: string
}

export interface TaskProgress {
  id: number
  taskId: number
  progressRate: number
  issueDesc?: string | null
  comment?: string | null
  updateTime: string
}

export interface CreateTaskProgressRequest {
  progressRate: number
  issueDesc?: string
  comment: string
}

export interface WeeklyReport {
  id: number
  memberId: number
  memberName?: string | null
  weekNo: string
  summary: string
  nextPlan: string
  managerComment?: string | null
  status: WeeklyReportStatus
  createdAt?: string
  updatedAt?: string
}

export interface WeeklyReportListQuery extends PageQuery {
  weekNo?: string
  memberId?: number
  status?: WeeklyReportStatus
}

export interface CreateWeeklyReportRequest {
  memberId: number
  weekNo: string
  summary: string
  nextPlan: string
}

export interface UpdateWeeklyReportRequest {
  summary: string
  nextPlan: string
}

export interface ReviewWeeklyReportRequest {
  status: 'REVIEWED' | 'RETURNED'
  managerComment?: string
}

export interface AssessmentScoreDetail {
  id: number
  assessmentId: number
  scorerUserId: number
  scorerName?: string | null
  scorerRole: RoleCode
  rawScore: number
  comment?: string | null
  scoreTime: string
}

export interface MonthlyAssessment {
  id: number
  memberId: number
  memberName: string
  teamId?: number | null
  teamName?: string | null
  assessMonth: string
  deptManagerScore?: number | null
  techDirectorScore?: number | null
  generalManagerScore?: number | null
  finalScore?: number | null
  ratingLevel?: string | null
  status: AssessmentStatus
  comment?: string | null
  scoreDetails?: AssessmentScoreDetail[]
  createdAt?: string
  updatedAt?: string
}

export interface AssessmentListQuery extends PageQuery {
  assessMonth?: string
  teamId?: number
  memberId?: number
  status?: AssessmentStatus
}

export interface GenerateAssessmentRequest {
  assessMonth: string
  teamId?: number | null
  overwriteExisting?: boolean
}

export interface GenerateAssessmentResult {
  assessMonth: string
  generatedCount: number
  skippedCount: number
}

export interface ScoreAssessmentRequest {
  scorerRole: RoleCode
  rawScore: number
  comment?: string
}

export interface FinalizeAssessmentRequest {
  comment?: string
}

export interface TeamComparisonItem {
  teamName: string
  averageScore: number
}

export interface MonthlyStatistics {
  month: string
  averageScore: number
  highestScore: number
  lowestScore: number
  assessedCount: number
  teamComparison?: TeamComparisonItem[]
}

export interface TrendPoint {
  month: string
  averageScore: number
}

export interface QuarterlyStatistics {
  quarter: string
  averageScore: number
  assessedCount: number
  trend: TrendPoint[]
}

export interface YearlyStatistics {
  year: string
  averageScore: number
  assessedCount: number
  monthlyTrend: TrendPoint[]
}

export interface DashboardData {
  currentMonthAverageScore: number
  pendingAssessments: number
  overdueTasks: number
  weeklyReportsPendingReview: number
  activeProjects: number
}

export interface AuditLog {
  id: number
  operatorId: number
  operatorName?: string | null
  actionType: string
  targetType: string
  targetId?: string | null
  result: AuditResult
  actionTime: string
  detail?: string | null
}

export interface AuditLogListQuery extends PageQuery {
  operatorId?: number
  actionType?: string
  targetType?: string
  result?: AuditResult
}
