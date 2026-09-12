import type {
  AssessmentStatus,
  AuditResult,
  MemberStatus,
  PriorityLevel,
  ProjectStatus,
  RoleCode,
  TaskStatus,
  TeamStatus,
  UserStatus,
  WeeklyReportStatus,
} from '@/types/api'

export interface LabeledOption<T extends string> {
  label: string
  value: T
}

export interface StatusMeta {
  label: string
  tagType: 'success' | 'warning' | 'danger' | 'info' | 'primary'
}

function createMetaMap<T extends string>(items: Array<[T, StatusMeta]>) {
  return Object.fromEntries(items) as Record<T, StatusMeta>
}

export const userStatusOptions: LabeledOption<UserStatus>[] = [
  { label: '启用', value: 'ENABLED' },
  { label: '禁用', value: 'DISABLED' },
  { label: '锁定', value: 'LOCKED' },
]

export const teamStatusOptions: LabeledOption<TeamStatus>[] = [
  { label: '启用', value: 'ACTIVE' },
  { label: '停用', value: 'INACTIVE' },
]

export const memberStatusOptions: LabeledOption<MemberStatus>[] = [
  { label: '入职中', value: 'ONBOARDING' },
  { label: '在职', value: 'ACTIVE' },
  { label: '暂停', value: 'SUSPENDED' },
  { label: '离职', value: 'LEFT' },
]

export const projectStatusOptions: LabeledOption<ProjectStatus>[] = [
  { label: '规划中', value: 'PLANNING' },
  { label: '进行中', value: 'IN_PROGRESS' },
  { label: '已暂停', value: 'ON_HOLD' },
  { label: '已完成', value: 'COMPLETED' },
  { label: '已关闭', value: 'CLOSED' },
]

export const taskStatusOptions: LabeledOption<TaskStatus>[] = [
  { label: '待处理', value: 'TODO' },
  { label: '进行中', value: 'IN_PROGRESS' },
  { label: '阻塞中', value: 'BLOCKED' },
  { label: '已完成', value: 'DONE' },
  { label: '已取消', value: 'CANCELLED' },
]

export const weeklyReportStatusOptions: LabeledOption<WeeklyReportStatus>[] = [
  { label: '草稿', value: 'DRAFT' },
  { label: '已提交', value: 'SUBMITTED' },
  { label: '已评审', value: 'REVIEWED' },
  { label: '已退回', value: 'RETURNED' },
]

export const assessmentStatusOptions: LabeledOption<AssessmentStatus>[] = [
  { label: '草稿', value: 'DRAFT' },
  { label: '评分中', value: 'SCORING' },
  { label: '待归档', value: 'PENDING_FINALIZE' },
  { label: '已归档', value: 'FINALIZED' },
]

export const priorityOptions: LabeledOption<PriorityLevel>[] = [
  { label: '低', value: 'LOW' },
  { label: '中', value: 'MEDIUM' },
  { label: '高', value: 'HIGH' },
  { label: '紧急', value: 'CRITICAL' },
]

export const auditResultOptions: LabeledOption<AuditResult>[] = [
  { label: '成功', value: 'SUCCESS' },
  { label: '失败', value: 'FAIL' },
]

export const assessmentScoreRoleOptions: LabeledOption<RoleCode>[] = [
  { label: '部门经理', value: 'DEPT_MANAGER' },
  { label: '技术总监', value: 'TECH_DIRECTOR' },
  { label: '总经理', value: 'GENERAL_MANAGER' },
]

export const userStatusMetaMap = createMetaMap<UserStatus>([
  ['ENABLED', { label: '启用', tagType: 'success' }],
  ['DISABLED', { label: '禁用', tagType: 'info' }],
  ['LOCKED', { label: '锁定', tagType: 'danger' }],
])

export const teamStatusMetaMap = createMetaMap<TeamStatus>([
  ['ACTIVE', { label: '启用', tagType: 'success' }],
  ['INACTIVE', { label: '停用', tagType: 'info' }],
])

export const memberStatusMetaMap = createMetaMap<MemberStatus>([
  ['ONBOARDING', { label: '入职中', tagType: 'warning' }],
  ['ACTIVE', { label: '在职', tagType: 'success' }],
  ['SUSPENDED', { label: '暂停', tagType: 'danger' }],
  ['LEFT', { label: '离职', tagType: 'info' }],
])

export const projectStatusMetaMap = createMetaMap<ProjectStatus>([
  ['PLANNING', { label: '规划中', tagType: 'info' }],
  ['IN_PROGRESS', { label: '进行中', tagType: 'warning' }],
  ['ON_HOLD', { label: '已暂停', tagType: 'danger' }],
  ['COMPLETED', { label: '已完成', tagType: 'success' }],
  ['CLOSED', { label: '已关闭', tagType: 'info' }],
])

export const taskStatusMetaMap = createMetaMap<TaskStatus>([
  ['TODO', { label: '待处理', tagType: 'info' }],
  ['IN_PROGRESS', { label: '进行中', tagType: 'warning' }],
  ['BLOCKED', { label: '阻塞中', tagType: 'danger' }],
  ['DONE', { label: '已完成', tagType: 'success' }],
  ['CANCELLED', { label: '已取消', tagType: 'info' }],
])

export const weeklyReportStatusMetaMap = createMetaMap<WeeklyReportStatus>([
  ['DRAFT', { label: '草稿', tagType: 'info' }],
  ['SUBMITTED', { label: '已提交', tagType: 'warning' }],
  ['REVIEWED', { label: '已评审', tagType: 'success' }],
  ['RETURNED', { label: '已退回', tagType: 'danger' }],
])

export const assessmentStatusMetaMap = createMetaMap<AssessmentStatus>([
  ['DRAFT', { label: '草稿', tagType: 'info' }],
  ['SCORING', { label: '评分中', tagType: 'warning' }],
  ['PENDING_FINALIZE', { label: '待归档', tagType: 'primary' }],
  ['FINALIZED', { label: '已归档', tagType: 'success' }],
])

export const priorityMetaMap = createMetaMap<PriorityLevel>([
  ['LOW', { label: '低', tagType: 'info' }],
  ['MEDIUM', { label: '中', tagType: 'warning' }],
  ['HIGH', { label: '高', tagType: 'danger' }],
  ['CRITICAL', { label: '紧急', tagType: 'danger' }],
])

export const auditResultMetaMap = createMetaMap<AuditResult>([
  ['SUCCESS', { label: '成功', tagType: 'success' }],
  ['FAIL', { label: '失败', tagType: 'danger' }],
])
