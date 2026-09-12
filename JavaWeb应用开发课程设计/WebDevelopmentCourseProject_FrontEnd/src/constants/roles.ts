import type { RoleCode } from '@/types/api'

export const roleOptions: Array<{ label: string; value: RoleCode }> = [
  { label: '系统管理员', value: 'SYSTEM_ADMIN' },
  { label: '部门经理', value: 'DEPT_MANAGER' },
  { label: '技术总监', value: 'TECH_DIRECTOR' },
  { label: '总经理', value: 'GENERAL_MANAGER' },
  { label: '审计管理员', value: 'AUDIT_ADMIN' },
  { label: '团队成员', value: 'TEAM_MEMBER' },
]

export const roleLabelMap = Object.fromEntries(roleOptions.map((item) => [item.value, item.label])) as Record<
  RoleCode,
  string
>

export const defaultRouteByRole: Record<RoleCode, string> = {
  SYSTEM_ADMIN: '/system/users',
  DEPT_MANAGER: '/dashboard',
  TECH_DIRECTOR: '/dashboard',
  GENERAL_MANAGER: '/dashboard',
  AUDIT_ADMIN: '/audit-logs',
  TEAM_MEMBER: '/tasks',
}

export type { RoleCode }
