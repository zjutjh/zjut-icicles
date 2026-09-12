import type { AppMenuItem } from '@/types/menu'

export const appMenus: AppMenuItem[] = [
  {
    key: 'dashboard',
    label: '仪表盘',
    icon: 'Odometer',
    to: '/dashboard',
    roles: ['SYSTEM_ADMIN', 'DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'AUDIT_ADMIN', 'TEAM_MEMBER'],
  },
  {
    key: 'users',
    label: '用户管理',
    icon: 'User',
    to: '/system/users',
    roles: ['SYSTEM_ADMIN'],
  },
  {
    key: 'teams',
    label: '团队与成员',
    icon: 'UserFilled',
    to: '/organization/teams',
    roles: ['SYSTEM_ADMIN', 'DEPT_MANAGER'],
  },
  {
    key: 'projects',
    label: '项目管理',
    icon: 'Management',
    to: '/projects',
    roles: ['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER'],
  },
  {
    key: 'tasks',
    label: '任务管理',
    icon: 'List',
    to: '/tasks',
    roles: ['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'TEAM_MEMBER'],
  },
  {
    key: 'reports',
    label: '周报管理',
    icon: 'Document',
    to: '/reports',
    roles: ['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'TEAM_MEMBER'],
  },
  {
    key: 'assessments',
    label: '月度考核',
    icon: 'DataAnalysis',
    to: '/assessments',
    roles: ['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'TEAM_MEMBER'],
  },
  {
    key: 'statistics',
    label: '统计分析',
    icon: 'TrendCharts',
    to: '/statistics',
    roles: ['SYSTEM_ADMIN', 'DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'AUDIT_ADMIN'],
  },
  {
    key: 'audit-logs',
    label: '审计日志',
    icon: 'Tickets',
    to: '/audit-logs',
    roles: ['AUDIT_ADMIN', 'SYSTEM_ADMIN'],
  },
]
