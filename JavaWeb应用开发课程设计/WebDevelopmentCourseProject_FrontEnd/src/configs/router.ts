import { createRouter, createWebHistory, type RouteRecordRaw } from 'vue-router'

import { appTitle, defaultRoutePath } from '@/configs/app'
import MainLayout from '@/layouts/MainLayout.vue'
import { useAuthStore } from '@/stores/auth'
import type { RoleCode } from '@/types/api'

const allRoles: RoleCode[] = [
  'SYSTEM_ADMIN',
  'DEPT_MANAGER',
  'TECH_DIRECTOR',
  'GENERAL_MANAGER',
  'AUDIT_ADMIN',
  'TEAM_MEMBER',
]

const routes: RouteRecordRaw[] = [
  {
    path: '/login',
    name: 'login',
    component: () => import('@/views/login/LoginView.vue'),
    meta: {
      title: '登录',
      guestOnly: true,
    },
  },
  {
    path: '/',
    component: MainLayout,
    redirect: defaultRoutePath,
    children: [
      {
        path: 'dashboard',
        name: 'dashboard',
        component: () => import('@/views/dashboard/DashboardView.vue'),
        meta: {
          title: '仪表盘',
          roles: allRoles,
        },
      },
      {
        path: 'system/users',
        name: 'users',
        component: () => import('@/views/system/UserManagementView.vue'),
        meta: {
          title: '用户管理',
          roles: ['SYSTEM_ADMIN'],
        },
      },
      {
        path: 'organization/teams',
        name: 'teams',
        component: () => import('@/views/organization/TeamManagementView.vue'),
        meta: {
          title: '团队与成员',
          roles: ['SYSTEM_ADMIN', 'DEPT_MANAGER'],
        },
      },
      {
        path: 'projects',
        name: 'projects',
        component: () => import('@/views/projects/ProjectManagementView.vue'),
        meta: {
          title: '项目管理',
          roles: ['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER'],
        },
      },
      {
        path: 'tasks',
        name: 'tasks',
        component: () => import('@/views/tasks/TaskManagementView.vue'),
        meta: {
          title: '任务管理',
          roles: ['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'TEAM_MEMBER'],
        },
      },
      {
        path: 'reports',
        name: 'reports',
        component: () => import('@/views/reports/WeeklyReportView.vue'),
        meta: {
          title: '周报管理',
          roles: ['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'TEAM_MEMBER'],
        },
      },
      {
        path: 'assessments',
        name: 'assessments',
        component: () => import('@/views/assessments/AssessmentView.vue'),
        meta: {
          title: '月度考核',
          roles: ['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'TEAM_MEMBER'],
        },
      },
      {
        path: 'statistics',
        name: 'statistics',
        component: () => import('@/views/statistics/StatisticsView.vue'),
        meta: {
          title: '统计分析',
          roles: ['SYSTEM_ADMIN', 'DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'AUDIT_ADMIN'],
        },
      },
      {
        path: 'audit-logs',
        name: 'audit-logs',
        component: () => import('@/views/audit/AuditLogView.vue'),
        meta: {
          title: '审计日志',
          roles: ['AUDIT_ADMIN', 'SYSTEM_ADMIN'],
        },
      },
    ],
  },
  {
    path: '/:pathMatch(.*)*',
    name: 'not-found',
    component: () => import('@/views/status/NotFoundView.vue'),
    meta: {
      title: '页面不存在',
    },
  },
]

export const routerConfig = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes,
})

routerConfig.beforeEach((to) => {
  const authStore = useAuthStore()
  const requiresGuest = Boolean(to.meta.guestOnly)

  document.title = `${String(to.meta.title ?? appTitle)} | ${appTitle}`

  if (requiresGuest && authStore.isLoggedIn) {
    return authStore.getDefaultRoute()
  }

  if (!requiresGuest) {
    if (!authStore.token) {
      return '/login'
    }

    if (!authStore.currentUser) {
      return authStore.fetchCurrentUser().then(
        () => {
          const allowedRoles = to.meta.roles as RoleCode[] | undefined
          if (allowedRoles && !authStore.hasAnyRole(allowedRoles)) {
            return authStore.getDefaultRoute()
          }

          return true
        },
        () => {
          authStore.clearSession()
          return '/login'
        },
      )
    }
  }

  const allowedRoles = to.meta.roles as RoleCode[] | undefined
  if (allowedRoles && !authStore.hasAnyRole(allowedRoles)) {
    return authStore.getDefaultRoute()
  }

  return true
})
