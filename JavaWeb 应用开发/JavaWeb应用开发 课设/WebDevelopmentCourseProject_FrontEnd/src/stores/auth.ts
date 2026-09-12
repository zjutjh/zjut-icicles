import { defineStore } from 'pinia'
import { computed, ref } from 'vue'

import { defaultRoutePath } from '@/configs/app'
import { defaultRouteByRole, type RoleCode, roleLabelMap } from '@/constants/roles'
import { authService } from '@/services/auth-service'
import type { CurrentUser, LoginResponseData } from '@/types/api'
import { storageKeys } from '@/utils/storage'

export const useAuthStore = defineStore('auth', () => {
  const token = ref(readToken())
  const currentUser = ref<CurrentUser | null>(readCurrentUser())

  const isLoggedIn = computed(() => Boolean(token.value && currentUser.value))
  const roleCodes = computed(() => currentUser.value?.roleCodes ?? [])
  const primaryRole = computed<RoleCode | null>(() => roleCodes.value[0] ?? null)
  const roleLabel = computed(() => (primaryRole.value ? roleLabelMap[primaryRole.value] : '未分配角色'))

  async function login(payload: { username: string; password: string }) {
    const result = await authService.login(payload)
    setSession(result)
    return result
  }

  async function fetchCurrentUser() {
    const user = await authService.getCurrentUser()
    currentUser.value = user
    persistCurrentUser(user)
    return user
  }

  async function logout() {
    try {
      if (token.value) {
        await authService.logout()
      }
    } finally {
      clearSession()
    }
  }

  function setSession(result: LoginResponseData) {
    token.value = result.token
    currentUser.value = result.user
    window.localStorage.setItem(storageKeys.token, result.token)
    persistCurrentUser(result.user)
  }

  function clearSession() {
    token.value = ''
    currentUser.value = null
    window.localStorage.removeItem(storageKeys.token)
    window.localStorage.removeItem(storageKeys.currentUser)
  }

  function getDefaultRoute() {
    if (!primaryRole.value) {
      return defaultRoutePath
    }

    return defaultRouteByRole[primaryRole.value]
  }

  function hasAnyRole(allowedRoles?: RoleCode[]) {
    if (!allowedRoles?.length) {
      return true
    }

    return allowedRoles.some((role) => roleCodes.value.includes(role))
  }

  return {
    token,
    currentUser,
    isLoggedIn,
    roleCodes,
    primaryRole,
    roleLabel,
    login,
    logout,
    fetchCurrentUser,
    setSession,
    clearSession,
    getDefaultRoute,
    hasAnyRole,
  }
})

function readToken() {
  return window.localStorage.getItem(storageKeys.token) ?? ''
}

function readCurrentUser(): CurrentUser | null {
  const raw = window.localStorage.getItem(storageKeys.currentUser)
  if (!raw) {
    return null
  }

  try {
    return JSON.parse(raw) as CurrentUser
  } catch {
    return null
  }
}

function persistCurrentUser(user: CurrentUser) {
  window.localStorage.setItem(storageKeys.currentUser, JSON.stringify(user))
}
