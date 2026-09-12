import { computed } from 'vue'

import { useAuthStore } from '@/stores/auth'
import type { RoleCode } from '@/types/api'

const globalTeamRoles: RoleCode[] = ['SYSTEM_ADMIN', 'TECH_DIRECTOR', 'GENERAL_MANAGER', 'AUDIT_ADMIN']

export function useTeamScope() {
  const authStore = useAuthStore()

  const currentTeamId = computed(() => authStore.currentUser?.teamId ?? null)
  const currentTeamName = computed(() => authStore.currentUser?.teamName ?? null)
  const hasGlobalTeamAccess = computed(() => authStore.hasAnyRole(globalTeamRoles))
  const shouldLimitToOwnTeam = computed(() => !hasGlobalTeamAccess.value)

  function isAccessibleTeamId(teamId?: number | null) {
    if (hasGlobalTeamAccess.value) {
      return true
    }

    if (currentTeamId.value == null || teamId == null) {
      return false
    }

    return teamId === currentTeamId.value
  }

  return {
    currentTeamId,
    currentTeamName,
    hasGlobalTeamAccess,
    shouldLimitToOwnTeam,
    isAccessibleTeamId,
  }
}
