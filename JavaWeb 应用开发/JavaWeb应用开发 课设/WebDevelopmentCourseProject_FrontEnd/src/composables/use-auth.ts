import { useMutation } from '@tanstack/vue-query'

import { authService } from '@/services/auth-service'
import { useAuthStore } from '@/stores/auth'
import type { ChangePasswordRequest, LoginRequest } from '@/types/api'

export function useLoginMutation() {
  const authStore = useAuthStore()

  return useMutation({
    mutationFn: (payload: LoginRequest) => authStore.login(payload),
  })
}

export function useChangePasswordMutation() {
  return useMutation({
    mutationFn: (payload: ChangePasswordRequest) => authService.changePassword(payload),
  })
}
