import { keepPreviousData, useMutation, useQuery, useQueryClient } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { QUERY_KEY } from '@/constants/query-key'
import { userService } from '@/services/user-service'
import type {
  CreateUserRequest,
  UpdateUserRequest,
  UserListQuery,
  UserStatusUpdateRequest,
} from '@/types/api'
import { compactObject } from '@/utils/query'

export function useRolesQuery() {
  return useQuery({
    queryKey: QUERY_KEY.roles,
    queryFn: () => userService.getRoles(),
    staleTime: 5 * 60 * 1000,
  })
}

export function useUsersQuery(params: MaybeRefOrGetter<UserListQuery>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.users(resolvedParams.value)),
    queryFn: () => userService.getUsers(resolvedParams.value as UserListQuery),
    placeholderData: keepPreviousData,
  })
}

export function useCreateUserMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: (payload: CreateUserRequest) => userService.createUser(payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['users'] }),
  })
}

export function useUpdateUserMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ userId, payload }: { userId: number; payload: UpdateUserRequest }) =>
      userService.updateUser(userId, payload),
    onSuccess: (_, variables) => {
      queryClient.invalidateQueries({ queryKey: ['users'] })
      queryClient.invalidateQueries({ queryKey: QUERY_KEY.userDetail(variables.userId) })
    },
  })
}

export function useUpdateUserStatusMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ userId, payload }: { userId: number; payload: UserStatusUpdateRequest }) =>
      userService.updateUserStatus(userId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['users'] }),
  })
}
