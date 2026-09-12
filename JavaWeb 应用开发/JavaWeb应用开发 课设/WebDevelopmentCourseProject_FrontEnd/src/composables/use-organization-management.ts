import { keepPreviousData, useMutation, useQuery, useQueryClient } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { QUERY_KEY } from '@/constants/query-key'
import { memberService } from '@/services/member-service'
import { teamService } from '@/services/team-service'
import type {
  CreateMemberRequest,
  CreateTeamRequest,
  MemberListQuery,
  MemberStatusUpdateRequest,
  TeamListQuery,
  UpdateMemberRequest,
  UpdateTeamRequest,
} from '@/types/api'
import { compactObject } from '@/utils/query'

export function useTeamsQuery(params: MaybeRefOrGetter<TeamListQuery>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.teams(resolvedParams.value)),
    queryFn: () => teamService.getTeams(resolvedParams.value as TeamListQuery),
    placeholderData: keepPreviousData,
  })
}

export function useMembersQuery(params: MaybeRefOrGetter<MemberListQuery>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.members(resolvedParams.value)),
    queryFn: () => memberService.getMembers(resolvedParams.value as MemberListQuery),
    placeholderData: keepPreviousData,
  })
}

export function useCreateTeamMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: (payload: CreateTeamRequest) => teamService.createTeam(payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['teams'] }),
  })
}

export function useUpdateTeamMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ teamId, payload }: { teamId: number; payload: UpdateTeamRequest }) =>
      teamService.updateTeam(teamId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['teams'] }),
  })
}

export function useCreateMemberMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: (payload: CreateMemberRequest) => memberService.createMember(payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['members'] }),
  })
}

export function useUpdateMemberMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ memberId, payload }: { memberId: number; payload: UpdateMemberRequest }) =>
      memberService.updateMember(memberId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['members'] }),
  })
}

export function useUpdateMemberStatusMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ memberId, payload }: { memberId: number; payload: MemberStatusUpdateRequest }) =>
      memberService.updateMemberStatus(memberId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['members'] }),
  })
}
