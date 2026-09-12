import { useQuery } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { memberService } from '@/services/member-service'
import { projectService } from '@/services/project-service'
import { teamService } from '@/services/team-service'
import { userService } from '@/services/user-service'
import type { MemberListQuery, ProjectListQuery, TeamListQuery, UserListQuery } from '@/types/api'
import { compactObject } from '@/utils/query'

const optionQueryDefaults = {
  pageNo: 1,
  pageSize: 200,
}

export function useUserOptionsQuery(params: MaybeRefOrGetter<Partial<UserListQuery>> = {}) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => ['option-users', resolvedParams.value]),
    queryFn: () =>
      userService.getUsers({
        ...optionQueryDefaults,
        ...resolvedParams.value,
      }),
    staleTime: 60 * 1000,
  })
}

export function useTeamOptionsQuery(params: MaybeRefOrGetter<Partial<TeamListQuery>> = {}) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => ['option-teams', resolvedParams.value]),
    queryFn: () =>
      teamService.getTeams({
        ...optionQueryDefaults,
        ...resolvedParams.value,
      }),
    staleTime: 60 * 1000,
  })
}

export function useMemberOptionsQuery(params: MaybeRefOrGetter<Partial<MemberListQuery>> = {}) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => ['option-members', resolvedParams.value]),
    queryFn: () =>
      memberService.getMembers({
        ...optionQueryDefaults,
        ...resolvedParams.value,
      }),
    staleTime: 60 * 1000,
  })
}

export function useProjectOptionsQuery(params: MaybeRefOrGetter<Partial<ProjectListQuery>> = {}) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => ['option-projects', resolvedParams.value]),
    queryFn: () =>
      projectService.getProjects({
        ...optionQueryDefaults,
        ...resolvedParams.value,
      }),
    staleTime: 60 * 1000,
  })
}
