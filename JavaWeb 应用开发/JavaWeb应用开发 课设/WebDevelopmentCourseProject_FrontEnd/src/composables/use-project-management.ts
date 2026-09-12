import { keepPreviousData, useMutation, useQuery, useQueryClient } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { QUERY_KEY } from '@/constants/query-key'
import { projectService } from '@/services/project-service'
import type {
  CreateProjectRequest,
  ProjectListQuery,
  ProjectStatusUpdateRequest,
  UpdateProjectRequest,
} from '@/types/api'
import { compactObject } from '@/utils/query'

export function useProjectsQuery(params: MaybeRefOrGetter<ProjectListQuery>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.projects(resolvedParams.value)),
    queryFn: () => projectService.getProjects(resolvedParams.value as ProjectListQuery),
    placeholderData: keepPreviousData,
  })
}

export function useCreateProjectMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: (payload: CreateProjectRequest) => projectService.createProject(payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['projects'] }),
  })
}

export function useUpdateProjectMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ projectId, payload }: { projectId: number; payload: UpdateProjectRequest }) =>
      projectService.updateProject(projectId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['projects'] }),
  })
}

export function useUpdateProjectStatusMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ projectId, payload }: { projectId: number; payload: ProjectStatusUpdateRequest }) =>
      projectService.updateProjectStatus(projectId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['projects'] }),
  })
}
