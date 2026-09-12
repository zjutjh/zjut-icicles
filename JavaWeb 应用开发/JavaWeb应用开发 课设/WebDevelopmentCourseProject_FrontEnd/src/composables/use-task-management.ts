import { keepPreviousData, useMutation, useQuery, useQueryClient } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { QUERY_KEY } from '@/constants/query-key'
import { taskService } from '@/services/task-service'
import type {
  CreateTaskProgressRequest,
  CreateTaskRequest,
  TaskListQuery,
  TaskStatusUpdateRequest,
  UpdateTaskRequest,
} from '@/types/api'
import { compactObject } from '@/utils/query'

export function useTasksQuery(params: MaybeRefOrGetter<TaskListQuery>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.tasks(resolvedParams.value)),
    queryFn: () => taskService.getTasks(resolvedParams.value as TaskListQuery),
    placeholderData: keepPreviousData,
  })
}

export function useTaskProgressQuery(taskId: MaybeRefOrGetter<number | null>) {
  const resolvedTaskId = computed(() => toValue(taskId))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.taskProgress(resolvedTaskId.value ?? undefined)),
    queryFn: () => taskService.getTaskProgress(resolvedTaskId.value as number),
    enabled: computed(() => Boolean(resolvedTaskId.value)),
  })
}

export function useCreateTaskMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: (payload: CreateTaskRequest) => taskService.createTask(payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['tasks'] }),
  })
}

export function useUpdateTaskMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ taskId, payload }: { taskId: number; payload: UpdateTaskRequest }) =>
      taskService.updateTask(taskId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['tasks'] }),
  })
}

export function useUpdateTaskStatusMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ taskId, payload }: { taskId: number; payload: TaskStatusUpdateRequest }) =>
      taskService.updateTaskStatus(taskId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['tasks'] }),
  })
}

export function useCreateTaskProgressMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ taskId, payload }: { taskId: number; payload: CreateTaskProgressRequest }) =>
      taskService.createTaskProgress(taskId, payload),
    onSuccess: (_, variables) => {
      queryClient.invalidateQueries({ queryKey: ['tasks'] })
      queryClient.invalidateQueries({ queryKey: QUERY_KEY.taskProgress(variables.taskId) })
    },
  })
}
