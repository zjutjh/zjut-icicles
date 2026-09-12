import type { AxiosRequestConfig } from 'axios'

import type {
  CreateTaskProgressRequest,
  CreateTaskRequest,
  PageResult,
  Task,
  TaskListQuery,
  TaskProgress,
  TaskStatusUpdateRequest,
  UpdateTaskRequest,
} from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class TaskService extends BaseService<AxiosRequestConfig> {
  getTasks(params: TaskListQuery) {
    return this.request<PageResult<Task>>({
      url: this.genBaseURL('/tasks'),
      method: 'GET',
      params,
    })
  }

  createTask(data: CreateTaskRequest) {
    return this.request<Task>({
      url: this.genBaseURL('/tasks'),
      method: 'POST',
      data,
    })
  }

  getTask(taskId: number) {
    return this.request<Task>({
      url: this.genBaseURL(`/tasks/${taskId}`),
      method: 'GET',
    })
  }

  updateTask(taskId: number, data: UpdateTaskRequest) {
    return this.request<Task>({
      url: this.genBaseURL(`/tasks/${taskId}`),
      method: 'PUT',
      data,
    })
  }

  updateTaskStatus(taskId: number, data: TaskStatusUpdateRequest) {
    return this.request<Task>({
      url: this.genBaseURL(`/tasks/${taskId}/status`),
      method: 'PATCH',
      data,
    })
  }

  getTaskProgress(taskId: number) {
    return this.request<TaskProgress[]>({
      url: this.genBaseURL(`/tasks/${taskId}/progress`),
      method: 'GET',
    })
  }

  createTaskProgress(taskId: number, data: CreateTaskProgressRequest) {
    return this.request<TaskProgress>({
      url: this.genBaseURL(`/tasks/${taskId}/progress`),
      method: 'POST',
      data,
    })
  }
}

export const taskService = new TaskService({
  request,
})
