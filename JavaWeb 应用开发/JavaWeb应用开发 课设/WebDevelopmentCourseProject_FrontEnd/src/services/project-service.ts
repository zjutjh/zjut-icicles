import type { AxiosRequestConfig } from 'axios'

import type {
  CreateProjectRequest,
  PageResult,
  Project,
  ProjectListQuery,
  ProjectStatusUpdateRequest,
  UpdateProjectRequest,
} from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class ProjectService extends BaseService<AxiosRequestConfig> {
  getProjects(params: ProjectListQuery) {
    return this.request<PageResult<Project>>({
      url: this.genBaseURL('/projects'),
      method: 'GET',
      params,
    })
  }

  createProject(data: CreateProjectRequest) {
    return this.request<Project>({
      url: this.genBaseURL('/projects'),
      method: 'POST',
      data,
    })
  }

  getProject(projectId: number) {
    return this.request<Project>({
      url: this.genBaseURL(`/projects/${projectId}`),
      method: 'GET',
    })
  }

  updateProject(projectId: number, data: UpdateProjectRequest) {
    return this.request<Project>({
      url: this.genBaseURL(`/projects/${projectId}`),
      method: 'PUT',
      data,
    })
  }

  updateProjectStatus(projectId: number, data: ProjectStatusUpdateRequest) {
    return this.request<Project>({
      url: this.genBaseURL(`/projects/${projectId}/status`),
      method: 'PATCH',
      data,
    })
  }
}

export const projectService = new ProjectService({
  request,
})
