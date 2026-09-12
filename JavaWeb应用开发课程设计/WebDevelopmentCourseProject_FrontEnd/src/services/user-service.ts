import type { AxiosRequestConfig } from 'axios'

import type {
  CreateUserRequest,
  PageResult,
  Role,
  UpdateUserRequest,
  User,
  UserListQuery,
  UserStatusUpdateRequest,
} from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class UserService extends BaseService<AxiosRequestConfig> {
  getRoles() {
    return this.request<Role[]>({
      url: this.genBaseURL('/roles'),
      method: 'GET',
    })
  }

  getUsers(params: UserListQuery) {
    return this.request<PageResult<User>>({
      url: this.genBaseURL('/users'),
      method: 'GET',
      params,
    })
  }

  createUser(data: CreateUserRequest) {
    return this.request<User>({
      url: this.genBaseURL('/users'),
      method: 'POST',
      data,
    })
  }

  getUser(userId: number) {
    return this.request<User>({
      url: this.genBaseURL(`/users/${userId}`),
      method: 'GET',
    })
  }

  updateUser(userId: number, data: UpdateUserRequest) {
    return this.request<User>({
      url: this.genBaseURL(`/users/${userId}`),
      method: 'PUT',
      data,
    })
  }

  updateUserStatus(userId: number, data: UserStatusUpdateRequest) {
    return this.request<User>({
      url: this.genBaseURL(`/users/${userId}/status`),
      method: 'PATCH',
      data,
    })
  }
}

export const userService = new UserService({
  request,
})
