import type { AxiosRequestConfig } from 'axios'

import type { ChangePasswordRequest, CurrentUser, LoginRequest, LoginResponseData } from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class AuthService extends BaseService<AxiosRequestConfig> {
  login(data: LoginRequest) {
    return this.request<LoginResponseData>({
      url: this.genBaseURL('/auth/login'),
      method: 'POST',
      data,
    })
  }

  logout() {
    return this.request<boolean>({
      url: this.genBaseURL('/auth/logout'),
      method: 'POST',
    })
  }

  getCurrentUser() {
    return this.request<CurrentUser>({
      url: this.genBaseURL('/auth/me'),
      method: 'GET',
    })
  }

  changePassword(data: ChangePasswordRequest) {
    return this.request<boolean>({
      url: this.genBaseURL('/auth/change-password'),
      method: 'POST',
      data,
    })
  }
}

export const authService = new AuthService({
  request,
})
