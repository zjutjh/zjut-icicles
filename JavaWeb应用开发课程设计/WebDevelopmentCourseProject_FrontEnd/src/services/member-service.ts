import type { AxiosRequestConfig } from 'axios'

import type {
  CreateMemberRequest,
  Member,
  MemberListQuery,
  MemberStatusUpdateRequest,
  PageResult,
  UpdateMemberRequest,
} from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class MemberService extends BaseService<AxiosRequestConfig> {
  getMembers(params: MemberListQuery) {
    return this.request<PageResult<Member>>({
      url: this.genBaseURL('/members'),
      method: 'GET',
      params,
    })
  }

  createMember(data: CreateMemberRequest) {
    return this.request<Member>({
      url: this.genBaseURL('/members'),
      method: 'POST',
      data,
    })
  }

  getMember(memberId: number) {
    return this.request<Member>({
      url: this.genBaseURL(`/members/${memberId}`),
      method: 'GET',
    })
  }

  updateMember(memberId: number, data: UpdateMemberRequest) {
    return this.request<Member>({
      url: this.genBaseURL(`/members/${memberId}`),
      method: 'PUT',
      data,
    })
  }

  updateMemberStatus(memberId: number, data: MemberStatusUpdateRequest) {
    return this.request<Member>({
      url: this.genBaseURL(`/members/${memberId}/status`),
      method: 'PATCH',
      data,
    })
  }
}

export const memberService = new MemberService({
  request,
})
