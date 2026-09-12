import type { AxiosRequestConfig } from 'axios'

import type { CreateTeamRequest, PageResult, Team, TeamListQuery, UpdateTeamRequest } from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class TeamService extends BaseService<AxiosRequestConfig> {
  getTeams(params: TeamListQuery) {
    return this.request<PageResult<Team>>({
      url: this.genBaseURL('/teams'),
      method: 'GET',
      params,
    })
  }

  createTeam(data: CreateTeamRequest) {
    return this.request<Team>({
      url: this.genBaseURL('/teams'),
      method: 'POST',
      data,
    })
  }

  getTeam(teamId: number) {
    return this.request<Team>({
      url: this.genBaseURL(`/teams/${teamId}`),
      method: 'GET',
    })
  }

  updateTeam(teamId: number, data: UpdateTeamRequest) {
    return this.request<Team>({
      url: this.genBaseURL(`/teams/${teamId}`),
      method: 'PUT',
      data,
    })
  }
}

export const teamService = new TeamService({
  request,
})
