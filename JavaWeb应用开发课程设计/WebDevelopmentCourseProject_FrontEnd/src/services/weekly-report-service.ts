import type { AxiosRequestConfig } from 'axios'

import type {
  CreateWeeklyReportRequest,
  PageResult,
  ReviewWeeklyReportRequest,
  UpdateWeeklyReportRequest,
  WeeklyReport,
  WeeklyReportListQuery,
} from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class WeeklyReportService extends BaseService<AxiosRequestConfig> {
  getReports(params: WeeklyReportListQuery) {
    return this.request<PageResult<WeeklyReport>>({
      url: this.genBaseURL('/reports'),
      method: 'GET',
      params,
    })
  }

  createReport(data: CreateWeeklyReportRequest) {
    return this.request<WeeklyReport>({
      url: this.genBaseURL('/reports'),
      method: 'POST',
      data,
    })
  }

  getReport(reportId: number) {
    return this.request<WeeklyReport>({
      url: this.genBaseURL(`/reports/${reportId}`),
      method: 'GET',
    })
  }

  updateReport(reportId: number, data: UpdateWeeklyReportRequest) {
    return this.request<WeeklyReport>({
      url: this.genBaseURL(`/reports/${reportId}`),
      method: 'PUT',
      data,
    })
  }

  reviewReport(reportId: number, data: ReviewWeeklyReportRequest) {
    return this.request<WeeklyReport>({
      url: this.genBaseURL(`/reports/${reportId}/review`),
      method: 'POST',
      data,
    })
  }
}

export const weeklyReportService = new WeeklyReportService({
  request,
})
