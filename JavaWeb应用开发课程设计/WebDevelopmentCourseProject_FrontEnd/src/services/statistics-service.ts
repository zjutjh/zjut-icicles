import type { AxiosRequestConfig } from 'axios'

import type {
  DashboardData,
  MonthlyStatistics,
  QuarterlyStatistics,
  YearlyStatistics,
} from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class StatisticsService extends BaseService<AxiosRequestConfig> {
  getDashboard() {
    return this.request<DashboardData>({
      url: this.genBaseURL('/statistics/dashboard'),
      method: 'GET',
    })
  }

  getMonthly(params: { month?: string; teamId?: number }) {
    return this.request<MonthlyStatistics>({
      url: this.genBaseURL('/statistics/monthly'),
      method: 'GET',
      params,
    })
  }

  getQuarterly(params: { quarter?: string; teamId?: number }) {
    return this.request<QuarterlyStatistics>({
      url: this.genBaseURL('/statistics/quarterly'),
      method: 'GET',
      params,
    })
  }

  getYearly(params: { year?: string; teamId?: number }) {
    return this.request<YearlyStatistics>({
      url: this.genBaseURL('/statistics/yearly'),
      method: 'GET',
      params,
    })
  }
}

export const statisticsService = new StatisticsService({
  request,
})
