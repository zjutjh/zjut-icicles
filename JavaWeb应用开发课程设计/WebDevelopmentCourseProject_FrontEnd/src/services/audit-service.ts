import type { AxiosRequestConfig } from 'axios'

import type { AuditLog, AuditLogListQuery, PageResult } from '@/types/api'
import { BaseService } from '@/utils/base-service'
import { request } from '@/utils/service'

class AuditService extends BaseService<AxiosRequestConfig> {
  getAuditLogs(params: AuditLogListQuery) {
    return this.request<PageResult<AuditLog>>({
      url: this.genBaseURL('/audit-logs'),
      method: 'GET',
      params,
    })
  }

  getAuditLog(logId: number) {
    return this.request<AuditLog>({
      url: this.genBaseURL(`/audit-logs/${logId}`),
      method: 'GET',
    })
  }
}

export const auditService = new AuditService({
  request,
})
