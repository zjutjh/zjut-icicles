import { keepPreviousData, useQuery } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { QUERY_KEY } from '@/constants/query-key'
import { auditService } from '@/services/audit-service'
import type { AuditLogListQuery } from '@/types/api'
import { compactObject } from '@/utils/query'

export function useAuditLogsQuery(params: MaybeRefOrGetter<AuditLogListQuery>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.auditLogs(resolvedParams.value)),
    queryFn: () => auditService.getAuditLogs(resolvedParams.value as AuditLogListQuery),
    placeholderData: keepPreviousData,
  })
}

export function useAuditLogDetailQuery(logId: MaybeRefOrGetter<number | null>) {
  const resolvedLogId = computed(() => toValue(logId))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.auditLogDetail(resolvedLogId.value ?? undefined)),
    queryFn: () => auditService.getAuditLog(resolvedLogId.value as number),
    enabled: computed(() => Boolean(resolvedLogId.value)),
  })
}
