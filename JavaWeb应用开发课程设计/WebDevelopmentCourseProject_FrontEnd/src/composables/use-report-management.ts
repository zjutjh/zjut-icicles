import { keepPreviousData, useMutation, useQuery, useQueryClient } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { QUERY_KEY } from '@/constants/query-key'
import { weeklyReportService } from '@/services/weekly-report-service'
import type {
  CreateWeeklyReportRequest,
  ReviewWeeklyReportRequest,
  UpdateWeeklyReportRequest,
  WeeklyReportListQuery,
} from '@/types/api'
import { compactObject } from '@/utils/query'

export function useReportsQuery(params: MaybeRefOrGetter<WeeklyReportListQuery>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => QUERY_KEY.reports(resolvedParams.value)),
    queryFn: () => weeklyReportService.getReports(resolvedParams.value as WeeklyReportListQuery),
    placeholderData: keepPreviousData,
  })
}

export function useCreateReportMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: (payload: CreateWeeklyReportRequest) => weeklyReportService.createReport(payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reports'] }),
  })
}

export function useUpdateReportMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ reportId, payload }: { reportId: number; payload: UpdateWeeklyReportRequest }) =>
      weeklyReportService.updateReport(reportId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reports'] }),
  })
}

export function useReviewReportMutation() {
  const queryClient = useQueryClient()

  return useMutation({
    mutationFn: ({ reportId, payload }: { reportId: number; payload: ReviewWeeklyReportRequest }) =>
      weeklyReportService.reviewReport(reportId, payload),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reports'] }),
  })
}
