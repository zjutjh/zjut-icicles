import { useQuery } from '@tanstack/vue-query'
import { computed, type MaybeRefOrGetter, toValue } from 'vue'

import { statisticsService } from '@/services/statistics-service'
import { compactObject } from '@/utils/query'

export function useMonthlyStatisticsQuery(params: MaybeRefOrGetter<{ month?: string; teamId?: number }>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => ['statistics', 'monthly', resolvedParams.value]),
    queryFn: () => statisticsService.getMonthly(resolvedParams.value),
  })
}

export function useQuarterlyStatisticsQuery(params: MaybeRefOrGetter<{ quarter?: string; teamId?: number }>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => ['statistics', 'quarterly', resolvedParams.value]),
    queryFn: () => statisticsService.getQuarterly(resolvedParams.value),
  })
}

export function useYearlyStatisticsQuery(params: MaybeRefOrGetter<{ year?: string; teamId?: number }>) {
  const resolvedParams = computed(() => compactObject(toValue(params)))

  return useQuery({
    queryKey: computed(() => ['statistics', 'yearly', resolvedParams.value]),
    queryFn: () => statisticsService.getYearly(resolvedParams.value),
  })
}
