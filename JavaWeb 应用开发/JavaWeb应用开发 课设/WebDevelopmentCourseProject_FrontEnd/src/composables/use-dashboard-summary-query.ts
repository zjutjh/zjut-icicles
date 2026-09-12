import { useQuery } from '@tanstack/vue-query'

import { QUERY_KEY } from '@/constants/query-key'
import { statisticsService } from '@/services/statistics-service'

export function useDashboardSummaryQuery() {
  return useQuery({
    queryKey: QUERY_KEY.dashboard,
    queryFn: () => statisticsService.getDashboard(),
  })
}
