import { QueryCache, QueryClient, type QueryClientConfig } from '@tanstack/vue-query'

const ONE_SECOND = 1000

const globalQueryClientConfig: QueryClientConfig = {
  queryCache: new QueryCache({
    onError: (error) => {
      console.error(error)
    },
  }),
  defaultOptions: {
    queries: {
      retry: 2,
      staleTime: 0 * ONE_SECOND,
      refetchOnWindowFocus: false,
    },
  },
}

export const globalQueryClient = new QueryClient(globalQueryClientConfig)
