/* eslint-disable @typescript-eslint/no-explicit-any */
interface RequestFnParams {
  url: string
  method: 'GET' | 'POST' | 'PUT' | 'PATCH' | 'DELETE'
  data?: Partial<Record<string, any>>
  params?: Partial<Record<string, any>>
}

export interface ServiceOptions<T> {
  baseURL?: string
  request<R>(params: RequestFnParams, options?: T): Promise<R>
}

export abstract class BaseService<T> {
  #baseURL = ''
  request: ServiceOptions<T>['request'] = () => {
    throw new Error('Service.request is undefined')
  }

  constructor(options: ServiceOptions<T>) {
    this.request = options.request
    this.#baseURL = options.baseURL ?? ''
  }

  genBaseURL(path: `/${string}`) {
    return this.#baseURL + path
  }
}

export const SERVICE_TIMEOUT = 15000 as const
