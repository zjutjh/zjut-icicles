import axios, { AxiosError, type AxiosRequestConfig } from 'axios'

import { RESP_CODE } from '@/constants/response-code'
import { getResolvedApiBaseUrl } from '@/services/api-base-url'
import type { ApiResponseEnvelope } from '@/types/api'

import type { ServiceOptions } from './base-service'
import { SERVICE_TIMEOUT } from './base-service'
import { RequestError } from './request-error'
import { storageKeys } from './storage'

const axiosInstance = axios.create({
  baseURL: getResolvedApiBaseUrl(),
  timeout: SERVICE_TIMEOUT,
  withCredentials: true,
})

axiosInstance.interceptors.response.use(
  (response) => {
    const responseType = response.config.responseType
    if (responseType === 'blob' || responseType === 'arraybuffer') {
      return response
    }

    const body: ApiResponseEnvelope<unknown> = response.data

    if (body.code !== RESP_CODE.OK) {
      throw new RequestError(body.message ?? body.msg ?? '请求失败', body.code)
    }

    return response
  },
  (axiosError: AxiosError) => Promise.reject(RequestError.fromAxiosError(axiosError)),
)

axiosInstance.interceptors.request.use(
  (config) => {
    const token = window.localStorage.getItem(storageKeys.token)
    if (token) {
      config.headers.Authorization = `Bearer ${token}`
    }
    return config
  },
  (error) => Promise.reject(error),
)

export const request: ServiceOptions<AxiosRequestConfig>['request'] = async (req, options) => {
  const response = await axiosInstance({
    url: req.url,
    method: req.method,
    params: req.params,
    data: req.data,
    ...options,
  })

  const responseType = response.config.responseType
  if (responseType === 'blob' || responseType === 'arraybuffer') {
    return response.data
  }

  const body = response.data as ApiResponseEnvelope<unknown>
  return body.data as never
}

export { axiosInstance }
