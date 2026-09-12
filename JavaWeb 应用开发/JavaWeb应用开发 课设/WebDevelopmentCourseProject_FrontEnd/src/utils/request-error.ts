import type { AxiosError } from 'axios'

type RequestErrorCodeType = string | number

export class RequestError extends Error {
  public override message: string
  public code: RequestErrorCodeType
  public originError: unknown = null

  constructor(message: string, code: RequestErrorCodeType) {
    super()
    this.message = message
    this.code = code
  }

  static fromAxiosError(error: AxiosError) {
    const requestError = new RequestError('网络异常，请稍后重试', String(error.code ?? ''))
    requestError.originError = error
    return requestError
  }
}
