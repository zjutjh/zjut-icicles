function trimTrailingSlash(url: string) {
  return url.replace(/\/+$/, '')
}

export function getResolvedApiBaseUrl() {
  const envApiBaseUrl = import.meta.env.VITE_API_BASE_URL?.trim()

  if (import.meta.env.PROD) {
    if (!envApiBaseUrl) {
      return '/api'
    }

    const normalized = trimTrailingSlash(envApiBaseUrl)

    if (normalized.startsWith('http://') || normalized.startsWith('https://')) {
      return normalized
    }

    if (normalized.startsWith('/')) {
      return normalized || '/api'
    }

    throw new Error('VITE_API_BASE_URL must be an absolute HTTP(S) URL or a same-origin path like /api')
  }

  return envApiBaseUrl || '/api'
}
