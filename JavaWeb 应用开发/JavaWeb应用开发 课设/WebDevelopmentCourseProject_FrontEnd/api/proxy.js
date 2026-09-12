function trimTrailingSlash(url) {
  return url.replace(/\/+$/, '')
}

function buildForwardHeaders(headers) {
  const forwardHeaders = new Headers()

  const headerNames = [
    'accept',
    'accept-language',
    'authorization',
    'content-type',
    'cookie',
    'user-agent',
    'x-requested-with',
  ]

  for (const name of headerNames) {
    const value = headers[name]
    if (typeof value === 'string' && value) {
      forwardHeaders.set(name, value)
    }
  }

  return forwardHeaders
}

function buildRequestBody(req) {
  if (req.method === 'GET' || req.method === 'HEAD') {
    return undefined
  }

  if (req.body == null) {
    return undefined
  }

  if (typeof req.body === 'string' || Buffer.isBuffer(req.body)) {
    return req.body
  }

  return JSON.stringify(req.body)
}

function buildTargetUrl(req) {
  const baseUrl = process.env.API_PROXY_TARGET
  if (!baseUrl) {
    throw new Error('Missing API_PROXY_TARGET environment variable')
  }

  const path = Array.isArray(req.query.path) ? req.query.path.join('/') : req.query.path || ''
  const normalizedBaseUrl = trimTrailingSlash(baseUrl)
  const targetUrl = new URL(`${normalizedBaseUrl}/${path}`)

  for (const [key, value] of Object.entries(req.query)) {
    if (key === 'path' || value == null) {
      continue
    }

    if (Array.isArray(value)) {
      for (const item of value) {
        targetUrl.searchParams.append(key, item)
      }
      continue
    }

    targetUrl.searchParams.set(key, String(value))
  }

  return targetUrl
}

export default async function handler(req, res) {
  try {
    const targetUrl = buildTargetUrl(req)
    const response = await fetch(targetUrl, {
      method: req.method,
      headers: buildForwardHeaders(req.headers),
      body: buildRequestBody(req),
      redirect: 'manual',
    })

    res.status(response.status)

    const contentType = response.headers.get('content-type')
    if (contentType) {
      res.setHeader('content-type', contentType)
    }

    const setCookie = response.headers.get('set-cookie')
    if (setCookie) {
      res.setHeader('set-cookie', setCookie)
    }

    const arrayBuffer = await response.arrayBuffer()
    res.send(Buffer.from(arrayBuffer))
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Proxy request failed'
    res.status(502).json({
      code: 502,
      message,
      data: null,
    })
  }
}
