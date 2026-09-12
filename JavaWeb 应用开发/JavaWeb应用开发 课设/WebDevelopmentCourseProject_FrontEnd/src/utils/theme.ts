export function getThemeCssVar(name: string, fallback: string) {
  if (typeof window === 'undefined') {
    return fallback
  }

  const value = window.getComputedStyle(document.documentElement).getPropertyValue(name).trim()
  return value || fallback
}
