import { roleLabelMap } from '@/constants/roles'
import type { RoleCode } from '@/types/api'

export function formatDateTime(value?: string | null, fallback = '-') {
  if (!value) {
    return fallback
  }

  const date = new Date(value)
  if (Number.isNaN(date.getTime())) {
    return value
  }

  return date.toLocaleString('zh-CN', {
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
  })
}

export function formatDate(value?: string | null, fallback = '-') {
  if (!value) {
    return fallback
  }

  const date = new Date(value)
  if (Number.isNaN(date.getTime())) {
    return value
  }

  return date.toLocaleDateString('zh-CN')
}

export function formatScore(value?: number | null, digits = 1) {
  if (value === undefined || value === null) {
    return '-'
  }

  return Number(value).toFixed(digits)
}

export function formatPercent(value?: number | null) {
  if (value === undefined || value === null) {
    return '-'
  }

  return `${value}%`
}

export function formatRoleList(roleCodes?: RoleCode[]) {
  if (!roleCodes?.length) {
    return '-'
  }

  return roleCodes.map((role) => roleLabelMap[role]).join('、')
}
