import type { RouteLocationRaw } from 'vue-router'

import type { RoleCode } from '@/constants/roles'

export interface AppMenuItem {
  key: string
  label: string
  icon: string
  to: RouteLocationRaw
  roles: RoleCode[]
}
