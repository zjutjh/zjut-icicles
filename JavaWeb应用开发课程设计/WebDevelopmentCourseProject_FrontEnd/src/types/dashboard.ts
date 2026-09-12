export interface DashboardMetric {
  label: string
  value: string
  trend: string
  tone: 'primary' | 'success' | 'warning' | 'danger' | 'info'
  description?: string
}
