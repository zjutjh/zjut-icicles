<script setup lang="ts">
import {
  Bell,
  Clock,
  DataAnalysis,
  Warning,
} from '@element-plus/icons-vue'
import { computed } from 'vue'

import type { DashboardMetric } from '@/types/dashboard'

const props = defineProps<{
  item: DashboardMetric
}>()

const iconMap = {
  primary: Bell,
  success: DataAnalysis,
  warning: Clock,
  danger: Warning,
  info: Bell,
} as const

const metricIcon = computed(() => iconMap[props.item.tone] ?? Bell)
</script>

<template>
  <el-card class="metric-card" shadow="never">
    <div class="metric-card__top">
      <div>
        <p class="metric-card__label">{{ item.label }}</p>
        <strong class="metric-card__value">{{ item.value }}</strong>
      </div>
      <div class="metric-card__icon" :data-tone="item.tone">
        <el-icon><component :is="metricIcon" /></el-icon>
      </div>
    </div>

    <p v-if="item.description" class="metric-card__hint">{{ item.description }}</p>

    <div class="metric-card__footer">
      <el-tag class="metric-card__tag" :type="item.tone" effect="light">{{ item.trend }}</el-tag>
    </div>
  </el-card>
</template>

<style scoped lang="scss">
.metric-card {
  border: 1px solid var(--theme-border);
  border-radius: 12px;
  box-shadow: var(--theme-shadow-soft);
}

.metric-card__top {
  display: flex;
  gap: 16px;
  align-items: flex-start;
  justify-content: space-between;
  margin-bottom: 18px;
}

.metric-card__label {
  margin-bottom: 14px;
  color: var(--theme-foreground-secondary);
  font-size: 14px;
}

.metric-card__value {
  display: block;
  color: var(--theme-foreground);
  font-size: 32px;
  font-weight: 700;
  line-height: 1.1;
}

.metric-card__icon {
  display: grid;
  place-items: center;
  width: 38px;
  height: 38px;
  border: 1px solid #e6ebf1;
  border-radius: 10px;
  background: #f8fafc;
  color: var(--theme-accent);
  flex-shrink: 0;
}

.metric-card__icon[data-tone='success'] {
  color: var(--el-color-success);
  background: var(--el-color-success-light-9);
}

.metric-card__icon[data-tone='warning'] {
  color: var(--el-color-warning);
  background: var(--el-color-warning-light-9);
}

.metric-card__icon[data-tone='danger'] {
  color: var(--el-color-danger);
  background: var(--el-color-danger-light-9);
}

.metric-card__icon[data-tone='primary'] {
  color: var(--theme-accent);
  background: #eef3f8;
}

.metric-card__hint {
  min-height: 38px;
  margin: 0 0 16px;
  color: var(--theme-foreground-secondary);
  font-size: 13px;
  line-height: 1.7;
}

.metric-card__footer {
  display: flex;
  align-items: center;
}

.metric-card__tag {
  border-color: transparent;
  font-weight: 500;
}

:deep(.metric-card__tag.el-tag--success) {
  color: #5f8f72;
  background: #eef6f1;
}

:deep(.metric-card__tag.el-tag--warning) {
  color: #a78457;
  background: #faf4eb;
}

:deep(.metric-card__tag.el-tag--danger) {
  color: #b1646b;
  background: #fbf0f1;
}

:deep(.metric-card__tag.el-tag--primary) {
  color: #3f5e7a;
  background: #eef3f8;
}

:deep(.metric-card__tag.el-tag--info) {
  color: #7b8794;
  background: #f3f5f7;
}
</style>
