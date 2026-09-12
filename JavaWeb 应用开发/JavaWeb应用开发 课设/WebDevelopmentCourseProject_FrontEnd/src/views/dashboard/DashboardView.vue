<script setup lang="ts">
import {
  Bell,
  DataAnalysis,
  Histogram,
  TrendCharts,
} from '@element-plus/icons-vue'
import { computed } from 'vue'

import LineTrendChart from '@/components/charts/LineTrendChart.vue'
import RadarScoreChart from '@/components/charts/RadarScoreChart.vue'
import PageSection from '@/components/common/PageSection.vue'
import MetricCard from '@/components/dashboard/MetricCard.vue'
import { useDashboardSummaryQuery } from '@/composables/use-dashboard-summary-query'
import { useMonthlyStatisticsQuery, useYearlyStatisticsQuery } from '@/composables/use-statistics'
import type { DashboardMetric } from '@/types/dashboard'
import { formatScore } from '@/utils/format'

const currentDate = new Date()
const currentMonth = `${currentDate.getFullYear()}-${String(currentDate.getMonth() + 1).padStart(2, '0')}`
const currentYear = String(currentDate.getFullYear())

const summaryQuery = useDashboardSummaryQuery()
const monthlyQuery = useMonthlyStatisticsQuery(
  computed(() => ({
    month: currentMonth,
  })),
)
const yearlyQuery = useYearlyStatisticsQuery(
  computed(() => ({
    year: currentYear,
  })),
)

const metrics = computed<DashboardMetric[]>(() => {
  const summary = summaryQuery.data.value
  const monthly = monthlyQuery.data.value

  return [
    {
      label: '本月平均得分',
      value: formatScore(summary?.currentMonthAverageScore ?? monthly?.averageScore),
      trend: `最高分 ${formatScore(monthly?.highestScore)}`,
      tone: 'success',
      description: `最低分 ${formatScore(monthly?.lowestScore)}，已考核 ${monthly?.assessedCount ?? 0} 人`,
    },
    {
      label: '待处理考核',
      value: String(summary?.pendingAssessments ?? 0),
      trend: '月度评分待推进',
      tone: 'warning',
      description: '涉及待评分、待归档的月度考核单',
    },
    {
      label: '逾期任务数',
      value: String(summary?.overdueTasks ?? 0),
      trend: '任务闭环重点跟踪',
      tone: 'danger',
      description: '建议优先检查阻塞任务和进度滞后项',
    },
    {
      label: '周报待评审',
      value: String(summary?.weeklyReportsPendingReview ?? 0),
      trend: `活跃项目 ${summary?.activeProjects ?? 0} 个`,
      tone: 'primary',
      description: '用于经理和总监快速定位本周审批压力',
    },
  ]
})

const trendLabels = computed(() => yearlyQuery.data.value?.monthlyTrend.map((item) => item.month) ?? [])
const trendValues = computed(() => yearlyQuery.data.value?.monthlyTrend.map((item) => item.averageScore) ?? [])
const teamComparison = computed(() => monthlyQuery.data.value?.teamComparison ?? [])

const radarIndicators = computed(() =>
  teamComparison.value.map((item) => ({
    name: item.teamName,
    max: 100,
  })),
)

const radarValues = computed(() => teamComparison.value.map((item) => item.averageScore))

const alerts = computed(() => {
  const summary = summaryQuery.data.value
  const monthly = monthlyQuery.data.value

  return [
    {
      title: '考核推进',
      value: `${summary?.pendingAssessments ?? 0} 项`,
      description: '本月待推进的月度考核数量。',
      note: '建议优先处理待归档项',
      tone: 'warning',
      icon: Histogram,
    },
    {
      title: '周报审批',
      value: `${summary?.weeklyReportsPendingReview ?? 0} 份`,
      description: '待经理评审的周报总量。',
      note: '可集中安排本周评审',
      tone: 'primary',
      icon: Bell,
    },
    {
      title: '成绩波动',
      value: monthly ? `${formatScore(monthly.highestScore)} / ${formatScore(monthly.lowestScore)}` : '-',
      description: '当前月份的最高分与最低分。',
      note: '关注波动明显的团队',
      tone: 'info',
      icon: TrendCharts,
    },
  ]
})
</script>

<template>
  <section class="dashboard-grid">
    <div class="dashboard-grid__metrics">
      <MetricCard v-for="item in metrics" :key="item.label" :item="item" />
    </div>
  </section>

  <section class="content-grid">
    <PageSection title="年度得分趋势" description="按月份查看平均得分变化，便于判断整体绩效走势。">
      <template v-if="trendLabels.length">
        <LineTrendChart :labels="trendLabels" :values="trendValues" series-name="平均得分" />
      </template>
      <div v-else class="dashboard-empty">
        <el-icon><TrendCharts /></el-icon>
        <strong>暂无年度趋势数据</strong>
        <p>当前没有可用于展示的年度统计结果。</p>
      </div>
    </PageSection>

    <PageSection title="本月团队对比" description="查看当前月份各团队平均分差异，便于横向比较。">
      <template v-if="radarIndicators.length">
        <RadarScoreChart :indicators="radarIndicators" :values="radarValues" />
      </template>
      <div v-else class="dashboard-empty">
        <el-icon><DataAnalysis /></el-icon>
        <strong>暂无团队对比数据</strong>
        <p>当前月份尚未生成团队维度的对比结果。</p>
      </div>
    </PageSection>
  </section>

  <PageSection title="重点提醒" description="将当前最值得跟进的事项压缩到首页，便于快速处理。">
    <div class="dashboard-alert-grid">
      <article v-for="item in alerts" :key="item.title" class="reminder-card" :data-tone="item.tone">
        <div class="reminder-card__top">
          <div class="reminder-card__icon">
            <el-icon><component :is="item.icon" /></el-icon>
          </div>
          <span class="reminder-card__note">{{ item.note }}</span>
        </div>

        <strong class="reminder-card__title">{{ item.title }}</strong>
        <p class="reminder-card__value">{{ item.value }}</p>
        <p class="reminder-card__description">{{ item.description }}</p>
      </article>
    </div>
  </PageSection>
</template>

<style scoped lang="scss">
.dashboard-grid__metrics {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 20px;
  margin-bottom: 20px;
}

.content-grid {
  display: grid;
  grid-template-columns: 1.4fr 1fr;
  gap: 20px;
}

.dashboard-empty {
  display: flex;
  flex-direction: column;
  gap: 10px;
  align-items: center;
  justify-content: center;
  min-height: 300px;
  border: 1px dashed #e7ecf1;
  border-radius: 10px;
  background: #fafbfd;
  text-align: center;
}

.dashboard-empty .el-icon {
  color: #9aa5b4;
  font-size: 24px;
}

.dashboard-empty strong {
  color: var(--theme-foreground);
  font-size: 15px;
  font-weight: 600;
}

.dashboard-empty p {
  color: var(--theme-foreground-secondary);
  font-size: 13px;
}

.dashboard-alert-grid {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 16px;
}

.reminder-card {
  padding: 18px 18px 16px;
  border: 1px solid #edf1f5;
  border-radius: 12px;
  background: #fbfcfd;
}

.reminder-card__top {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  margin-bottom: 18px;
}

.reminder-card__icon {
  display: grid;
  place-items: center;
  width: 34px;
  height: 34px;
  border-radius: 10px;
  background: #eef3f8;
  color: #3f5e7a;
}

.reminder-card[data-tone='warning'] .reminder-card__icon {
  background: #faf4eb;
  color: #a78457;
}

.reminder-card[data-tone='info'] .reminder-card__icon {
  background: #f3f5f7;
  color: #7b8794;
}

.reminder-card__note {
  color: var(--theme-foreground-tertiary);
  font-size: 12px;
}

.reminder-card__title {
  display: block;
  margin-bottom: 12px;
  color: var(--theme-foreground);
  font-size: 15px;
  font-weight: 600;
}

.reminder-card__value {
  margin-bottom: 8px;
  color: var(--theme-foreground);
  font-size: 26px;
  font-weight: 700;
  line-height: 1.2;
}

.reminder-card__description {
  color: var(--theme-foreground-secondary);
  font-size: 13px;
  line-height: 1.7;
}

@media (max-width: 1440px) {
  .dashboard-grid__metrics {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }

  .content-grid,
  .dashboard-alert-grid {
    grid-template-columns: 1fr;
  }
}

@media (max-width: 768px) {
  .dashboard-grid__metrics {
    grid-template-columns: 1fr;
    gap: 16px;
  }

  .dashboard-empty {
    min-height: 260px;
  }
}
</style>
