<script setup lang="ts">
import { computed, reactive } from 'vue'

import LineTrendChart from '@/components/charts/LineTrendChart.vue'
import PageSection from '@/components/common/PageSection.vue'
import MetricCard from '@/components/dashboard/MetricCard.vue'
import { useTeamOptionsQuery } from '@/composables/use-reference-data'
import { useMonthlyStatisticsQuery, useQuarterlyStatisticsQuery, useYearlyStatisticsQuery } from '@/composables/use-statistics'
import type { DashboardMetric } from '@/types/dashboard'
import { formatScore } from '@/utils/format'

const filters = reactive({
  month: `${new Date().getFullYear()}-${String(new Date().getMonth() + 1).padStart(2, '0')}`,
  quarter: `${new Date().getFullYear()}-Q${Math.ceil((new Date().getMonth() + 1) / 3)}`,
  year: String(new Date().getFullYear()),
  teamId: undefined as number | undefined,
})

const teamOptionsQuery = useTeamOptionsQuery()
const monthlyQuery = useMonthlyStatisticsQuery(computed(() => ({ month: filters.month, teamId: filters.teamId })))
const quarterlyQuery = useQuarterlyStatisticsQuery(computed(() => ({ quarter: filters.quarter, teamId: filters.teamId })))
const yearlyQuery = useYearlyStatisticsQuery(computed(() => ({ year: filters.year, teamId: filters.teamId })))

const metricCards = computed<DashboardMetric[]>(() => {
  const monthly = monthlyQuery.data.value
  const quarterly = quarterlyQuery.data.value
  const yearly = yearlyQuery.data.value

  return [
    {
      label: '月均得分',
      value: formatScore(monthly?.averageScore),
      trend: `最高 ${formatScore(monthly?.highestScore)}`,
      tone: 'success',
      description: `最低 ${formatScore(monthly?.lowestScore)}，已考核 ${monthly?.assessedCount ?? 0} 人`,
    },
    {
      label: '季度均分',
      value: formatScore(quarterly?.averageScore),
      trend: `季度人数 ${quarterly?.assessedCount ?? 0}`,
      tone: 'warning',
      description: `统计区间 ${quarterly?.quarter ?? '-'}`,
    },
    {
      label: '年度均分',
      value: formatScore(yearly?.averageScore),
      trend: `年度人数 ${yearly?.assessedCount ?? 0}`,
      tone: 'primary',
      description: `统计年份 ${yearly?.year ?? '-'}`,
    },
  ]
})

const quarterTrendLabels = computed(() => quarterlyQuery.data.value?.trend.map((item) => item.month) ?? [])
const quarterTrendValues = computed(() => quarterlyQuery.data.value?.trend.map((item) => item.averageScore) ?? [])
const yearTrendLabels = computed(() => yearlyQuery.data.value?.monthlyTrend.map((item) => item.month) ?? [])
const yearTrendValues = computed(() => yearlyQuery.data.value?.monthlyTrend.map((item) => item.averageScore) ?? [])
</script>

<template>
  <PageSection title="统计分析" description="从月、季度、年度三个层级查看绩效趋势与团队对比。">
    <div class="page-toolbar">
      <div class="page-toolbar__filters">
        <el-date-picker v-model="filters.month" type="month" value-format="YYYY-MM" placeholder="月份" />
        <el-input v-model="filters.quarter" placeholder="季度，如 2026-Q2" style="width: 160px" />
        <el-input v-model="filters.year" placeholder="年份，如 2026" style="width: 140px" />
        <el-select v-model="filters.teamId" placeholder="全部团队" clearable style="width: 180px">
          <el-option
            v-for="item in teamOptionsQuery.data.value?.records ?? []"
            :key="item.id"
            :label="item.teamName"
            :value="item.id"
          />
        </el-select>
      </div>
    </div>

    <section class="stats-grid">
      <MetricCard v-for="item in metricCards" :key="item.label" :item="item" />
    </section>

    <section class="sub-grid mb-16">
      <PageSection title="季度趋势" description="季度内部的月度均分走势。">
        <template v-if="quarterTrendLabels.length">
          <LineTrendChart :labels="quarterTrendLabels" :values="quarterTrendValues" series-name="季度均分" />
        </template>
        <div v-else class="empty-block">暂无季度趋势数据</div>
      </PageSection>

      <PageSection title="年度趋势" description="年度维度的月均分变化。">
        <template v-if="yearTrendLabels.length">
          <LineTrendChart :labels="yearTrendLabels" :values="yearTrendValues" series-name="年度均分" />
        </template>
        <div v-else class="empty-block">暂无年度趋势数据</div>
      </PageSection>
    </section>

    <PageSection title="月度团队对比" description="展示当前月份各团队平均分，方便横向分析。">
      <el-table :data="monthlyQuery.data.value?.teamComparison ?? []">
        <el-table-column prop="teamName" label="团队" min-width="180" />
        <el-table-column prop="averageScore" label="平均分" min-width="120" />
      </el-table>
    </PageSection>
  </PageSection>
</template>

<style scoped lang="scss">
.page-toolbar {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 16px;
}

.page-toolbar__filters {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
  align-items: center;
}

.stats-grid {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 16px;
  margin-bottom: 16px;
}

.sub-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 16px;
}

.empty-block {
  display: grid;
  place-items: center;
  min-height: 220px;
  color: var(--theme-foreground-secondary);
}

@media (max-width: 1440px) {
  .stats-grid,
  .sub-grid {
    grid-template-columns: 1fr;
  }
}

@media (max-width: 768px) {
  .page-toolbar {
    flex-direction: column;
    align-items: stretch;
  }
}
</style>
