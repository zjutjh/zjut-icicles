<script setup lang="ts">
import * as echarts from 'echarts'
import { nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'

import { getThemeCssVar } from '@/utils/theme'

const props = withDefaults(
  defineProps<{
    labels: string[]
    values: number[]
    seriesName?: string
  }>(),
  {
    seriesName: '趋势',
  },
)

const chartRef = ref<HTMLDivElement>()
let chart: echarts.ECharts | null = null

function renderChart() {
  if (!chartRef.value) {
    return
  }

  const accent = getThemeCssVar('--theme-accent', '#4b6584')
  const accentGhost = getThemeCssVar('--theme-accent-ghost-strong', 'rgba(75, 101, 132, 0.18)')
  const foreground = getThemeCssVar('--theme-foreground', '#2f3542')
  const foregroundSecondary = getThemeCssVar('--theme-foreground-secondary', '#7b8794')
  const border = getThemeCssVar('--theme-border', '#e5eaf0')

  chart ??= echarts.init(chartRef.value)
  chart.setOption({
    tooltip: {
      trigger: 'axis',
      backgroundColor: '#ffffff',
      borderColor: border,
      borderWidth: 1,
      textStyle: {
        color: foreground,
      },
      extraCssText: 'box-shadow: 0 10px 24px rgba(47,53,66,0.08); border-radius: 10px;',
    },
    textStyle: {
      color: foregroundSecondary,
      fontFamily: 'Geist, PingFang SC, Microsoft YaHei, sans-serif',
    },
    grid: {
      left: 24,
      right: 20,
      top: 22,
      bottom: 18,
    },
    xAxis: {
      type: 'category',
      data: props.labels,
      boundaryGap: false,
      axisLine: {
        lineStyle: {
          color: border,
        },
      },
      axisLabel: {
        color: foregroundSecondary,
      },
      axisTick: {
        show: false,
      },
    },
    yAxis: {
      type: 'value',
      minInterval: 1,
      splitLine: {
        lineStyle: {
          color: border,
          type: 'dashed',
        },
      },
      axisLine: {
        show: false,
      },
      axisTick: {
        show: false,
      },
      axisLabel: {
        color: foregroundSecondary,
      },
    },
    series: [
      {
        name: props.seriesName,
        type: 'line',
        smooth: true,
        data: props.values,
        symbol: 'circle',
        symbolSize: 7,
        areaStyle: {
          color: new echarts.graphic.LinearGradient(0, 0, 0, 1, [
            { offset: 0, color: accentGhost },
            { offset: 1, color: 'rgba(255,255,255,0.1)' },
          ]),
        },
        lineStyle: {
          color: accent,
          width: 2.5,
        },
        itemStyle: {
          color: accent,
          borderColor: '#ffffff',
          borderWidth: 2,
        },
        emphasis: {
          focus: 'series',
        },
      },
    ],
  })
  chart.resize()
}

function resizeChart() {
  chart?.resize()
}

onMounted(async () => {
  await nextTick()
  renderChart()
  window.addEventListener('resize', resizeChart)
})

watch(
  () => [props.labels, props.values],
  () => {
    renderChart()
  },
  { deep: true },
)

onBeforeUnmount(() => {
  window.removeEventListener('resize', resizeChart)
  chart?.dispose()
})
</script>

<template>
  <div ref="chartRef" class="chart-panel"></div>
</template>

<style scoped lang="scss">
.chart-panel {
  width: 100%;
  height: 320px;
}
</style>
