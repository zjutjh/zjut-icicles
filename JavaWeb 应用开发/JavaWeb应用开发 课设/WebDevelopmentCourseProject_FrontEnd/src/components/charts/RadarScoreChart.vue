<script setup lang="ts">
import * as echarts from 'echarts'
import { nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'

import { getThemeCssVar } from '@/utils/theme'

const props = defineProps<{
  indicators: Array<{ name: string; max: number }>
  values: number[]
}>()

const chartRef = ref<HTMLDivElement>()
let chart: echarts.ECharts | null = null

function renderChart() {
  if (!chartRef.value) {
    return
  }

  const accent = getThemeCssVar('--theme-accent', '#4b6584')
  const accentGhost = getThemeCssVar('--theme-accent-ghost-strong', 'rgba(75, 101, 132, 0.18)')
  const accentSoft = getThemeCssVar('--theme-accent-soft', '#dbe5ef')
  const foreground = getThemeCssVar('--theme-foreground', '#2f3542')
  const foregroundSecondary = getThemeCssVar('--theme-foreground-secondary', '#7b8794')
  const border = getThemeCssVar('--theme-border', '#e5eaf0')

  chart ??= echarts.init(chartRef.value)
  chart.setOption({
    tooltip: {
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
    radar: {
      indicator: props.indicators,
      radius: '64%',
      splitLine: {
        lineStyle: {
          color: border,
        },
      },
      splitArea: {
        areaStyle: {
          color: ['transparent'],
        },
      },
      axisLine: {
        lineStyle: {
          color: accentSoft,
        },
      },
      name: {
        color: foregroundSecondary,
        fontSize: 13,
      },
    },
    series: [
      {
        type: 'radar',
        data: [
          {
            value: props.values,
            areaStyle: {
              color: accentGhost,
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
          },
        ],
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
  () => [props.indicators, props.values],
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
