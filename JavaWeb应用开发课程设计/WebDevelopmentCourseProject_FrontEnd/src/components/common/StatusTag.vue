<script setup lang="ts">
import type { StatusMeta } from '@/constants/options'

defineProps<{
  meta?: StatusMeta
}>()
</script>

<template>
  <el-tag :type="meta?.tagType ?? 'info'" effect="light">
    {{ meta?.label ?? '--' }}
  </el-tag>
</template>
