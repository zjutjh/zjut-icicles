<script setup lang="ts">
defineProps<{
  title: string
  description?: string
}>()
</script>

<template>
  <el-card shadow="never" class="page-section">
    <template #header>
      <div class="page-section__header">
        <div>
          <strong>{{ title }}</strong>
        </div>
        <slot name="extra" />
      </div>
    </template>
    <slot />
  </el-card>
</template>

<style scoped lang="scss">
.page-section {
  border: 1px solid var(--theme-border);
  border-radius: 12px;
  background: var(--theme-surface-strong);
  box-shadow: var(--theme-shadow-soft);
}

.page-section__header {
  display: flex;
  justify-content: space-between;
  gap: 16px;
  align-items: flex-start;
}

.page-section__header strong {
  display: block;
  font-size: 18px;
  color: var(--theme-foreground);
}
</style>
