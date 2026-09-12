<template>
  <div class="status-page">
    <el-result icon="warning" title="页面不存在" sub-title="你访问的页面可能已移动，或者当前角色没有对应入口。">
      <template #extra>
        <el-button type="primary" @click="$router.push('/dashboard')">返回仪表盘</el-button>
      </template>
    </el-result>
  </div>
</template>

<style scoped lang="scss">
.status-page {
  display: grid;
  place-items: center;
  min-height: 100vh;
}
</style>
