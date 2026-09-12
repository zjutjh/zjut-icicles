<script setup lang="ts">
import { computed, reactive, ref } from 'vue'

import PageSection from '@/components/common/PageSection.vue'
import StatusTag from '@/components/common/StatusTag.vue'
import { useAuditLogDetailQuery, useAuditLogsQuery } from '@/composables/use-audit-log-management'
import { useUserOptionsQuery } from '@/composables/use-reference-data'
import { auditResultMetaMap, auditResultOptions } from '@/constants/options'
import type { AuditResult } from '@/types/api'
import { formatDateTime } from '@/utils/format'

const filters = reactive({
  operatorId: undefined as number | undefined,
  actionType: '',
  targetType: '',
  result: undefined as AuditResult | undefined,
})

const pagination = reactive({
  pageNo: 1,
  pageSize: 10,
})

const queryParams = computed(() => ({
  operatorId: filters.operatorId,
  actionType: filters.actionType || undefined,
  targetType: filters.targetType || undefined,
  result: filters.result,
  pageNo: pagination.pageNo,
  pageSize: pagination.pageSize,
}))

const auditLogsQuery = useAuditLogsQuery(queryParams)
const userOptionsQuery = useUserOptionsQuery()

const drawerVisible = ref(false)
const selectedLogId = ref<number | null>(null)
const detailQuery = useAuditLogDetailQuery(computed(() => selectedLogId.value))

const tableData = computed(() => auditLogsQuery.data.value?.records ?? [])

function getAuditResultMeta(result: AuditResult) {
  return auditResultMetaMap[result]
}

function resetFilters() {
  filters.operatorId = undefined
  filters.actionType = ''
  filters.targetType = ''
  filters.result = undefined
  pagination.pageNo = 1
}

function openDetail(logId: number) {
  selectedLogId.value = logId
  drawerVisible.value = true
}
</script>

<template>
  <PageSection title="审计日志" description="记录关键操作的操作者、目标对象、结果和详细内容。">
    <div class="page-toolbar">
      <div class="page-toolbar__filters">
        <el-select v-model="filters.operatorId" placeholder="全部操作人" clearable style="width: 180px">
          <el-option
            v-for="item in userOptionsQuery.data.value?.records ?? []"
            :key="item.id"
            :label="item.realName"
            :value="item.id"
          />
        </el-select>
        <el-input v-model="filters.actionType" placeholder="操作类型" clearable style="max-width: 180px" />
        <el-input v-model="filters.targetType" placeholder="目标类型" clearable style="max-width: 180px" />
        <el-select v-model="filters.result" placeholder="全部结果" clearable style="width: 160px">
          <el-option v-for="item in auditResultOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
        <el-button type="primary" @click="pagination.pageNo = 1">查询</el-button>
        <el-button @click="resetFilters">重置</el-button>
      </div>
    </div>

    <el-table v-loading="auditLogsQuery.isLoading.value" :data="tableData">
      <el-table-column prop="operatorName" label="操作人" min-width="120" />
      <el-table-column prop="actionType" label="操作类型" min-width="140" />
      <el-table-column prop="targetType" label="目标类型" min-width="140" />
      <el-table-column prop="targetId" label="目标 ID" min-width="120" />
      <el-table-column label="结果" width="110">
        <template #default="{ row }">
          <StatusTag :meta="getAuditResultMeta(row.result)" />
        </template>
      </el-table-column>
      <el-table-column label="操作时间" min-width="180">
        <template #default="{ row }">
          {{ formatDateTime(row.actionTime) }}
        </template>
      </el-table-column>
      <el-table-column label="详情" min-width="120" fixed="right">
        <template #default="{ row }">
          <el-button link type="primary" @click="openDetail(row.id)">查看</el-button>
        </template>
      </el-table-column>
    </el-table>

    <div class="pagination-bar">
      <el-pagination
        v-model:current-page="pagination.pageNo"
        v-model:page-size="pagination.pageSize"
        :total="auditLogsQuery.data.value?.total ?? 0"
        :page-sizes="[10, 20, 50]"
        layout="total, sizes, prev, pager, next"
      />
    </div>
  </PageSection>

  <el-drawer v-model="drawerVisible" title="日志详情" size="520px">
    <div v-if="detailQuery.data.value" class="detail-grid">
      <div class="detail-grid__item">
        <span class="detail-grid__label">操作人</span>
        <strong>{{ detailQuery.data.value.operatorName || '-' }}</strong>
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">操作类型</span>
        <strong>{{ detailQuery.data.value.actionType }}</strong>
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">目标类型</span>
        <strong>{{ detailQuery.data.value.targetType }}</strong>
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">目标 ID</span>
        <strong>{{ detailQuery.data.value.targetId || '-' }}</strong>
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">执行结果</span>
        <StatusTag :meta="getAuditResultMeta(detailQuery.data.value.result)" />
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">操作时间</span>
        <strong>{{ formatDateTime(detailQuery.data.value.actionTime) }}</strong>
      </div>
      <div class="detail-grid__item" style="grid-column: 1 / -1">
        <span class="detail-grid__label">详细内容</span>
        <strong>{{ detailQuery.data.value.detail || '-' }}</strong>
      </div>
    </div>
  </el-drawer>
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

.pagination-bar {
  display: flex;
  justify-content: flex-end;
  margin-top: 16px;
}

.detail-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px 20px;
}

.detail-grid__item {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.detail-grid__label {
  color: var(--theme-foreground-secondary);
  font-size: 13px;
}

@media (max-width: 768px) {
  .page-toolbar {
    flex-direction: column;
    align-items: stretch;
  }

  .detail-grid {
    grid-template-columns: 1fr;
  }
}
</style>
