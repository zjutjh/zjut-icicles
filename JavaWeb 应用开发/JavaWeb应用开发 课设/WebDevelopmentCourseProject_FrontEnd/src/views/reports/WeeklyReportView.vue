<script setup lang="ts">
import type { FormInstance } from 'element-plus'
import { ElMessage } from 'element-plus'
import { computed, reactive, ref } from 'vue'

import PageSection from '@/components/common/PageSection.vue'
import StatusTag from '@/components/common/StatusTag.vue'
import { useMemberOptionsQuery } from '@/composables/use-reference-data'
import {
  useCreateReportMutation,
  useReportsQuery,
  useReviewReportMutation,
  useUpdateReportMutation,
} from '@/composables/use-report-management'
import { useTeamScope } from '@/composables/use-team-scope'
import { weeklyReportStatusMetaMap, weeklyReportStatusOptions } from '@/constants/options'
import { useAuthStore } from '@/stores/auth'
import type {
  CreateWeeklyReportRequest,
  ReviewWeeklyReportRequest,
  WeeklyReport,
  WeeklyReportStatus,
} from '@/types/api'
import { formatDateTime } from '@/utils/format'

const TEXT = {
  pageTitle: '\u5468\u62a5\u7ba1\u7406',
  pageDescription: '\u56f4\u7ed5\u6210\u5458\u5468\u62a5\u7684\u63d0\u4ea4\u3001\u4fee\u6539\u548c\u8bc4\u5ba1\u5f62\u6210\u95ed\u73af\u3002',
  weekPlaceholder: '\u5468\u6b21\uff0c\u5982 2026-W24',
  allMembers: '\u5168\u90e8\u6210\u5458',
  allStatuses: '\u5168\u90e8\u72b6\u6001',
  search: '\u67e5\u8be2',
  reset: '\u91cd\u7f6e',
  createReport: '\u65b0\u5efa\u5468\u62a5',
  member: '\u6210\u5458',
  weekNo: '\u5468\u6b21',
  status: '\u72b6\u6001',
  summary: '\u672c\u5468\u603b\u7ed3',
  nextPlan: '\u4e0b\u5468\u8ba1\u5212',
  managerComment: '\u8bc4\u5ba1\u610f\u89c1',
  updatedAt: '\u66f4\u65b0\u65f6\u95f4',
  actions: '\u64cd\u4f5c',
  edit: '\u7f16\u8f91',
  review: '\u8bc4\u5ba1',
  editReport: '\u7f16\u8f91\u5468\u62a5',
  selectMember: '\u8bf7\u9009\u62e9\u6210\u5458',
  weekExample: '\u4f8b\u5982 2026-W24',
  cancel: '\u53d6\u6d88',
  save: '\u4fdd\u5b58',
  reviewDialogTitle: '\u5468\u62a5\u8bc4\u5ba1',
  reviewResult: '\u8bc4\u5ba1\u7ed3\u679c',
  approved: '\u901a\u8fc7',
  returned: '\u9000\u56de',
  submitReview: '\u63d0\u4ea4\u8bc4\u5ba1',
  noComment: '-',
  ruleMember: '\u8bf7\u9009\u62e9\u6210\u5458',
  ruleWeekNo: '\u8bf7\u8f93\u5165\u5468\u6b21',
  ruleSummary: '\u8bf7\u586b\u5199\u672c\u5468\u603b\u7ed3',
  ruleNextPlan: '\u8bf7\u586b\u5199\u4e0b\u5468\u8ba1\u5212',
  messageUpdated: '\u5468\u62a5\u5df2\u66f4\u65b0',
  messageCreated: '\u5468\u62a5\u5df2\u521b\u5efa',
  messageReviewed: '\u5468\u62a5\u8bc4\u5ba1\u5df2\u63d0\u4ea4',
} as const

const authStore = useAuthStore()
const { currentTeamId, shouldLimitToOwnTeam } = useTeamScope()
const canReview = computed(() => authStore.hasAnyRole(['DEPT_MANAGER', 'TECH_DIRECTOR', 'GENERAL_MANAGER']))

const filters = reactive({
  weekNo: '',
  memberId: undefined as number | undefined,
  status: undefined as WeeklyReportStatus | undefined,
})

const pagination = reactive({
  pageNo: 1,
  pageSize: 10,
})

const queryParams = computed(() => ({
  weekNo: filters.weekNo || undefined,
  memberId: filters.memberId,
  status: filters.status,
  pageNo: shouldLimitToOwnTeam.value ? 1 : pagination.pageNo,
  pageSize: shouldLimitToOwnTeam.value ? 500 : pagination.pageSize,
}))

const reportsQuery = useReportsQuery(queryParams)
const memberOptionsQuery = useMemberOptionsQuery(
  computed(() => (shouldLimitToOwnTeam.value && currentTeamId.value ? { teamId: currentTeamId.value } : {})),
)
const createReportMutation = useCreateReportMutation()
const updateReportMutation = useUpdateReportMutation()
const reviewReportMutation = useReviewReportMutation()

const dialogVisible = ref(false)
const editingReport = ref<WeeklyReport | null>(null)
const formRef = ref<FormInstance>()
const form = reactive({
  memberId: undefined as number | undefined,
  weekNo: '',
  summary: '',
  nextPlan: '',
})

const rules = {
  memberId: [{ required: true, message: TEXT.ruleMember, trigger: 'change' }],
  weekNo: [{ required: true, message: TEXT.ruleWeekNo, trigger: 'blur' }],
  summary: [{ required: true, message: TEXT.ruleSummary, trigger: 'blur' }],
  nextPlan: [{ required: true, message: TEXT.ruleNextPlan, trigger: 'blur' }],
}

const reviewDialogVisible = ref(false)
const reviewTarget = ref<WeeklyReport | null>(null)
const reviewForm = reactive({
  status: 'REVIEWED' as 'REVIEWED' | 'RETURNED',
  managerComment: '',
})

const accessibleMemberIds = computed(() => new Set((memberOptionsQuery.data.value?.records ?? []).map((item) => item.id)))
const filteredReportRows = computed(() => {
  const rows = reportsQuery.data.value?.records ?? []

  if (!shouldLimitToOwnTeam.value) {
    return rows
  }

  return rows.filter((row) => accessibleMemberIds.value.has(row.memberId))
})
const tableData = computed(() => {
  if (!shouldLimitToOwnTeam.value) {
    return filteredReportRows.value
  }

  const start = (pagination.pageNo - 1) * pagination.pageSize
  return filteredReportRows.value.slice(start, start + pagination.pageSize)
})
const total = computed(() =>
  shouldLimitToOwnTeam.value ? filteredReportRows.value.length : (reportsQuery.data.value?.total ?? 0),
)

function isReportAccessible(report: WeeklyReport) {
  return !shouldLimitToOwnTeam.value || accessibleMemberIds.value.has(report.memberId)
}

function getReportStatusMeta(status: WeeklyReportStatus) {
  return weeklyReportStatusMetaMap[status]
}

function resetFilters() {
  filters.weekNo = ''
  filters.memberId = undefined
  filters.status = undefined
  pagination.pageNo = 1
}

function resetForm() {
  form.memberId = undefined
  form.weekNo = ''
  form.summary = ''
  form.nextPlan = ''
  formRef.value?.clearValidate()
}

function openCreateDialog() {
  editingReport.value = null
  resetForm()
  dialogVisible.value = true
}

function openEditDialog(report: WeeklyReport) {
  if (!isReportAccessible(report)) {
    ElMessage.error('只能查看和维护本团队成员的周报')
    return
  }

  editingReport.value = report
  form.memberId = report.memberId
  form.weekNo = report.weekNo
  form.summary = report.summary
  form.nextPlan = report.nextPlan
  dialogVisible.value = true
}

async function submitReport() {
  const valid = await formRef.value?.validate().catch(() => false)
  if (!valid) {
    return
  }

  if (shouldLimitToOwnTeam.value && !accessibleMemberIds.value.has(form.memberId as number)) {
    ElMessage.error('只能为本团队成员提交周报')
    return
  }

  if (editingReport.value) {
    if (!isReportAccessible(editingReport.value)) {
      ElMessage.error('只能查看和维护本团队成员的周报')
      return
    }

    await updateReportMutation.mutateAsync({
      reportId: editingReport.value.id,
      payload: {
        summary: form.summary,
        nextPlan: form.nextPlan,
      },
    })
    ElMessage.success(TEXT.messageUpdated)
  } else {
    const payload: CreateWeeklyReportRequest = {
      memberId: form.memberId as number,
      weekNo: form.weekNo,
      summary: form.summary,
      nextPlan: form.nextPlan,
    }

    await createReportMutation.mutateAsync(payload)
    ElMessage.success(TEXT.messageCreated)
  }

  dialogVisible.value = false
  resetForm()
}

function openReviewDialog(report: WeeklyReport) {
  if (!isReportAccessible(report)) {
    ElMessage.error('只能评审本团队成员的周报')
    return
  }

  reviewTarget.value = report
  reviewForm.status = report.status === 'RETURNED' ? 'RETURNED' : 'REVIEWED'
  reviewForm.managerComment = report.managerComment ?? ''
  reviewDialogVisible.value = true
}

async function submitReview() {
  if (!reviewTarget.value || !isReportAccessible(reviewTarget.value)) {
    if (reviewTarget.value) {
      ElMessage.error('只能评审本团队成员的周报')
    }

    return
  }

  const payload: ReviewWeeklyReportRequest = {
    status: reviewForm.status,
    managerComment: reviewForm.managerComment || undefined,
  }

  await reviewReportMutation.mutateAsync({
    reportId: reviewTarget.value.id,
    payload,
  })
  ElMessage.success(TEXT.messageReviewed)
  reviewDialogVisible.value = false
}
</script>

<template>
  <PageSection :title="TEXT.pageTitle" :description="TEXT.pageDescription">
    <div class="page-toolbar">
      <div class="page-toolbar__filters">
        <el-input v-model="filters.weekNo" :placeholder="TEXT.weekPlaceholder" clearable style="max-width: 180px" />
        <el-select v-model="filters.memberId" :placeholder="TEXT.allMembers" clearable style="width: 180px">
          <el-option
            v-for="item in memberOptionsQuery.data.value?.records ?? []"
            :key="item.id"
            :label="item.name"
            :value="item.id"
          />
        </el-select>
        <el-select v-model="filters.status" :placeholder="TEXT.allStatuses" clearable style="width: 160px">
          <el-option
            v-for="item in weeklyReportStatusOptions"
            :key="item.value"
            :label="item.label"
            :value="item.value"
          />
        </el-select>
        <el-button type="primary" @click="pagination.pageNo = 1">{{ TEXT.search }}</el-button>
        <el-button @click="resetFilters">{{ TEXT.reset }}</el-button>
      </div>
      <div class="page-toolbar__actions">
        <el-button type="primary" @click="openCreateDialog">{{ TEXT.createReport }}</el-button>
      </div>
    </div>

    <el-table v-loading="reportsQuery.isLoading.value" :data="tableData">
      <el-table-column prop="memberName" :label="TEXT.member" min-width="120" />
      <el-table-column prop="weekNo" :label="TEXT.weekNo" min-width="110" />
      <el-table-column :label="TEXT.status" width="110">
        <template #default="{ row }">
          <StatusTag :meta="getReportStatusMeta(row.status)" />
        </template>
      </el-table-column>
      <el-table-column :label="TEXT.summary" min-width="220">
        <template #default="{ row }">
          <div class="line-clamp-2">{{ row.summary }}</div>
        </template>
      </el-table-column>
      <el-table-column :label="TEXT.nextPlan" min-width="220">
        <template #default="{ row }">
          <div class="line-clamp-2">{{ row.nextPlan }}</div>
        </template>
      </el-table-column>
      <el-table-column :label="TEXT.managerComment" min-width="180">
        <template #default="{ row }">
          <div class="line-clamp-2">{{ row.managerComment || TEXT.noComment }}</div>
        </template>
      </el-table-column>
      <el-table-column :label="TEXT.updatedAt" min-width="180">
        <template #default="{ row }">
          {{ formatDateTime(row.updatedAt) }}
        </template>
      </el-table-column>
      <el-table-column :label="TEXT.actions" min-width="180" fixed="right">
        <template #default="{ row }">
          <div class="table-ops">
            <el-button link type="primary" @click="openEditDialog(row)">{{ TEXT.edit }}</el-button>
            <el-button v-if="canReview" link @click="openReviewDialog(row)">{{ TEXT.review }}</el-button>
          </div>
        </template>
      </el-table-column>
    </el-table>

    <div class="pagination-bar">
      <el-pagination
        v-model:current-page="pagination.pageNo"
        v-model:page-size="pagination.pageSize"
        :total="total"
        :page-sizes="[10, 20, 50]"
        layout="total, sizes, prev, pager, next"
      />
    </div>
  </PageSection>

  <el-dialog v-model="dialogVisible" :title="editingReport ? TEXT.editReport : TEXT.createReport" width="760px" @closed="resetForm">
    <el-form ref="formRef" :model="form" :rules="rules" label-position="top">
      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item :label="TEXT.member" prop="memberId">
            <el-select v-model="form.memberId" :disabled="Boolean(editingReport)" :placeholder="TEXT.selectMember">
              <el-option
                v-for="item in memberOptionsQuery.data.value?.records ?? []"
                :key="item.id"
                :label="item.name"
                :value="item.id"
              />
            </el-select>
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item :label="TEXT.weekNo" prop="weekNo">
            <el-input v-model="form.weekNo" :disabled="Boolean(editingReport)" :placeholder="TEXT.weekExample" />
          </el-form-item>
        </el-col>
      </el-row>
      <el-form-item :label="TEXT.summary" prop="summary">
        <el-input v-model="form.summary" type="textarea" :rows="4" maxlength="500" show-word-limit />
      </el-form-item>
      <el-form-item :label="TEXT.nextPlan" prop="nextPlan">
        <el-input v-model="form.nextPlan" type="textarea" :rows="4" maxlength="500" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="dialogVisible = false">{{ TEXT.cancel }}</el-button>
        <el-button
          type="primary"
          :loading="createReportMutation.isPending.value || updateReportMutation.isPending.value"
          @click="submitReport"
        >
          {{ TEXT.save }}
        </el-button>
      </el-space>
    </template>
  </el-dialog>

  <el-dialog v-model="reviewDialogVisible" :title="TEXT.reviewDialogTitle" width="520px">
    <el-form label-position="top">
      <el-form-item :label="TEXT.reviewResult">
        <el-radio-group v-model="reviewForm.status">
          <el-radio-button label="REVIEWED">{{ TEXT.approved }}</el-radio-button>
          <el-radio-button label="RETURNED">{{ TEXT.returned }}</el-radio-button>
        </el-radio-group>
      </el-form-item>
      <el-form-item :label="TEXT.managerComment">
        <el-input v-model="reviewForm.managerComment" type="textarea" :rows="4" maxlength="300" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="reviewDialogVisible = false">{{ TEXT.cancel }}</el-button>
        <el-button type="primary" :loading="reviewReportMutation.isPending.value" @click="submitReview">
          {{ TEXT.submitReview }}
        </el-button>
      </el-space>
    </template>
  </el-dialog>
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

.page-toolbar__filters,
.page-toolbar__actions,
.table-ops {
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

.line-clamp-2 {
  display: -webkit-box;
  overflow: hidden;
  -webkit-box-orient: vertical;
  -webkit-line-clamp: 2;
}

@media (max-width: 768px) {
  .page-toolbar {
    flex-direction: column;
    align-items: stretch;
  }
}
</style>
