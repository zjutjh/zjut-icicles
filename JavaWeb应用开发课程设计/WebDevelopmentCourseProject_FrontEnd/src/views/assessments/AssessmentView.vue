<script setup lang="ts">
import { ElMessage } from 'element-plus'
import { computed, reactive, ref } from 'vue'

import PageSection from '@/components/common/PageSection.vue'
import StatusTag from '@/components/common/StatusTag.vue'
import { useAssessmentDetailQuery, useAssessmentsQuery, useFinalizeAssessmentMutation, useGenerateAssessmentsMutation, useScoreAssessmentMutation } from '@/composables/use-assessment-management'
import { useMemberOptionsQuery, useTeamOptionsQuery } from '@/composables/use-reference-data'
import { useTeamScope } from '@/composables/use-team-scope'
import { assessmentScoreRoleOptions, assessmentStatusMetaMap, assessmentStatusOptions } from '@/constants/options'
import { useAuthStore } from '@/stores/auth'
import type { AssessmentStatus, MonthlyAssessment, RoleCode } from '@/types/api'
import { formatDateTime, formatScore } from '@/utils/format'

const authStore = useAuthStore()
const { currentTeamId, shouldLimitToOwnTeam, isAccessibleTeamId } = useTeamScope()

const filters = reactive({
  assessMonth: '',
  teamId: undefined as number | undefined,
  memberId: undefined as number | undefined,
  status: undefined as AssessmentStatus | undefined,
})

const pagination = reactive({
  pageNo: 1,
  pageSize: 10,
})

const queryParams = computed(() => ({
  assessMonth: filters.assessMonth || undefined,
  teamId: shouldLimitToOwnTeam.value ? currentTeamId.value ?? undefined : filters.teamId,
  memberId: filters.memberId,
  status: filters.status,
  pageNo: shouldLimitToOwnTeam.value ? 1 : pagination.pageNo,
  pageSize: shouldLimitToOwnTeam.value ? 500 : pagination.pageSize,
}))

const assessmentsQuery = useAssessmentsQuery(queryParams)
const teamOptionsQuery = useTeamOptionsQuery()
const memberOptionsQuery = useMemberOptionsQuery(
  computed(() => (shouldLimitToOwnTeam.value && currentTeamId.value ? { teamId: currentTeamId.value } : {})),
)
const generateMutation = useGenerateAssessmentsMutation()
const scoreMutation = useScoreAssessmentMutation()
const finalizeMutation = useFinalizeAssessmentMutation()

const detailDrawerVisible = ref(false)
const detailTargetId = ref<number | null>(null)
const detailQuery = useAssessmentDetailQuery(computed(() => detailTargetId.value))

const generateDialogVisible = ref(false)
const generateForm = reactive({
  assessMonth: '',
  teamId: undefined as number | undefined,
  overwriteExisting: false,
})

const scoreDialogVisible = ref(false)
const scoreTarget = ref<MonthlyAssessment | null>(null)
const allowedScoreRoles = computed(() => {
  const role = authStore.primaryRole

  if (role && assessmentScoreRoleOptions.some((item) => item.value === role)) {
    return [role] as RoleCode[]
  }

  return [] as RoleCode[]
})
const canScoreWithCurrentRole = computed(() => allowedScoreRoles.value.length > 0)
const scoreRoleOptions = computed(() =>
  assessmentScoreRoleOptions.filter((item) => allowedScoreRoles.value.includes(item.value)),
)
const scoreForm = reactive({
  scorerRole: (allowedScoreRoles.value[0] ?? 'DEPT_MANAGER') as RoleCode,
  rawScore: 80,
  comment: '',
})

const finalizeDialogVisible = ref(false)
const finalizeTarget = ref<MonthlyAssessment | null>(null)
const finalizeComment = ref('')

const teamOptions = computed(() => {
  const rows = teamOptionsQuery.data.value?.records ?? []

  if (!shouldLimitToOwnTeam.value) {
    return rows
  }

  return rows.filter((item) => isAccessibleTeamId(item.id))
})
const accessibleMemberIds = computed(() => new Set((memberOptionsQuery.data.value?.records ?? []).map((item) => item.id)))
const filteredAssessmentRows = computed(() => {
  const rows = assessmentsQuery.data.value?.records ?? []

  if (!shouldLimitToOwnTeam.value) {
    return rows
  }

  return rows.filter((row) => isAccessibleTeamId(row.teamId) && accessibleMemberIds.value.has(row.memberId))
})
const tableData = computed(() => {
  if (!shouldLimitToOwnTeam.value) {
    return filteredAssessmentRows.value
  }

  const start = (pagination.pageNo - 1) * pagination.pageSize
  return filteredAssessmentRows.value.slice(start, start + pagination.pageSize)
})
const total = computed(() =>
  shouldLimitToOwnTeam.value ? filteredAssessmentRows.value.length : (assessmentsQuery.data.value?.total ?? 0),
)

function getAssessmentStatusMeta(status: AssessmentStatus) {
  return assessmentStatusMetaMap[status]
}

function resetFilters() {
  filters.assessMonth = ''
  filters.teamId = shouldLimitToOwnTeam.value ? currentTeamId.value ?? undefined : undefined
  filters.memberId = undefined
  filters.status = undefined
  pagination.pageNo = 1
}

function openDetail(row: MonthlyAssessment) {
  if (shouldLimitToOwnTeam.value && !isAccessibleTeamId(row.teamId)) {
    ElMessage.error('只能查看本团队的考核记录')
    return
  }

  detailTargetId.value = row.id
  detailDrawerVisible.value = true
}

function openGenerateDialog() {
  generateForm.assessMonth = generateForm.assessMonth || `${new Date().getFullYear()}-${String(new Date().getMonth() + 1).padStart(2, '0')}`
  generateForm.teamId = shouldLimitToOwnTeam.value ? currentTeamId.value ?? undefined : undefined
  generateForm.overwriteExisting = false
  generateDialogVisible.value = true
}

async function submitGenerate() {
  if (shouldLimitToOwnTeam.value && !generateForm.teamId) {
    generateForm.teamId = currentTeamId.value ?? undefined
  }

  await generateMutation.mutateAsync({
    assessMonth: generateForm.assessMonth,
    teamId: generateForm.teamId,
    overwriteExisting: generateForm.overwriteExisting,
  })
  ElMessage.success('考核单生成完成')
  generateDialogVisible.value = false
}

function openScoreDialog(row: MonthlyAssessment) {
  if (shouldLimitToOwnTeam.value && !isAccessibleTeamId(row.teamId)) {
    ElMessage.error('只能处理本团队的考核记录')
    return
  }

  if (!canScoreWithCurrentRole.value) {
    ElMessage.error('当前身份不允许提交该类评分')
    return
  }

  scoreTarget.value = row
  scoreForm.scorerRole = allowedScoreRoles.value[0] ?? 'DEPT_MANAGER'
  scoreForm.rawScore = row.finalScore ?? row.deptManagerScore ?? row.techDirectorScore ?? row.generalManagerScore ?? 80
  scoreForm.comment = row.comment ?? ''
  scoreDialogVisible.value = true
}

async function submitScore() {
  if (!scoreTarget.value || (shouldLimitToOwnTeam.value && !isAccessibleTeamId(scoreTarget.value.teamId))) {
    if (scoreTarget.value) {
      ElMessage.error('只能处理本团队的考核记录')
    }

    return
  }

  if (!allowedScoreRoles.value.includes(scoreForm.scorerRole)) {
    ElMessage.error('只能提交与当前身份对应的评分')
    return
  }

  await scoreMutation.mutateAsync({
    assessmentId: scoreTarget.value.id,
    payload: {
      scorerRole: scoreForm.scorerRole,
      rawScore: scoreForm.rawScore,
      comment: scoreForm.comment || undefined,
    },
  })
  ElMessage.success('评分已提交')
  scoreDialogVisible.value = false
}

function openFinalizeDialog(row: MonthlyAssessment) {
  if (shouldLimitToOwnTeam.value && !isAccessibleTeamId(row.teamId)) {
    ElMessage.error('只能处理本团队的考核记录')
    return
  }

  finalizeTarget.value = row
  finalizeComment.value = row.comment ?? ''
  finalizeDialogVisible.value = true
}

async function submitFinalize() {
  if (!finalizeTarget.value || (shouldLimitToOwnTeam.value && !isAccessibleTeamId(finalizeTarget.value.teamId))) {
    if (finalizeTarget.value) {
      ElMessage.error('只能处理本团队的考核记录')
    }

    return
  }

  await finalizeMutation.mutateAsync({
    assessmentId: finalizeTarget.value.id,
    payload: {
      comment: finalizeComment.value || undefined,
    },
  })
  ElMessage.success('考核单已归档')
  finalizeDialogVisible.value = false
}
</script>

<template>
  <PageSection title="月度考核" description="支持月度考核单生成、评分、归档与明细查看。">
    <div class="page-toolbar">
      <div class="page-toolbar__filters">
        <el-date-picker v-model="filters.assessMonth" type="month" value-format="YYYY-MM" placeholder="考核月份" />
        <el-select v-model="filters.teamId" :disabled="shouldLimitToOwnTeam" placeholder="全部团队" clearable style="width: 180px">
          <el-option
            v-for="item in teamOptions"
            :key="item.id"
            :label="item.teamName"
            :value="item.id"
          />
        </el-select>
        <el-select v-model="filters.memberId" placeholder="全部成员" clearable style="width: 180px">
          <el-option
            v-for="item in memberOptionsQuery.data.value?.records ?? []"
            :key="item.id"
            :label="item.name"
            :value="item.id"
          />
        </el-select>
        <el-select v-model="filters.status" placeholder="全部状态" clearable style="width: 160px">
          <el-option v-for="item in assessmentStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
        <el-button type="primary" @click="pagination.pageNo = 1">查询</el-button>
        <el-button @click="resetFilters">重置</el-button>
      </div>
      <div class="page-toolbar__actions">
        <el-button type="primary" @click="openGenerateDialog">生成考核单</el-button>
      </div>
    </div>

    <el-table v-loading="assessmentsQuery.isLoading.value" :data="tableData">
      <el-table-column prop="memberName" label="成员" min-width="120" />
      <el-table-column prop="teamName" label="团队" min-width="160" />
      <el-table-column prop="assessMonth" label="考核月份" min-width="120" />
      <el-table-column label="部门经理分" min-width="120">
        <template #default="{ row }">
          {{ formatScore(row.deptManagerScore) }}
        </template>
      </el-table-column>
      <el-table-column label="技术总监分" min-width="120">
        <template #default="{ row }">
          {{ formatScore(row.techDirectorScore) }}
        </template>
      </el-table-column>
      <el-table-column label="总经理分" min-width="120">
        <template #default="{ row }">
          {{ formatScore(row.generalManagerScore) }}
        </template>
      </el-table-column>
      <el-table-column label="最终得分" min-width="120">
        <template #default="{ row }">
          {{ formatScore(row.finalScore) }}
        </template>
      </el-table-column>
      <el-table-column label="状态" width="120">
        <template #default="{ row }">
          <StatusTag :meta="getAssessmentStatusMeta(row.status)" />
        </template>
      </el-table-column>
      <el-table-column label="操作" min-width="240" fixed="right">
        <template #default="{ row }">
          <div class="table-ops">
            <el-button link type="primary" @click="openDetail(row)">详情</el-button>
            <el-button v-if="canScoreWithCurrentRole" link @click="openScoreDialog(row)">评分</el-button>
            <el-button link @click="openFinalizeDialog(row)">归档</el-button>
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

  <el-dialog v-model="generateDialogVisible" title="生成月度考核单" width="460px">
    <el-form label-position="top">
      <el-form-item label="考核月份">
        <el-date-picker v-model="generateForm.assessMonth" type="month" value-format="YYYY-MM" class="w-full" />
      </el-form-item>
      <el-form-item label="指定团队">
        <el-select v-model="generateForm.teamId" :disabled="shouldLimitToOwnTeam" clearable placeholder="为空时生成全部团队">
          <el-option
            v-for="item in teamOptions"
            :key="item.id"
            :label="item.teamName"
            :value="item.id"
          />
        </el-select>
      </el-form-item>
      <el-form-item>
        <el-checkbox v-model="generateForm.overwriteExisting">覆盖已有考核单</el-checkbox>
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="generateDialogVisible = false">取消</el-button>
        <el-button type="primary" :loading="generateMutation.isPending.value" @click="submitGenerate">生成</el-button>
      </el-space>
    </template>
  </el-dialog>

  <el-dialog v-model="scoreDialogVisible" title="提交评分" width="460px">
    <el-form label-position="top">
      <el-form-item label="评分角色">
        <el-select v-model="scoreForm.scorerRole" disabled>
          <el-option v-for="item in scoreRoleOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
      </el-form-item>
      <el-form-item label="分数">
        <el-input-number v-model="scoreForm.rawScore" :min="0" :max="100" class="w-full" />
      </el-form-item>
      <el-form-item label="评语">
        <el-input v-model="scoreForm.comment" type="textarea" :rows="4" maxlength="300" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="scoreDialogVisible = false">取消</el-button>
        <el-button type="primary" :loading="scoreMutation.isPending.value" @click="submitScore">提交评分</el-button>
      </el-space>
    </template>
  </el-dialog>

  <el-dialog v-model="finalizeDialogVisible" title="归档考核单" width="460px">
    <el-form label-position="top">
      <el-form-item label="归档备注">
        <el-input v-model="finalizeComment" type="textarea" :rows="4" maxlength="300" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="finalizeDialogVisible = false">取消</el-button>
        <el-button type="primary" :loading="finalizeMutation.isPending.value" @click="submitFinalize">确认归档</el-button>
      </el-space>
    </template>
  </el-dialog>

  <el-drawer v-model="detailDrawerVisible" title="考核详情" size="560px">
    <div v-if="detailQuery.data.value" class="detail-grid">
      <div class="detail-grid__item">
        <span class="detail-grid__label">成员</span>
        <strong>{{ detailQuery.data.value.memberName }}</strong>
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">团队</span>
        <strong>{{ detailQuery.data.value.teamName || '-' }}</strong>
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">考核月份</span>
        <strong>{{ detailQuery.data.value.assessMonth }}</strong>
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">状态</span>
        <StatusTag :meta="getAssessmentStatusMeta(detailQuery.data.value.status)" />
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">最终得分</span>
        <strong>{{ formatScore(detailQuery.data.value.finalScore) }}</strong>
      </div>
      <div class="detail-grid__item">
        <span class="detail-grid__label">更新时间</span>
        <strong>{{ formatDateTime(detailQuery.data.value.updatedAt) }}</strong>
      </div>
    </div>

    <PageSection title="评分明细" class="mb-16">
      <el-table :data="detailQuery.data.value?.scoreDetails ?? []" size="small">
        <el-table-column prop="scorerName" label="评分人" min-width="120" />
        <el-table-column prop="scorerRole" label="角色" min-width="120" />
        <el-table-column prop="rawScore" label="分数" min-width="100" />
        <el-table-column prop="comment" label="评语" min-width="180" show-overflow-tooltip />
      </el-table>
    </PageSection>
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

.detail-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px 20px;
  margin-bottom: 16px;
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
