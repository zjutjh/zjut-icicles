<script setup lang="ts">
import type { FormInstance } from 'element-plus'
import { ElMessage } from 'element-plus'
import { computed, reactive, ref } from 'vue'

import PageSection from '@/components/common/PageSection.vue'
import StatusTag from '@/components/common/StatusTag.vue'
import { useMemberOptionsQuery, useProjectOptionsQuery } from '@/composables/use-reference-data'
import {
  useCreateTaskMutation,
  useCreateTaskProgressMutation,
  useTaskProgressQuery,
  useTasksQuery,
  useUpdateTaskMutation,
  useUpdateTaskStatusMutation,
} from '@/composables/use-task-management'
import { useTeamScope } from '@/composables/use-team-scope'
import { priorityMetaMap, priorityOptions, taskStatusMetaMap, taskStatusOptions } from '@/constants/options'
import type {
  CreateTaskRequest,
  PriorityLevel,
  Task,
  TaskStatus,
  UpdateTaskRequest,
} from '@/types/api'
import { formatDate, formatPercent } from '@/utils/format'

const TEXT = {
  pageTitle: '\u4efb\u52a1\u7ba1\u7406',
  pageDescription: '\u56f4\u7ed5\u5468\u4efb\u52a1\u3001\u8d23\u4efb\u4eba\u3001\u72b6\u6001\u6d41\u8f6c\u548c\u8fdb\u5ea6\u8bb0\u5f55\u5f62\u6210\u95ed\u73af\u3002',
  weekPlaceholder: '\u5468\u6b21\uff0c\u5982 2026-W24',
  allProjects: '\u5168\u90e8\u9879\u76ee',
  allAssignees: '\u5168\u90e8\u8d23\u4efb\u4eba',
  allStatuses: '\u5168\u90e8\u72b6\u6001',
  allPriorities: '\u5168\u90e8\u4f18\u5148\u7ea7',
  search: '\u67e5\u8be2',
  reset: '\u91cd\u7f6e',
  createTask: '\u65b0\u5efa\u4efb\u52a1',
  emptyHint: '\u5f53\u524d\u6682\u65e0\u8865\u5145\u8bf4\u660e\uff0c\u53ef\u901a\u8fc7\u72b6\u6001\u66f4\u65b0\u6216\u8fdb\u5ea6\u8bb0\u5f55\u7ee7\u7eed\u5b8c\u5584\u3002',
  noProject: '\u672a\u5173\u8054\u9879\u76ee',
  projectLabel: '\u6240\u5c5e\u9879\u76ee',
  assigneeLabel: '\u8d23\u4efb\u4eba',
  weekLabel: '\u5468\u6b21',
  deadlineLabel: '\u622a\u6b62\u65e5\u671f',
  planRangeLabel: '\u8ba1\u5212\u5468\u671f',
  taskProgress: '\u4efb\u52a1\u8fdb\u5ea6',
  edit: '\u7f16\u8f91',
  updateStatus: '\u66f4\u65b0\u72b6\u6001',
  progressRecord: '\u8fdb\u5ea6\u8bb0\u5f55',
  emptyTasks: '\u6682\u65e0\u4efb\u52a1\u6570\u636e\uff0c\u8c03\u6574\u7b5b\u9009\u6761\u4ef6\u540e\u518d\u8bd5\u8bd5\u3002',
  editTask: '\u7f16\u8f91\u4efb\u52a1',
  taskTitle: '\u4efb\u52a1\u6807\u9898',
  selectAssignee: '\u8bf7\u9009\u62e9\u8d23\u4efb\u4eba',
  optional: '\u53ef\u9009',
  priorityLabel: '\u4f18\u5148\u7ea7',
  weekIdLabel: '\u5468\u6b21',
  weekIdPlaceholder: '\u4f8b\u5982 2026-W24',
  planStartLabel: '\u8ba1\u5212\u5f00\u59cb',
  planEndLabel: '\u8ba1\u5212\u7ed3\u675f',
  issueDescLabel: '\u95ee\u9898\u4e0e\u5907\u6ce8',
  cancel: '\u53d6\u6d88',
  save: '\u4fdd\u5b58',
  statusDialogTitle: '\u66f4\u65b0\u4efb\u52a1\u72b6\u6001',
  taskStatusLabel: '\u4efb\u52a1\u72b6\u6001',
  progressPercentLabel: '\u8fdb\u5ea6\u767e\u5206\u6bd4',
  issueExplainLabel: '\u95ee\u9898\u8bf4\u660e',
  progressDrawerTitle: '\u4efb\u52a1\u8fdb\u5ea6\u8bb0\u5f55',
  currentTask: '\u5f53\u524d\u4efb\u52a1\uff1a',
  progressCommentLabel: '\u8fdb\u5ea6\u5907\u6ce8',
  addProgress: '\u65b0\u589e\u8fdb\u5ea6\u8bb0\u5f55',
  progressPrefix: '\u8fdb\u5ea6\uff1a',
  commentPrefix: '\u5907\u6ce8\uff1a',
  issuePrefix: '\u95ee\u9898\uff1a',
  ruleTaskTitle: '\u8bf7\u8f93\u5165\u4efb\u52a1\u6807\u9898',
  ruleAssignee: '\u8bf7\u9009\u62e9\u8d23\u4efb\u4eba',
  rulePriority: '\u8bf7\u9009\u62e9\u4f18\u5148\u7ea7',
  ruleWeekNo: '\u8bf7\u8f93\u5165\u5468\u6b21\u6807\u8bc6',
  ruleProgressRate: '\u8bf7\u586b\u5199\u8fdb\u5ea6\u767e\u5206\u6bd4',
  ruleProgressComment: '\u8bf7\u586b\u5199\u8fdb\u5ea6\u8bf4\u660e',
  messageTaskUpdated: '\u4efb\u52a1\u5df2\u66f4\u65b0',
  messageTaskCreated: '\u4efb\u52a1\u5df2\u521b\u5efa',
  messageStatusUpdated: '\u4efb\u52a1\u72b6\u6001\u5df2\u66f4\u65b0',
  messageProgressAdded: '\u8fdb\u5ea6\u8bb0\u5f55\u5df2\u65b0\u589e',
  dash: '-',
} as const

const filters = reactive({
  weekNo: '',
  projectId: undefined as number | undefined,
  assigneeId: undefined as number | undefined,
  status: undefined as TaskStatus | undefined,
  priority: undefined as PriorityLevel | undefined,
})

const pagination = reactive({
  pageNo: 1,
  pageSize: 10,
})

const { currentTeamId, shouldLimitToOwnTeam } = useTeamScope()

const queryParams = computed(() => ({
  weekNo: filters.weekNo || undefined,
  projectId: filters.projectId,
  assigneeId: filters.assigneeId,
  status: filters.status,
  priority: filters.priority,
  pageNo: shouldLimitToOwnTeam.value ? 1 : pagination.pageNo,
  pageSize: shouldLimitToOwnTeam.value ? 500 : pagination.pageSize,
}))

const tasksQuery = useTasksQuery(queryParams)
const projectOptionsQuery = useProjectOptionsQuery()
const memberOptionsQuery = useMemberOptionsQuery(
  computed(() => (shouldLimitToOwnTeam.value && currentTeamId.value ? { teamId: currentTeamId.value } : {})),
)
const createTaskMutation = useCreateTaskMutation()
const updateTaskMutation = useUpdateTaskMutation()
const updateTaskStatusMutation = useUpdateTaskStatusMutation()
const createTaskProgressMutation = useCreateTaskProgressMutation()

const dialogVisible = ref(false)
const editingTask = ref<Task | null>(null)
const formRef = ref<FormInstance>()

const form = reactive({
  title: '',
  projectId: undefined as number | undefined,
  assigneeId: undefined as number | undefined,
  priority: 'MEDIUM' as PriorityLevel,
  weekNo: '',
  deadline: '',
  planStartDate: '',
  planEndDate: '',
  issueDesc: '',
})

const rules = {
  title: [{ required: true, message: TEXT.ruleTaskTitle, trigger: 'blur' }],
  assigneeId: [{ required: true, message: TEXT.ruleAssignee, trigger: 'change' }],
  priority: [{ required: true, message: TEXT.rulePriority, trigger: 'change' }],
  weekNo: [{ required: true, message: TEXT.ruleWeekNo, trigger: 'blur' }],
}

const statusDialogVisible = ref(false)
const statusTarget = ref<Task | null>(null)
const statusForm = reactive({
  status: 'TODO' as TaskStatus,
  progressRate: 0,
  issueDesc: '',
})

const progressDrawerVisible = ref(false)
const progressTask = ref<Task | null>(null)
const progressFormRef = ref<FormInstance>()
const progressForm = reactive({
  progressRate: 0,
  issueDesc: '',
  comment: '',
})

const progressRules = {
  progressRate: [{ required: true, message: TEXT.ruleProgressRate, trigger: 'change' }],
  comment: [{ required: true, message: TEXT.ruleProgressComment, trigger: 'blur' }],
}

const progressQuery = useTaskProgressQuery(computed(() => progressTask.value?.id ?? null))
const accessibleMemberIds = computed(() => new Set((memberOptionsQuery.data.value?.records ?? []).map((item) => item.id)))
const filteredTaskRows = computed(() => {
  const rows = tasksQuery.data.value?.records ?? []

  if (!shouldLimitToOwnTeam.value) {
    return rows
  }

  return rows.filter((row) => accessibleMemberIds.value.has(row.assigneeId))
})
const tableData = computed(() => {
  if (!shouldLimitToOwnTeam.value) {
    return filteredTaskRows.value
  }

  const start = (pagination.pageNo - 1) * pagination.pageSize
  return filteredTaskRows.value.slice(start, start + pagination.pageSize)
})
const total = computed(() =>
  shouldLimitToOwnTeam.value ? filteredTaskRows.value.length : (tasksQuery.data.value?.total ?? 0),
)

function isTaskAccessible(task: Task) {
  return !shouldLimitToOwnTeam.value || accessibleMemberIds.value.has(task.assigneeId)
}

function getTaskStatusMeta(status: TaskStatus) {
  return taskStatusMetaMap[status]
}

function getPriorityStatusMeta(priority: PriorityLevel) {
  return priorityMetaMap[priority]
}

function resetFilters() {
  filters.weekNo = ''
  filters.projectId = undefined
  filters.assigneeId = undefined
  filters.status = undefined
  filters.priority = undefined
  pagination.pageNo = 1
}

function resetForm() {
  form.title = ''
  form.projectId = undefined
  form.assigneeId = undefined
  form.priority = 'MEDIUM'
  form.weekNo = ''
  form.deadline = ''
  form.planStartDate = ''
  form.planEndDate = ''
  form.issueDesc = ''
  formRef.value?.clearValidate()
}

function openCreateDialog() {
  editingTask.value = null
  resetForm()
  dialogVisible.value = true
}

function openEditDialog(task: Task) {
  if (!isTaskAccessible(task)) {
    ElMessage.error('只能管理本团队成员的任务')
    return
  }

  editingTask.value = task
  form.title = task.title
  form.projectId = task.projectId ?? undefined
  form.assigneeId = task.assigneeId
  form.priority = task.priority
  form.weekNo = task.weekNo ?? ''
  form.deadline = task.deadline ?? ''
  form.planStartDate = task.planStartDate ?? ''
  form.planEndDate = task.planEndDate ?? ''
  form.issueDesc = task.issueDesc ?? ''
  dialogVisible.value = true
}

async function submitTask() {
  const valid = await formRef.value?.validate().catch(() => false)
  if (!valid) {
    return
  }

  if (shouldLimitToOwnTeam.value && !accessibleMemberIds.value.has(form.assigneeId as number)) {
    ElMessage.error('只能为本团队成员创建或分配任务')
    return
  }

  if (editingTask.value) {
    if (!isTaskAccessible(editingTask.value)) {
      ElMessage.error('只能管理本团队成员的任务')
      return
    }

    const payload: UpdateTaskRequest = {
      title: form.title,
      projectId: form.projectId,
      assigneeId: form.assigneeId as number,
      priority: form.priority,
      deadline: form.deadline || undefined,
      planStartDate: form.planStartDate || undefined,
      planEndDate: form.planEndDate || undefined,
      issueDesc: form.issueDesc || undefined,
    }

    await updateTaskMutation.mutateAsync({
      taskId: editingTask.value.id,
      payload,
    })
    ElMessage.success(TEXT.messageTaskUpdated)
  } else {
    const payload: CreateTaskRequest = {
      title: form.title,
      projectId: form.projectId,
      assigneeId: form.assigneeId as number,
      priority: form.priority,
      weekNo: form.weekNo,
      deadline: form.deadline || undefined,
      planStartDate: form.planStartDate || undefined,
      planEndDate: form.planEndDate || undefined,
      issueDesc: form.issueDesc || undefined,
    }

    await createTaskMutation.mutateAsync(payload)
    ElMessage.success(TEXT.messageTaskCreated)
  }

  dialogVisible.value = false
  resetForm()
}

function openStatusDialog(task: Task) {
  if (!isTaskAccessible(task)) {
    ElMessage.error('只能管理本团队成员的任务')
    return
  }

  statusTarget.value = task
  statusForm.status = task.status
  statusForm.progressRate = task.progressRate ?? 0
  statusForm.issueDesc = task.issueDesc ?? ''
  statusDialogVisible.value = true
}

async function submitStatus() {
  if (!statusTarget.value || !isTaskAccessible(statusTarget.value)) {
    if (statusTarget.value) {
      ElMessage.error('只能管理本团队成员的任务')
    }

    return
  }

  await updateTaskStatusMutation.mutateAsync({
    taskId: statusTarget.value.id,
    payload: {
      status: statusForm.status,
      progressRate: statusForm.progressRate,
      issueDesc: statusForm.issueDesc || undefined,
    },
  })
  ElMessage.success(TEXT.messageStatusUpdated)
  statusDialogVisible.value = false
}

function openProgressDrawer(task: Task) {
  if (!isTaskAccessible(task)) {
    ElMessage.error('只能管理本团队成员的任务')
    return
  }

  progressTask.value = task
  progressForm.progressRate = task.progressRate ?? 0
  progressForm.issueDesc = task.issueDesc ?? ''
  progressForm.comment = ''
  progressDrawerVisible.value = true
}

async function submitProgress() {
  const valid = await progressFormRef.value?.validate().catch(() => false)
  if (!valid || !progressTask.value || !isTaskAccessible(progressTask.value)) {
    if (progressTask.value && !isTaskAccessible(progressTask.value)) {
      ElMessage.error('只能管理本团队成员的任务')
    }

    return
  }

  await createTaskProgressMutation.mutateAsync({
    taskId: progressTask.value.id,
    payload: {
      progressRate: progressForm.progressRate,
      issueDesc: progressForm.issueDesc || undefined,
      comment: progressForm.comment,
    },
  })

  ElMessage.success(TEXT.messageProgressAdded)
  progressForm.comment = ''
}
</script>

<template>
  <PageSection :title="TEXT.pageTitle" :description="TEXT.pageDescription">
    <div class="page-toolbar">
      <div class="page-toolbar__filters">
        <el-input v-model="filters.weekNo" :placeholder="TEXT.weekPlaceholder" clearable />
        <el-select v-model="filters.projectId" :placeholder="TEXT.allProjects" clearable>
          <el-option
            v-for="item in projectOptionsQuery.data.value?.records ?? []"
            :key="item.id"
            :label="item.projectName"
            :value="item.id"
          />
        </el-select>
        <el-select v-model="filters.assigneeId" :placeholder="TEXT.allAssignees" clearable>
          <el-option
            v-for="item in memberOptionsQuery.data.value?.records ?? []"
            :key="item.id"
            :label="item.name"
            :value="item.id"
          />
        </el-select>
        <el-select v-model="filters.status" :placeholder="TEXT.allStatuses" clearable>
          <el-option v-for="item in taskStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
        <el-select v-model="filters.priority" :placeholder="TEXT.allPriorities" clearable>
          <el-option v-for="item in priorityOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
      </div>

      <div class="page-toolbar__actions">
        <el-button type="primary" @click="pagination.pageNo = 1">{{ TEXT.search }}</el-button>
        <el-button @click="resetFilters">{{ TEXT.reset }}</el-button>
        <el-button type="primary" plain @click="openCreateDialog">{{ TEXT.createTask }}</el-button>
      </div>
    </div>

    <div v-loading="tasksQuery.isLoading.value" class="task-board">
      <template v-if="tableData.length">
        <article v-for="row in tableData" :key="row.id" class="task-card">
          <div class="task-card__header">
            <div class="task-card__title-wrap">
              <h3 class="task-card__title">{{ row.title }}</h3>
              <p class="task-card__subtitle">
                {{ row.issueDesc || TEXT.emptyHint }}
              </p>
            </div>

            <div class="task-card__tags">
              <StatusTag :meta="getTaskStatusMeta(row.status)" />
              <StatusTag :meta="getPriorityStatusMeta(row.priority)" />
            </div>
          </div>

          <div class="task-card__meta">
            <div class="task-meta-item">
              <span class="task-meta-item__label">{{ TEXT.projectLabel }}</span>
              <strong>{{ row.projectName || TEXT.noProject }}</strong>
            </div>
            <div class="task-meta-item">
              <span class="task-meta-item__label">{{ TEXT.assigneeLabel }}</span>
              <strong>{{ row.assigneeName || TEXT.dash }}</strong>
            </div>
            <div class="task-meta-item">
              <span class="task-meta-item__label">{{ TEXT.weekLabel }}</span>
              <strong>{{ row.weekNo || TEXT.dash }}</strong>
            </div>
            <div class="task-meta-item">
              <span class="task-meta-item__label">{{ TEXT.deadlineLabel }}</span>
              <strong>{{ formatDate(row.deadline) }}</strong>
            </div>
            <div class="task-meta-item">
              <span class="task-meta-item__label">{{ TEXT.planRangeLabel }}</span>
              <strong>{{ formatDate(row.planStartDate) }} - {{ formatDate(row.planEndDate) }}</strong>
            </div>
          </div>

          <div class="task-card__footer">
            <div class="task-progress">
              <div class="task-progress__head">
                <span>{{ TEXT.taskProgress }}</span>
                <strong>{{ formatPercent(row.progressRate) }}</strong>
              </div>
              <el-progress
                :percentage="row.progressRate ?? 0"
                :show-text="false"
                :stroke-width="8"
                color="#4B6584"
              />
            </div>

            <div class="task-card__actions">
              <el-button class="task-action-button" @click="openEditDialog(row)">{{ TEXT.edit }}</el-button>
              <el-button class="task-action-button" @click="openStatusDialog(row)">{{ TEXT.updateStatus }}</el-button>
              <el-button class="task-action-button task-action-button--primary" @click="openProgressDrawer(row)">
                {{ TEXT.progressRecord }}
              </el-button>
            </div>
          </div>
        </article>
      </template>

      <el-empty v-else :description="TEXT.emptyTasks" />
    </div>

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

  <el-dialog v-model="dialogVisible" :title="editingTask ? TEXT.editTask : TEXT.createTask" width="820px" @closed="resetForm">
    <el-form ref="formRef" :model="form" :rules="rules" label-position="top">
      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item :label="TEXT.taskTitle" prop="title">
            <el-input v-model="form.title" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item :label="TEXT.assigneeLabel" prop="assigneeId">
            <el-select v-model="form.assigneeId" :placeholder="TEXT.selectAssignee">
              <el-option
                v-for="item in memberOptionsQuery.data.value?.records ?? []"
                :key="item.id"
                :label="item.name"
                :value="item.id"
              />
            </el-select>
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="8">
          <el-form-item :label="TEXT.projectLabel">
            <el-select v-model="form.projectId" clearable :placeholder="TEXT.optional">
              <el-option
                v-for="item in projectOptionsQuery.data.value?.records ?? []"
                :key="item.id"
                :label="item.projectName"
                :value="item.id"
              />
            </el-select>
          </el-form-item>
        </el-col>
        <el-col :span="8">
          <el-form-item :label="TEXT.priorityLabel" prop="priority">
            <el-select v-model="form.priority">
              <el-option v-for="item in priorityOptions" :key="item.value" :label="item.label" :value="item.value" />
            </el-select>
          </el-form-item>
        </el-col>
        <el-col :span="8">
          <el-form-item :label="TEXT.weekIdLabel" prop="weekNo">
            <el-input v-model="form.weekNo" :disabled="Boolean(editingTask)" :placeholder="TEXT.weekIdPlaceholder" />
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="8">
          <el-form-item :label="TEXT.planStartLabel">
            <el-date-picker v-model="form.planStartDate" type="date" value-format="YYYY-MM-DD" class="w-full" />
          </el-form-item>
        </el-col>
        <el-col :span="8">
          <el-form-item :label="TEXT.planEndLabel">
            <el-date-picker v-model="form.planEndDate" type="date" value-format="YYYY-MM-DD" class="w-full" />
          </el-form-item>
        </el-col>
        <el-col :span="8">
          <el-form-item :label="TEXT.deadlineLabel">
            <el-date-picker v-model="form.deadline" type="date" value-format="YYYY-MM-DD" class="w-full" />
          </el-form-item>
        </el-col>
      </el-row>

      <el-form-item :label="TEXT.issueDescLabel">
        <el-input v-model="form.issueDesc" type="textarea" :rows="4" maxlength="300" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="dialogVisible = false">{{ TEXT.cancel }}</el-button>
        <el-button
          type="primary"
          :loading="createTaskMutation.isPending.value || updateTaskMutation.isPending.value"
          @click="submitTask"
        >
          {{ TEXT.save }}
        </el-button>
      </el-space>
    </template>
  </el-dialog>

  <el-dialog v-model="statusDialogVisible" :title="TEXT.statusDialogTitle" width="460px">
    <el-form label-position="top">
      <el-form-item :label="TEXT.taskStatusLabel">
        <el-select v-model="statusForm.status">
          <el-option v-for="item in taskStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
      </el-form-item>
      <el-form-item :label="TEXT.progressPercentLabel">
        <el-input-number v-model="statusForm.progressRate" :min="0" :max="100" class="w-full" />
      </el-form-item>
      <el-form-item :label="TEXT.issueExplainLabel">
        <el-input v-model="statusForm.issueDesc" type="textarea" :rows="3" maxlength="200" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="statusDialogVisible = false">{{ TEXT.cancel }}</el-button>
        <el-button type="primary" :loading="updateTaskStatusMutation.isPending.value" @click="submitStatus">
          {{ TEXT.save }}
        </el-button>
      </el-space>
    </template>
  </el-dialog>

  <el-drawer v-model="progressDrawerVisible" :title="TEXT.progressDrawerTitle" size="520px">
    <div class="drawer-task-title">{{ TEXT.currentTask }}{{ progressTask?.title ?? TEXT.dash }}</div>

    <el-form ref="progressFormRef" :model="progressForm" :rules="progressRules" label-position="top" class="mb-16">
      <el-form-item :label="TEXT.progressPercentLabel" prop="progressRate">
        <el-input-number v-model="progressForm.progressRate" :min="0" :max="100" class="w-full" />
      </el-form-item>
      <el-form-item :label="TEXT.issueExplainLabel">
        <el-input v-model="progressForm.issueDesc" type="textarea" :rows="3" maxlength="200" show-word-limit />
      </el-form-item>
      <el-form-item :label="TEXT.progressCommentLabel" prop="comment">
        <el-input v-model="progressForm.comment" type="textarea" :rows="3" maxlength="300" show-word-limit />
      </el-form-item>
      <el-button type="primary" :loading="createTaskProgressMutation.isPending.value" @click="submitProgress">
        {{ TEXT.addProgress }}
      </el-button>
    </el-form>

    <el-timeline>
      <el-timeline-item
        v-for="item in progressQuery.data.value ?? []"
        :key="item.id"
        :timestamp="formatDate(item.updateTime)"
        placement="top"
      >
        <el-card shadow="never" class="progress-record-card">
          <p>{{ TEXT.progressPrefix }}{{ formatPercent(item.progressRate) }}</p>
          <p class="text-muted">{{ TEXT.commentPrefix }}{{ item.comment || TEXT.dash }}</p>
          <p class="text-muted">{{ TEXT.issuePrefix }}{{ item.issueDesc || TEXT.dash }}</p>
        </el-card>
      </el-timeline-item>
    </el-timeline>
  </el-drawer>
</template>

<style scoped lang="scss">
.page-toolbar {
  display: flex;
  flex-wrap: wrap;
  align-items: flex-start;
  justify-content: space-between;
  gap: 16px;
  margin-bottom: 20px;
}

.page-toolbar__filters,
.page-toolbar__actions {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
}

.page-toolbar__filters :deep(.el-input),
.page-toolbar__filters :deep(.el-select) {
  width: 180px;
}

.task-board {
  display: grid;
  gap: 16px;
  min-height: 280px;
}

.task-card {
  border: 1px solid var(--theme-border);
  border-radius: 14px;
  background: #fff;
  box-shadow: 0 10px 24px rgba(63, 94, 122, 0.05);
  padding: 20px 22px;
}

.task-card__header {
  display: flex;
  align-items: flex-start;
  justify-content: space-between;
  gap: 16px;
  padding-bottom: 16px;
  border-bottom: 1px solid rgba(229, 234, 240, 0.9);
}

.task-card__title-wrap {
  min-width: 0;
  flex: 1;
}

.task-card__title {
  margin: 0;
  font-size: 18px;
  font-weight: 700;
  line-height: 1.4;
  color: #2f3542;
}

.task-card__subtitle {
  margin: 8px 0 0;
  font-size: 13px;
  line-height: 1.7;
  color: #7b8794;
}

.task-card__tags {
  display: flex;
  flex-wrap: wrap;
  justify-content: flex-end;
  gap: 8px;
}

.task-card__meta {
  display: grid;
  grid-template-columns: repeat(5, minmax(0, 1fr));
  gap: 14px 16px;
  padding: 18px 0;
}

.task-meta-item {
  min-width: 0;
  padding: 12px 14px;
  border: 1px solid rgba(229, 234, 240, 0.85);
  border-radius: 12px;
  background: #f9fbfd;
}

.task-meta-item__label {
  display: block;
  margin-bottom: 8px;
  font-size: 12px;
  color: #7b8794;
}

.task-meta-item strong {
  display: block;
  color: #2f3542;
  font-size: 14px;
  line-height: 1.5;
  word-break: break-word;
}

.task-card__footer {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 18px;
  padding-top: 18px;
  border-top: 1px solid rgba(229, 234, 240, 0.9);
}

.task-progress {
  flex: 1;
  min-width: 0;
}

.task-progress__head {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 10px;
  font-size: 13px;
  color: #7b8794;
}

.task-progress__head strong {
  font-size: 15px;
  color: #2f3542;
}

.task-card__actions {
  display: flex;
  flex-wrap: wrap;
  justify-content: flex-end;
  gap: 10px;
}

.task-action-button {
  min-width: 92px;
  margin: 0;
  border-color: #d8e1ea;
  color: #4b6584;
  background: #fff;
}

.task-action-button:hover {
  border-color: #b9c7d6;
  color: #3f5e7a;
  background: #f7fafd;
}

.task-action-button--primary {
  border-color: #c6d3e1;
  background: #eef3f8;
}

.pagination-bar {
  display: flex;
  justify-content: flex-end;
  margin-top: 20px;
}

.drawer-task-title {
  margin-bottom: 16px;
  color: #7b8794;
}

.progress-record-card {
  border: 1px solid var(--theme-border);
  border-radius: 12px;
}

.progress-record-card p {
  margin: 0 0 8px;
}

.progress-record-card p:last-child {
  margin-bottom: 0;
}

.text-muted {
  color: var(--theme-foreground-secondary);
}

@media (max-width: 1280px) {
  .task-card__meta {
    grid-template-columns: repeat(3, minmax(0, 1fr));
  }
}

@media (max-width: 900px) {
  .page-toolbar,
  .task-card__header,
  .task-card__footer {
    flex-direction: column;
    align-items: stretch;
  }

  .page-toolbar__actions,
  .task-card__actions,
  .task-card__tags {
    justify-content: flex-start;
  }

  .task-card__meta {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
}

@media (max-width: 640px) {
  .page-toolbar__filters :deep(.el-input),
  .page-toolbar__filters :deep(.el-select),
  .page-toolbar__actions :deep(.el-button) {
    width: 100%;
  }

  .task-card {
    padding: 16px;
  }

  .task-card__meta {
    grid-template-columns: 1fr;
  }

  .task-card__actions :deep(.el-button) {
    width: 100%;
  }
}
</style>
