<script setup lang="ts">
import type { FormInstance } from 'element-plus'
import { ElMessage } from 'element-plus'
import { computed, reactive, ref } from 'vue'

import PageSection from '@/components/common/PageSection.vue'
import StatusTag from '@/components/common/StatusTag.vue'
import {
  useCreateProjectMutation,
  useProjectsQuery,
  useUpdateProjectMutation,
  useUpdateProjectStatusMutation,
} from '@/composables/use-project-management'
import { useUserOptionsQuery } from '@/composables/use-reference-data'
import { priorityMetaMap, priorityOptions, projectStatusMetaMap, projectStatusOptions } from '@/constants/options'
import type {
  CreateProjectRequest,
  PriorityLevel,
  Project,
  ProjectStatus,
  UpdateProjectRequest,
} from '@/types/api'
import { formatDate } from '@/utils/format'

const filters = reactive({
  keyword: '',
  status: undefined as ProjectStatus | undefined,
  priority: undefined as PriorityLevel | undefined,
})

const pagination = reactive({
  pageNo: 1,
  pageSize: 10,
})

const queryParams = computed(() => ({
  keyword: filters.keyword || undefined,
  status: filters.status,
  priority: filters.priority,
  pageNo: pagination.pageNo,
  pageSize: pagination.pageSize,
}))

const projectsQuery = useProjectsQuery(queryParams)
const ownerOptionsQuery = useUserOptionsQuery()
const createProjectMutation = useCreateProjectMutation()
const updateProjectMutation = useUpdateProjectMutation()
const updateProjectStatusMutation = useUpdateProjectStatusMutation()

const dialogVisible = ref(false)
const editingProject = ref<Project | null>(null)
const formRef = ref<FormInstance>()

const form = reactive({
  projectName: '',
  ownerId: undefined as number | undefined,
  status: 'PLANNING' as ProjectStatus,
  priority: 'MEDIUM' as PriorityLevel,
  startDate: '',
  endDate: '',
  milestone: '',
  description: '',
})

const rules = {
  projectName: [{ required: true, message: '请输入项目名称', trigger: 'blur' }],
  priority: [{ required: true, message: '请选择优先级', trigger: 'change' }],
  startDate: [{ required: true, message: '请选择开始日期', trigger: 'change' }],
}

const statusDialogVisible = ref(false)
const statusTarget = ref<Project | null>(null)
const statusForm = reactive({
  status: 'PLANNING' as ProjectStatus,
  comment: '',
})

const tableData = computed(() => projectsQuery.data.value?.records ?? [])

function getProjectStatusMeta(status: ProjectStatus) {
  return projectStatusMetaMap[status]
}

function getPriorityStatusMeta(priority: PriorityLevel) {
  return priorityMetaMap[priority]
}

function resetFilters() {
  filters.keyword = ''
  filters.status = undefined
  filters.priority = undefined
  pagination.pageNo = 1
}

function resetForm() {
  form.projectName = ''
  form.ownerId = undefined
  form.status = 'PLANNING'
  form.priority = 'MEDIUM'
  form.startDate = ''
  form.endDate = ''
  form.milestone = ''
  form.description = ''
  formRef.value?.clearValidate()
}

function openCreateDialog() {
  editingProject.value = null
  resetForm()
  dialogVisible.value = true
}

function openEditDialog(project: Project) {
  editingProject.value = project
  form.projectName = project.projectName
  form.ownerId = project.ownerId ?? undefined
  form.status = project.status
  form.priority = project.priority
  form.startDate = project.startDate
  form.endDate = project.endDate ?? ''
  form.milestone = project.milestone ?? ''
  form.description = project.description ?? ''
  dialogVisible.value = true
}

async function submitProject() {
  const valid = await formRef.value?.validate().catch(() => false)
  if (!valid) {
    return
  }

  if (editingProject.value) {
    const payload: UpdateProjectRequest = {
      projectName: form.projectName,
      ownerId: form.ownerId,
      priority: form.priority,
      startDate: form.startDate,
      endDate: form.endDate || undefined,
      milestone: form.milestone || undefined,
      description: form.description || undefined,
    }

    await updateProjectMutation.mutateAsync({
      projectId: editingProject.value.id,
      payload,
    })
    ElMessage.success('项目已更新')
  } else {
    const payload: CreateProjectRequest = {
      projectName: form.projectName,
      ownerId: form.ownerId,
      status: form.status,
      priority: form.priority,
      startDate: form.startDate,
      endDate: form.endDate || undefined,
      milestone: form.milestone || undefined,
      description: form.description || undefined,
    }

    await createProjectMutation.mutateAsync(payload)
    ElMessage.success('项目已创建')
  }

  dialogVisible.value = false
  resetForm()
}

function openStatusDialog(project: Project) {
  statusTarget.value = project
  statusForm.status = project.status
  statusForm.comment = ''
  statusDialogVisible.value = true
}

async function submitStatus() {
  if (!statusTarget.value) {
    return
  }

  await updateProjectStatusMutation.mutateAsync({
    projectId: statusTarget.value.id,
    payload: {
      status: statusForm.status,
      comment: statusForm.comment || undefined,
    },
  })
  ElMessage.success('项目状态已更新')
  statusDialogVisible.value = false
}
</script>

<template>
  <PageSection title="项目管理" description="维护项目负责人、周期、里程碑和整体状态。">
    <div class="page-toolbar">
      <div class="page-toolbar__filters">
        <el-input v-model="filters.keyword" placeholder="搜索项目名称" clearable style="max-width: 220px" />
        <el-select v-model="filters.status" placeholder="全部状态" clearable style="width: 160px">
          <el-option v-for="item in projectStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
        <el-select v-model="filters.priority" placeholder="全部优先级" clearable style="width: 160px">
          <el-option v-for="item in priorityOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
        <el-button type="primary" @click="pagination.pageNo = 1">查询</el-button>
        <el-button @click="resetFilters">重置</el-button>
      </div>

      <div class="page-toolbar__actions">
        <el-button type="primary" @click="openCreateDialog">新建项目</el-button>
      </div>
    </div>

    <el-table v-loading="projectsQuery.isLoading.value" :data="tableData">
      <el-table-column prop="projectName" label="项目名称" min-width="220" />
      <el-table-column prop="ownerName" label="负责人" min-width="120" />
      <el-table-column label="状态" width="110">
        <template #default="{ row }">
          <StatusTag :meta="getProjectStatusMeta(row.status)" />
        </template>
      </el-table-column>
      <el-table-column label="优先级" width="110">
        <template #default="{ row }">
          <StatusTag :meta="getPriorityStatusMeta(row.priority)" />
        </template>
      </el-table-column>
      <el-table-column label="开始日期" min-width="120">
        <template #default="{ row }">
          {{ formatDate(row.startDate) }}
        </template>
      </el-table-column>
      <el-table-column label="结束日期" min-width="120">
        <template #default="{ row }">
          {{ formatDate(row.endDate) }}
        </template>
      </el-table-column>
      <el-table-column prop="milestone" label="里程碑" min-width="180" show-overflow-tooltip />
      <el-table-column label="操作" min-width="180" fixed="right">
        <template #default="{ row }">
          <div class="table-ops">
            <el-button link type="primary" @click="openEditDialog(row)">编辑</el-button>
            <el-button link @click="openStatusDialog(row)">更新状态</el-button>
          </div>
        </template>
      </el-table-column>
    </el-table>

    <div class="pagination-bar">
      <el-pagination
        v-model:current-page="pagination.pageNo"
        v-model:page-size="pagination.pageSize"
        :total="projectsQuery.data.value?.total ?? 0"
        :page-sizes="[10, 20, 50]"
        layout="total, sizes, prev, pager, next"
      />
    </div>
  </PageSection>

  <el-dialog v-model="dialogVisible" :title="editingProject ? '编辑项目' : '新建项目'" width="760px" @closed="resetForm">
    <el-form ref="formRef" :model="form" :rules="rules" label-position="top">
      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="项目名称" prop="projectName">
            <el-input v-model="form.projectName" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="负责人">
            <el-select v-model="form.ownerId" clearable placeholder="可选">
              <el-option
                v-for="item in ownerOptionsQuery.data.value?.records ?? []"
                :key="item.id"
                :label="item.realName"
                :value="item.id"
              />
            </el-select>
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="8">
          <el-form-item v-if="!editingProject" label="初始状态">
            <el-select v-model="form.status">
              <el-option v-for="item in projectStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
            </el-select>
          </el-form-item>
        </el-col>
        <el-col :span="8">
          <el-form-item label="优先级" prop="priority">
            <el-select v-model="form.priority">
              <el-option v-for="item in priorityOptions" :key="item.value" :label="item.label" :value="item.value" />
            </el-select>
          </el-form-item>
        </el-col>
        <el-col :span="8">
          <el-form-item label="里程碑">
            <el-input v-model="form.milestone" />
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="开始日期" prop="startDate">
            <el-date-picker v-model="form.startDate" type="date" value-format="YYYY-MM-DD" class="w-full" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="结束日期">
            <el-date-picker v-model="form.endDate" type="date" value-format="YYYY-MM-DD" class="w-full" />
          </el-form-item>
        </el-col>
      </el-row>

      <el-form-item label="项目描述">
        <el-input v-model="form.description" type="textarea" :rows="4" maxlength="300" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="dialogVisible = false">取消</el-button>
        <el-button
          type="primary"
          :loading="createProjectMutation.isPending.value || updateProjectMutation.isPending.value"
          @click="submitProject"
        >
          保存
        </el-button>
      </el-space>
    </template>
  </el-dialog>

  <el-dialog v-model="statusDialogVisible" title="更新项目状态" width="420px">
    <el-form label-position="top">
      <el-form-item label="状态">
        <el-select v-model="statusForm.status">
          <el-option v-for="item in projectStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
      </el-form-item>
      <el-form-item label="备注">
        <el-input v-model="statusForm.comment" type="textarea" :rows="3" maxlength="200" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="statusDialogVisible = false">取消</el-button>
        <el-button type="primary" :loading="updateProjectStatusMutation.isPending.value" @click="submitStatus">
          保存
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

@media (max-width: 768px) {
  .page-toolbar {
    flex-direction: column;
    align-items: stretch;
  }
}
</style>
