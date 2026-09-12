<script setup lang="ts">
import type { FormInstance } from 'element-plus'
import { ElMessage } from 'element-plus'
import { computed, reactive, ref } from 'vue'

import PageSection from '@/components/common/PageSection.vue'
import StatusTag from '@/components/common/StatusTag.vue'
import {
  useCreateMemberMutation,
  useCreateTeamMutation,
  useMembersQuery,
  useTeamsQuery,
  useUpdateMemberMutation,
  useUpdateMemberStatusMutation,
  useUpdateTeamMutation,
} from '@/composables/use-organization-management'
import { useTeamOptionsQuery, useUserOptionsQuery } from '@/composables/use-reference-data'
import { useTeamScope } from '@/composables/use-team-scope'
import { memberStatusMetaMap, memberStatusOptions, teamStatusMetaMap, teamStatusOptions } from '@/constants/options'
import type {
  CreateMemberRequest,
  CreateTeamRequest,
  Member,
  MemberStatus,
  Team,
  TeamStatus,
  UpdateMemberRequest,
  UpdateTeamRequest,
} from '@/types/api'
import { formatDate, formatDateTime } from '@/utils/format'

const activeTab = ref<'teams' | 'members'>('teams')
const { currentTeamId, shouldLimitToOwnTeam, isAccessibleTeamId } = useTeamScope()

const teamFilters = reactive({
  keyword: '',
  status: undefined as TeamStatus | undefined,
})

const teamPagination = reactive({
  pageNo: 1,
  pageSize: 10,
})

const memberFilters = reactive({
  keyword: '',
  teamId: undefined as number | undefined,
  status: undefined as MemberStatus | undefined,
})

const memberPagination = reactive({
  pageNo: 1,
  pageSize: 10,
})

const teamsQuery = useTeamsQuery(
  computed(() => ({
    keyword: teamFilters.keyword || undefined,
    status: teamFilters.status,
    pageNo: shouldLimitToOwnTeam.value ? 1 : teamPagination.pageNo,
    pageSize: shouldLimitToOwnTeam.value ? 500 : teamPagination.pageSize,
  })),
)

const membersQuery = useMembersQuery(
  computed(() => ({
    keyword: memberFilters.keyword || undefined,
    teamId: shouldLimitToOwnTeam.value ? currentTeamId.value ?? undefined : memberFilters.teamId,
    status: memberFilters.status,
    pageNo: shouldLimitToOwnTeam.value ? 1 : memberPagination.pageNo,
    pageSize: shouldLimitToOwnTeam.value ? 500 : memberPagination.pageSize,
  })),
)

const managerOptionsQuery = useUserOptionsQuery()
const teamOptionsQuery = useTeamOptionsQuery()
const createTeamMutation = useCreateTeamMutation()
const updateTeamMutation = useUpdateTeamMutation()
const createMemberMutation = useCreateMemberMutation()
const updateMemberMutation = useUpdateMemberMutation()
const updateMemberStatusMutation = useUpdateMemberStatusMutation()

const teamDialogVisible = ref(false)
const editingTeam = ref<Team | null>(null)
const teamFormRef = ref<FormInstance>()
const teamForm = reactive({
  teamName: '',
  departmentName: '',
  managerId: undefined as number | undefined,
  description: '',
  status: 'ACTIVE' as TeamStatus,
})

const memberDialogVisible = ref(false)
const editingMember = ref<Member | null>(null)
const memberFormRef = ref<FormInstance>()
const memberForm = reactive({
  name: '',
  teamId: undefined as number | undefined,
  position: '',
  jobLevel: '',
  phone: '',
  email: '',
  entryDate: '',
})

const teamRules = {
  teamName: [{ required: true, message: '请输入团队名称', trigger: 'blur' }],
  departmentName: [{ required: true, message: '请输入部门名称', trigger: 'blur' }],
}

const memberRules = {
  name: [{ required: true, message: '请输入成员姓名', trigger: 'blur' }],
}

const filteredManagerOptions = computed(() => {
  const rows = managerOptionsQuery.data.value?.records ?? []

  if (!shouldLimitToOwnTeam.value) {
    return rows
  }

  return rows.filter((item) => isAccessibleTeamId(item.teamId))
})
const filteredTeamOptions = computed(() => {
  const rows = teamOptionsQuery.data.value?.records ?? []

  if (!shouldLimitToOwnTeam.value) {
    return rows
  }

  return rows.filter((item) => isAccessibleTeamId(item.id))
})
const filteredTeamRows = computed(() => {
  const rows = teamsQuery.data.value?.records ?? []

  if (!shouldLimitToOwnTeam.value) {
    return rows
  }

  return rows.filter((item) => isAccessibleTeamId(item.id))
})
const teamRows = computed(() => {
  if (!shouldLimitToOwnTeam.value) {
    return filteredTeamRows.value
  }

  const start = (teamPagination.pageNo - 1) * teamPagination.pageSize
  return filteredTeamRows.value.slice(start, start + teamPagination.pageSize)
})
const teamTotal = computed(() =>
  shouldLimitToOwnTeam.value ? filteredTeamRows.value.length : (teamsQuery.data.value?.total ?? 0),
)
const filteredMemberRows = computed(() => {
  const rows = membersQuery.data.value?.records ?? []

  if (!shouldLimitToOwnTeam.value) {
    return rows
  }

  return rows.filter((item) => isAccessibleTeamId(item.teamId))
})
const memberRows = computed(() => {
  if (!shouldLimitToOwnTeam.value) {
    return filteredMemberRows.value
  }

  const start = (memberPagination.pageNo - 1) * memberPagination.pageSize
  return filteredMemberRows.value.slice(start, start + memberPagination.pageSize)
})
const memberTotal = computed(() =>
  shouldLimitToOwnTeam.value ? filteredMemberRows.value.length : (membersQuery.data.value?.total ?? 0),
)

function getTeamStatusMeta(status: TeamStatus) {
  return teamStatusMetaMap[status]
}

function getMemberStatusMeta(status: MemberStatus) {
  return memberStatusMetaMap[status]
}

function resetTeamFilters() {
  teamFilters.keyword = ''
  teamFilters.status = undefined
  teamPagination.pageNo = 1
}

function resetMemberFilters() {
  memberFilters.keyword = ''
  memberFilters.teamId = shouldLimitToOwnTeam.value ? currentTeamId.value ?? undefined : undefined
  memberFilters.status = undefined
  memberPagination.pageNo = 1
}

function resetTeamForm() {
  teamForm.teamName = ''
  teamForm.departmentName = ''
  teamForm.managerId = undefined
  teamForm.description = ''
  teamForm.status = 'ACTIVE'
  teamFormRef.value?.clearValidate()
}

function openCreateTeamDialog() {
  if (shouldLimitToOwnTeam.value) {
    ElMessage.error('只能维护本团队信息，不能新增其他团队')
    return
  }

  editingTeam.value = null
  resetTeamForm()
  teamDialogVisible.value = true
}

function openEditTeamDialog(team: Team) {
  if (shouldLimitToOwnTeam.value && !isAccessibleTeamId(team.id)) {
    ElMessage.error('只能维护本团队信息')
    return
  }

  editingTeam.value = team
  teamForm.teamName = team.teamName
  teamForm.departmentName = team.departmentName
  teamForm.managerId = team.managerId ?? undefined
  teamForm.description = team.description ?? ''
  teamForm.status = team.status
  teamDialogVisible.value = true
}

async function submitTeam() {
  const valid = await teamFormRef.value?.validate().catch(() => false)
  if (!valid) {
    return
  }

  if (editingTeam.value) {
    if (shouldLimitToOwnTeam.value && !isAccessibleTeamId(editingTeam.value.id)) {
      ElMessage.error('只能维护本团队信息')
      return
    }

    const payload: UpdateTeamRequest = {
      teamName: teamForm.teamName,
      departmentName: teamForm.departmentName,
      managerId: teamForm.managerId,
      description: teamForm.description || undefined,
      status: teamForm.status,
    }

    await updateTeamMutation.mutateAsync({
      teamId: editingTeam.value.id,
      payload,
    })
    ElMessage.success('团队信息已更新')
  } else {
    if (shouldLimitToOwnTeam.value) {
      ElMessage.error('只能维护本团队信息，不能新增其他团队')
      return
    }

    const payload: CreateTeamRequest = {
      teamName: teamForm.teamName,
      departmentName: teamForm.departmentName,
      managerId: teamForm.managerId,
      description: teamForm.description || undefined,
    }

    await createTeamMutation.mutateAsync(payload)
    ElMessage.success('团队已创建')
  }

  teamDialogVisible.value = false
  resetTeamForm()
}

function resetMemberForm() {
  memberForm.name = ''
  memberForm.teamId = undefined
  memberForm.position = ''
  memberForm.jobLevel = ''
  memberForm.phone = ''
  memberForm.email = ''
  memberForm.entryDate = ''
  memberFormRef.value?.clearValidate()
}

function openCreateMemberDialog() {
  editingMember.value = null
  resetMemberForm()
  if (shouldLimitToOwnTeam.value) {
    memberForm.teamId = currentTeamId.value ?? undefined
  }
  memberDialogVisible.value = true
}

function openEditMemberDialog(member: Member) {
  if (shouldLimitToOwnTeam.value && !isAccessibleTeamId(member.teamId)) {
    ElMessage.error('只能维护本团队成员')
    return
  }

  editingMember.value = member
  memberForm.name = member.name
  memberForm.teamId = member.teamId ?? undefined
  memberForm.position = member.position ?? ''
  memberForm.jobLevel = member.jobLevel ?? ''
  memberForm.phone = ''
  memberForm.email = member.email ?? ''
  memberForm.entryDate = member.entryDate ?? ''
  memberDialogVisible.value = true
}

async function submitMember() {
  const valid = await memberFormRef.value?.validate().catch(() => false)
  if (!valid) {
    return
  }

  if (shouldLimitToOwnTeam.value) {
    memberForm.teamId = currentTeamId.value ?? undefined
  }

  if (editingMember.value) {
    if (shouldLimitToOwnTeam.value && !isAccessibleTeamId(editingMember.value.teamId)) {
      ElMessage.error('只能维护本团队成员')
      return
    }

    const payload: UpdateMemberRequest = {
      name: memberForm.name,
      teamId: memberForm.teamId,
      position: memberForm.position || undefined,
      jobLevel: memberForm.jobLevel || undefined,
      phone: memberForm.phone || undefined,
      email: memberForm.email || undefined,
      entryDate: memberForm.entryDate || undefined,
    }

    await updateMemberMutation.mutateAsync({
      memberId: editingMember.value.id,
      payload,
    })
    ElMessage.success('成员信息已更新')
  } else {
    const payload: CreateMemberRequest = {
      name: memberForm.name,
      teamId: memberForm.teamId,
      position: memberForm.position || undefined,
      jobLevel: memberForm.jobLevel || undefined,
      phone: memberForm.phone || undefined,
      email: memberForm.email || undefined,
      entryDate: memberForm.entryDate || undefined,
    }

    await createMemberMutation.mutateAsync(payload)
    ElMessage.success('成员已创建')
  }

  memberDialogVisible.value = false
  resetMemberForm()
}

async function changeMemberStatus(member: Member, status: MemberStatus) {
  if (shouldLimitToOwnTeam.value && !isAccessibleTeamId(member.teamId)) {
    ElMessage.error('只能维护本团队成员')
    return
  }

  await updateMemberStatusMutation.mutateAsync({
    memberId: member.id,
    payload: {
      status,
    },
  })
  ElMessage.success('成员状态已更新')
}
</script>

<template>
  <PageSection title="团队与成员" description="维护团队组织结构、负责人分配以及成员状态。">
    <el-tabs v-model="activeTab">
      <el-tab-pane label="团队管理" name="teams">
        <div class="page-toolbar">
          <div class="page-toolbar__filters">
            <el-input v-model="teamFilters.keyword" placeholder="搜索团队或部门" clearable style="max-width: 220px" />
            <el-select v-model="teamFilters.status" placeholder="全部状态" clearable style="width: 160px">
              <el-option v-for="item in teamStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
            </el-select>
            <el-button type="primary" @click="teamPagination.pageNo = 1">查询</el-button>
            <el-button @click="resetTeamFilters">重置</el-button>
          </div>
          <div class="page-toolbar__actions">
            <el-button v-if="!shouldLimitToOwnTeam" type="primary" @click="openCreateTeamDialog">新建团队</el-button>
          </div>
        </div>

        <el-table v-loading="teamsQuery.isLoading.value" :data="teamRows">
          <el-table-column prop="teamName" label="团队名称" min-width="180" />
          <el-table-column prop="departmentName" label="所属部门" min-width="180" />
          <el-table-column prop="managerName" label="负责人" min-width="120" />
          <el-table-column prop="memberCount" label="成员数" width="100" />
          <el-table-column label="状态" width="110">
            <template #default="{ row }">
              <StatusTag :meta="getTeamStatusMeta(row.status)" />
            </template>
          </el-table-column>
          <el-table-column label="更新时间" min-width="180">
            <template #default="{ row }">
              {{ formatDateTime(row.updatedAt) }}
            </template>
          </el-table-column>
          <el-table-column label="操作" min-width="120" fixed="right">
            <template #default="{ row }">
              <el-button link type="primary" @click="openEditTeamDialog(row)">编辑</el-button>
            </template>
          </el-table-column>
        </el-table>

        <div class="pagination-bar">
          <el-pagination
            v-model:current-page="teamPagination.pageNo"
            v-model:page-size="teamPagination.pageSize"
            :total="teamTotal"
            :page-sizes="[10, 20, 50]"
            layout="total, sizes, prev, pager, next"
          />
        </div>
      </el-tab-pane>

      <el-tab-pane label="成员管理" name="members">
        <div class="page-toolbar">
          <div class="page-toolbar__filters">
            <el-input v-model="memberFilters.keyword" placeholder="搜索姓名或岗位" clearable style="max-width: 220px" />
            <el-select v-model="memberFilters.teamId" :disabled="shouldLimitToOwnTeam" placeholder="全部团队" clearable style="width: 180px">
              <el-option
                v-for="item in filteredTeamOptions"
                :key="item.id"
                :label="item.teamName"
                :value="item.id"
              />
            </el-select>
            <el-select v-model="memberFilters.status" placeholder="全部状态" clearable style="width: 160px">
              <el-option v-for="item in memberStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
            </el-select>
            <el-button type="primary" @click="memberPagination.pageNo = 1">查询</el-button>
            <el-button @click="resetMemberFilters">重置</el-button>
          </div>
          <div class="page-toolbar__actions">
            <el-button type="primary" @click="openCreateMemberDialog">新建成员</el-button>
          </div>
        </div>

        <el-table v-loading="membersQuery.isLoading.value" :data="memberRows">
          <el-table-column prop="name" label="姓名" min-width="120" />
          <el-table-column prop="teamName" label="团队" min-width="160" />
          <el-table-column prop="position" label="岗位" min-width="140" />
          <el-table-column prop="jobLevel" label="职级" min-width="120" />
          <el-table-column label="入职日期" min-width="140">
            <template #default="{ row }">
              {{ formatDate(row.entryDate) }}
            </template>
          </el-table-column>
          <el-table-column label="状态" width="110">
            <template #default="{ row }">
              <StatusTag :meta="getMemberStatusMeta(row.status)" />
            </template>
          </el-table-column>
          <el-table-column label="操作" min-width="300" fixed="right">
            <template #default="{ row }">
              <div class="table-ops">
                <el-button link type="primary" @click="openEditMemberDialog(row)">编辑</el-button>
                <el-button v-if="row.status !== 'ACTIVE'" link @click="changeMemberStatus(row, 'ACTIVE')">设为在职</el-button>
                <el-button v-if="row.status !== 'SUSPENDED'" link @click="changeMemberStatus(row, 'SUSPENDED')">暂停</el-button>
                <el-button v-if="row.status !== 'LEFT'" link type="danger" @click="changeMemberStatus(row, 'LEFT')">离职</el-button>
              </div>
            </template>
          </el-table-column>
        </el-table>

        <div class="pagination-bar">
          <el-pagination
            v-model:current-page="memberPagination.pageNo"
            v-model:page-size="memberPagination.pageSize"
            :total="memberTotal"
            :page-sizes="[10, 20, 50]"
            layout="total, sizes, prev, pager, next"
          />
        </div>
      </el-tab-pane>
    </el-tabs>
  </PageSection>

  <el-dialog
    v-model="teamDialogVisible"
    :title="editingTeam ? '编辑团队' : '新建团队'"
    width="680px"
    @closed="resetTeamForm"
  >
    <el-form ref="teamFormRef" :model="teamForm" :rules="teamRules" label-position="top">
      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="团队名称" prop="teamName">
            <el-input v-model="teamForm.teamName" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="所属部门" prop="departmentName">
            <el-input v-model="teamForm.departmentName" />
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="负责人">
            <el-select v-model="teamForm.managerId" clearable placeholder="可选">
              <el-option
                v-for="item in filteredManagerOptions"
                :key="item.id"
                :label="item.realName"
                :value="item.id"
              />
            </el-select>
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item v-if="editingTeam" label="状态">
            <el-select v-model="teamForm.status">
              <el-option v-for="item in teamStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
            </el-select>
          </el-form-item>
        </el-col>
      </el-row>

      <el-form-item label="团队描述">
        <el-input v-model="teamForm.description" type="textarea" :rows="4" maxlength="200" show-word-limit />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="teamDialogVisible = false">取消</el-button>
        <el-button type="primary" :loading="createTeamMutation.isPending.value || updateTeamMutation.isPending.value" @click="submitTeam">
          保存
        </el-button>
      </el-space>
    </template>
  </el-dialog>

  <el-dialog
    v-model="memberDialogVisible"
    :title="editingMember ? '编辑成员' : '新建成员'"
    width="760px"
    @closed="resetMemberForm"
  >
    <el-form ref="memberFormRef" :model="memberForm" :rules="memberRules" label-position="top">
      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="姓名" prop="name">
            <el-input v-model="memberForm.name" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="所属团队">
            <el-select v-model="memberForm.teamId" :disabled="shouldLimitToOwnTeam" clearable placeholder="可选">
              <el-option
                v-for="item in filteredTeamOptions"
                :key="item.id"
                :label="item.teamName"
                :value="item.id"
              />
            </el-select>
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="岗位">
            <el-input v-model="memberForm.position" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="职级">
            <el-input v-model="memberForm.jobLevel" />
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="手机号">
            <el-input v-model="memberForm.phone" placeholder="编辑时留空表示不变更" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="邮箱">
            <el-input v-model="memberForm.email" />
          </el-form-item>
        </el-col>
      </el-row>

      <el-form-item label="入职日期">
        <el-date-picker v-model="memberForm.entryDate" type="date" value-format="YYYY-MM-DD" placeholder="选择日期" />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="memberDialogVisible = false">取消</el-button>
        <el-button
          type="primary"
          :loading="createMemberMutation.isPending.value || updateMemberMutation.isPending.value"
          @click="submitMember"
        >
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
