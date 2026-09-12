<script setup lang="ts">
import type { FormInstance } from 'element-plus'
import { ElMessage } from 'element-plus'
import { computed, reactive, ref } from 'vue'

import PageSection from '@/components/common/PageSection.vue'
import StatusTag from '@/components/common/StatusTag.vue'
import { useMemberOptionsQuery, useTeamOptionsQuery } from '@/composables/use-reference-data'
import {
  useCreateUserMutation,
  useRolesQuery,
  useUpdateUserMutation,
  useUpdateUserStatusMutation,
  useUsersQuery,
} from '@/composables/use-user-management'
import { userStatusMetaMap, userStatusOptions } from '@/constants/options'
import { type RoleCode, roleLabelMap, roleOptions } from '@/constants/roles'
import type { CreateUserRequest, UpdateUserRequest, User, UserStatus } from '@/types/api'
import { formatDateTime, formatRoleList } from '@/utils/format'

const filters = reactive({
  keyword: '',
  roleCode: undefined as RoleCode | undefined,
  status: undefined as UserStatus | undefined,
})

const pagination = reactive({
  pageNo: 1,
  pageSize: 10,
})

const queryParams = computed(() => ({
  keyword: filters.keyword || undefined,
  roleCode: filters.roleCode,
  status: filters.status,
  pageNo: pagination.pageNo,
  pageSize: pagination.pageSize,
}))

const usersQuery = useUsersQuery(queryParams)
const rolesQuery = useRolesQuery()
const teamOptionsQuery = useTeamOptionsQuery()
const memberOptionsQuery = useMemberOptionsQuery()
const createUserMutation = useCreateUserMutation()
const updateUserMutation = useUpdateUserMutation()
const updateUserStatusMutation = useUpdateUserStatusMutation()

const dialogVisible = ref(false)
const editingUser = ref<User | null>(null)
const formRef = ref<FormInstance>()

const form = reactive({
  username: '',
  password: '',
  realName: '',
  mobile: '',
  email: '',
  roleCodes: [] as RoleCode[],
  teamId: undefined as number | undefined,
  memberId: undefined as number | undefined,
  mustChangePassword: true,
})

const rules = {
  username: [{ required: true, message: '请输入账号', trigger: 'blur' }],
  password: [{ required: true, message: '请输入初始密码', trigger: 'blur' }],
  realName: [{ required: true, message: '请输入姓名', trigger: 'blur' }],
  roleCodes: [{ required: true, message: '请至少选择一个角色', trigger: 'change' }],
}

const tableData = computed(() => usersQuery.data.value?.records ?? [])
const total = computed(() => usersQuery.data.value?.total ?? 0)
const availableRoles = computed(() => {
  if (rolesQuery.data.value?.length) {
    return rolesQuery.data.value.map((item) => ({
      label: roleLabelMap[item.code],
      value: item.code,
    }))
  }

  return roleOptions
})

function getUserStatusMeta(status: UserStatus) {
  return userStatusMetaMap[status]
}

function handleSearch() {
  pagination.pageNo = 1
}

function resetFilters() {
  filters.keyword = ''
  filters.roleCode = undefined
  filters.status = undefined
  pagination.pageNo = 1
}

function openCreateDialog() {
  editingUser.value = null
  resetForm()
  dialogVisible.value = true
}

function openEditDialog(user: User) {
  editingUser.value = user
  form.username = user.username
  form.password = ''
  form.realName = user.realName
  form.mobile = ''
  form.email = user.email ?? ''
  form.roleCodes = [...user.roleCodes]
  form.teamId = user.teamId ?? undefined
  form.memberId = user.memberId ?? undefined
  form.mustChangePassword = false
  dialogVisible.value = true
}

function resetForm() {
  form.username = ''
  form.password = ''
  form.realName = ''
  form.mobile = ''
  form.email = ''
  form.roleCodes = []
  form.teamId = undefined
  form.memberId = undefined
  form.mustChangePassword = true
  formRef.value?.clearValidate()
}

async function handleSubmit() {
  const valid = await formRef.value?.validate().catch(() => false)
  if (!valid) {
    return
  }

  if (editingUser.value) {
    const payload: UpdateUserRequest = {
      realName: form.realName,
      mobile: form.mobile || undefined,
      email: form.email || undefined,
      roleCodes: [...form.roleCodes],
      teamId: form.teamId,
      memberId: form.memberId,
    }

    await updateUserMutation.mutateAsync({
      userId: editingUser.value.id,
      payload,
    })
    ElMessage.success('用户信息已更新')
  } else {
    const payload: CreateUserRequest = {
      username: form.username,
      password: form.password,
      realName: form.realName,
      mobile: form.mobile || undefined,
      email: form.email || undefined,
      roleCodes: [...form.roleCodes],
      teamId: form.teamId,
      memberId: form.memberId,
      mustChangePassword: form.mustChangePassword,
    }

    await createUserMutation.mutateAsync(payload)
    ElMessage.success('用户已创建')
  }

  dialogVisible.value = false
  resetForm()
}

async function handleChangeStatus(user: User, status: UserStatus) {
  await updateUserStatusMutation.mutateAsync({
    userId: user.id,
    payload: {
      status,
    },
  })
  ElMessage.success('状态更新成功')
}
</script>

<template>
  <PageSection title="用户管理" description="维护系统账号、角色分配、团队归属与启停状态。">
    <div class="page-toolbar">
      <div class="page-toolbar__filters">
        <el-input v-model="filters.keyword" placeholder="搜索账号或姓名" clearable style="max-width: 220px" @keyup.enter="handleSearch" />
        <el-select v-model="filters.roleCode" placeholder="全部角色" clearable style="width: 180px">
          <el-option v-for="item in availableRoles" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
        <el-select v-model="filters.status" placeholder="全部状态" clearable style="width: 160px">
          <el-option v-for="item in userStatusOptions" :key="item.value" :label="item.label" :value="item.value" />
        </el-select>
        <el-button type="primary" @click="handleSearch">查询</el-button>
        <el-button @click="resetFilters">重置</el-button>
      </div>

      <div class="page-toolbar__actions">
        <el-button type="primary" @click="openCreateDialog">新建用户</el-button>
      </div>
    </div>

    <el-table v-loading="usersQuery.isLoading.value" :data="tableData">
      <el-table-column prop="username" label="账号" min-width="140" />
      <el-table-column prop="realName" label="姓名" min-width="120" />
      <el-table-column label="角色" min-width="220">
        <template #default="{ row }">
          {{ formatRoleList(row.roleCodes) }}
        </template>
      </el-table-column>
      <el-table-column label="所属团队" min-width="140">
        <template #default="{ row }">
          {{ row.teamId ? `#${row.teamId}` : '-' }}
        </template>
      </el-table-column>
      <el-table-column prop="email" label="邮箱" min-width="200" />
      <el-table-column label="状态" width="110">
        <template #default="{ row }">
          <StatusTag :meta="getUserStatusMeta(row.status)" />
        </template>
      </el-table-column>
      <el-table-column label="最近登录" min-width="180">
        <template #default="{ row }">
          {{ formatDateTime(row.lastLoginAt) }}
        </template>
      </el-table-column>
      <el-table-column label="操作" min-width="280" fixed="right">
        <template #default="{ row }">
          <div class="table-ops">
            <el-button link type="primary" @click="openEditDialog(row)">编辑</el-button>
            <el-button v-if="row.status !== 'ENABLED'" link @click="handleChangeStatus(row, 'ENABLED')">启用</el-button>
            <el-button v-if="row.status !== 'DISABLED'" link @click="handleChangeStatus(row, 'DISABLED')">禁用</el-button>
            <el-button v-if="row.status !== 'LOCKED'" link type="danger" @click="handleChangeStatus(row, 'LOCKED')">锁定</el-button>
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

  <el-dialog
    v-model="dialogVisible"
    :title="editingUser ? '编辑用户' : '新建用户'"
    width="720px"
    @closed="resetForm"
  >
    <el-form ref="formRef" :model="form" :rules="rules" label-position="top">
      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="账号" prop="username">
            <el-input v-model="form.username" :disabled="Boolean(editingUser)" placeholder="例如 admin" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item v-if="!editingUser" label="初始密码" prop="password">
            <el-input v-model="form.password" type="password" show-password placeholder="至少 6 位" />
          </el-form-item>
          <el-form-item v-else label="密码提示">
            <el-input value="如需重置密码，请由后端或后续功能处理" disabled />
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="姓名" prop="realName">
            <el-input v-model="form.realName" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="角色" prop="roleCodes">
            <el-select v-model="form.roleCodes" multiple placeholder="请选择角色">
              <el-option v-for="item in availableRoles" :key="item.value" :label="item.label" :value="item.value" />
            </el-select>
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="手机号">
            <el-input v-model="form.mobile" placeholder="编辑时如无需变更可留空" />
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="邮箱">
            <el-input v-model="form.email" />
          </el-form-item>
        </el-col>
      </el-row>

      <el-row :gutter="16">
        <el-col :span="12">
          <el-form-item label="所属团队">
            <el-select v-model="form.teamId" clearable placeholder="可选">
              <el-option
                v-for="item in teamOptionsQuery.data.value?.records ?? []"
                :key="item.id"
                :label="item.teamName"
                :value="item.id"
              />
            </el-select>
          </el-form-item>
        </el-col>
        <el-col :span="12">
          <el-form-item label="关联成员">
            <el-select v-model="form.memberId" clearable placeholder="可选">
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

      <el-form-item v-if="!editingUser">
        <el-checkbox v-model="form.mustChangePassword">首次登录强制修改密码</el-checkbox>
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="dialogVisible = false">取消</el-button>
        <el-button
          type="primary"
          :loading="createUserMutation.isPending.value || updateUserMutation.isPending.value"
          @click="handleSubmit"
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
