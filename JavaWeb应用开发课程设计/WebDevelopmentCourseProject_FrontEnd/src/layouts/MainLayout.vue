<script setup lang="ts">
import { Expand, Fold, Operation } from '@element-plus/icons-vue'
import type { FormInstance } from 'element-plus'
import { ElMessage } from 'element-plus'
import { computed, reactive, ref, watch } from 'vue'
import { useRoute, useRouter } from 'vue-router'

import { useChangePasswordMutation } from '@/composables/use-auth'
import { appTitle } from '@/configs/app'
import { appMenus } from '@/configs/menu'
import { useAuthStore } from '@/stores/auth'
import { iconMap } from '@/utils/icons'

const TEXT = {
  brandTitle: '\u540e\u53f0\u7ba1\u7406\u7cfb\u7edf',
  oldPasswordRequired: '\u8bf7\u8f93\u5165\u539f\u5bc6\u7801',
  newPasswordRequired: '\u8bf7\u8f93\u5165\u65b0\u5bc6\u7801',
  newPasswordLength: '\u65b0\u5bc6\u7801\u81f3\u5c11 6 \u4f4d',
  confirmPasswordRequired: '\u8bf7\u518d\u6b21\u8f93\u5165\u65b0\u5bc6\u7801',
  passwordMismatch: '\u4e24\u6b21\u8f93\u5165\u7684\u65b0\u5bc6\u7801\u4e0d\u4e00\u81f4',
  logoutSuccess: '\u5df2\u9000\u51fa\u767b\u5f55',
  changePasswordSuccess: '\u5bc6\u7801\u4fee\u6539\u6210\u529f\uff0c\u8bf7\u4f7f\u7528\u65b0\u5bc6\u7801\u767b\u5f55',
  defaultAvatar: '\u7528',
  defaultUser: '\u672a\u767b\u5f55\u7528\u6237',
  changePassword: '\u4fee\u6539\u5bc6\u7801',
  logout: '\u9000\u51fa\u767b\u5f55',
  dialogTitle: '\u4fee\u6539\u5bc6\u7801',
  oldPasswordLabel: '\u539f\u5bc6\u7801',
  newPasswordLabel: '\u65b0\u5bc6\u7801',
  confirmPasswordLabel: '\u786e\u8ba4\u65b0\u5bc6\u7801',
  cancel: '\u53d6\u6d88',
  save: '\u4fdd\u5b58',
} as const

const route = useRoute()
const router = useRouter()
const authStore = useAuthStore()
const changePasswordMutation = useChangePasswordMutation()

const activeMenu = computed(() => route.path)
const visibleMenus = computed(() => appMenus.filter((menu) => authStore.hasAnyRole(menu.roles)))
const isSidebarExpanded = ref(true)
const showPasswordDialog = ref(false)
const showMobileMenu = ref(false)
const passwordFormRef = ref<FormInstance>()

const passwordForm = reactive({
  oldPassword: '',
  newPassword: '',
  confirmPassword: '',
})

const passwordRules = {
  oldPassword: [{ required: true, message: TEXT.oldPasswordRequired, trigger: 'blur' }],
  newPassword: [
    { required: true, message: TEXT.newPasswordRequired, trigger: 'blur' },
    { min: 6, message: TEXT.newPasswordLength, trigger: 'blur' },
  ],
  confirmPassword: [
    { required: true, message: TEXT.confirmPasswordRequired, trigger: 'blur' },
    {
      validator: (_rule: unknown, value: string, callback: (error?: Error) => void) => {
        if (value !== passwordForm.newPassword) {
          callback(new Error(TEXT.passwordMismatch))
          return
        }

        callback()
      },
      trigger: 'blur',
    },
  ],
}

async function handleCommand(command: string) {
  if (command === 'logout') {
    await authStore.logout()
    ElMessage.success(TEXT.logoutSuccess)
    await router.push('/login')
    return
  }

  if (command === 'change-password') {
    showPasswordDialog.value = true
  }
}

async function handleChangePassword() {
  const valid = await passwordFormRef.value?.validate().catch(() => false)
  if (!valid) {
    return
  }

  await changePasswordMutation.mutateAsync({
    oldPassword: passwordForm.oldPassword,
    newPassword: passwordForm.newPassword,
  })

  ElMessage.success(TEXT.changePasswordSuccess)
  showPasswordDialog.value = false
  resetPasswordForm()
}

function resetPasswordForm() {
  passwordForm.oldPassword = ''
  passwordForm.newPassword = ''
  passwordForm.confirmPassword = ''
  passwordFormRef.value?.clearValidate()
}

function toggleSidebar() {
  isSidebarExpanded.value = !isSidebarExpanded.value
}

watch(
  () => route.path,
  () => {
    showMobileMenu.value = false
  },
)
</script>

<template>
  <el-container class="app-shell" :style="{ '--sidebar-width': isSidebarExpanded ? '244px' : '84px' }">
    <el-aside
      class="app-shell__aside"
      :class="{ 'app-shell__aside--expanded': isSidebarExpanded }"
      :width="isSidebarExpanded ? '244px' : '84px'"
    >
      <div class="app-shell__brand">
        <div class="app-shell__brand-icon" aria-hidden="true">
          <span class="app-shell__brand-mark-frame" />
          <span class="app-shell__brand-mark-bar app-shell__brand-mark-bar--tall" />
          <span class="app-shell__brand-mark-bar app-shell__brand-mark-bar--short" />
          <span class="app-shell__brand-mark-dot" />
        </div>
        <strong class="app-shell__brand-title">{{ TEXT.brandTitle }}</strong>
      </div>

      <el-menu :default-active="activeMenu" class="app-shell__menu" router>
        <el-menu-item v-for="menu in visibleMenus" :key="menu.key" :index="String(menu.to)">
          <el-icon>
            <component :is="iconMap[menu.icon]" />
          </el-icon>
          <span>{{ menu.label }}</span>
        </el-menu-item>
      </el-menu>

      <div class="app-shell__aside-footer">
        <el-button class="app-shell__collapse-button" circle @click="toggleSidebar">
          <el-icon>
            <component :is="isSidebarExpanded ? Fold : Expand" />
          </el-icon>
        </el-button>
      </div>
    </el-aside>

    <el-container class="app-shell__content">
      <el-header class="app-shell__header">
        <div class="app-shell__heading">
          <el-button class="app-shell__menu-trigger" :icon="Operation" circle @click="showMobileMenu = true" />
          <div>
            <span class="app-shell__eyebrow">Management Console</span>
            <h1>{{ route.meta.title ?? appTitle }}</h1>
          </div>
        </div>

        <el-dropdown placement="bottom-end" @command="handleCommand">
          <div class="app-shell__user">
            <el-avatar class="app-shell__avatar">{{ authStore.currentUser?.realName?.slice(0, 1) ?? TEXT.defaultAvatar }}</el-avatar>
            <strong>{{ authStore.currentUser?.realName ?? TEXT.defaultUser }}</strong>
          </div>
          <template #dropdown>
            <el-dropdown-menu>
              <el-dropdown-item command="change-password">{{ TEXT.changePassword }}</el-dropdown-item>
              <el-dropdown-item command="logout">{{ TEXT.logout }}</el-dropdown-item>
            </el-dropdown-menu>
          </template>
        </el-dropdown>
      </el-header>

      <el-main class="app-shell__main">
        <div class="app-shell__main-inner">
          <router-view />
        </div>
      </el-main>
    </el-container>
  </el-container>

  <el-drawer v-model="showMobileMenu" class="app-shell__drawer" direction="ltr" size="244px" :with-header="false">
    <div class="app-shell__brand app-shell__brand--drawer">
      <div class="app-shell__brand-icon" aria-hidden="true">
        <span class="app-shell__brand-mark-frame" />
        <span class="app-shell__brand-mark-bar app-shell__brand-mark-bar--tall" />
        <span class="app-shell__brand-mark-bar app-shell__brand-mark-bar--short" />
        <span class="app-shell__brand-mark-dot" />
      </div>
      <strong class="app-shell__brand-title">{{ TEXT.brandTitle }}</strong>
    </div>

    <el-menu :default-active="activeMenu" class="app-shell__menu app-shell__menu--drawer" router>
      <el-menu-item v-for="menu in visibleMenus" :key="menu.key" :index="String(menu.to)">
        <el-icon>
          <component :is="iconMap[menu.icon]" />
        </el-icon>
        <span>{{ menu.label }}</span>
      </el-menu-item>
    </el-menu>
  </el-drawer>

  <el-dialog v-model="showPasswordDialog" :title="TEXT.dialogTitle" width="460px" @closed="resetPasswordForm">
    <el-form ref="passwordFormRef" :model="passwordForm" :rules="passwordRules" label-position="top">
      <el-form-item :label="TEXT.oldPasswordLabel" prop="oldPassword">
        <el-input v-model="passwordForm.oldPassword" type="password" show-password />
      </el-form-item>
      <el-form-item :label="TEXT.newPasswordLabel" prop="newPassword">
        <el-input v-model="passwordForm.newPassword" type="password" show-password />
      </el-form-item>
      <el-form-item :label="TEXT.confirmPasswordLabel" prop="confirmPassword">
        <el-input v-model="passwordForm.confirmPassword" type="password" show-password />
      </el-form-item>
    </el-form>

    <template #footer>
      <el-space>
        <el-button @click="showPasswordDialog = false">{{ TEXT.cancel }}</el-button>
        <el-button type="primary" :loading="changePasswordMutation.isPending.value" @click="handleChangePassword">
          {{ TEXT.save }}
        </el-button>
      </el-space>
    </template>
  </el-dialog>
</template>

<style scoped lang="scss">
.app-shell {
  min-height: 100vh;
  background: #f7f8fa;
}

.app-shell__aside {
  position: fixed;
  top: 0;
  left: 0;
  bottom: 0;
  z-index: 20;
  display: flex;
  flex-direction: column;
  width: var(--sidebar-width) !important;
  height: 100vh;
  border-right: 1px solid var(--theme-border);
  background: #ffffff;
  transition: width 0.22s ease;
  overflow: hidden;
}

.app-shell__aside:not(.app-shell__aside--expanded) {
  width: var(--sidebar-width) !important;
}

.app-shell__brand {
  display: flex;
  gap: 0;
  align-items: center;
  justify-content: flex-start;
  min-height: 92px;
  padding: 22px 20px 18px;
}

.app-shell__brand-title {
  display: block;
  color: var(--theme-foreground);
  font-size: 17px;
  font-weight: 600;
  line-height: 1;
  white-space: nowrap;
  opacity: 1;
  transition: opacity 0.16s ease;
}

.app-shell__aside:not(.app-shell__aside--expanded) .app-shell__brand {
  justify-content: center;
  min-height: 76px;
  padding: 18px 0 12px;
}

.app-shell__aside:not(.app-shell__aside--expanded) .app-shell__brand-title {
  opacity: 0;
  width: 0;
  overflow: hidden;
  pointer-events: none;
}

.app-shell__brand-icon {
  position: relative;
  display: flex;
  flex-shrink: 0;
  width: 28px;
  height: 28px;
  margin-right: 12px;
  opacity: 0;
  transform: scale(0.92);
  transition:
    opacity 0.16s ease,
    transform 0.16s ease,
    margin 0.16s ease;
}

.app-shell__aside:not(.app-shell__aside--expanded) .app-shell__brand-icon {
  width: 24px;
  height: 24px;
  margin-right: 0;
  opacity: 1;
  transform: scale(1);
}

.app-shell__brand-mark-frame {
  position: absolute;
  inset: 1px;
  border: 1.5px solid rgba(75, 101, 132, 0.2);
  border-radius: 9px;
  background: linear-gradient(180deg, rgba(255, 255, 255, 0.96), rgba(243, 247, 251, 0.96));
}

.app-shell__brand-mark-bar,
.app-shell__brand-mark-dot {
  position: absolute;
  background: var(--theme-accent);
}

.app-shell__brand-mark-bar {
  bottom: 6px;
  left: 7px;
  width: 4px;
  border-radius: 999px;
  opacity: 0.96;
}

.app-shell__brand-mark-bar--tall {
  height: 14px;
}

.app-shell__brand-mark-bar--short {
  left: 14px;
  width: 7px;
  height: 4px;
  bottom: 9px;
}

.app-shell__brand-mark-dot {
  top: 7px;
  right: 7px;
  width: 5px;
  height: 5px;
  border-radius: 50%;
  opacity: 0.72;
}

.app-shell__menu {
  display: flex;
  flex: 1 1 auto;
  flex-direction: column;
  align-items: stretch;
  justify-content: flex-start;
  border-right: none;
  background: transparent;
  padding: 10px 0;
  overflow: hidden;
}

.app-shell__menu :deep(.el-menu-item) {
  position: relative;
  display: flex;
  gap: 10px;
  flex: 0 0 44px;
  height: 44px;
  margin: 4px 12px;
  align-items: center;
  color: var(--theme-foreground-secondary);
  font-size: 14px;
}

.app-shell__menu :deep(.el-menu-item .el-icon) {
  color: #8f9baa;
  font-size: 16px;
}

.app-shell__aside:not(.app-shell__aside--expanded) .app-shell__menu {
  padding-top: 6px;
}

.app-shell__aside:not(.app-shell__aside--expanded) .app-shell__menu :deep(.el-menu-item) {
  justify-content: center;
  gap: 0;
  flex-basis: 40px;
  height: 40px;
  margin-left: 10px;
  margin-right: 10px;
  padding: 0 !important;
}

.app-shell__aside:not(.app-shell__aside--expanded) .app-shell__menu :deep(.el-menu-item .el-icon) {
  margin-right: 0;
  font-size: 17px;
}

.app-shell__aside:not(.app-shell__aside--expanded) .app-shell__menu :deep(.el-menu-item span) {
  width: 0;
  overflow: hidden;
  opacity: 0;
}

.app-shell__aside:not(.app-shell__aside--expanded) .app-shell__menu :deep(.el-menu-item.is-active::before) {
  top: 8px;
  height: 22px;
}

.app-shell__menu :deep(.el-menu-item:hover) {
  color: var(--theme-accent);
  background: #f7fafd;
}

.app-shell__menu :deep(.el-menu-item.is-active) {
  color: #3f5e7a;
  background: #eef3f8;
}

.app-shell__menu :deep(.el-menu-item.is-active .el-icon) {
  color: #3f5e7a;
}

.app-shell__menu :deep(.el-menu-item.is-active::before) {
  position: absolute;
  top: 10px;
  left: 0;
  width: 3px;
  height: 24px;
  border-radius: 0 999px 999px 0;
  background: var(--theme-accent);
  content: '';
}

.app-shell__aside-footer {
  display: flex;
  justify-content: center;
  padding: 10px 0 18px;
}

.app-shell__collapse-button {
  width: 36px;
  height: 36px;
  border-color: var(--theme-border);
  color: var(--theme-accent);
  background: #ffffff;
  box-shadow: none;
}

.app-shell__collapse-button:hover {
  border-color: #c9d5e1;
  color: #3f5e7a;
  background: #f7fafd;
}

.app-shell__content {
  margin-left: var(--sidebar-width);
  min-height: 100vh;
  background:
    radial-gradient(circle at top center, rgba(75, 101, 132, 0.04), transparent 28%),
    linear-gradient(180deg, #f7f8fa 0%, #f5f7fa 100%);
  transition: margin-left 0.22s ease;
}

.app-shell__header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 16px;
  height: 88px;
  padding: 18px 24px 10px;
  background: transparent;
}

.app-shell__heading {
  display: flex;
  gap: 12px;
  align-items: flex-start;
}

.app-shell__eyebrow {
  display: inline-block;
  margin-bottom: 6px;
  color: var(--theme-foreground-tertiary);
  font-size: 12px;
  letter-spacing: 0.08em;
  text-transform: uppercase;
}

.app-shell__header h1 {
  margin: 0;
  font-size: 24px;
  font-weight: 600;
  color: var(--theme-foreground);
}

.app-shell__menu-trigger {
  display: none;
  width: 38px;
  height: 38px;
  border-color: var(--theme-border);
  color: var(--theme-accent);
  background: #ffffff;
  box-shadow: none;
}

.app-shell__user {
  display: flex;
  min-width: 0;
  gap: 10px;
  align-items: center;
  padding: 10px 14px;
  border: 1px solid var(--theme-border);
  border-radius: 999px;
  background: #ffffff;
  box-shadow: var(--theme-shadow-soft);
  cursor: pointer;
}

.app-shell__avatar {
  color: #ffffff;
  background: var(--theme-accent);
}

.app-shell__user strong {
  color: var(--theme-foreground);
  font-size: 14px;
  font-weight: 600;
  line-height: 1;
}

.app-shell__main {
  padding: 6px 24px 24px;
}

.app-shell__main-inner {
  min-height: calc(100vh - 112px);
}

.app-shell__drawer :deep(.el-drawer) {
  background: #ffffff;
}

.app-shell__brand--drawer {
  padding-top: 8px;
}

.app-shell__menu--drawer {
  padding-top: 4px;
}

@media (max-width: 768px) {
  .app-shell__aside {
    display: none;
  }

  .app-shell__content {
    margin-left: 0;
  }

  .app-shell__header {
    align-items: flex-start;
    height: auto;
    padding: 16px 16px 8px;
  }

  .app-shell__heading {
    width: 100%;
  }

  .app-shell__menu-trigger {
    display: inline-flex;
    flex-shrink: 0;
  }

  .app-shell__header h1 {
    font-size: 22px;
  }

  .app-shell__main {
    padding: 8px 16px 16px;
  }

  .app-shell__main-inner {
    min-height: auto;
  }

  .app-shell__user {
    align-self: stretch;
  }
}
</style>
