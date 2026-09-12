<script setup lang="ts">
import { DataAnalysis, Lock, Opportunity, User } from '@element-plus/icons-vue'
import type { FormInstance } from 'element-plus'
import { ElMessage } from 'element-plus'
import { reactive, ref } from 'vue'
import { useRouter } from 'vue-router'

import { useLoginMutation } from '@/composables/use-auth'
import { appTitle } from '@/configs/app'
import { useAuthStore } from '@/stores/auth'

const router = useRouter()
const authStore = useAuthStore()
const loginMutation = useLoginMutation()
const formRef = ref<FormInstance>()

const form = reactive({
  username: '',
  password: '',
})

const rules = {
  username: [{ required: true, message: '请输入账号', trigger: 'blur' }],
  password: [{ required: true, message: '请输入密码', trigger: 'blur' }],
}

async function handleLogin() {
  const valid = await formRef.value?.validate().catch(() => false)
  if (!valid) {
    return
  }

  try {
    const result = await loginMutation.mutateAsync({
      username: form.username,
      password: form.password,
    })

    ElMessage.success(`欢迎回来，${result.user.realName}`)
    if (result.user.mustChangePassword) {
      ElMessage.warning('当前账号需要尽快修改初始密码')
    }

    await router.push(authStore.getDefaultRoute())
  } catch {
    ElMessage.error('账号不存在或密码错误，请重新输入！')
  }
}
</script>

<template>
  <div class="login-page">
    <div class="login-page__wrap">
      <section class="login-page__intro" aria-label="系统简介">
        <p class="login-page__eyebrow">MANAGEMENT SYSTEM</p>
        <h1>{{ appTitle }}</h1>
        <p class="login-page__description">科学绩效管理，高效团队协作，助力企业人才发展。</p>

        <div class="login-page__features">
          <div class="login-page__feature">
            <el-icon><Opportunity /></el-icon>
            <div>
              <strong>数据安全</strong>
              <span>多重安全保障</span>
            </div>
          </div>
          <div class="login-page__feature">
            <el-icon><DataAnalysis /></el-icon>
            <div>
              <strong>高效管理</strong>
              <span>提升管理效率</span>
            </div>
          </div>
          <div class="login-page__feature">
            <el-icon><User /></el-icon>
            <div>
              <strong>团队协作</strong>
              <span>促进团队成长</span>
            </div>
          </div>
        </div>
      </section>

      <el-card class="login-page__card" shadow="never">
        <div class="login-page__card-header">
          <strong>系统登录</strong>
          <p>请输入账号和密码登录系统</p>
        </div>

        <el-form ref="formRef" :model="form" :rules="rules" label-position="top" @submit.prevent="handleLogin">
          <el-form-item label="账号" prop="username">
            <el-input v-model="form.username" :prefix-icon="User" placeholder="请输入账号" />
          </el-form-item>
          <el-form-item label="密码" prop="password">
            <el-input v-model="form.password" :prefix-icon="Lock" type="password" show-password placeholder="请输入密码" />
          </el-form-item>
          <el-button native-type="submit" type="primary" class="login-page__submit" :loading="loginMutation.isPending.value">
            登录
          </el-button>
        </el-form>
      </el-card>
    </div>

    <p class="login-page__copyright">© 2024 软件企业技术团队绩效考核管理系统</p>
  </div>
</template>

<style scoped lang="scss">
.login-page {
  position: relative;
  overflow: hidden;
  display: flex;
  flex-direction: column;
  min-height: 100vh;
  box-sizing: border-box;
  padding: 24px 24px 20px;
  background:
    radial-gradient(circle at 52% 10%, rgba(75, 101, 132, 0.08), transparent 32%),
    radial-gradient(circle at 16% 22%, rgba(75, 101, 132, 0.05), transparent 24%),
    linear-gradient(132deg, transparent 0 68%, rgba(75, 101, 132, 0.035) 68%, transparent 68.5%),
    linear-gradient(180deg, #f7f9fc 0%, #f5f7fa 100%);
  color: #2f3542;
}

.login-page::before,
.login-page::after {
  position: absolute;
  content: '';
  pointer-events: none;
}

.login-page::before {
  left: -10%;
  bottom: -10%;
  width: 60vw;
  min-width: 720px;
  height: 34vw;
  min-height: 380px;
  background:
    radial-gradient(96% 100% at 8% 100%, rgba(75, 101, 132, 0.045) 0 28%, transparent 28.8%),
    radial-gradient(88% 92% at 14% 100%, rgba(75, 101, 132, 0.09) 0 22%, transparent 22.8%),
    radial-gradient(76% 84% at 18% 100%, rgba(75, 101, 132, 0.06) 0 16%, transparent 16.8%),
    radial-gradient(64% 72% at 24% 100%, rgba(255, 255, 255, 0.86) 0 12%, transparent 12.8%),
    linear-gradient(24deg, rgba(75, 101, 132, 0.03) 0 1px, transparent 1px 100%);
  opacity: 0.85;
}

.login-page::after {
  display: none;
}

.login-page__wrap {
  display: grid;
  grid-template-columns: minmax(360px, 520px) minmax(420px, 560px);
  gap: 72px;
  align-items: center;
  justify-content: center;
  flex: 1;
  min-height: 0;
  position: relative;
  z-index: 1;
}

.login-page__wrap::before {
  position: absolute;
  left: -120px;
  bottom: 24px;
  width: 86px;
  height: 68px;
  content: '';
  background-image: radial-gradient(circle, rgba(75, 101, 132, 0.08) 1.15px, transparent 1.2px);
  background-size: 14px 14px;
  opacity: 0.7;
  pointer-events: none;
}

.login-page__wrap::after {
  position: absolute;
  top: 5%;
  right: 2%;
  width: 280px;
  height: 280px;
  content: '';
  border: 1px solid rgba(75, 101, 132, 0.08);
  border-radius: 50%;
  background:
    radial-gradient(circle at 68% 32%, rgba(75, 101, 132, 0.04) 0 2px, transparent 2.4px),
    linear-gradient(135deg, transparent 0 74%, rgba(75, 101, 132, 0.035) 74%, transparent 74.8%);
  box-shadow:
    -34px 34px 0 -33px rgba(75, 101, 132, 0.07),
    -68px 68px 0 -67px rgba(75, 101, 132, 0.05),
    -102px 102px 0 -101px rgba(75, 101, 132, 0.035);
  opacity: 0.8;
  pointer-events: none;
}

.login-page__intro {
  max-width: 520px;
}

.login-page__eyebrow {
  margin: 0 0 18px;
  color: #4b6584;
  font-size: 12px;
  font-weight: 600;
  letter-spacing: 0.12em;
}

.login-page__intro h1 {
  max-width: 420px;
  margin: 0 0 22px;
  padding-left: 24px;
  border-left: 5px solid #4b6584;
  color: #2f3542;
  font-size: 38px;
  font-weight: 700;
  line-height: 1.45;
}

.login-page__description {
  margin: 0 0 36px;
  color: #7b8794;
  font-size: 15px;
  line-height: 1.8;
}

.login-page__features {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 20px;
}

.login-page__feature {
  display: flex;
  gap: 12px;
  align-items: flex-start;
  min-width: 0;
}

.login-page__feature + .login-page__feature {
  padding-left: 20px;
  border-left: 1px solid #e5eaf0;
}

.login-page__feature .el-icon {
  margin-top: 2px;
  color: #4b6584;
  font-size: 22px;
}

.login-page__feature strong {
  display: block;
  margin-bottom: 4px;
  color: #2f3542;
  font-size: 16px;
  font-weight: 600;
}

.login-page__feature span {
  color: #7b8794;
  font-size: 13px;
}

.login-page__card {
  width: min(100%, 560px);
  border: 1px solid #e5eaf0;
  border-radius: 16px;
  background: #ffffff;
  box-shadow: 0 16px 48px rgba(47, 53, 66, 0.06);
}

.login-page__card-header {
  margin-bottom: 24px;
  text-align: center;
}

.login-page__card-header strong {
  display: block;
  margin-bottom: 14px;
  color: #2f3542;
  font-size: 28px;
  font-weight: 700;
}

.login-page__card-header p {
  margin: 0;
  color: #7b8794;
  font-size: 15px;
}

:deep(.el-card__body) {
  padding: 40px 40px 32px;
}

:deep(.el-form-item) {
  margin-bottom: 22px;
}

:deep(.el-form-item__label) {
  color: #2f3542;
  font-weight: 500;
  font-size: 15px;
  margin-bottom: 8px;
}

:deep(.el-input__wrapper) {
  min-height: 60px;
  padding-inline: 16px;
  border: 1px solid #e5eaf0;
  border-radius: 10px;
  box-shadow: none;
  background: #ffffff;
}

:deep(.el-input__wrapper.is-focus) {
  border-color: #4b6584;
  box-shadow: 0 0 0 3px rgba(75, 101, 132, 0.08);
}

:deep(.el-input__prefix-inner) {
  color: #a7b0bc;
  font-size: 18px;
}

:deep(.el-input__inner) {
  color: #2f3542;
  font-size: 16px;
}

:deep(.el-input__inner::placeholder) {
  color: #b0b8c4;
}

:deep(.el-input__suffix-inner) {
  color: #a7b0bc;
}

.login-page__submit {
  width: 100%;
  height: 52px;
  border-color: #4b6584;
  border-radius: 8px;
  background: #4b6584;
  color: #ffffff;
  font-size: 16px;
  font-weight: 600;
}

.login-page__submit:hover,
.login-page__submit:focus {
  border-color: #425a76;
  background: #425a76;
}

.login-page__copyright {
  position: relative;
  z-index: 1;
  margin: 0;
  color: #8d97a6;
  font-size: 13px;
  line-height: 1.4;
  text-align: center;
}

@media (max-width: 1440px) {
  .login-page__wrap {
    gap: 40px;
  }
}

@media (max-width: 1100px) {
  .login-page__wrap {
    grid-template-columns: minmax(0, 540px);
    gap: 28px;
  }

  .login-page__intro {
    max-width: none;
  }

  .login-page__intro h1 {
    max-width: none;
  }

  .login-page__wrap::after {
    top: auto;
    right: -18px;
    bottom: -36px;
    width: 220px;
    height: 220px;
  }
}

@media (max-width: 900px) {
  .login-page__features {
    grid-template-columns: 1fr;
    gap: 16px;
  }

  .login-page__feature + .login-page__feature {
    padding-left: 0;
    border-left: none;
  }
}

@media (max-width: 768px) {
  .login-page {
    padding: 16px 16px 20px;
  }

  .login-page__wrap {
    align-content: center;
  }

  .login-page__intro h1 {
    padding-left: 16px;
    font-size: 28px;
  }

  .login-page__description {
    margin-bottom: 24px;
  }

  .login-page__card {
    width: 100%;
  }

  :deep(.el-card__body) {
    padding: 28px 20px 24px;
  }

  :deep(.el-input__wrapper) {
    min-height: 52px;
  }

  .login-page__submit {
    height: 50px;
  }

  .login-page::before {
    width: 82vw;
    min-width: 0;
    height: 48vw;
    min-height: 0;
  }

  .login-page::after {
    display: none;
  }

  .login-page__wrap::before {
    left: -36px;
    bottom: 8px;
    width: 72px;
    height: 56px;
  }

  .login-page__wrap::after {
    display: none;
  }
}
</style>
