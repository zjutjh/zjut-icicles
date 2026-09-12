# WebDevelopmentCourseProject_FrontEnd — AGENTS.md

软件企业技术团队绩效考核管理系统前端，使用 Vue 3 + TypeScript + Vite 开发，是一个课程设计管理后台项目。

---

## 项目概览

| 属性 | 值 |
|------|-----|
| 框架 | Vue 3 + Composition API + Vite |
| 语言 | TypeScript + SCSS |
| UI 组件库 | Element Plus |
| 图标库 | `@element-plus/icons-vue` |
| 图表库 | ECharts |
| 状态管理 | Pinia |
| 路由 | Vue Router |
| 数据请求 | `@tanstack/vue-query` + `src/services/` |
| 包管理 | npm |
| 主题方案 | `scss` 变量包装 `var()`，主题源在 `src/styles/theme.scss` |

---

## 常用命令

| 用途 | 命令 |
|------|------|
| 开发 | `npm run dev` |
| Lint 检查 | `npm run lint` |
| 构建 | `npm run build` |
| 类型检查 | `npm run type-check` |

---

## 目录结构

```text
src/
├── api/            # 对 service 的轻量导出层；不放新的核心请求实现
├── components/     # 全局共享组件
├── composables/    # Vue Query hooks / 复用业务逻辑
├── configs/        # 项目级配置与单例配置
├── constants/      # 常量、枚举、query keys、状态码
├── layouts/        # 布局组件
├── services/       # API service 类（请求主入口）
├── stores/         # Pinia store
├── styles/         # 全局样式与主题 token
├── types/          # 全局 TypeScript 类型
├── utils/          # 工具函数与请求底座
└── views/          # 路由页面
```

---

## 强制约束

### 请求与数据

- 页面、组件里 **禁止直接调用 `axios`**。
- 新请求统一写在 `src/services/`，基于 `BaseService`。
- 页面里的服务端数据请求状态，**必须优先使用 `@tanstack/vue-query`**。
- 新增查询或变更请求时，优先走这条路径：
  `types -> service -> composable(useQuery/useMutation) -> view`
- `src/api/` 当前只作为轻量导出/兼容层，不要把它当成新的主请求层。

### UI 与组件

- **优先使用 Element Plus 组件**，不要随意自造基础组件。
- 只有当 Element Plus 明显不适合时，才新增自定义组件。
- 表单、表格、弹窗、抽屉、分页、按钮、输入框、下拉框等基础交互，默认先选 Element Plus。

### 图标与图表

- **图标统一使用 `@element-plus/icons-vue`**。
- 不要随意再引入新的 icon 库。
- **图表统一使用 ECharts**。
- 不要混用别的图表方案。

### 样式与主题

- 主题颜色统一来源于 `src/styles/theme.scss`。
- 新样式 **优先消费现有 SCSS 变量**，这些变量本质上包装了 CSS `var(...)`。
- 不要在业务代码里随意硬编码新的十六进制颜色。
- 避免内联 `style`，除非是非常局部且明显更合适的动态样式。
- 保持当前暖色浅色主题，不要引入无关的蓝紫色默认视觉。

### 配置与目录

- **只使用 `src/configs/`，不要重新引入 `src/config/`。**
- 项目级配置都放在 `src/configs/`：
  如 `router.ts`、`vue-query.ts`、`app.ts`、`menu.ts`。
- 不要把新路由配置写回旧的 `src/router/` 目录。

---

## 代码规范

### 通用原则

- 使用 **Composition API + `<script setup lang="ts">`**。
- 不使用 Options API。
- 新增逻辑优先保持小文件、单一职责。
- 页面私有逻辑优先提取到 `composables` 或子组件，不要把页面写成巨型单文件。

### 文件职责

- `views/` 只放页面。
- `components/` 放全局复用组件。
- `services/` 放接口服务类。
- `composables/` 放查询、变更和可复用业务逻辑。
- `constants/` 放 query key、角色、状态码、静态映射。
- `configs/` 放项目配置，不放业务请求实现。

### 文案与编码

- 项目里有中文文案。
- PowerShell 有时会把 UTF-8 中文显示成乱码。
- **不要仅因为终端显示乱码，就批量修改中文文案。**
- 修改中文前，先确认是真乱码还是终端显示问题。

---

## 新增功能推荐流程

1. 在 `src/types/` 中补请求/响应类型。
2. 在 `src/services/` 中新增或扩展 `*-service.ts`。
3. 在 `src/constants/query-key.ts` 中补 query key。
4. 在 `src/composables/` 中新增 `useXxxQuery` 或 `useXxxMutation`。
5. 在 `src/views/` 中实现页面。
6. 在 `src/configs/router.ts` 与 `src/configs/menu.ts` 中接入页面。

---

## Lint 与校验

- 每次完成任务后，至少执行：
  - `npm run lint`
  - `npm run build`
- 如果只改了类型相关逻辑，也建议额外看一下 `npm run type-check`。

---

## 明确禁止

- 禁止在页面里直写请求 URL。
- 禁止页面里直调 `axios`。
- 禁止新增第二套路由配置目录。
- 禁止新增第二套主题来源。
- 禁止随意新增新的 UI 库、图标库、图表库。
- 禁止为了赶进度绕开 Vue Query，直接在页面里手搓请求状态管理。
