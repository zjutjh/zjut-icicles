# 技术团队绩效考核管理系统前端

这个仓库是 Web 开发课设《软件企业技术团队绩效考核管理系统》的前端部分。

当前除了管理台页面骨架，也已经把一套可复用的工程基建补齐了，基于你给的 `AI-Interview-Front` 做了适配。现在后续开发可以基本按下面这条链路推进：

`types -> service -> query/composable -> view`

## 当前技术栈

- Vue 3
- TypeScript
- Vite
- Vue Router
- Pinia
- Element Plus
- Axios
- TanStack Vue Query
- ECharts
- Sass
- ESLint + Prettier

## 已补齐的基建

- `src/configs/router.ts`
  路由实例、路由守卫、页面标题、角色控制统一收口。

- `src/configs/vue-query.ts`
  全局 `QueryClient`、重试策略、缓存默认配置。

- `src/utils/base-service.ts`
  通用 `BaseService` 抽象，后续每个业务模块都能按类来写接口。

- `src/utils/service.ts`
  统一 `axios` 实例、超时配置、业务响应码判断、错误抛出。

- `src/utils/request-error.ts`
  统一请求错误对象，方便后面做消息提示、鉴权跳转、全局异常处理。

- `src/services/api-base-url.ts`
  统一解析 `VITE_API_BASE_URL`，开发和生产环境都能稳定工作。

- `src/constants/response-code.ts`
  后端通用业务状态码常量。

- `src/constants/query-key.ts`
  Vue Query 的查询 key 统一管理。

- `src/services/*.ts`
  已提供 `auth-service`、`dashboard-service` 作为示例写法。

- `src/composables/*.ts`
  已提供 `use-dashboard-summary-query` 作为 `service + query` 的接法示例。

- `vite.config.ts`
  已支持 `VITE_API_PROXY_TARGET` 代理配置，便于本地联调。

- `eslint.config.ts`
  已切到和参考项目同一风格的 `@zjutjh/eslint-config` 配置方式。

## 目录建议

```text
src
├─ api                 # 对 service 的轻量导出或兼容层
├─ components          # 通用组件
├─ composables         # Vue Query / 业务复用逻辑
├─ config              # 业务配置
├─ configs             # 工程级配置
├─ constants           # 常量
├─ layouts             # 布局
├─ services            # 业务 service 类
├─ stores              # Pinia 状态
├─ styles              # 全局样式
├─ types               # 通用类型
├─ utils               # 基础工具与请求底座
└─ views               # 页面
```

## 环境变量

参考 [.env.example](/D:/code/project/WebDevelopmentCourseProject_FrontEnd/.env.example)：

```bash
VITE_API_BASE_URL=
VITE_API_PROXY_TARGET=http://127.0.0.1:8080
```

说明：

- `VITE_API_BASE_URL`
  前端请求基础路径，通常留空或设为 `/api`。

- `VITE_API_PROXY_TARGET`
  本地开发时代理到的后端地址。

## 开发约定

新增一个业务模块时，推荐这样落：

1. 在 `src/types` 里定义请求/响应类型。
2. 在 `src/services` 里新增对应的 `xxx-service.ts`。
3. 如果页面需要查询缓存，在 `src/constants/query-key.ts` 补 key，并在 `src/composables` 里写 query hook。
4. 在 `src/views` 里写页面。
5. 在 `src/configs/router.ts` 里挂路由。

## 启动与检查

```bash
npm install
npm run dev
npm run lint
npm run build
```
