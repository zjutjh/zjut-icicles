import 'element-plus/dist/index.css'
import './styles/index.scss'

import { VueQueryPlugin } from '@tanstack/vue-query'
import ElementPlus from 'element-plus'
import { createPinia } from 'pinia'
import { createApp } from 'vue'

import App from './App.vue'
import { routerConfig } from './configs/router'
import { globalQueryClient } from './configs/vue-query'

const app = createApp(App)

app.use(createPinia())
app.use(routerConfig)
app.use(VueQueryPlugin, {
  queryClient: globalQueryClient,
})
app.use(ElementPlus)

app.mount('#app')
