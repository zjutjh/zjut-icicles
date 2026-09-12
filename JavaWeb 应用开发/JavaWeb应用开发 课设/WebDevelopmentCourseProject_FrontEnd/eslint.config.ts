import zjutjh from '@zjutjh/eslint-config'

export default zjutjh({
  vue: true,
  ts: true,
}, {
  rules: {
    'unicorn/filename-case': 'off',
    '@stylistic/quotes': ['error', 'single'],
    '@stylistic/semi': ['error', 'never'],
    '@stylistic/comma-dangle': ['error', 'always-multiline'],
    'vue/component-name-in-template-casing': 'off',
    'vue/max-attributes-per-line': 'off',
    'vue/singleline-html-element-content-newline': 'off',
    'vue/html-self-closing': 'off',
  },
})
