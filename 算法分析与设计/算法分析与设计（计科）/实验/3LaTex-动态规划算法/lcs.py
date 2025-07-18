def lcs_dp(s1, s2):
    m, n = len(s1), len(s2)
    
    # 创建dp表，dp[i][j]表示s1[0:i]和s2[0:j]的最长公共子序列长度
    dp = [[0] * (n + 1) for _ in range(m + 1)]
    
    # 填充dp表
    for i in range(1, m + 1):
        for j in range(1, n + 1):
            if s1[i - 1] == s2[j - 1]:
                dp[i][j] = dp[i - 1][j - 1] + 1
            else:
                dp[i][j] = max(dp[i - 1][j], dp[i][j - 1])
    
    # 打印dp表
    print('dp表:')
    print('     ', end='')
    print('  '.join([''] + list(s2)))
    for i in range(m + 1):
        if i == 0:
            print('  ', end='')
        else:
            print(s1[i-1], end=' ')
        for j in range(n + 1):
            print(dp[i][j], end='  ')
        print()
    
    # 回溯构建LCS
    lcs = []
    i, j = m, n
    while i > 0 and j > 0:
        if s1[i - 1] == s2[j - 1]:
            lcs.append(s1[i - 1])
            i -= 1
            j -= 1
        elif dp[i - 1][j] > dp[i][j - 1]:
            i -= 1
        else:
            j -= 1
    
    return ''.join(reversed(lcs))

if __name__ == "__main__":
    s1 = 'AGGTAB'
    s2 = 'GXTXAYB'
    lcs = lcs_dp(s1, s2)
    print(f'\n最长公共子序列: {lcs}')
    print(f'长度: {len(lcs)}') 