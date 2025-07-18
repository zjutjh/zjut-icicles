"""
问题2：基因序列比对
查找两个基因序列的最长公共子序列(LCS)
"""

def lcs_brute_force(s1, s2, i=0, j=0, current=""):
    """
    暴力穷举法求最长公共子序列
    通过递归遍历所有可能的子序列
    时间复杂度：O(2^(m+n))，其中m和n分别是两个序列的长度
    """
    # 如果任一序列达到末尾，返回当前构建的子序列
    if i == len(s1) or j == len(s2):
        return current
    
    # 如果当前字符相同，则将其加入子序列
    if s1[i] == s2[j]:
        return lcs_brute_force(s1, s2, i + 1, j + 1, current + s1[i])
    
    # 否则，尝试跳过s1或s2中的当前字符，取较长的结果
    skip_s1 = lcs_brute_force(s1, s2, i + 1, j, current)
    skip_s2 = lcs_brute_force(s1, s2, i, j + 1, current)
    
    if len(skip_s1) > len(skip_s2):
        return skip_s1
    else:
        return skip_s2

def lcs_dp(s1, s2):
    """
    动态规划法求最长公共子序列
    时间复杂度：O(m*n)，其中m和n分别是两个序列的长度
    空间复杂度：O(m*n)
    """
    m, n = len(s1), len(s2)
    
    # 创建dp表，dp[i][j]表示s1[0:i]和s2[0:j]的最长公共子序列长度
    dp = [[0] * (n + 1) for _ in range(m + 1)]
    
    # 填充dp表
    for i in range(1, m + 1):
        for j in range(1, n + 1):
            if s1[i - 1] == s2[j - 1]:
                # 如果当前字符相同，则最长公共子序列长度加1
                dp[i][j] = dp[i - 1][j - 1] + 1
            else:
                # 否则，取跳过s1或s2当前字符后的较大值
                dp[i][j] = max(dp[i - 1][j], dp[i][j - 1])
    
    # 构建最长公共子序列
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
    
    # 返回最长公共子序列（需要反转）
    return ''.join(reversed(lcs)), dp

def print_dp_table(s1, s2, dp):
    """打印动态规划表"""
    m, n = len(s1), len(s2)
    print("动态规划表:")
    
    # 打印表头
    print("   ", end="")
    print("   ", end="")
    for j in range(n):
        print(f" {s2[j]} ", end="")
    print()
    
    # 打印表内容
    for i in range(m + 1):
        if i == 0:
            print("   ", end="")
        else:
            print(f" {s1[i-1]} ", end="")
        
        for j in range(n + 1):
            print(f" {dp[i][j]} ", end="")
        print()

def main():
    # 测试用例1
    s1 = "AGGTAC"
    s2 = "GUTUAYC"
    lcs, dp = lcs_dp(s1, s2)
    print(f"序列1: {s1}")
    print(f"序列2: {s2}")
    print(f"最长公共子序列: {lcs}")
    print(f"长度: {len(lcs)}")
    print_dp_table(s1, s2, dp)
    
    # 测试用例2
    s1 = "AGGTAB"
    s2 = "GXTXAYB"
    lcs, dp = lcs_dp(s1, s2)
    print(f"\n序列1: {s1}")
    print(f"序列2: {s2}")
    print(f"最长公共子序列: {lcs}")
    print(f"长度: {len(lcs)}")
    print_dp_table(s1, s2, dp)

if __name__ == "__main__":
    main() 