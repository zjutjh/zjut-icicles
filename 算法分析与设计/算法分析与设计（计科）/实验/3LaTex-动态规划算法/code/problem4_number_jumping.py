"""
问题4：数字跳棋游戏（可选题）
找出递增的数字序列，使序列长度最大
本题实际上是求解最长递增子序列(LIS - Longest Increasing Subsequence)问题
"""

def lis_brute_force(nums, curr_idx=0, prev_val=float('-inf'), curr_sequence=None):
    """
    暴力穷举法求解最长递增子序列
    时间复杂度：O(2^n)，其中n是数组长度
    """
    if curr_sequence is None:
        curr_sequence = []
    
    # 基本情况：到达数组末尾
    if curr_idx == len(nums):
        return curr_sequence
    
    # 选择1：跳过当前数字
    skip_result = lis_brute_force(nums, curr_idx + 1, prev_val, curr_sequence.copy())
    
    # 选择2：如果当前数字大于前一个选择的数字，则选择当前数字
    take_result = []
    if nums[curr_idx] > prev_val:
        new_sequence = curr_sequence.copy()
        new_sequence.append(nums[curr_idx])
        take_result = lis_brute_force(nums, curr_idx + 1, nums[curr_idx], new_sequence)
    
    # 返回长度更大的序列
    if len(take_result) > len(skip_result):
        return take_result
    else:
        return skip_result

def lis_dp(nums):
    """
    动态规划法求解最长递增子序列
    时间复杂度：O(n^2)，其中n是数组长度
    空间复杂度：O(n)
    """
    if not nums:
        return []
    
    n = len(nums)
    # dp[i]表示以第i个元素结尾的最长递增子序列的长度
    dp = [1] * n
    # prev[i]记录以第i个元素结尾的最长递增子序列的前一个元素的索引
    prev = [-1] * n
    
    # 填充dp数组
    for i in range(1, n):
        for j in range(i):
            if nums[i] > nums[j] and dp[i] < dp[j] + 1:
                dp[i] = dp[j] + 1
                prev[i] = j
    
    # 找到最长递增子序列的结束位置
    max_length = max(dp)
    end_index = dp.index(max_length)
    
    # 重建最长递增子序列
    lis = []
    while end_index != -1:
        lis.append(nums[end_index])
        end_index = prev[end_index]
    
    # 逆序返回，得到正确的顺序
    return list(reversed(lis))

def lis_dp_optimized(nums):
    """
    优化的动态规划法求解最长递增子序列
    时间复杂度：O(n log n)，其中n是数组长度
    空间复杂度：O(n)
    """
    if not nums:
        return []
    
    n = len(nums)
    # piles[i]存储牌堆顶部的数字
    piles = []
    # 记录每个元素在哪个牌堆
    pile_idx = [-1] * n
    
    for i, num in enumerate(nums):
        # 二分查找：查找第一个大于等于num的牌堆
        left, right = 0, len(piles)
        while left < right:
            mid = (left + right) // 2
            if piles[mid] < num:
                left = mid + 1
            else:
                right = mid
                
        # 如果找不到牌堆，则新建一个
        if left == len(piles):
            piles.append(num)
        else:
            piles[left] = num
            
        pile_idx[i] = left
    
    # 牌堆数量就是最长递增子序列的长度
    max_length = len(piles)
    
    # 重建最长递增子序列（这种方法只能重建一种可能的LIS，不一定是原始顺序）
    lis = []
    curr_pile = max_length - 1
    for i in range(n - 1, -1, -1):
        if pile_idx[i] == curr_pile:
            lis.append(nums[i])
            curr_pile -= 1
            if curr_pile < 0:
                break
    
    return list(reversed(lis))

def print_dp_table(nums, dp):
    """打印动态规划表"""
    print("棋子值:  ", end="")
    for num in nums:
        print(f"{num:<4}", end="")
    print()
    
    print("LIS长度:", end="")
    for d in dp:
        print(f"{d:<4}", end="")
    print()

def main():
    # 测试用例
    test_case = [10, 22, 9, 33, 21, 50, 41, 60, 80]
    
    # 使用动态规划求解
    n = len(test_case)
    dp = [1] * n
    prev = [-1] * n
    
    for i in range(1, n):
        for j in range(i):
            if test_case[i] > test_case[j] and dp[i] < dp[j] + 1:
                dp[i] = dp[j] + 1
                prev[i] = j
    
    # 找到最长递增子序列的结束位置
    max_length = max(dp)
    end_index = dp.index(max_length)
    
    # 重建最长递增子序列
    lis = []
    while end_index != -1:
        lis.append(test_case[end_index])
        end_index = prev[end_index]
    
    # 逆序得到正确的顺序
    lis = list(reversed(lis))
    
    print("测试用例:")
    print(f"棋子: {test_case}")
    print(f"最长递增子序列: {lis}")
    print(f"得分: {len(lis)}")
    print_dp_table(test_case, dp)

if __name__ == "__main__":
    main() 