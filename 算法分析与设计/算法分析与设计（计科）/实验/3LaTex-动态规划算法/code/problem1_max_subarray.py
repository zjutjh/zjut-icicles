"""
问题1：选择数字游戏
在一系列整数中选择相邻的几个数字，使得它们的和最大
"""

def max_subarray_brute_force(arr):
    """
    暴力穷举法求解最大子数组和
    时间复杂度：O(n^2)，其中n是数组长度
    """
    n = len(arr)
    max_sum = float('-inf')
    result_start = result_end = -1
    
    # 枚举所有可能的子数组
    for i in range(n):
        current_sum = 0
        for j in range(i, n):
            current_sum += arr[j]
            if current_sum > max_sum:
                max_sum = current_sum
                result_start = i
                result_end = j
    
    return max_sum, result_start, result_end

def max_subarray_divide_conquer(arr, low, high):
    """
    分治法求解最大子数组和
    时间复杂度：O(n log n)，其中n是数组长度
    """
    # 基本情况：只有一个元素
    if low == high:
        return arr[low], low, high
    
    # 分解问题
    mid = (low + high) // 2
    
    # 递归求解左半部分
    left_max_sum, left_start, left_end = max_subarray_divide_conquer(arr, low, mid)
    
    # 递归求解右半部分
    right_max_sum, right_start, right_end = max_subarray_divide_conquer(arr, mid + 1, high)
    
    # 求解跨越中点的最大子数组
    cross_max_sum, cross_start, cross_end = max_crossing_subarray(arr, low, mid, high)
    
    # 比较三种情况，返回最大值
    if left_max_sum >= right_max_sum and left_max_sum >= cross_max_sum:
        return left_max_sum, left_start, left_end
    elif right_max_sum >= left_max_sum and right_max_sum >= cross_max_sum:
        return right_max_sum, right_start, right_end
    else:
        return cross_max_sum, cross_start, cross_end

def max_crossing_subarray(arr, low, mid, high):
    """计算跨越中点的最大子数组"""
    # 计算左半部分的最大和（从中点向左）
    left_sum = float('-inf')
    current_sum = 0
    max_left = mid
    
    for i in range(mid, low - 1, -1):
        current_sum += arr[i]
        if current_sum > left_sum:
            left_sum = current_sum
            max_left = i
    
    # 计算右半部分的最大和（从中点向右）
    right_sum = float('-inf')
    current_sum = 0
    max_right = mid + 1
    
    for i in range(mid + 1, high + 1):
        current_sum += arr[i]
        if current_sum > right_sum:
            right_sum = current_sum
            max_right = i
    
    # 返回跨越中点的最大子数组和及其起止位置
    return left_sum + right_sum, max_left, max_right

def max_subarray_dp(arr):
    """
    动态规划法求解最大子数组和
    时间复杂度：O(n)，其中n是数组长度
    """
    n = len(arr)
    # dp[i]表示以第i个元素结尾的最大子数组和
    dp = [0] * n
    dp[0] = arr[0]
    
    # 记录全局最大和及其结束位置
    max_sum = dp[0]
    end_index = 0
    
    for i in range(1, n):
        # 状态转移方程：要么将当前元素加入前面的子数组，要么重新开始一个子数组
        dp[i] = max(dp[i-1] + arr[i], arr[i])
        if dp[i] > max_sum:
            max_sum = dp[i]
            end_index = i
    
    # 回溯找到子数组的起始位置
    start_index = end_index
    while start_index > 0 and dp[start_index-1] > 0:
        start_index -= 1
    
    return max_sum, start_index, end_index

def main():
    # 测试用例1
    arr1 = [4, -6, 5, 2, -1]
    max_sum, start, end = max_subarray_dp(arr1)
    print(f"测试用例1: {arr1}")
    print(f"最大子数组和: {max_sum}")
    print(f"子数组: {arr1[start:end+1]}")
    
    # 测试用例2
    arr2 = [-13, -3, -25, -20, -3, -16, -23, -12, -5, -22, -15, -4, -7]
    max_sum, start, end = max_subarray_dp(arr2)
    print(f"\n测试用例2: {arr2}")
    print(f"最大子数组和: {max_sum}")
    print(f"子数组: {arr2[start:end+1]}")

if __name__ == "__main__":
    main() 