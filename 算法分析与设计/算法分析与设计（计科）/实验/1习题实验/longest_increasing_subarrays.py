# -*- coding: utf-8 -*-
import sys
import io

# 设置标准输出编码为utf-8
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def count_long_subarrays(A):
    if not A:
        return 0
    
    n = len(A)
    # 存储当前递增子数组的长度
    current_length = 1
    # 存储最长递增子数组的长度
    max_length = 1
    # 存储最长递增子数组的个数
    count = 1
    
    for i in range(1, n):
        if A[i] > A[i-1]:
            # 当前元素大于前一个元素，递增子数组长度加1
            current_length += 1
        else:
            # 当遇到不递增的情况时，重置当前长度
            current_length = 1
        
        if current_length > max_length:
            # 如果找到更长的递增子数组，更新最大长度和计数
            max_length = current_length
            count = 1
        elif current_length == max_length:
            # 如果找到相同长度的递增子数组，增加计数
            count += 1
    
    return count

def main():
    # 测试用例
    A = [1, 3, 4, 2, 7, 5, 6, 9, 8]
    result = count_long_subarrays(A)
    print(u"输入：A = {0}".format(A))
    print(u"输出：count = {0}".format(result))
    print(u"解释：最长递增子数组为 (1,3,4) 和 (5,6,9)，长度都是3")
    
    # 额外的测试用例
    test_cases = [
        [1, 2, 3, 2, 3, 4, 5],  # 应该返回1，最长递增子数组是(2,3,4,5)
        [1, 2, 1, 2, 1, 2],     # 应该返回3，有三个长度为2的递增子数组
        [5, 4, 3, 2, 1],        # 应该返回5，每个单独的数字都是长度为1的递增子数组
        [1, 2, 3, 4, 5]         # 应该返回1，整个数组就是一个递增子数组
    ]
    
    print(u"\n额外测试用例：")
    for i, test_case in enumerate(test_cases, 1):
        result = count_long_subarrays(test_case)
        print(u"测试用例 {0}：".format(i))
        print(u"输入：A = {0}".format(test_case))
        print(u"输出：count = {0}\n".format(result))

if __name__ == "__main__":
    main() 