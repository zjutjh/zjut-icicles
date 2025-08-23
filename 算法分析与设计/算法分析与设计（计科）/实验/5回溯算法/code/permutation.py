def permute(chars, start, end, result):
    if start == end:
        result.append(''.join(chars))
    else:
        for i in range(start, end + 1):
            # 交换位置
            chars[start], chars[i] = chars[i], chars[start]
            # 递归处理
            permute(chars, start + 1, end, result)
            # 回溯，恢复原状
            chars[start], chars[i] = chars[i], chars[start]

def print_all_permutations(input_string):
    chars = list(input_string)
    result = []
    n = len(chars)
    permute(chars, 0, n - 1, result)
    return result

# 测试用例
if __name__ == "__main__":
    test_input = "XYR"
    permutations = print_all_permutations(test_input)
    print(f"字符串 '{test_input}' 的全排列：")
    for p in permutations:
        print(p) 