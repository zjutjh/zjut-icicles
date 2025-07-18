def loading_problem(weights, capacity):
    n = len(weights)
    result = {'max_weight': 0, 'solution': []}
    current = []
    
    def backtrack(index, current_weight):
        # 找到一个更好的解
        if current_weight > result['max_weight'] and current_weight <= capacity:
            result['max_weight'] = current_weight
            result['solution'] = current.copy()
        
        # 已经考虑了所有物品
        if index == n:
            return
        
        # 尝试装载当前物品
        if current_weight + weights[index] <= capacity:
            current.append(index)
            backtrack(index + 1, current_weight + weights[index])
            current.pop()
        
        # 尝试不装载当前物品
        backtrack(index + 1, current_weight)
    
    backtrack(0, 0)
    return result

# 测试用例
if __name__ == "__main__":
    # 物品重量
    weights = [10, 40, 30, 50, 35, 25]
    # 船的最大载重
    capacity = 100
    
    # 解决装载问题
    result = loading_problem(weights, capacity)
    
    print(f"物品重量: {weights}")
    print(f"船的最大载重: {capacity}")
    print(f"能够装载的最大重量: {result['max_weight']}")
    selected_weights = [weights[i] for i in result['solution']]
    print(f"选择的物品索引: {result['solution']}")
    print(f"选择的物品重量: {selected_weights}") 