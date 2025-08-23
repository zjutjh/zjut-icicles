# 活动选择问题的贪心算法实现
def activity_selection(activities):
    # activities: [(start, finish), ...]
    # 按结束时间排序
    activities.sort(key=lambda x: x[1])
    n = len(activities)
    res = []
    last_end = -float('inf')
    for s, f in activities:
        if s >= last_end:
            res.append((s, f))
            last_end = f
    return res

# 示例
activities = [(1, 4), (3, 5), (0, 6), (5, 7), (3, 8), (5, 9), (6, 10), (8, 11), (8, 12), (2, 13), (12, 14)]
selected = activity_selection(activities)
print("最多可选的报告数：", len(selected))
print("选择的报告：", selected)