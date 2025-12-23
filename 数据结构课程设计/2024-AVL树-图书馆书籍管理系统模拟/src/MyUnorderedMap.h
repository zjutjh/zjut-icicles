// MyUnorderedMap.h
#ifndef MY_UNORDERED_MAP_H
#define MY_UNORDERED_MAP_H

#include <vector>
#include <functional>
#include <cstddef>   // size_t
#include <utility>   // std::pair
#include <stdexcept> // std::out_of_range

template <typename Key, typename Value,
         typename Hash = std::hash<Key>,
         typename KeyEqual = std::equal_to<Key>>
class MyUnorderedMap {
public:
    // 键值对结构体
    struct KeyValuePair {
        Key   key;
        Value value;
        KeyValuePair(const Key& k, const Value& v): key(k), value(v) {}
    };

private:
    std::vector<std::vector<KeyValuePair>> buckets; // 哈希桶，每个桶是一个KeyValuePair的链表
    size_t bucket_count;
    Hash hash_func;
    KeyEqual key_equal;
    size_t element_count;

    // 根据key计算hash bucket下标
    size_t bucket_index(const Key& key) const {
        return hash_func(key) % bucket_count;
    }

public:
    MyUnorderedMap(size_t bucket_count_hint = 8)
        : bucket_count(bucket_count_hint), element_count(0)
    {
        buckets.resize(bucket_count);
    }

    // 插入操作，如果键不存在则插入，存在则更新值
    void insert(const Key& key, const Value& value) {
        size_t idx = bucket_index(key);
        for (auto& kv : buckets[idx]) {
            if (key_equal(kv.key, key)) {
                kv.value = value; // 更新值
                return;
            }
        }
        // 没有相同Key，直接插入
        buckets[idx].emplace_back(key, value);
        ++element_count;

        // 简易负载控制：当装载因子过大时扩容
        if (load_factor() > 1.0) {
            rehash(bucket_count * 2);
        }
    }

    // 查找操作，如果找到返回指向值的指针，否则返回nullptr
    Value* find(const Key& key) {
        size_t idx = bucket_index(key);
        for (auto& kv : buckets[idx]) {
            if (key_equal(kv.key, key)) {
                return &kv.value;
            }
        }
        return nullptr;
    }

    const Value* find(const Key& key) const {
        size_t idx = bucket_index(key);
        for (const auto& kv : buckets[idx]) {
            if (key_equal(kv.key, key)) {
                return &kv.value;
            }
        }
        return nullptr;
    }

    // 删除操作，如果存在key则删除对应元素并返回true，否则返回false
    bool erase(const Key& key) {
        size_t idx = bucket_index(key);
        auto& chain = buckets[idx];
        for (auto it = chain.begin(); it != chain.end(); ++it) {
            if (key_equal(it->key, key)) {
                chain.erase(it);
                --element_count;
                return true;
            }
        }
        return false;
    }

    // 返回当前哈希表中元素个数
    size_t size() const {
        return element_count;
    }

    // 返回当前bucket的数量
    size_t bucket_size() const {
        return bucket_count;
    }

    // 计算并返回装载因子
    double load_factor() const {
        return static_cast<double>(element_count) / static_cast<double>(bucket_count);
    }

    // 重哈希：给定新的bucket数量并重新分配元素
    void rehash(size_t new_bucket_count) {
        std::vector<std::vector<KeyValuePair>> new_buckets(new_bucket_count);
        for (auto& chain : buckets) {
            for (auto& kv : chain) {
                size_t new_idx = hash_func(kv.key) % new_bucket_count;
                new_buckets[new_idx].emplace_back(kv.key, kv.value);
            }
        }
        buckets.swap(new_buckets);
        bucket_count = new_bucket_count;
    }

    // 获取所有键值对（用于迭代）
    std::vector<std::pair<Key, Value>> getAllKeyValuePairs() const {
        std::vector<std::pair<Key, Value>> allPairs;
        for (const auto& chain : buckets) {
            for (const auto& kv : chain) {
                allPairs.emplace_back(std::make_pair(kv.key, kv.value));
            }
        }
        return allPairs;
    }

    // operator[] 实现
    Value& operator[](const Key& key) {
        size_t idx = bucket_index(key);
        for (auto& kv : buckets[idx]) {
            if (key_equal(kv.key, key)) {
                return kv.value;
            }
        }
        // 如果键不存在，插入默认值并返回
        buckets[idx].emplace_back(key, Value());
        ++element_count;

        // 简易负载控制：当装载因子过大时扩容
        if (load_factor() > 1.0) {
            rehash(bucket_count * 2);
            // 重新计算索引，因为bucket_count已经改变
            idx = bucket_index(key);
        }

        return buckets[idx].back().value;
    }

    // const at()
    const Value& at(const Key& key) const {
        size_t idx = bucket_index(key);
        for (const auto& kv : buckets[idx]) {
            if (key_equal(kv.key, key)) {
                return kv.value;
            }
        }
        throw std::out_of_range("Key not found in MyUnorderedMap::at()");
    }

    // 非const at()
    Value& at(const Key& key) {
        size_t idx = bucket_index(key);
        for (auto& kv : buckets[idx]) {
            if (key_equal(kv.key, key)) {
                return kv.value;
            }
        }
        throw std::out_of_range("Key not found in MyUnorderedMap::at()");
    }
};

#endif // MY_UNORDERED_MAP_H
