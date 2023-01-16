```java
HashMap<T, S>map = null;

map.containsKey(key);

map.get(key);

map.put(key, value);

map.remove(key);

map.clear();
```

## 遍历

```java
map.values() -> Collection<S>

for ( Iterator<S> i = map.values().iterator(); i.hasNext(); ) {
  S item = (S) i.next();
}
```