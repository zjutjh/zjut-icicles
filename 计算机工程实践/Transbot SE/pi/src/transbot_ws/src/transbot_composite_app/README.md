# transbot_composite_app

本包用于集成 transbot_ws 下各功能包，实现复合主程序流程。

## 目录结构

- scripts/   主程序与调度脚本
- launch/    启动文件
- config/    配置文件
- src/       可选的C++源码

## 依赖
- rospy
- std_msgs
- transbot_msgs

## 用法
1. 在 scripts/ 下编写主流程脚本（如 main.py）
2. 在 launch/ 下编写一键启动文件
3. 在 config/ 下放置参数配置

---

本包为复合流程开发的入口，建议所有自定义主流程节点均放于此包。
