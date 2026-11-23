# Robot Joint Order Check Tool Online

机器人关节顺序在线检查工具 / Online robot joint order checker (URDF · MJCF).

[![GitHub Pages](https://img.shields.io/badge/GitHub%20Pages-Live-brightgreen?logo=github)](https://imchong.github.io/Robot_Joint_Order_Check_Tool_Online/)
[![Deploy GitHub Pages](https://github.com/ImChong/Robot_Joint_Order_Check_Tool_Online/actions/workflows/pages.yml/badge.svg)](https://github.com/ImChong/Robot_Joint_Order_Check_Tool_Online/actions/workflows/pages.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**[打开在线工具](https://imchong.github.io/Robot_Joint_Order_Check_Tool_Online/)**

上传 URDF 或 MJCF，对比 Isaac Gym、Isaac Sim / Isaac Lab、MuJoCo、Genesis、Newton、Gazebo、PyBullet 和 ros2_control 的关节顺序。

**核心功能**：差异定位 · 运动学树 · Python / C++ 索引重映射 · URDF 顺序统一建议 · CSV / JSON 导出 · 四元数约定对照

文件仅在浏览器本地解析，不上传服务器。结果基于各框架默认规则静态推导；实际部署请以运行时关节列表为准。

## 本地运行

```bash
python3 -m http.server 8000
```

访问 <http://localhost:8000/>。无需构建。

本项目采用 [MIT License](LICENSE)。
