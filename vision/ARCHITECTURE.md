# Vision Architecture

当前目录已经重建为可复用视觉框架。第一阶段保持原有可运行行为不变：

```text
main.py -> app.py -> app_legacy.py
```

`app_legacy.py` 保存原来的稳定视觉主循环。新目录中的模块是后续逐步迁移的目标边界，避免一次性重构打断当前可用识别和串口闭环。

## Target Layout

```text
main.py                 启动入口
app.py                  主应用调度
app_legacy.py           当前稳定旧逻辑，后续逐步拆出
config.py               参数集中管理
registry.py             组件注册表

core/                   通用数据结构、接口、几何、滤波、状态机、仲裁器
detectors/              YOLO 和传统检测器
trackers/               ROI 传统视觉、运动预测、混合跟踪
preprocess/             数字调焦、激光 mask、图像工具
comm/                   UART 通信与 STM32 状态解析
control/                瞄准点、dx/dy 输出限幅
ui/                     触摸、候选面板、渲染
diagnostics/            日志、性能统计、记录
```

## Migration Order

1. 把 `app_legacy.py` 中模型加载迁移到 `detectors/yolo_maix.py`。
2. 把数字调焦和激光 mask 迁移到 `preprocess/`。
3. 把 STM32 状态解析和 dx/dy 发送迁移到 `comm/` 与 `control/`。
4. 把候选列表、触摸选择、绘制迁移到 `ui/`。
5. 把锁定、平滑、丢失保持、重捕迁移到 `trackers/hybrid_tracker.py`。
6. 最后让 `app.py` 直接调度各模块，删除对 `app_legacy.py` 的依赖。

## Hybrid Principle

```text
YOLO          负责识别、候选、重捕
传统视觉 ROI 负责锁定后的连续中心输出
Arbiter       决定当前帧信 YOLO、传统视觉、预测还是丢失
STM32 PID     只接收稳定后的 dx/dy
```
