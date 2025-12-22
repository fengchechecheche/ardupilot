# 固定翼风速故障保护脚本分析

这是一个用于固定翼飞机的**风速监控和故障保护脚本**，当风速超过预设阈值时提供警告，并在风速过大时自动触发安全措施。

## 主要功能

### 1. **风速实时监控**
- 通过AHRS获取风估计向量
- 计算二维水平风速（忽略垂直分量）
- 每秒检查一次风速变化

### 2. **两级风速响应**
- **警告阈值**：10米/秒（36公里/小时）
  - 当风速超过此值时，每15秒发送一次警告消息
- **故障保护阈值**：15米/秒（54公里/小时）
  - 当风速超过此值时，立即触发故障保护，执行返航（RTL）

### 3. **智能通知机制**
- **警告消息**：使用优先级4的消息，每15秒发送一次，避免消息泛滥
- **故障保护消息**：使用优先级0的消息，立即发送
- **时间间隔控制**：使用`warning_last_sent_ms`记录上次发送时间

## 技术实现

### 1. **风速计算**
```lua
-- 获取三维风向量
local wind = ahrs:wind_estimate()

-- 创建二维向量（忽略垂直分量）
wind_xy = Vector2f()
wind_xy:x(wind:x())
wind_xy:y(wind:y())

-- 计算风速（向量长度）
speed = wind_xy:length()
```

### 2. **风估计来源**
- AHRS（姿态和航向参考系统）估计的风速
- 基于空速和地速的向量差计算
- 需要飞机有可靠的空速传感器

### 3. **故障保护触发**
```lua
if speed > failsafe_speed then
    gcs:send_text(0, "Wind failsafe at " .. speed .. " metres/second")
    vehicle:set_mode(11) -- 切换到RTL模式
    return  -- 停止脚本执行
end
```

## 工作流程

### 1. **初始化设置**
- 警告阈值：10米/秒
- 故障保护阈值：15米/秒
- 警告间隔：15秒
- 检查频率：1Hz

### 2. **主循环（每秒执行一次）**
```
开始
  ↓
获取风估计向量
  ↓
计算二维风速
  ↓
风速 > 15米/秒？ → 是 → 发送故障保护消息 → 执行RTL返航 → 停止脚本
  ↓否
风速 > 10米/秒？ → 是 → 检查是否超过15秒间隔？ → 发送警告消息
  ↓否
等待1秒
  ↓
返回主循环
```

### 3. **消息优先级**
- **优先级0**：故障保护消息（最高优先级）
- **优先级4**：警告消息（中等优先级）

## 飞行模式映射

```lua
vehicle:set_mode(11)  -- 11对应RTL（返航）模式
```
- 在ArduPilot Plane中，飞行模式11对应RTL（Return to Launch）
- 注释中提到"应该是枚举"，但直接使用数值11确保兼容性

## 应用场景

### 1. **固定翼安全飞行**
- 防止在强风条件下飞行
- 避免因风速过大导致飞机失控
- 特别适用于轻型固定翼飞机

### 2. **远程飞行监控**
- 当操作员可能无法及时响应时，自动触发安全措施
- 为自主飞行提供额外的安全层

### 3. **天气条件限制**
- 确保飞机在安全风速范围内运行
- 防止在突发的阵风中飞行

## 安全特性

### 1. **渐进式响应**
- 先警告，后保护
- 给操作员反应时间
- 避免不必要的故障保护触发

### 2. **消息间隔控制**
- 警告消息每15秒发送一次
- 防止地面站消息过载
- 保持操作界面清晰

### 3. **故障保护后停止**
```lua
return  -- 触发故障保护后停止脚本
```
- 避免重复触发故障保护
- 让飞控系统接管后续操作
- 简化故障恢复流程

## 配置参数

### 1. **阈值调整**
```lua
local warn_speed = 10      -- 警告阈值（米/秒）
local failsafe_speed = 15  -- 故障保护阈值（米/秒）
```
- 可根据飞机类型和性能调整
- 建议值：
  - 轻型飞机：10/15米/秒
  - 中型飞机：15/20米/秒
  - 重型飞机：20/25米/秒

### 2. **时间间隔**
```lua
local warning_interval_ms = uint32_t(15000)  -- 15秒
```
- 平衡及时性和避免消息过载
- 可根据操作需求调整

### 3. **检查频率**
```lua
return update, 1000  -- 1秒间隔
```
- 足够响应风速变化
- 不会给飞控系统带来过大负担

## 局限性

### 1. **风估计准确性**
- 依赖于AHRS的风估计算法
- 需要可靠的空速和GPS数据
- 在机动飞行时可能不准确

### 2. **仅水平风分量**
- 仅考虑水平风分量
- 忽略垂直风（上升/下降气流）
- 对于山区或复杂地形可能不够

### 3. **固定翼专用**
- 脚本明确标注仅适用于ArduPlane
- 其他机型（多旋翼、直升机）可能需要调整

## 改进建议

### 1. **动态阈值**
```lua
-- 根据飞机重量或电池电量调整阈值
local battery_remaining = battery:capacity_remaining_pct()
local adjusted_failsafe = failsafe_speed * (battery_remaining / 100)
```

### 2. **风向考虑**
```lua
-- 考虑风向与航向的关系
local wind_direction = math.deg(math.atan2(wind_xy:y(), wind_xy:x()))
local heading = math.deg(ahrs:get_yaw())
local headwind_component = speed * math.cos(math.rad(wind_direction - heading))
```

### 3. **高级故障保护选项**
```lua
-- 提供多种故障保护动作选项
local failsafe_action = param:get("WIND_FS_ACTION")
if failsafe_action == 0 then
    vehicle:set_mode(11)  -- RTL
elseif failsafe_action == 1 then
    vehicle:set_mode(6)   -- FBWA（稳定辅助）
elseif failsafe_action == 2 then
    -- 逐渐降低高度
end
```

### 4. **历史记录**
```lua
-- 记录风速历史用于分析
local wind_history = {}
function record_wind_speed(speed)
    table.insert(wind_history, {time=millis(), speed=speed})
    if #wind_history > 60 then
        table.remove(wind_history, 1)  -- 保持最近60秒记录
    end
end
```

## 使用注意事项

### 1. **传感器要求**
- 必需：GPS接收器
- 必需：空速传感器
- 推荐：高精度气压计

### 2. **飞行前测试**
- 在地面测试脚本功能
- 验证风估计的准确性
- 调整阈值以适应具体飞机

### 3. **与其他故障保护协调**
- 确保不与飞控内置故障保护冲突
- 考虑电池、GPS等其他故障保护条件
- 设置合理的优先级

这个脚本提供了一个简单而有效的风速监控方案，特别适合在风况多变的地区进行固定翼飞行。通过自动化的风速检测和故障保护，大大增强了飞行安全性，减少了操作员的工作负担。