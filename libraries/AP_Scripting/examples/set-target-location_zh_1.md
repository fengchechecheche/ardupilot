这是一个**简化版自动返航与降落脚本**，通过遥控器触发，让无人机在引导模式下返回家点上方并自动降落。以下是详细功能分析：

## 核心功能
这是一个**遥控器触发的一键返航系统**，当遥控器第7通道PWM值大于1800时，自动执行返航和降落流程。

## 详细分析

### 1. **参数配置**
```lua
local wp_radius = 2                    -- 到达半径（米），距离家点2米内触发降落
local target_alt_above_home = 10       -- 目标高度（家点上方10米）
local copter_guided_mode_num = 4       -- 多旋翼引导模式编号
local copter_land_mode_num = 9         -- 多旋翼降落模式编号
local sent_target = false              -- 目标发送标志
```

### 2. **遥控器触发条件**
```lua
pwm7 = rc:get_pwm(7)                   -- 读取第7通道PWM值
if pwm7 and pwm7 > 1800 then           -- PWM值大于1800时触发
```
- **第7通道**：通常用于模式切换或辅助功能
- **1800阈值**：典型三段开关的高位置阈值（1000=低，1500=中，2000=高）
- **逻辑**：需要持续保持开关在高位置才能执行整个流程

## 工作流程

### 条件检查阶段：
```
1. 检查是否已解锁（未解锁则重置状态）
2. 读取遥控器第7通道PWM值
3. 如果PWM > 1800 且未发送目标：
   a. 如果不是引导模式，切换到引导模式
   b. 如果是引导模式，计算家点上方10米位置
   c. 设置目标位置并标记已发送目标
4. 如果已发送目标且未在降落模式：
   a. 计算当前位置与家点的水平距离
   b. 距离 < 2米时切换到降落模式
```

### 详细执行流程：

#### 阶段1：触发和切换模式
```lua
if not sent_target then
  if not (mode == copter_guided_mode_num) then
    vehicle:set_mode(copter_guided_mode_num)  -- 切换到引导模式
  else
    -- 计算家点上方10米位置
    local above_home = ahrs:get_home()
    above_home:alt(above_home:alt() + (target_alt_above_home * 100))
    sent_target = vehicle:set_target_location(above_home)  -- 设置目标
  end
end
```

#### 阶段2：到达检查和降落
```lua
if sent_target then
  if not (mode == copter_land_mode_num) then
    local home = ahrs:get_home()
    local curr_loc = ahrs:get_location()
    local home_dist = curr_loc:get_distance(home)  -- 水平距离
    if (home_dist < wp_radius) then
      vehicle:set_mode(copter_land_mode_num)  -- 切换到降落模式
    end
  end
end
```

## 关键API功能

### 1. **位置高度调整**
```lua
above_home:alt(above_home:alt() + (target_alt_above_home * 100))
```
- `:alt()`获取/设置高度值，单位为厘米
- `target_alt_above_home * 100`：米转厘米
- 最终高度 = 家点高度 + 1000厘米（10米）

### 2. **水平距离计算**
```lua
curr_loc:get_distance(home)  -- 仅计算水平距离，忽略高度差
```
- 返回两个位置之间的水平距离（米）
- 用于判断是否到达家点水平位置

### 3. **目标位置设置**
```lua
sent_target = vehicle:set_target_location(above_home)
```
- 在引导模式下设置目标位置
- 无人机将自主飞向该位置
- 返回布尔值表示设置是否成功

## 状态管理

### 1. **sent_target标志**
- **false**：尚未发送目标位置，可以执行阶段1
- **true**：已发送目标位置，执行阶段2
- **重置**：当无人机解除武装时重置为false

### 2. **模式状态**
- 必须处于引导模式才能发送目标位置
- 到达目标附近后切换到降落模式
- 降落模式后不再执行任何操作

## 与标准RTL对比

### 相似之处：
1. 返回家点上方固定高度
2. 水平到达后执行降落
3. 自动模式切换

### 不同之处：
| 特性 | 标准RTL | 此脚本 |
|------|---------|--------|
| **触发方式** | 遥控器模式开关或故障保护 | 遥控器第7通道PWM值 |
| **高度设置** | 通过参数RTL_ALT设置 | 脚本固定10米 |
| **到达半径** | 通过参数WP_RADIUS设置 | 脚本固定2米 |
| **降落条件** | 到达后自动降落 | 需要保持触发条件 |
| **中断处理** | 切换模式即可中断 | 需要开关回中位 |

## 安全性设计

### 1. **解锁状态检查**
```lua
if not arming:is_armed() then
    sent_target = false  -- 解锁时重置状态
end
```
- 确保只在已解锁状态下执行
- 解锁时重置，防止意外触发

### 2. **模式检查**
- 确保在引导模式下发送目标
- 确保不在降落模式时才检查距离
- 防止重复发送目标

### 3. **位置有效性检查**
```lua
if above_home then  -- 检查家点是否有效
if home and curr_loc then  -- 检查当前位置是否有效
```

## 潜在问题

### 1. **触发条件保持**
```lua
if pwm7 and pwm7 > 1800 then
```
- 需要持续保持开关在高位置
- 如果中途开关回中位，流程会中断
- 可能导致悬停在家点上方不降落

### 2. **无超时保护**
- 如果无法到达目标位置，脚本会一直运行
- 没有最大飞行时间限制
- 没有低电量检查

### 3. **高度单位混淆**
```lua
target_alt_above_home * 100  -- 米转厘米
```
- 正确转换，但可能令人困惑
- 最好添加注释说明单位

## 改进建议

### 1. **添加超时保护**
```lua
local start_time = 0
local max_mission_time = 300  -- 5分钟最大任务时间

if sent_target and (millis() - start_time > max_mission_time * 1000) then
    gcs:send_text(0, "Mission timeout, forcing land")
    vehicle:set_mode(copter_land_mode_num)
    return update, 1000
end
```

### 2. **添加低电量保护**
```lua
local battery_voltage = battery:voltage(0)
if battery_voltage < 10.5 then
    gcs:send_text(0, "Low battery, forcing land")
    vehicle:set_mode(copter_land_mode_num)
    return update, 1000
end
```

### 3. **添加视觉/声音反馈**
```lua
if not sent_target then
    gcs:send_text(0, "RTL initiated, climbing to 10m")
    notify:play_tune("L8C8")  -- 播放提示音
elseif vehicle:get_mode() == copter_land_mode_num then
    gcs:send_text(0, "Starting landing sequence")
end
```

### 4. **添加参数化配置**
```lua
local trigger_channel = param:get("RTL_SCRIPT_CH") or 7
local trigger_threshold = param:get("RTL_SCRIPT_THR") or 1800
local rtl_altitude = param:get("RTL_SCRIPT_ALT") or 10
local arrival_radius = param:get("RTL_SCRIPT_RAD") or 2
```

### 5. **改进触发逻辑**
```lua
-- 使用开关触发而不是持续保持
local last_pwm7 = 0
local triggered = false

if pwm7 and pwm7 > 1800 and last_pwm7 <= 1800 then
    triggered = true  -- 开关从低/中位切换到高位时触发
end
last_pwm7 = pwm7

if triggered then
    -- 执行返航逻辑
end
```

### 6. **添加垂直距离检查**
```lua
-- 同时检查水平和垂直距离
local vertical_distance = math.abs(curr_loc:alt() - home:alt()) / 100  -- 转换为米
if home_dist < wp_radius and vertical_distance < 3 then  -- 水平和垂直都接近
    vehicle:set_mode(copter_land_mode_num)
end
```

## 实际应用场景

### 1. **紧急返航按钮**
- 将遥控器开关映射为紧急返航按钮
- 飞行员可一键触发安全返航
- 适合训练或紧急情况

### 2. **自动化测试**
- 测试自主返航和降落功能
- 验证位置控制精度
- 进行可靠性测试

### 3. **特定任务应用**
- 需要特定返航高度（非标准RTL高度）
- 需要自定义到达半径
- 需要与其他任务集成

### 4. **教学示例**
- 演示引导模式的使用
- 展示位置控制API
- 说明自动化任务设计

## 执行时间线示例

```
时间(秒) | 事件
--------|------
0       | 脚本启动，等待触发
10      | 飞行员将第7通道开关推到高位置
10.1    | 检测到PWM>1800，切换到引导模式
10.2    | 发送目标位置（家点上方10米）
15-30   | 无人机飞向目标位置
35      | 到达家点水平位置2米内
35.1    | 切换到降落模式
40-50   | 自动降落
55      | 降落完成，解锁重置状态
```

## 注意事项

### 1. **家点设置要求**
- 必须已设置有效的家点
- 家点高度需要准确
- GPS信号需要良好

### 2. **飞行环境**
- 确保10米高度无障碍物
- 确保降落区域安全
- 考虑风况影响

### 3. **遥控器配置**
- 第7通道需要正确映射
- PWM范围需要校准
- 避免意外触发

## 在无人机系统中的作用

### 1. **自定义自动化**
- 提供了标准RTL之外的自动化选项
- 允许自定义返航逻辑
- 支持特定应用需求

### 2. **安全增强**
- 增加手动触发的一键返航
- 可作为故障保护的补充
- 提供额外的安全层

### 3. **系统集成**
- 演示如何集成遥控器输入与自动化任务
- 展示多模式切换
- 说明状态机设计

这个脚本是一个实用的自定义返航系统，通过遥控器触发提供了灵活的自动化控制。它展示了如何结合遥控器输入、模式切换和位置控制来创建自定义任务，适合需要特定返航逻辑的应用场景。