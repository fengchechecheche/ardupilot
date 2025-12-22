这是一个**自动方形航线飞行脚本**，控制多旋翼无人机执行完整的自动化飞行任务，包括起飞、方形航线飞行和返航。以下是详细功能分析：

## 核心功能
这是一个**自动化飞行任务序列控制器**，通过遥控器触发，让无人机执行"起飞→20米正方形航线→返航"的完整自动化任务。

## 详细分析

### 1. **任务参数配置**
```lua
local takeoff_alt_above_home = 10   -- 起飞高度（家点上方10米）
local copter_guided_mode_num = 4    -- 引导模式编号
local copter_rtl_mode_num = 6       -- 返航模式编号
local stage = 0                     -- 当前阶段
local bottom_left_loc               -- 方形起点（左下角位置）
local square_side_length = 20       -- 正方形边长（20米）
```

### 2. **遥控器触发条件**
```lua
pwm6 = rc:get_pwm(6)
if pwm6 and pwm6 > 1800 then    -- 检查第6通道PWM值大于1800
```
- 使用遥控器第6通道作为任务启动开关
- 高电平（>1800）触发整个任务序列
- 需要保持开关在高位置直到任务完成

## 任务阶段分解

### 阶段0 → 阶段1：**切换到引导模式**
```lua
if (stage == 0) then
    if (vehicle:set_mode(copter_guided_mode_num)) then
        stage = stage + 1  -- 切换到阶段1
    end
end
```

### 阶段1 → 阶段2：**执行起飞**
```lua
elseif (stage == 1) then
    if (vehicle:start_takeoff(takeoff_alt_above_home)) then
        stage = stage + 1  -- 切换到阶段2
    end
end
```

### 阶段2 → 阶段3：**等待到达目标高度**
```lua
elseif (stage == 2) then
    local vec_from_home = home:get_distance_NED(curr_loc)
    if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
        stage = stage + 1  -- 切换到阶段3
        bottom_left_loc = curr_loc  -- 记录方形起点
    end
end
```

### 阶段3-6：**执行方形航线**
```
阶段3：向北飞行20米，速度2m/s
阶段4：向东飞行20米，速度2m/s  
阶段5：向南飞行20米，速度2m/s
阶段6：向西飞行20米，速度2m/s
```

### 阶段7：**切换返航模式**
```lua
elseif (stage == 7) then
    vehicle:set_mode(copter_rtl_mode_num)
    stage = stage + 1
    gcs:send_text(0, "finished square, switching to RTL")
end
```

## 方形航线算法详解

### 1. **位置参考点**
```lua
bottom_left_loc = curr_loc  -- 起飞完成时的位置作为方形左下角
```

### 2. **距离计算**
```lua
local dist_NE = bottom_left_loc:get_distance_NE(curr_loc)
```
- 返回当前位置相对于起点的东北方向距离
- `dist_NE:x()`：向北的距离（正值为北）
- `dist_NE:y()`：向东的距离（正值为东）

### 3. **四边飞行逻辑**

#### 北边（阶段3）：
```lua
target_vel:x(2)  -- 向北2m/s
if (dist_NE:x() >= square_side_length) then
    stage = stage + 1  -- 完成北边，进入阶段4
end
```

#### 东边（阶段4）：
```lua
target_vel:y(2)  -- 向东2m/s  
if (dist_NE:y() >= square_side_length) then
    stage = stage + 1  -- 完成东边，进入阶段5
end
```

#### 南边（阶段5）：
```lua
target_vel:x(-2)  -- 向南2m/s（北向负速度）
if (dist_NE:x() <= 2) then  -- 回到起点北向2米范围内
    stage = stage + 1  -- 完成南边，进入阶段6
end
```

#### 西边（阶段6）：
```lua
target_vel:y(-2)  -- 向西2m/s（东向负速度）
if (dist_NE:y() <= 2) then  -- 回到起点东向2米范围内
    stage = stage + 1  -- 完成西边，进入阶段7
end
```

## 坐标系和方向

### NED坐标系：
- **X轴**：正北方向，负南方向
- **Y轴**：正东方向，负西方向  
- **Z轴**：正下方，负上方（向下为正）

### 航线形状：
```
起点(0,0) → 北20m → 东20m → 南20m → 西20m → 返回起点
     ↓          ↓         ↓         ↓
    (0,0)    (20,0)   (20,20)   (0,20)   (0,0)
```

## 执行时间预估

### 飞行时间计算：
- **每边距离**：20米
- **飞行速度**：2米/秒
- **每边时间**：20 ÷ 2 = 10秒
- **总航线时间**：4 × 10 = 40秒

### 加上起飞和返航：
- **起飞时间**：约10-20秒（取决于上升速度）
- **航线时间**：40秒
- **总任务时间**：约50-60秒

## 安全特性

### 1. **状态重置**
```lua
if not arming:is_armed() then
    stage = 0  -- 解锁时重置所有状态
end
```

### 2. **阶段检查**
- 每个阶段执行前检查必要条件
- 阶段之间有序过渡
- 防止跳过关键步骤

### 3. **高度检查**
```lua
if (math.abs(takeoff_alt_above_home + vec_from_home:z()) < 1) then
```
- 检查是否到达目标高度（误差1米内）
- NED坐标系中Z向下为正，所以高度是`-vec_from_home:z()`
- 条件：`|目标高度 - (-Z)| < 1` 即 `|10 + Z| < 1`

## 代码中的问题

### 1. **注释不一致**
```lua
-- 脚本注释说检查RC7，但代码检查RC6
pwm6 = rc:get_pwm(6)  -- 应该是第6通道，注释写成了RC7
```

### 2. **误差处理不足**
```lua
if (dist_NE:x() <= 2) then  -- 南边和西边使用2米容差
```
- 北边和东边使用`>= square_side_length`（20米）
- 南边和西边使用`<= 2`米
- 不对称的误差处理可能导致航线不闭合

### 3. **缺乏超时保护**
- 如果某个阶段卡住，脚本会永远等待
- 没有最大时间限制

## 改进建议

### 1. **参数化配置**
```lua
local trigger_channel = param:get("SQUARE_CH") or 6
local trigger_threshold = param:get("SQUARE_THR") or 1800
local square_size = param:get("SQUARE_SIZE") or 20
local fly_speed = param:get("SQUARE_SPEED") or 2
local takeoff_alt = param:get("SQUARE_ALT") or 10
```

### 2. **对称误差处理**
```lua
-- 所有边使用相同的误差逻辑
local tolerance = 2  -- 2米误差容限

if (stage == 3) and (dist_NE:x() >= square_side_length - tolerance) then
    stage = stage + 1
end
-- 类似处理其他边
```

### 3. **添加超时机制**
```lua
local stage_start_time = {}
local stage_timeout = {
    [0] = 5000,   -- 模式切换5秒
    [1] = 10000,  -- 起飞10秒
    [2] = 30000,  -- 爬升30秒
    [3] = 15000,  -- 每边飞行15秒
    [4] = 15000,
    [5] = 15000,
    [6] = 15000,
    [7] = 5000    -- 模式切换5秒
}

-- 检查超时
local elapsed = millis() - stage_start_time[stage]
if elapsed > stage_timeout[stage] then
    gcs:send_text(0, string.format("Stage %d timeout, forcing RTL", stage))
    vehicle:set_mode(copter_rtl_mode_num)
    return update, 1000
end
```

### 4. **添加航点可视化**
```lua
-- 计算方形航点
local waypoints = {
    bottom_left_loc,  -- 起点
    bottom_left_loc:copy():offset(20, 0, 0),      -- 北20米
    bottom_left_loc:copy():offset(20, 20, 0),     -- 东北角
    bottom_left_loc:copy():offset(0, 20, 0),      -- 东20米
    bottom_left_loc                                -- 返回起点
}

-- 发送航点到地面站显示
if stage == 2 then
    for i, wp in ipairs(waypoints) do
        gcs:send_text(0, string.format("Waypoint %d: N%.1f, E%.1f", i, wp:lat(), wp:lng()))
    end
end
```

### 5. **改进速度控制**
```lua
-- 根据剩余距离调整速度，实现平滑停止
local function calculate_velocity(target_distance, current_distance, max_speed)
    local remaining = target_distance - current_distance
    if remaining < 2 then  -- 最后2米减速
        return max_speed * (remaining / 2)
    else
        return max_speed
    end
end

if (stage == 3) then
    local speed = calculate_velocity(square_side_length, dist_NE:x(), 2)
    target_vel:x(speed)
end
```

## 实际应用场景

### 1. **自动巡检**
- 建筑物或农田的方形区域巡检
- 定期自动飞行检查
- 固定路线数据采集

### 2. **飞行测试**
- 测试自动驾驶性能
- 验证导航系统精度
- 评估飞行控制稳定性

### 3. **航拍任务**
- 自动方形区域拍摄
- 创建全景或网格照片
- 重复性拍摄任务

### 4. **教学演示**
- 展示完整的自动化任务
- 演示多阶段任务控制
- 说明速度控制方法

## 任务时间线示例

```
时间(秒) | 阶段 | 动作
--------|------|------
0       | 0    | 等待触发
5       | 0→1  | 检测到RC6>1800，切换引导模式
6       | 1→2  | 发送起飞命令
15      | 2→3  | 到达10米高度，记录起点
15-25   | 3    | 向北飞行20米（2m/s）
25-35   | 4    | 向东飞行20米
35-45   | 5    | 向南飞行20米
45-55   | 6    | 向西飞行20米
55      | 6→7  | 返回起点，切换返航模式
56      | 7→8  | 执行返航
```

## 与其他脚本的对比

### 与`set-target-location.lua`对比：
| 特性 | set-target-location.lua | set-target-velocity.lua |
|------|------------------------|-------------------------|
| **控制方式** | 位置控制 | 速度控制 |
| **任务类型** | 简单返航 | 复杂航线 |
| **阶段数量** | 2个阶段 | 8个阶段 |
| **运动模式** | 直接飞向目标 | 分段速度控制 |
| **适用场景** | 紧急返航 | 预定航线飞行 |

## 在无人机系统中的作用

### 1. **高级自动化演示**
- 展示复杂任务序列控制
- 演示速度控制方法
- 实现完整的自动化流程

### 2. **系统能力测试**
- 测试长时间自主飞行能力
- 验证多个飞行模式的切换
- 评估系统可靠性

### 3. **应用开发基础**
- 为更复杂的航线任务提供模板
- 支持自定义航线形状
- 可作为其他自动化任务的基础

这个脚本展示了一个完整的自动化飞行任务，从触发、起飞、航线飞行到返航，涵盖了无人机自主飞行的多个关键环节。适合用于测试、演示和特定应用场景。