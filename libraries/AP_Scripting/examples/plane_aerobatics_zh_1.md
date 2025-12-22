# 固定翼特技飞行脚本分析

这是一个用于固定翼飞机在自动模式下执行特技飞行的复杂Lua脚本，实现了三种不同的特技动作。

## 主要功能

### 1. **三种特技动作执行**
- **轴向横滚（Axial Roll）**：以恒定横滚率绕纵轴旋转360°
- **循环（Loop）**：执行垂直方向上的筋斗动作
- **横滚圆周（Rolling Circle）**：在圆形轨迹上同时进行横滚动作

### 2. **高级飞行控制**
- 多通道PID控制器实现精确的姿态和高度控制
- 支持倒飞状态下的控制逻辑
- 实时调整油门以补偿俯仰角变化

### 3. **任务系统集成**
- 通过任务指令触发特技动作
- 支持DO_JUMP（177）指令的解析
- 自动计算航点间航向

## 脚本结构

### 1. **参数绑定与初始化**
```lua
-- 绑定用户参数用于控制器调参
local SCR_USER1 = bind_param("SCR_USER1") -- 高度P增益
local SCR_USER2 = bind_param("SCR_USER2") -- 高度I增益
local SCR_USER3 = bind_param("SCR_USER3") -- 俯仰油门前馈
local SCR_USER4 = bind_param("SCR_USER4") -- 刀锋飞行俯仰补偿
local SCR_USER5 = bind_param("SCR_USER5") -- 速度P增益
local SCR_USER6 = bind_param("SCR_USER6") -- 速度I增益
```

### 2. **控制器实现**
- **PI控制器对象**：通用的比例-积分控制器
- **高度控制器**：基于PI控制，增加刀锋飞行补偿
- **速度控制器**：维持空速的PI控制器
- **横滚零控制器**：处理倒飞情况的横滚控制
- **俯仰控制器**：同时控制俯仰和偏航的地球/机体坐标系转换

### 3. **特技动作函数**
1. **`do_axial_roll(arg1, arg2)`**：执行轴向横滚
   - `arg1`：横滚率（度/秒）
   - `arg2`：油门百分比

2. **`do_loop(arg1, arg2)`**：执行循环
   - `arg1`：俯仰率（度/秒）
   - `arg2`：未使用

3. **`do_rolling_circle(arg1, arg2)`**：执行横滚圆周
   - `arg1`：圆周半径（米）
   - `arg2`：横滚圈数

## 技术实现

### 1. **姿态控制算法**
```lua
-- 零横滚角控制器（支持倒飞）
function roll_zero_controller(tconst)
   local roll_deg = math.deg(ahrs:get_roll())
   -- 处理接近90度俯仰（刀锋飞行）的情况
   -- 处理超过±90度的横滚（倒飞）情况
end

-- 俯仰和偏航控制（地球到机体坐标系转换）
function pitch_controller(target_pitch_deg, target_yaw_deg, tconst)
   -- 计算地球坐标系下的俯仰和偏航率
   -- 转换到机体坐标系
   return bf_pitch_rate, bf_yaw_rate
end
```

### 2. **PI控制器实现**
```lua
local function PI_controller(kP,kI,iMax)
   -- 完整的PID控制器实现
   -- 包含积分限幅、抗饱和等特性
   -- 支持参数动态调整
   -- 集成数据记录功能
end
```

### 3. **特技状态机**
每个特技动作都使用状态机管理：
- **阶段0**：进入条件检查
- **阶段1**：执行特技
- **阶段2**：完成特技

### 4. **油门控制逻辑**
```lua
-- 根据俯仰角调整油门
function throttle_controller(tconst)
   local pitch_rad = ahrs:get_pitch()
   local thr_ff = SCR_USER3:get()  -- 前馈增益
   local throttle = TRIM_THROTTLE:get() + math.sin(pitch_rad) * thr_ff
   return constrain(throttle, 0.0, 100.0)
end
```

## 特技动作详解

### 1. **轴向横滚（Axial Roll）**
- **目标**：绕纵轴旋转360°，保持高度和航向
- **控制方式**：
  - 恒定横滚率
  - 高度PI控制器维持高度
  - 俯仰控制器维持航向
- **阶段**：
  1. 横滚超过45°时进入阶段1
  2. 横滚接近0°（±5°）时完成

### 2. **循环（Loop）**
- **目标**：执行垂直筋斗，绕横轴旋转360°
- **控制方式**：
  - 恒定俯仰率
  - 横滚零控制器保持机翼水平
  - 油门控制器补偿重力
- **阶段**：
  1. 俯仰超过60°时进入阶段1
  2. 俯仰接近0°且横滚接近水平时完成

### 3. **横滚圆周（Rolling Circle）**
- **目标**：在圆形轨迹上连续横滚
- **控制方式**：
  - 根据半径和地速计算偏航率
  - 根据横滚圈数计算横滚率
  - 高度和速度PI控制器维持高度和空速
- **阶段**：
  1. 偏航变化超过10°时进入阶段1
  2. 完成360°圆周时结束

## 任务系统集成

### 1. **指令接收**
```lua
-- 接收任务中的脚本时间指令
id, cmd, arg1, arg2 = vehicle:nav_script_time()
```

### 2. **指令映射**
- `cmd == 1`：轴向横滚
- `cmd == 2`：循环
- `cmd == 3`：横滚圆周

### 3. **航点计算**
```lua
-- 计算前一个航点到下一个航点的航向
local loc_prev = get_wp_location(cnum-1)
local loc_next = get_wp_location(resolve_jump(cnum+1))
wp_yaw_deg = math.deg(loc_prev:get_bearing(loc_next))
```

## 参数配置

### 1. **飞行控制参数**
- `RLL2SRV_TCONST`：横滚时间常数（用于控制器调参）
- `PTCH2SRV_TCONST`：俯仰时间常数
- `TRIM_THROTTLE`：平飞油门
- `TRIM_ARSPD_CM`：目标空速（厘米/秒）

### 2. **脚本用户参数**
- `SCR_USER1` - `SCR_USER6`：6个用户可调参数
- 分别控制高度、速度控制器的P、I增益和前馈

### 3. **控制器参数**
- 高度PI控制器：P和I增益，积分限幅20
- 速度PI控制器：P和I增益，积分限幅100

## 安全特性

### 1. **状态检查**
- 检查特技完成条件
- 防止重复触发
- 确保飞机处于安全状态

### 2. **控制限幅**
```lua
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end
```

### 3. **异常处理**
- 参数绑定失败时断言错误
- 航点解析错误处理
- 控制器积分限幅防止积分饱和

## 数据记录

### 1. **控制器日志**
```lua
-- 记录控制器内部状态
logger.write('HPI','Targ,Curr,P,I,Total,Add','ffffff',...)
logger.write('SPI','Targ,Curr,P,I,Total,Add','ffffff',...)
```

### 2. **地面站消息**
- 特技开始和完成通知
- 实时状态反馈
- 错误和警告消息

## 使用说明

### 1. **任务设置**
在任务中添加特技指令：
- 轴向横滚：`DO_SCRIPT_TIME`命令，参数1=1，参数2=横滚率，参数3=油门
- 循环：`DO_SCRIPT_TIME`命令，参数1=2，参数2=俯仰率
- 横滚圆周：`DO_SCRIPT_TIME`命令，参数1=3，参数2=半径，参数3=横滚圈数

### 2. **参数调参**
根据飞机特性调整：
- `SCR_USER1`-`SCR_USER6`：控制器增益
- `RLL2SRV_TCONST`和`PTCH2SRV_TCONST`：时间常数

### 3. **飞行测试**
- 在足够高度开始特技
- 确保飞机有足够的速度和功率
- 逐步增加特技难度

这个脚本展示了ArduPilot Lua脚本的高级能力，实现了复杂的飞行控制算法和特技动作。通过精心设计的控制器和状态机，能够在自动模式下安全地执行专业的特技飞行。对于航模爱好者和专业无人机操作员来说，这是一个强大的飞行表演工具。

---
# 1️⃣ 补充问题：请分析这个脚本中对哪些输出通道进行了控制，又是如何解决飞控本身计算的输出通道控制量与脚本计算的输出通道控制量之间的冲突的？
# 通道控制分析

## 1. **脚本控制的输出通道**

在这个脚本中，**脚本不直接控制任何输出通道**，而是通过**高层控制指令**间接影响输出通道。具体来说：

### 间接控制方式：
```lua
vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
```

### 控制的目标：
- **油门**：百分比（0-100%）
- **横滚角速度**：度/秒
- **俯仰角速度**：度/秒
- **偏航角速度**：度/秒

## 2. **输出通道控制的层级结构**

```
┌─────────────────────────────────────┐
│        Lua脚本（高层指令）           │
│  set_target_throttle_rate_rpy()     │
└───────────────┬─────────────────────┘
                │ 设置目标值（油门、角速度）
                ▼
┌─────────────────────────────────────┐
│       ArduPilot飞行控制器           │
│  - 姿态控制器（PID）                │
│  - 混控器（Mixing）                 │
│  - 输出限制（Slew Rate）            │
└───────────────┬─────────────────────┘
                │ 计算PWM值（1000-2000）
                ▼
┌─────────────────────────────────────┐
│          输出通道（PWM）             │
│  - 副翼、升降舵、方向舵、油门        │
└─────────────────────────────────────┘
```

## 3. **冲突解决机制**

### 3.1 **统一控制接口**
脚本使用与飞控内部相同的控制接口：
```lua
-- 脚本使用高层控制API
vehicle:set_target_throttle_rate_rpy()

-- 飞控内部也使用相同接口（在C++代码中）
AP::vehicle()->set_target_throttle_rate_rpy()
```

### 3.2 **优先级系统**
ArduPilot的控制系统有**明确的优先级层次**：

1. **最高优先级**：安全相关控制（失控保护、急停）
2. **高优先级**：手动控制（遥控器输入）
3. **中优先级**：自动控制（任务、自动模式）
4. **低优先级**：辅助控制（脚本、外部命令）

**脚本处于中优先级**，这意味着：
- 当手动控制激活时，脚本指令会被覆盖
- 脚本不能覆盖安全控制
- 多个脚本同时运行时，只有一个能获得控制权

### 3.3 **帧同步机制**
```lua
-- 脚本以10Hz频率运行（每100ms）
return update, 10

-- 飞控主循环通常以400Hz运行
-- 脚本指令会被插值到主循环中
```

### 3.4 **超时保护**
虽然这个脚本没有显式设置超时，但`set_target_throttle_rate_rpy()`内部有**隐含的超时保护**：
- 如果脚本停止发送指令，控制器会自动恢复到默认行为
- 在指定时间内没有新指令，控制权返回给更高优先级的系统

## 4. **与传统直接PWM控制的比较**

### 传统方式（直接控制）：
```lua
-- 直接设置PWM值（存在冲突风险）
SRV_Channels:set_output_pwm_chan_timeout(0, 1500, 1000)
```
**问题**：
- 与飞控的控制计算直接冲突
- 需要手动处理优先级
- 容易造成不稳定的混控

### 本脚本方式（间接控制）：
```lua
-- 设置目标物理量（无冲突）
vehicle:set_target_throttle_rate_rpy(50, 30, 0, 0)
```
**优势**：
- 由飞控统一处理所有控制输入
- 自动进行混控和优先级处理
- 保持控制系统的完整性

## 5. **具体示例分析**

### 轴向横滚中的控制：
```lua
function do_axial_roll(arg1, arg2)
    -- ...
    target_pitch = height_PI.update(initial_height)
    pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST)
    
    -- 设置目标值，由飞控计算实际输出
    vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
end
```

**飞控内部处理流程**：
1. **接收脚本目标值**：
   - 油门：50%
   - 横滚率：30°/s
   - 俯仰率：-2°/s
   - 偏航率：1°/s

2. **姿态控制器计算**：
   - 根据当前姿态计算误差
   - PID控制器输出舵面偏转角

3. **混控器计算**：
   ```c++
   // 简化的混控示例
   aileron = roll_PID_output * roll_mix_gain + yaw_PID_output * yaw_mix_gain;
   elevator = pitch_PID_output * pitch_mix_gain;
   rudder = yaw_PID_output * yaw_mix_gain - roll_PID_output * roll_mix_gain;
   ```

4. **输出到通道**：
   - 通道1（副翼）：1500 + aileron * scale
   - 通道2（升降舵）：1500 + elevator * scale
   - 通道3（油门）：1100 + throttle * scale
   - 通道4（方向舵）：1500 + rudder * scale

## 6. **特殊情况处理**

### 6.1 **多控制源合并**
当多个控制源同时存在时（如脚本+遥控器），飞控使用**混合策略**：
```c++
// 伪代码示例
if (manual_control_active) {
    // 手动控制为主
    output = manual_input * 0.8 + script_input * 0.2;
} else {
    // 脚本控制为主
    output = script_input;
}
```

### 6.2 **刀锋飞行补偿**
脚本中的特殊处理：
```lua
local ke_add = math.abs(math.sin(roll_rad)) * KnifeEdge:get()
target_pitch = target_pitch + ke_add
```
这个补偿值被送入控制器，最终影响的是**俯仰率目标**，而不是直接改变舵面输出。

## 7. **优势总结**

### 7.1 **安全性**
- 脚本不直接控制硬件
- 飞控保持最终决策权
- 内置的安全限制仍然有效

### 7.2 **兼容性**
- 与其他控制模式（手动、自动、悬停）无缝集成
- 支持所有机型配置（常规、V尾、飞翼等）
- 混控参数仍然生效

### 7.3 **可维护性**
- 脚本只关注高层逻辑
- 飞控处理底层细节
- 易于调试和调整

## 8. **与其他脚本的对比**

### `opendog_demo.lua`（直接控制）：
```lua
SRV_Channels:set_output_pwm_chan_timeout(i-1, pwm[i], 1000)
```
- **直接控制**：设置具体PWM值
- **冲突风险**：可能与其他控制冲突
- **超时必需**：必须设置超时恢复

### `plane_aerobatics.lua`（间接控制）：
```lua
vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
```
- **间接控制**：设置物理量目标
- **无冲突**：由飞控统一处理
- **更安全**：利用飞控的安全机制

## 9. **实际应用建议**

### 推荐使用间接控制的情况：
1. **飞行控制脚本**（如本脚本）
2. **高级飞行模式**
3. **自动化任务**

### 可能需要直接控制的情况：
1. **特殊伺服设备**（如相机云台、投放机构）
2. **非飞行控制功能**
3. **测试和校准**

## 结论

这个脚本采用了**最安全、最符合飞控架构**的控制方式：通过高层控制接口设置物理目标值，让飞控的统一控制系统计算具体的输出通道值。这种方式：

1. **完全避免冲突**：所有控制输入都由飞控统一混合处理
2. **保持安全性**：飞控的安全限制和故障保护仍然有效
3. **提高兼容性**：与所有飞行模式和机型配置兼容
4. **简化开发**：脚本开发者无需了解底层混控细节

这种设计体现了ArduPilot Lua脚本系统的一个重要原则：**脚本应该增强飞控的能力，而不是绕过飞控的控制系统**。
