# 固定翼操纵面双脉冲测试脚本分析

这是一个用于固定翼飞机执行**操纵面双脉冲测试**的脚本，用于**飞机动力学参数辨识**或**控制响应测试**。

## 主要功能

### 1. **操纵面双脉冲测试**
- **升降舵双脉冲**：当选择开关处于低位时，对升降舵进行双脉冲测试
- **方向舵双脉冲**：当选择开关处于高位时，对方向舵进行双脉冲测试
- **标准化脉冲信号**：先向一个方向偏转，然后向相反方向偏转，最后回到中立位置

### 2. **测试条件控制**
- **隔离测试通道**：固定非测试通道（副翼、方向舵/升降舵、油门）在配平位置
- **保持配平状态**：使用伺服超时覆盖来维持测试期间的稳定状态
- **飞行模式管理**：自动切换到手动模式进行测试，测试后恢复原模式

### 3. **安全恢复机制**
- **提前中止恢复**：如果测试中途释放开关，自动进入FBWA（稳定辅助）模式
- **超时自动清除**：所有伺服覆盖都设有超时，确保控制权自动恢复
- **完整状态清理**：测试结束后清除所有覆盖，恢复正常控制

## 工作流程

### 初始化阶段
1. **配置参数**：
   - 动作通道（RC6）：启动测试的瞬时开关
   - 选择通道（RC7）：选择升降舵（低）或方向舵（高）
   - 脉冲幅度：6（基于45度比例）
   - 脉冲时间：500ms

2. **获取伺服参数**：
   - 最小、最大和配平PWM值
   - 伺服通道映射

### 主循环逻辑

#### 1. **测试启动条件**
```lua
if arming:is_armed() == true and rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 and end_time == -1 then
```
- 飞机已解锁
- 动作通道PWM > 1700（开关高位）
- 测试未完成

#### 2. **测试阶段划分**
- **阶段1**：记录开始时间，配置测试参数，固定非测试通道
- **阶段2**（前250ms）：操纵面向一个方向偏转
- **阶段3**（后250ms）：操纵面向相反方向偏转
- **阶段4**（1000ms）：保持配平位置，观察响应
- **阶段5**：测试完成，清理状态

#### 3. **测试中止处理**
```lua
elseif now ~= -1 then
    -- 停止前完成。恢复到水平姿态
    gcs:send_text(6, "FBWA RECOVER")
```
- 如果开关在测试完成前释放
- 进入FBWA模式辅助恢复
- 清除所有伺服覆盖

## 技术实现

### 1. **伺服覆盖控制**
```lua
SRV_Channels:set_output_pwm_chan_timeout(trim_chan, trim_pwm, DOUBLET_TIME * 2)
```
- **参数1**：通道索引
- **参数2**：目标PWM值
- **参数3**：超时时间（毫秒），超时后恢复飞控控制

### 2. **脉冲幅度计算**
```lua
-- 向下偏转计算
down = doublet_srv_trim - math.floor((doublet_srv_trim - doublet_srv_min) * (DOUBLET_MAGNITUDE / 45))

-- 向上偏转计算
up = doublet_srv_trim + math.floor((doublet_srv_max - doublet_srv_trim) * (DOUBLET_MAGNITUDE / 45))
```
- 基于伺服行程范围计算
- `DOUBLET_MAGNITUDE/45`：将幅度转换为行程比例

### 3. **飞行模式管理**
```lua
function retry_set_mode(mode)
    if vehicle:set_mode(mode) then
        desired_mode = -1
        return doublet, 1
    else
        desired_mode = mode
        return retry_set_mode, 1
    end
end
```
- 尝试设置飞行模式
- 如果失败，持续重试
- 成功后才继续执行

## 测试配置

### 1. **升降舵测试**
- **选择条件**：`doublet_choice_pwm < 1500`
- **固定通道**：副翼（K_AILERON）和方向舵（K_RUDDER）
- **脉冲幅度**：12（基于45度比例）
- **配平位置**：使用SERVO_TRIM参数

### 2. **方向舵测试**
- **选择条件**：`doublet_choice_pwm >= 1500`
- **固定通道**：副翼（K_AILERON）
- **升降舵固定**：固定在当前位置（非配平位置）
- **脉冲幅度**：15（基于45度比例）

### 3. **油门管理**
- 固定为测试开始时的油门位置
- 超时时间：`DOUBLET_TIME * 3`（1500ms）

## 安全机制

### 1. **多层级超时设置**
```lua
-- 不同通道设置不同的超时时间
SRV_Channels:set_output_pwm_chan_timeout(trim_chan, trim_pwm, DOUBLET_TIME * 2)  -- 副翼/方向舵
SRV_Channels:set_output_pwm_chan_timeout(elevator_chan, pre_doublet_elevator, DOUBLET_TIME * 4)  -- 升降舵
SRV_Channels:set_output_pwm_chan_timeout(throttle_chan, pre_doublet_throttle, DOUBLET_TIME * 3)  -- 油门
```

### 2. **控制权恢复**
```lua
-- 测试结束后清除所有覆盖
control_functions = {K_AILERON, K_ELEVATOR, K_THROTTLE, K_RUDDER}
for i = 1, 4 do
    local control_chan = SRV_Channels:find_channel(control_functions[i])
    SRV_Channels:set_output_pwm_chan_timeout(control_chan, param:get("SERVO" .. control_chan + 1 .. "_TRIM"), 0)
end
```

### 3. **飞行模式安全切换**
- 测试前：保存当前飞行模式
- 测试中：切换到手动模式（MODE_MANUAL）
- 测试后：恢复原飞行模式
- 中止时：切换到FBWA模式辅助恢复

## 应用场景

### 1. **飞机参数辨识**
- 通过双脉冲响应识别飞机的动态特性
- 用于控制系统设计和参数调优

### 2. **操纵面有效性测试**
- 验证操纵面的偏转范围和响应速度
- 检测伺服系统性能

### 3. **飞行品质评估**
- 评估飞机的阻尼特性和稳定性
- 为飞行控制律设计提供数据

### 4. **安全测试**
- 在受控条件下测试极限操纵
- 验证故障安全机制

## 测试步骤

### 1. **准备阶段**
- 飞机在FBWB模式下稳定平飞
- 进行配平调整（SLUF状态）
- 确保足够的安全高度

### 2. **测试执行**
1. 将选择开关置于所需位置（升降舵或方向舵）
2. 按下动作开关启动测试
3. 保持开关按下直到看到"DOUBLET FINISHED"
4. 释放开关

### 3. **数据记录**
- 使用数据闪存记录测试过程
- 分析姿态、角速度和操纵面响应
- 提取飞机动态模型参数

## 消息系统

### 1. **状态通知**
- **启动**："STARTING DOUBLET [通道号]"
- **完成**："DOUBLET FINISHED"
- **恢复**："FBWA RECOVER" 或 "RETURN TO PREVIOUS FLIGHT MODE"

### 2. **错误提示**
- 如果RC通道仍保持高位："RC6 still high"
- 理论上不应到达的状态："this should not be reached"

### 3. **运行状态**
- 脚本启动："plane-doublets.lua is running"

## 配置建议

### 1. **安全高度**
- 建议至少100米安全高度
- 确保有足够的恢复空间

### 2. **初始条件**
- 直线、水平、非加速飞行（SLUF）
- 稳定的空速和姿态
- 完成配平调整

### 3. **参数调整**
- **DOUBLET_MAGNITUDE**：根据飞机响应调整，从较小值开始
- **DOUBLET_TIME**：根据飞机动态特性调整，典型值200-1000ms
- 伺服限幅：确保不超过物理限制

## 技术特点

### 1. **精确的时间控制**
- 使用`millis()`进行精确计时
- 分阶段控制脉冲形状
- 快速回调（测试期间10ms间隔）

### 2. **参数化设计**
- 所有关键参数可通过变量调整
- 支持不同飞机配置
- 易于扩展更多测试类型

### 3. **鲁棒的错误处理**
- 模式切换重试机制
- 全面的状态清理
- 多重安全恢复路径

这个脚本是一个专业的飞机测试工具，通过标准化的双脉冲测试方法，为飞机动态特性分析提供了可靠的手段。它体现了工程测试中的严谨性和安全性考虑，适用于飞机研发、系统辨识和飞行品质评估等专业应用。