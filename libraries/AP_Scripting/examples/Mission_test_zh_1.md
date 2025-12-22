# 任务系统测试脚本分析

这是一个用于测试ArduPilot任务系统绑定的脚本，具有任务监控、脚本命令接收和任务跳转功能。

## 主要功能

### 1. **任务状态监控**
- 实时监控任务执行状态
- 检测任务完成和停止状态
- 跟踪当前导航命令索引的变化

### 2. **脚本命令接收**
- 通过`mission_receive()`接收任务中的脚本命令（DO命令）
- 解析并显示脚本命令的时间戳和参数

### 3. **任务跳转逻辑**
- 当任务执行到倒数第二个命令时，随机跳转到前面的任务项
- 避免跳转到最后一个任务项，防止无限循环
- 提供任务跳转的成功/失败反馈

## 工作流程

### 初始化
1. **记录初始任务索引**：
   ```lua
   local last_mission_index = mission:get_current_nav_index()
   ```

### 主循环（1Hz运行）
1. **检查脚本命令**：
   - 调用`mission_receive()`获取任务中的脚本命令
   - 如果有命令，显示时间戳和四个参数

2. **检查任务状态**：
   - 如果任务完成：发送"Mission Complete"消息
   - 如果任务停止：发送"Mission stopped"消息

3. **监控任务索引变化**：
   - 比较当前任务索引与上次记录的索引
   - 如果发生变化：
     - 发送"New Mission Item"消息
     - 显示前一个和当前导航命令的ID
     - 更新记录的任务索引

4. **任务跳转逻辑**：
   - 计算任务长度（排除home命令）
   - 如果任务长度大于1且当前索引等于任务长度（即将完成）
   - 随机选择一个跳转目标（1到任务长度-1之间）
   - 执行跳转并显示结果

## 技术实现

### 1. **脚本命令接收**
```lua
local time_ms, param1, param2, param3, param4 = mission_receive()
```
- 返回脚本命令的时间戳和四个参数
- 时间戳为`time_ms`类型，使用`:tofloat()`转换为浮点数

### 2. **任务状态检查**
```lua
local mission_state = mission:state()
if mission_state == mission.MISSION_COMPLETE then
    -- 任务完成处理
elseif mission_state == mission.MISSION_STOPPED then
    -- 任务停止处理
end
```

### 3. **任务索引管理**
```lua
local mission_index = mission:get_current_nav_index()
if mission_index ~= last_mission_index then
    -- 任务索引变化处理
    last_mission_index = mission_index
end
```

### 4. **任务跳转**
```lua
local mission_length = mission:num_commands() - 1  -- 排除home命令
if mission_length > 1 and mission_index == mission_length then
    local jump_to = math.random(mission_length - 1)
    if mission:set_current_cmd(jump_to) then
        -- 跳转成功
    else
        -- 跳转失败
    end
end
```

## 消息输出

### 1. **脚本命令消息**
```
"Scripting CMD @ 12345 ms, 1, 2.50, 3.00, 4.00"
```

### 2. **任务状态消息**
- "LUA: Mission Complete"
- "LUA: Mission stopped"

### 3. **任务索引变化消息**
- "LUA: New Mission Item"
- "Prev: 16, Current: 17"（示例：前一个命令ID为16，当前命令ID为17）

### 4. **任务跳转消息**
- 成功："LUA: jumped to mission item 5"
- 失败："LUA: mission item jump failed"

## 应用场景

1. **任务系统测试**：测试AP_Mission绑定的功能和稳定性
2. **脚本命令调试**：验证任务中脚本命令的正确执行
3. **任务流程控制**：实现复杂的任务逻辑和跳转
4. **自动化测试**：用于任务系统的自动化测试框架

## 重要特性

### 1. **实时监控**
- 每秒检查一次任务状态
- 及时响应任务变化和脚本命令

### 2. **安全跳转**
- 避免跳转到最后一个任务项
- 检查跳转操作的返回值
- 提供跳转失败的处理

### 3. **兼容性考虑**
- 处理home命令（任务命令数减1）
- 支持随机跳转和固定跳转

## 脚本逻辑详解

### 1. **任务长度计算**
```lua
local mission_length = mission:num_commands() - 1
```
- `mission:num_commands()`返回包括home命令在内的总命令数
- 减1得到实际的导航命令数量

### 2. **跳转条件**
```lua
if mission_length > 1 and mission_index == mission_length then
```
- 任务长度大于1：确保有多个可执行命令
- 当前索引等于任务长度：即将执行最后一个命令

### 3. **随机跳转范围**
```lua
local jump_to = math.random(mission_length - 1)
```
- 使用`math.random()`生成随机数
- 范围：1到(任务长度-1)
- 避免跳转到最后一个命令

## 使用注意事项

### 1. **脚本命令格式**
- 脚本命令（DO命令）需要在任务中预先设置
- 参数1-4可以是整数或浮点数

### 2. **任务跳转限制**
- 只能在任务即将完成时触发跳转
- 跳转目标不能是最后一个命令
- 需要任务系统支持跳转功能

### 3. **性能考虑**
- 1Hz的检查频率对系统性能影响很小
- 跳转操作可能影响任务执行的连续性

## 扩展可能性

### 1. **增强跳转逻辑**
```lua
-- 添加条件跳转
if param1 == 1 then
    mission:set_current_cmd(param2)
end
```

### 2. **增加更多状态监控**
```lua
-- 监控任务暂停状态
if mission_state == mission.MISSION_PAUSED then
    gcs:send_text(0, "LUA: Mission paused")
end
```

### 3. **添加错误处理**
```lua
-- 捕获任务操作异常
local success, error_msg = pcall(function()
    mission:set_current_cmd(jump_to)
end)
```

### 4. **配置化跳转**
```lua
-- 从参数读取跳转配置
local jump_enabled = param:get("SCR_JUMP_ENABLE")
```

这个脚本是一个功能完整的任务系统测试工具，不仅展示了如何与ArduPilot的任务系统交互，还实现了实用的任务监控和跳转功能。对于开发和测试复杂的任务自动化流程，这是一个非常有价值的参考实现。