# 云台相机模式 Lua 脚本

此脚本在使用云台和相机时是一个实用工具。您可以用它在任务模式下切换手动与自动控制。它适用于带有空闲开关（三档）的遥控器。

## 工作原理

相机在自动模式下设置为拍照触发，但在手动模式下也可执行其他操作，例如录像。脚本会更改参数但不保存它们。
它会改变 `SERVO<X>_FUNCTION` 参数，并在 `RCIN<X>`（直通模式）、`mount_tilt`、`mount_roll` 和 `camera_trigger` 之间切换。
要查看存储的参数值，请禁用脚本（SCR_USER1）并重启飞控。

## 模式

三档开关设置的控制模式如下：

- **云台-相机模式 手动**：云台和相机处于手动控制模式（直通）

- **云台-相机模式 自动/手动**：云台自动，相机处于手动控制模式

- **云台-相机模式 自动**：云台和相机处于自动控制模式（由 ArduPilot 和任务控制）

## 设置与使用

- 如果尚未完成，请按照 ArduPilot 维基页面上的说明启用自动相机和云台控制。

- 同时配置用于手动控制的通道。

- 例如，我设置如下（转换数值请使用 https://ardupilot.org/copter/docs/parameters.html）：

  - SERVO8_FUNCTION = 7 (mount_tilt)   对应   MNT_RC_IN_TILT = 9 (RC9)     以及   RCIN9  (59)
  - SERVO9_FUNCTION = 8 (mount_roll)   对应   MNT_RC_IN_ROLL = 10 (RC10)   以及   RCIN10 (60)
  - SERVOn_FUNCTION = ? (mount_pan)    本例中未使用
  - SERVO10_FUNCTION = 10 (camera_trigger)   对应   RC12_OPTION = 9 (Camera Trigger)   以及   RCIN12 (62)

- 现在将这些值填入脚本 Gimbal_Camera_Mode.lua：

--------------------------------
     -- 低档：设置云台和相机为手动模式 (RCIN9, RCIN10, RCIN12)
    set_save_param('SERVO8_FUNCTION',59,false)
    set_save_param('SERVO9_FUNCTION',60,false)
    set_save_param('SERVO10_FUNCTION',62,false)
--------------------------------
     -- 中档：设置云台为自动，相机为手动模式 (mount_tilt, mount_roll, RCIN12)
    set_save_param('SERVO8_FUNCTION',7,false)
    set_save_param('SERVO9_FUNCTION',8,false)
    set_save_param('SERVO10_FUNCTION',62,false)
--------------------------------
     -- 高档：设置云台和相机为自动模式 (mount_tilt, mount_roll, camera_trigger)
    set_save_param('SERVO8_FUNCTION',7,false)
    set_save_param('SERVO9_FUNCTION',8,false)
    set_save_param('SERVO10_FUNCTION',10,false)
--------------------------------
   （'update()' 中的这些设置如果云台和相机已完全配置好，本可以自动推导出来，但这会增加脚本的内存占用，也可能增加出错行为的几率）
--------------------------------

- 为获得脚本的适当反馈，建议在设置期间将 `SCR_DEBUG_LVL` 设为 3。

- 将修改后的 Gimbal_Camera_Mode.lua 加载到飞控的 'scripts' 文件夹中。

- 重启飞控。

- 将您希望用于控制模式的开关所对应的 RC 输入通道的 `RC<X>_OPTION` 参数设置为 300。

- 将 `SCR_USER1` 设为 1 以启用脚本并允许其运行。

- 通过拨动开关并检查输出的日志消息来测试脚本。

## 输出消息

### "LUA: 云台-相机模式 已启动"

脚本已启动并正在运行。

此消息通常紧随 "LUA: 云台-相机模式 < X >"。如果**没有**，可能是脚本被禁用，请检查 `SCR_USER1`。设置 `SCR_USER1` 为 1 后脚本应立即运行（如果没有，可能是 Lua 脚本存在严重错误，因为无法读取参数 `SCR_USER1`）。

### "LUA: RC 通道未设置"

RC 通道未设置为脚本通道 (300)。请设置它！

### "LUA: 云台-相机模式 < X >"

已选择模式，参考上文的'模式'说明。

### "LUA: 参数设置失败"

设置参数失败。脚本将持续尝试设置参数而不会退出。如果此消息持续显示，请考虑降落并调查根本问题。

## 已知问题

手动和自动模式没有相同的行程，因为对于"直通" RCINn 通道会忽略 SERVOn_ 设置 - 需要引入并使用一个缩放且有边界的变体来解决此问题。