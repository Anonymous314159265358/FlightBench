# Bridge Note

## Sbus Bridge from ETH-RPG

### sbus_bridge_node.cpp
sbus_bridge_node -> sbus_bridge

### sbus_bridge.cpp

SBusBridge

1. `bool loadParameters()` -> `thrust_mapping.cpp`
2. check whether disable_thrust_mapping_
3. declare publisher
   1. low_level_feedback
   2. received_sbus_message
4. declare subscriber
   1. sbus_bridge/arm
   2. control_command
   3. battery_voltage
5. create low_level_feedback_pub_timer_: pub low_level_feedback by * Hz
6. setup sbus serial port (x)
7. start watchdog thread

~SBusBridge

#### watchdog
长时间收不到控制指令时候的安全处理

#### sbus_bridge/arm
如果data为true，arm the bridge
否则，disarm，如果当前bridge_state是ARMING or AUTONOMOUS_FLIGHT， 改成OFF

#### control_command
1. 检查controller是否armed
2. 如果bridge not armed & controller armed & not in RC_FLIGHT, throw a warning
3. 把control command转成sbus message
   - 在这一步里做thrust mapping
   - 仅支持attitude control和bodyrate control
4. limit max and min
