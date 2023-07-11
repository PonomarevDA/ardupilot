# Cyphal ESC Driver

Reference: https://opencyphal.org/specification/Cyphal_Specification.pdf

## Parameters

Please, consider the following paramers to enable scripting on CAN1:

||||
|-|-|-|
| SCR_ENABLE      | 1 | 1 means scripting is enabled, 0 means disabled
| CAN_D1_PROTOCOL | 10 | 10 means scripting
| CAN_P1_BITRATE  | 100000 | Default baudrate for most of the applications
| CAN_P1_DRIVER   | First driver
| CAN_D2_PROTOCOL | UAVCAN | Cyphal can coexist with DroneCAN on second bus
| CAN_P2_BITRATE  | 100000
| CAN_P2_DRIVER   | Second driver

> Here, the second CAN is configured as DroneCAN. It is not necessary, but it is used as an illustration that Cyphal and DroneCAN can coexist on different buses.

The Cyphal inroduces the following parameters:

||||
|-|-|-|
| CYP_ENABLE      | 1 | 1 means Cyphal is enabled, 0 means disabled
| CYP_NODE_ID     | 1 | 1 means Cyphal is enabled, 0 means disabled
| CYP_FB          | | ESC Feedback port id. Enabled if less then 8191, otherwise disabled.
| CYP_RD          | | Readiness port id. Enabled if less then 8191, otherwise disabled.
| CYP_SP          | | Setpoint port id. Enabled if less then 8191, otherwise disabled.
| CYP_TESTS       | 0 | If set to 1, unit and performance tests will be runned at the beginning of the application.
