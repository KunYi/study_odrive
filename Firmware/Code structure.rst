# ODrive 代碼結構

## 啟動流程

### 啟動程序
- **文件位置**: `Board/v3/startup_stm32f405xx.s`
- **功能**: 設置初始堆疊指針、配置中斷向量表、跳轉到`SystemInit()`

### 系統初始化
- **函數**: `SystemInit()`
- **文件位置**: `Board/v3/Src/system_stm32f4xx.c`
- **功能**: 重置系統狀態、配置向量表、設置系統時鐘

### 啟動前檢查
- **函數**: `early_start_checks()`
- **文件位置**: `MotorControl/main.cpp`
- **功能**: 檢查是否需要進入DFU模式（當AUX L/H 拉高或標誌位被設置時）

### 主函數
- **函數**: `main()`
- **文件位置**: `MotorControl/main.cpp`
- **功能**:
  - 讀取晶片UID生成USB設備描述符的序列號
  - 調用`system_init()`進行系統初始化
  - 透過`config_manager`載入系統設定
  - 調用`board_init()`進行板級與外設初始化
  - 初始化外設和控制系統
  - 設定GPIO 模式
  - 建立FreeRTOS 執行環境與創建預設執行Task, 執行 rtos_main()
  - 在rtos_main 進行USB/ADC/PWM/UART/Communication/AXES 相關初始化，並開啟axis 的task thread 後結束 rtos_main 自己
  - 系統透過 axis status machine 運作，執行Motors 的初始化與定位相關

## 系統初始化

### 板級初始化
- **函數**: `system_init()`
- **文件位置**: `Board/v3/board.cpp`
- **功能**:
  - 初始化STM32 HAL庫
  - 檢查板卡硬件版本
  - 配置系統時鐘和引腳狀態

## 外設初始化

### GPIO初始化
- **文件位置**: `Board/v3/Src/gpio.c`
- **相關函數**: `MX_GPIO_Init()`

### ADC初始化
- **文件位置**: `Board/v3/Src/adc.c`
- **相關函數**: `MX_ADC1_Init()`, `MX_ADC2_Init()`, `MX_ADC3_Init()`
- **功能**: 用於電流檢測、電壓檢測等

### 定時器初始化
- **文件位置**: `Board/v3/Src/tim.c`
- **相關函數**: `MX_TIM1_Init()`, `MX_TIM8_Init()`
- **功能**: 用於PWM生成控制電機

### SPI初始化
- **文件位置**: `Board/v3/Src/spi.c`
- **相關函數**: `MX_SPI3_Init()`
- **功能**: 用於編碼器接口

### CAN總線初始化
- **文件位置**: `Board/v3/Src/can.c`
- **相關函數**: `MX_CAN1_Init()`

### USB初始化
- **文件位置**: `Board/v3/Src/usbd_conf.c`, `Board/v3/Src/usbd_cdc_if.c`
- **相關函數**: `MX_USB_DEVICE_Init()`

## FOC電機控制

### 電機控制類
- **文件位置**: `MotorControl/motor.cpp`, `MotorControl/motor.hpp`
- **主要類**: `Motor`
- **功能**: 電機參數初始化、電流控制、速度控制、位置控制

### 編碼器接口
- **文件位置**: `MotorControl/encoder.cpp`, `MotorControl/encoder.hpp`
- **主要類**: `Encoder`
- **功能**: 編碼器初始化、位置讀取、速度計算

### FOC控制算法
- **文件位置**: `MotorControl/foc.cpp`, `MotorControl/foc.hpp`
- **主要函數**: `FOC_current()`, `FOC_voltage()`
- **功能**: 實現Field Oriented Control算法

### 軸控制
- **文件位置**: `MotorControl/axis.cpp`, `MotorControl/axis.hpp`
- **主要類**: `Axis`
- **功能**: 協調電機和編碼器，實現位置/速度閉環控制

### 軌跡規劃
- **文件位置**: `MotorControl/trajectory.cpp`, `MotorControl/trajectory.hpp`
- **主要類**: `Trajectory`
- **功能**: 實現平滑的運動軌跡規劃

## 通信接口

### USB通信
- **文件位置**: `MotorControl/usb_communication.cpp`, `MotorControl/usb_communication.hpp`
- **功能**: 處理USB通信，解析命令

### CAN總線通信
- **文件位置**: `MotorControl/can_communication.cpp`, `MotorControl/can_communication.hpp`
- **功能**: 處理CAN總線通信

### 命令處理
- **文件位置**: `MotorControl/commands.cpp`, `MotorControl/commands.hpp`
- **功能**: 解析和執行來自通信接口的命令

## 主循環與狀態機

### 主循環
- **文件位置**: `MotorControl/main.cpp`
- **功能**: 調用狀態機更新、處理通信、執行控制算法

### 狀態機
- **文件位置**: `MotorControl/state_machine.cpp`, `MotorControl/state_machine.hpp`
- **主要類**: `StateMachine`
- **功能**: 管理電機控制器的不同狀態（校準、閉環控制、錯誤狀態等）

## 安全檢查與錯誤處理

### 錯誤處理
- **文件位置**: `MotorControl/error.cpp`, `MotorControl/error.hpp`
- **功能**: 定義錯誤類型、處理錯誤狀態

### 安全監控
- **文件位置**: `MotorControl/safety.cpp`, `MotorControl/safety.hpp`
- **功能**: 監控過流、過壓、過溫等異常情況
