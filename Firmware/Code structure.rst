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



# ODrive 韌體架構分析

## 第一章：系統初始化與啟動 (System Initialization and Startup)

韌體的啟動流程由 rtos_main 函數開始
這個函數負責初始化核心周邊、通訊介面和馬達相關組件。

* USB 裝置初始化：首先會呼叫 MX_USB_DEVICE_Init() 初始化 USB 通訊。
* ADC 初始化：start_general_purpose_adc() 用於啟動 ADC，可能用於溫度和使用者自定義量測。
* 通訊初始化：init_communication() 負責設定不同的通訊介面，例如 UART、CAN 等。

  * 原始碼中提到 uart_poll() 函數，暗示了 UART 通訊的輪詢機制。
  * ODriveCAN can_; 的宣告表示支援 CAN Bus 通訊。
  *  程式碼中也看到對 enable_uart_a、enable_uart_b、enable_uart_c 和 enable_can_a 等配置的檢查，這些配置項在 BoardConfig_t 結構體中定義。

* PWM 輸入初始化：pwm0_input.init() 初始化 PWM 輸入捕獲模組，可能用於接收外部 PWM 訊號。
* 絕對式編碼器 CS 腳位設定：程式碼中針對具有絕對式編碼器的軸進行 CS (Chip Select) 腳位的初始化。
* 閘極驅動器初始化：每個軸的馬達物件會調用 setup() 函數，這可能包含閘極驅動器 (如 Drv8301) 的初始化。
* 編碼器初始化：每個軸的編碼器物件也會調用 setup() 函數，啟動編碼器硬體介面 (例如 HAL_TIM_Encoder_Start)。
* 組件連接：程式碼中將不同組件的輸出和輸入連接起來，例如感測器無感測估測器 (SensorlessEstimator) 的輸出連接到控制器的輸入。
* ADC PWM 同步啟動：start_adc_pwm() 啟動 PWM 產生和 ADC 中斷/回調，這是實現 FOC 的關鍵步驟。
* 類比線程啟動：start_analog_thread() 啟動一個專門處理類比訊號的線程。
* 等待馬達就緒：啟動流程會等待一段時間 (最長 2 秒) 讓馬達相關組件就緒，例如電流感測器的校準收斂。
* 載入使用者配置：韌體會嘗試載入儲存的使用者配置，如果成功則應用配置，否則清除並應用預設配置。
* ConfigManager 類別負責配置的載入和儲存。
* 板卡特定周邊初始化：board_init() 函數用於初始化特定硬體平台的周邊。
* GPIO 初始化：根據 odrv.config_.gpio_modes 中的配置，初始化每個 GPIO 腳位的功能 (例如數位輸入/輸出、類比輸入、UART、CAN、PWM、編碼器等)。

第二章：軸 (Axis) 管理

* ODrive 的核心概念之一是 軸 (Axis)。
一個 ODrive 裝置可以有多個軸 (**在 AXIS_COUNT** 中定義)，每個軸通常對應一個馬達。
* 軸物件：Axis 類別封裝了控制一個馬達所需的所有組件和狀態。
每個 Axis 物件都包含了以下子物件：
* Encoder: 負責馬達位置和速度的感測
    * Controller: 實現馬達的控制邏輯，包括位置、速度和扭矩控制模式。
    * Motor: 代表實際的馬達硬體，負責 PWM 控制、電流感測、溫度保護等。
    * TrapezoidalTrajectory: 用於產生梯形速度曲線，實現平滑的運動規劃。
    * Endstop: 用於限制馬達的運動範圍。
    * MechanicalBrake: 用於控制機械煞車 (如果存在)。

軸狀態機：每個軸都有一個狀態機 (current_state_)，定義了軸的不同操作階段，例如啟動、校準、閉迴路控制、空閒等
。task_chain_ 則定義了軸狀態轉換的順序
。
•
執行緒管理：每個軸都在一個獨立的 FreeRTOS 執行緒中運行其狀態機迴圈 (run_state_machine_loop)
。start_thread() 函數負責創建這些執行緒。
•
步進/方向介面：Axis 類別支援步進/方向訊號輸入，用於接收外部運動指令
。step_cb() 函數處理步進訊號，並更新馬達的目標位置。
•
錯誤處理：每個軸都有一個 error_ 變數，用於記錄該軸及其子組件發生的錯誤
。do_checks() 函數會檢查軸及其子組件的錯誤狀態
。
•
看門狗：watchdog_feed() 和 watchdog_check() 函數用於實現看門狗機制，防止韌體因錯誤而停止響應
。
第三章：通訊介面 (Communication Interfaces)
•
ODrive 韌體支援多種通訊介面，用於與外部設備進行控制和數據交換。
•
UART (Universal Asynchronous Receiver/Transmitter):
◦
程式碼中可以看到 UART_HandleTypeDef* uart_b = &huart2; 的宣告，表示可能使用 STM32 的 USART2 作為 UART B
。
◦
BoardConfig_t 中包含 enable_uart_a、enable_uart_b、enable_uart_c 和對應的 uart_a_baudrate 等配置項
。
◦
ODriveIntf::StreamProtocolType 用於定義 UART 的通訊協定
。
◦
uart_poll() 函數暗示了 UART 的資料接收可能採用輪詢方式
。
•
CAN (Controller Area Network):
◦
ODriveCAN can_; 的宣告表示支援 CAN Bus 通訊
。
◦
BoardConfig_t 中包含 enable_can_a 配置項
。
◦
GPIO 配置中包含 ODriveIntf::GPIO_MODE_CAN_A 選項，並指定了相關的 GPIO 腳位 (CAN_R, CAN_D)
。
◦
程式碼中檢查 GPIO 模式是否與 CAN 模式一致
。
•
USB (Universal Serial Bus):
◦
MX_USB_DEVICE_Init() 用於初始化 USB 裝置功能
。
◦
SystemStats_t 結構體中包含與 USB 相關的統計資訊，例如最大堆疊使用量 (max_stack_usage_usb) 和優先級 (prio_usb)
。
◦
ODriveIntf::StreamProtocolType usb_cdc_protocol 用於定義 USB CDC (Communication Device Class) 的通訊協定
。
•
GPIO 作為通訊介面:
◦
GPIO 可以配置為多種通訊相關的功能，例如 UART 和 I2C 的替代功能 (AF, Alternate Function)
。
第四章：馬達控制 (Motor Control)
•
ODrive 主要採用 磁場導向控制 (Field-Oriented Control, FOC)
。FieldOrientedController 類別實現了 FOC 演算法。
•
電流感測：馬達電流透過 ADC 進行感測
。Motor::current_meas_cb() 函數處理電流感測的回調，並進行零點校準 (DC_calib_) 和資料更新。phase_current_from_adcval() 函數將 ADC 值轉換為相電流
。
•
電壓應用：FOC 控制器計算出所需的 α-β 參考系電壓 (final_v_alpha_, final_v_beta_)，然後透過 空間向量脈寬調變 (Space Vector Modulation, SVM) 將其轉換為三個 PWM 訊號的佔空比 (pwm_timings)
。Motor::pwm_update_cb() 函數在 PWM 更新事件發生時被調用，並應用新的 PWM 時序
。
•
控制模式：Controller 類別支援多種控制模式
:
◦
位置控制 (Position Control): 將馬達控制到指定的角度。
◦
速度控制 (Velocity Control): 將馬達控制到指定的轉速。
◦
扭矩控制 (Torque Control): 直接控制馬達輸出的扭矩。
•
電流控制迴路：在電流控制模式下，FOC 控制器使用 PI (比例積分) 控制器 來追蹤目標 d 軸和 q 軸電流 (Idq_setpoint_)
。PI 控制器的增益 (pi_gains_) 可以在電阻和電感測量後自動設定
.
•
電壓控制迴路：在電壓控制模式下，FOC 控制器直接輸出目標 d 軸和 q 軸電壓 (Vdq_setpoint_)
。
•
前饋控制：FOC 控制器支援多種前饋控制項，例如 bEMF 前饋 (bEMF_FF_enable) 和 RI、wL*I 前饋 (R_wL_FF_enable)，以提高控制性能
。
•
扭矩限制和速度限制：控制器可以設定扭矩上限 (torque_limit) 和速度上限 (vel_limit)，以保護馬達和系統
。
•
反饋來源：控制器可以選擇不同的位置和速度反饋來源，例如編碼器或無感測估測器
.
•
開環控制：OpenLoopController 類別實現了開環控制模式，在沒有位置或速度反饋的情況下直接控制馬達的相位和電壓/電流
。這通常用於馬達參數校準階段。
第五章：位置和速度估測 (Position and Velocity Estimation)
•
韌體使用兩種主要方法來獲取馬達的位置和速度資訊：
◦
編碼器 (Encoder)：Encoder 類別處理來自各種編碼器的訊號
。支援增量式編碼器透過計數器 (timer_) 測量相對位置變化。也支援霍爾感測器，透過解碼霍爾訊號 (decode_hall_samples()) 獲取粗略的位置和速度資訊。對於絕對式編碼器，韌體透過 SPI 通訊 (abs_spi_start_transaction(), abs_spi_cb()) 讀取絕對位置。對於正餘弦編碼器，韌體讀取類比訊號 (sincos_sample_s_, sincos_sample_c_) 並計算相位。編碼器物件還實現了 鎖相迴路 (Phase-Locked Loop, PLL) (pll_)，用於濾除編碼器雜訊並估計速度
。
◦
無感測估測器 (Sensorless Estimator)：SensorlessEstimator 類別在沒有編碼器的情況下，基於馬達的電壓和電流資訊來估計轉子位置和速度
。它使用基於磁鏈觀測器的演算法，通過測量到的相電流和施加的電壓來估計磁鏈狀態 (flux_state_)，進而推斷轉子位置和速度。
•
模式切換：韌體可以根據配置在編碼器模式和無感測模式之間切換
。
第六章：校準程序 (Calibration Procedures)
•
為了獲得最佳的控制性能，ODrive 韌體包含多個校準程序：
◦
電流感測器零點校準 (DC Calibration)：Motor::dc_calib_cb() 函數用於校準電流感測器的偏移量 (DC_calib_)
。
◦
相電阻測量 (Phase Resistance Measurement)：Motor::measure_phase_resistance() 函數透過施加一個已知的測試電流並測量電壓來估計馬達的相電阻 (config_.phase_resistance)
。
◦
相電感測量 (Phase Inductance Measurement)：Motor::measure_phase_inductance() 函數透過快速切換輸出電壓並觀察電流漣波來估計馬達的相電感 (config_.phase_inductance)
。
◦
編碼器偏移校準 (Encoder Offset Calibration)：Encoder::run_offset_calibration() 函數用於確定編碼器計數和馬達電氣相位的對應關係 (config_.phase_offset, config_.phase_offset_float)
。這通常涉及在開環模式下旋轉馬達並採集編碼器讀數。
◦
霍爾感測器極性校準 (Hall Polarity Calibration)：Encoder::run_hall_polarity_calibration() 函數用於確定霍爾感測器的極性 (config_.hall_polarity)
。
◦
霍爾感測器相位校準 (Hall Phase Calibration)：Encoder::run_hall_phase_calibration() 函數用於校準霍爾感測器訊號和馬達電氣相位的對應關係 (config_.hall_edge_phcnt)
。
◦
方向尋找 (Direction Find)：Encoder::run_direction_find() 函數用於確定編碼器的方向 (config_.direction)
。
◦
反齒槽效應校準 (Anticogging Calibration)：Controller::anticogging_calibration() 函數用於測量並補償由馬達磁極齒槽效應引起的扭矩波動 (config_.anticogging.cogging_map)
。
第七章：配置管理 (Configuration Management)
•
ODrive 韌體使用 ConfigManager 類別來管理配置參數的儲存和載入
。
•
配置結構體：主要的配置參數儲存在 ODrive::config_ (BoardConfig_t) 和每個軸的 Axis::config_、Controller::config_、Motor::config_、Encoder::config_ 等結構體中
。
•
預設配置：程式碼中可以看到許多預設配置值的定義 (例如 DEFAULT_BRAKE_RESISTANCE, DEFAULT_MIN_DC_VOLTAGE, DEFAULT_GPIO_MODES)
。
•
載入配置：在啟動時，韌體會嘗試從非揮發性記憶體 (例如 Flash) 載入使用者配置。如果載入成功，則應用這些配置
(config_read_all(), config_apply_all()).
•
儲存配置：使用者可以透過通訊介面將目前的配置儲存到非揮發性記憶體 (config_write_all(), config_manager.start_store(), config_manager.finish_store()).
•
清除配置：韌體也提供清除所有使用者配置並恢復到預設值的機制 (config_clear_all()).
•
GPIO 模式配置：BoardConfig_t::gpio_modes 陣列定義了每個 GPIO 腳位的功能模式
。在啟動時，韌體會根據這些配置初始化 GPIO 腳位
。
第八章：錯誤處理與安全 (Error Handling and Safety)
•
ODrive 韌體具有完善的錯誤處理機制。
•
錯誤碼：每個主要組件 (ODrive 本身、Axis、Motor、Encoder、Controller、SensorlessEstimator) 都有一個 error_ 變數，用於記錄發生的錯誤
。錯誤碼通常是枚舉類型 (例如 ODrive::Error, Axis::Error, Motor::Error)。
•
設定錯誤：組件可以使用 set_error() 函數設定其錯誤狀態
。
•
檢查錯誤：Axis::do_checks() 函數會檢查軸及其子組件的錯誤狀態
。ODrive::any_error() 函數檢查整個系統是否存在任何錯誤
。
•
解除武裝 (Disarming)：在檢測到嚴重錯誤時，韌體會將馬達解除武裝 (Motor::disarm(), ODrive::disarm_with_error())，停止 PWM 輸出，以防止硬體損壞
。
•
煞車電阻控制 (Brake Resistor Control)：韌體支援使用煞車電阻來吸收再生能量，防止直流母線過壓
。brake_resistor_armed_, brake_resistor_saturated_, brake_resistor_current_ 等變數用於追蹤煞車電阻的狀態。update_brake_current() 函數計算並設定煞車電阻的佔空比。
•
電流限制：Motor::effective_current_lim() 函數計算有效的電流限制
。在 FOC 控制器中，電流會被限制在安全範圍內
。
•
溫度保護：OnboardThermistorCurrentLimiter 和 OffboardThermistorCurrentLimiter 類別用於監控 FET 和馬達的溫度，並在溫度過高時限制電流或觸發錯誤
。
•
看門狗計時器 (Watchdog Timer)：看門狗用於檢測韌體是否停止運行，並在超時時重置系統
。
第九章：即時作業系統 (Real-Time Operating System, RTOS)
•
ODrive 韌體基於 FreeRTOS 運行
。
•
執行緒 (Tasks)：系統和每個軸的功能都運行在不同的 FreeRTOS 執行緒中，實現並行處理
.
•
優先級 (Priorities)：每個執行緒可以設定不同的優先級 (例如 thread_priority_, osPriorityHigh)，以確保關鍵任務 (例如控制迴路) 能夠及時執行
.
•
訊號量 (Signals)：osSignalWait() 和 osSignalSet() 等 FreeRTOS API 可能用於執行緒間的同步和通訊 (例如 Axis::wait_for_control_iteration() 使用訊號量等待控制迴路完成)
.
•
互斥鎖 (Mutexes) 和臨界區段 (Critical Sections)：CRITICAL_SECTION() 宏用於保護共享資源免受多個執行緒同時存取的影響，防止競爭條件
.
•
堆疊大小 (Stack Size)：每個執行緒都有分配的堆疊空間 (stack_size_)，用於儲存局部變數和函數調用資訊
.
第十章：硬體抽象層 (Hardware Abstraction Layer, HAL)
•
ODrive 韌體廣泛使用 STM32 HAL (Hardware Abstraction Layer) 庫來與底層的 STM32 微控制器硬體進行互動
。
•
HAL 提供了對微控制器各種周邊 (例如 ADC、TIM、SPI、UART、GPIO) 的標準化 API，使得韌體開發人員可以更方便地存取和控制硬體，而無需深入了解底層硬體細節。
•
例如，HAL_ADC_Start_DMA(), HAL_TIM_Encoder_Start(), HAL_GPIO_Init(), HAL_SPI_TxRxCpltCallback() 等函數都是 HAL 庫提供的 API
.
•
韌體中定義了許多指向 HAL 結構體的指標 (例如 TIM_HandleTypeDef* timer_, ADC_HandleTypeDef hadc1, SPI_HandleTypeDef hspi3)，用於操作特定的硬體周邊
