# 🤖 Maze Solver Robot (Micromouse)

> **Dự án cuối kì - Nhập môn Nhúng (Embedded Systems)**  
> Đại học Công nghệ - ĐHQG Hà Nội

## 📋 Mục lục
- [Giới thiệu](#giới-thiệu)
- [Tính năng chính](#tính-năng-chính)
- [Kiến trúc phần cứng](#kiến-trúc-phần-cứng)
- [Kiến trúc phần mềm](#kiến-trúc-phần-mềm)
- [Hướng dẫn cài đặt](#hướng-dẫn-cài-đặt)
- [Hướng dẫn sử dụng](#hướng-dẫn-sử-dụng)
- [Cấu trúc project](#cấu-trúc-project)
- [Troubleshooting](#troubleshooting)

---

## 🎯 Giới thiệu

**Maze Solver Robot** là một robot tự hành nhỏ được thiết kế để tự động điều hướng trong một mê cung. Robot sử dụng:
- **Cảm biến hồng ngoại (IR)** để phát hiện tường
- **IMU 6-trục (MPU6500)** để định hướng chính xác
- **Encoder** để đo vị trí bánh xe
- **PID Controller** để điều khiển chuyển động mượt mà

### 🎓 Mục tiêu dự án
- Thiết kế và cài đặt hệ thống nhúng trên **STM32F4**
- Áp dụng các kỹ thuật **điều khiển tự động (PID control)**
- Phát triển **firmware** có tính modular và dễ mở rộng
- Tích hợp nhiều cảm biến và thực hiện **fusion algorithm**

---

## ✨ Tính năng chính

| Tính năng | Mô tả |
|----------|--------|
| **Định hướng tự động** | Sử dụng IMU + Encoder fusion để duy trì hướng chính xác |
| **Phát hiện tường** | 4 cảm biến IR phát hiện tường xung quanh |
| **Điều khiển PID** | PID control cho vận tốc và hướng đi |
| **Quản lý năng lượng** | Theo dõi điện áp pin realtime |
| **Chế độ test** | Multiple test modes (calibration, forward step, rotation step) |
| **Giao tiếp debug** | UART 115200 baud cho debugging và hiệu chuẩn |
| **Phản hồi audio** | Buzzer phát âm thanh để phản hồi người dùng |
| **Chỉ báo trạng thái** | 4 LED hiển thị chế độ hoạt động |
| **Motion profiling** | S-curve acceleration/deceleration profiles |

---

## ⚙️ Kiến trúc phần cứng

### Thông số kỹ thuật chính

| Thành phần | Chi tiết |
|-----------|---------|
| **MCU** | STM32F4 (ARM Cortex-M4), 168 MHz, 512KB Flash, 192KB RAM |
| **Cảm biến IMU** | MPU6500 (6-axis): Accelerometer + Gyroscope via SPI |
| **Cảm biến vị trí** | 2x Incremental Encoders (bánh xe trái/phải) |
| **Cảm biến IR** | 4x Analog Reflectance Sensors (phía trước-trái, trái, phải, phải trước) |
| **Động cơ** | 2x DC Motors điều khiển bằng PWM (TIM1) |
| **Nguồn điện** | Pin/Battery via ADC monitoring |
| **Xuất ra** | 4x LEDs, 1x Buzzer, UART debug |
| **Tần số Interrupt** | 1 kHz (SysTick) |

### Sơ đồ khối chức năng

```
┌─────────────────────────────────────┐
│     STM32F4 (168 MHz)              │
│  ┌───────────────────────────────┐  │
│  │  SysTick (1 kHz Interrupt)   │  │
│  │  - Update Sensors            │  │
│  │  - Calculate PID             │  │
│  │  - Update Motors             │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘
    ↓              ↓              ↓
  INPUT        CONTROL        OUTPUT
  ─────        ───────        ──────
  • MPU6500    • PID Ctrl     • Motor PWM
  • Encoders   • Motion Prof  • LEDs
  • IR Sensors • Fusion Algo  • Buzzer
  • ADC (Bat)
```

### Kết nối GPIO

| Chức năng | Pin | Ghi chú |
|----------|-----|--------|
| **Motor PWM L** | PA8 (TIM1_CH1) | PWM tần số ~60kHz |
| **Motor PWM R** | PA11 (TIM1_CH4) | PWM tần số ~60kHz |
| **Motor Dir L** | PA1 | Output |
| **Motor Dir R** | PB9 | Output |
| **LED 1** | PC13 | Mode 0 indicator |
| **LED 2** | PA7 | Mode 1 indicator |
| **LED 3** | PB0 | Mode 2 indicator |
| **LED 4** | PB1 | Mode 3 indicator |
| **Buzzer** | PB4 (TIM3_CH1) | PWM 1-2kHz |
| **Encoder L** | PB6 | GPIO input (pulse) |
| **Encoder R** | PB7 | GPIO input (pulse) |
| **IR Sensor R** | PA2 (ADC) | Cảm biến bên phải |
| **IR Sensor FR** | PA3 (ADC) | Cảm biến phía trước-phải |
| **IR Sensor FL** | PA4 (ADC) | Cảm biến phía trước-trái |
| **IR Sensor L** | PA5 (ADC) | Cảm biến bên trái |
| **Battery Monitor** | PA6 (ADC) | Theo dõi điện áp |
| **UART TX** | PA9 (UART1) | Debug console |
| **UART RX** | PA10 (UART1) | Debug input |
| **SPI SCK** | PB3 (SPI2) | MPU6500 clock |
| **SPI MOSI** | PB5 (SPI2) | MPU6500 data out |
| **SPI MISO** | PB4 (SPI2) | MPU6500 data in |
| **SPI CS** | PA4 | MPU6500 chip select |

---

## 🏗️ Kiến trúc phần mềm

### Cấu trúc module

```
main.cpp
├── SystemClock_Config()      // Clock 168MHz
├── main()
│   ├── Hardware Initialization
│   │   ├── mpu.begin()
│   │   ├── encoders.begin()
│   │   ├── motors.begin()
│   │   ├── adc.begin()
│   │   ├── systick.begin()
│   │   └── buzzer.begin()
│   │
│   └── Main Event Loop
│       ├── Read MPU
│       ├── Check Serial Input
│       ├── Handle Button Input
│       └── Execute Motion Profile
│
├── SysTick_Handler() [1 kHz]
│   ├── encoders.update()
│   ├── motion.update()
│   ├── motors.updateControl()
│   └── battery.update() [every 100ms]
```

### Các module chính

#### 1. **config.h** - Cấu hình toàn cệ
```cpp
// Robot Parameters
const float WheelDiameter = 34.0f;      // mm
const float CountPerRev = 270.0f;       // pulses/rev
const float MouseRadius = 76.935f;      // mm

// Motor PID Parameters
const float Fwd_Km = 590.3f;            // Motor gain
const float Fwd_Tm = 0.272f;            // Motor time constant
const float Fwd_Zeta = 1.0f;            // Damping ratio

// Control Loop
const float LOOP_FREQUENCY = 1000.0f;   // Hz (SysTick)
const float LOOP_INTERVAL = 0.001f;     // seconds
```

#### 2. **Motors** - Điều khiển động cơ
- PWM control (0-255)
- PID position control
- PID angle control
- Feed-forward compensation
- Dynamic controller selection (low/high speed)

#### 3. **Motion** - Quản lý chuyển động
- **Profile-based motion**: Controlled acceleration/deceleration
- **Position control**: Maintain target distance
- **Angle control**: Maintain target heading
- **Motion states**: ACCELERATING → CONSTANT → BRAKING → FINISHED

#### 4. **Encoders** - Đo vị trí bánh xe
- Incremental pulse counting
- Distance calculation (mm)
- Velocity calculation (mm/s)
- Rotation angle calculation

#### 5. **MPU6500** - Cảm biến IMU
- 6-axis sensor (accel + gyro)
- SPI communication
- Calibration with LED feedback
- Angle fusion (encoder + gyro + accel)

#### 6. **ADC_Sensor** - Cảm biến đầu vào
- 4x IR reflectance sensors
- Battery voltage monitoring
- LED on/off for IR measurement
- Peak-ambient method

#### 7. **Systick** - Timer ngắt chính
- 1 kHz interrupt
- Central update hub for all sensors and controllers

### Thuật toán điều khiển

#### PID Control
$$u(t) = K_p e(t) + K_d \frac{de(t)}{dt} + K_i \int_0^t e(\tau)d\tau$$

**Tham số điều khiển:**
| Mode | Kp | Kd | Ki | Ứng dụng |
|------|----|----|----|---------| 
| Fwd Low Speed | 1.0 | 2.0 | - | Tốc độ bình thường |
| Fwd High Speed | 2.5 | 5.0 | - | Tốc độ cao (>1000 mm/s) |
| Rot Normal | 0.4 | 10.0 | - | Quay bình thường |
| Rot High Speed | - | - | - | Quay nhanh |

#### Heading Fusion (Complementary Filter)
$$\theta_{estimate} = 0.98 \times \theta_{encoder} + 0.02 \times \theta_{MPU}$$

**Ưu điểm:**
- Encoder: Chính xác ngắn hạn, drift dài hạn
- MPU: Absolute, nhưng noise cao
- Fusion: Kết hợp tốt của cả hai

#### Motion Profiling (S-curve)
```
Velocity
   ↑
   │     ┌──────────┐
   │    ╱            ╲
   │  ╱              ╲
   └─╱────────────────╲──→ Time
     Acceleration    Deceleration
```

**Trạng thái:**
1. **Accelerating**: v += acc * dt
2. **Constant**: v = target_speed
3. **Braking**: v -= acc * dt
4. **Finished**: Stop

---

## 🔧 Hướng dẫn cài đặt

### Yêu cầu phần cứng
- **Board**: STM32F4 Discovery hoặc tương tự
- **Programmer**: ST-Link v2 hoặc tương tự
- **Cable**: USB để debug/programming

### Yêu cầu phần mềm
- **IDE**: STM32CubeIDE hoặc Keil uVision
- **Compiler**: ARM GCC (trong CubeIDE)
- **Tools**: STM32CubeMX (optional, để sinh code)
- **Driver**: ST-Link driver

### Các bước cài đặt

1. **Clone/Download project**
   ```bash
   git clone <project-repo>
   cd Project\ cuối\ kì\ nhập\ môn\ nhúng
   ```

2. **Mở trong STM32CubeIDE**
   - File → Open Projects from File System
   - Chọn thư mục project
   - Build project (Ctrl+B)

3. **Lập trình vào board**
   - Kết nối ST-Link
   - Right click project → Run As → STM32 C/C++ Application
   - Hoặc: Run → Run Configurations → Debug

4. **Thiết lập baud rate UART**
   ```
   Baud Rate: 115200
   Data Bits: 8
   Stop Bits: 1
   Parity: None
   ```

### Kiểm tra sau cài đặt
- [ ] LED phía PC13 sáng lên (khởi động)
- [ ] Nghe 2 tiếng beep (khởi động thành công)
- [ ] UART hiển thị: "Calibrate MPU..." khi gửi 'c'
- [ ] Pin voltage được in ra UART

---

## 📖 Hướng dẫn sử dụng

### Các chế độ hoạt động (Mode)

| Mode | Chức năng | Lệnh |
|------|----------|------|
| 0 | Calibrate Bias | Xoay encoder → nhấn nút |
| 1 | Forward Step Test | Quay encoder → nhấn nút |
| 2 | Rotation Step Test | Quay encoder → nhấn nút |
| 3-15 | Maze Solving Modes | ... (dùng cho giải đấu) |

### Lệnh UART

| Lệnh | Tác dụng |
|------|---------|
| `c` | Calibrate MPU (đặt trên bàn phẳng + 's' khi sẵn sàng) |
| `s` | Start calibration |
| `d` | Debug mode (in sensor values) |

### Thủ tục bắt đầu

```
1. Cấp nguồn cho robot
2. Nghe 2 tiếng beep → Robot sẵn sàng
3. Xoay encoder để chọn mode (LED sáng tương ứng)
4. Nhấn nút → Robot phát âm thanh xác nhận
5. Nhấn nút lần 2 → Bắt đầu thực thi
6. Nhấn nút để dừng/trở về chế độ chọn
```

### Hiệu chuẩn (Calibration)

#### Hiệu chuẩn MPU6500
```
1. Đặt robot trên bàn phẳng ngang
2. Gửi lệnh 'c' qua UART
3. Robot sẽ sáng LED liên tục trong 2 giây
4. Gửi 's' để xác nhận calibration
5. Robot sẽ lưu bias vào flash
```

#### Kiểm tra kết quả
```
// Mở Serial Monitor (115200 baud)
- Đặt robot, không di chuyển
- Giá trị gyro phải gần 0 độ/s
- Giá trị gia tốc Z ≈ 9.8 m/s²
```

---

## 📁 Cấu trúc project

```
Project cuối kì nhập môn nhúng/
│
├── Inc/                          # Header files
│   ├── main.h                    # Main header
│   ├── config.h                  # Global configuration
│   ├── motors.h                  # Motor control class
│   ├── encoders.h                # Encoder reading
│   ├── motion.h                  # Motion planning
│   ├── profile.h                 # Motion profiles (S-curve)
│   ├── mpu_spi.h                 # IMU (MPU6500) interface
│   ├── adc.h                     # ADC sensors
│   ├── uart.h                    # UART communication
│   ├── switches.h                # Buzzer & LED control
│   ├── batery.h                  # Battery monitoring
│   ├── systick.h                 # System tick timer
│   ├── SysID.h                   # System identification
│   └── stm32f4xx_hal_conf.h      # HAL configuration
│
├── Src/                          # Source files
│   ├── main.cpp                  # Main program
│   ├── stm32f4xx_it.c            # Interrupt handlers
│   ├── stm32f4xx_hal_msp.c       # MSP initialization
│   └── system_stm32f4xx.c        # System init
│
├── README.md                     # This file
└── CODE_EXPLANATION.txt          # Code documentation
```

### File chính

| File | Dòng code | Chức năng |
|------|-----------|----------|
| main.cpp | ~200 | Vòng lặp chính, quản lý states |
| motors.h | ~150 | PID control 2 động cơ |
| motion.h | ~100 | Quản lý chuyển động (forward/rotation) |
| profile.h | ~180 | Motion profiling (accel/decel) |
| encoders.h | ~80 | Đo vị trí bánh xe |
| mpu_spi.h | ~200 | Giao tiếp IMU qua SPI |
| adc.h | ~120 | Đọc 4 IR sensors + battery |

---

## 🐛 Troubleshooting

### Robot không chuyển động
**Nguyên nhân & Giải pháp:**
1. **Kiểm tra battery**
   - Đo điện áp qua UART
   - Phải ≥ 6V để vận hành
   
2. **Kiểm tra motor**
   - Xoay bánh bằng tay, có cảm thấy lực không?
   - Kết nối dây có tốt không?
   
3. **Kiểm tra encoder**
   - Gửi 'c' để calibrate (kiểm tra output)

### Robot không quay chính xác
**Nguyên nhân & Giải pháp:**
1. **Hiệu chuẩn IMU**
   - Calibrate MPU6500 (xem phần Calibration)
   
2. **Điều chỉnh tham số PID**
   - Tăng Rot_KP để quay nhanh hơn
   - Tăng Rot_KD để giảm overshoot

### LED không sáng
**Nguyên nhân & Giải pháp:**
1. **Kiểm tra GPIO**
   - Xác nhận pin (PC13, PA7, PB0, PB1)
   
2. **Kiểm tra phần cứng**
   - LED có bị mắt gẫy không?
   - Điện trở hạn dòng 330Ω có trong mạch không?

### UART không nhận dữ liệu
**Nguyên nhân & Giải pháp:**
1. **Kiểm tra baud rate**
   - Phải là 115200 baud
   
2. **Kiểm tra driver**
   - Cài đặt ST-Link driver
   
3. **Kiểm tra kết nối**
   - USB cable có tốt không?

### Robot đi lệch hướng
**Nguyên nhân & Giải pháp:**
1. **Kiểm tra encoder**
   - Cả 2 encoder có cùng CPR (270 pulses/rev)?
   - Bánh xe có bị chồi lệch không?
   
2. **Điều chỉnh bias motor**
   - Tăng BiasFF để tăng lực motor yếu
   
3. **Calibrate IMU**
   - Đặt trên mặt phẳng tuyệt đối

---

## 📊 Kết quả thử nghiệm

### Performance Metrics

| Chỉ số | Giá trị | Ghi chú |
|--------|--------|--------|
| **Tốc độ tối đa** | 1000 mm/s | Có thể điều chỉnh |
| **Gia tốc** | 1000 mm/s² | S-curve profile |
| **Độ chính xác hướng** | ±2° | Sau calibration |
| **Tần số điều khiển** | 1000 Hz | SysTick |
| **Lỗi vị trí** | <5 mm | Trên 1000 mm chuyển động |

---

## 📚 Tài liệu tham khảo

### Datasheets
- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
- [MPU6500 Datasheet](https://www.invensense.com/wp-content/uploads/2015/02/MPU-6500-Datasheet.pdf)
- [STM32CubeIDE Documentation](https://www.st.com/en/development-tools/stm32cubeide.html)

### Tài liệu hữu ích
- STM32 HAL Library User Manual
- ARM Cortex-M4 Programmer's Manual
- Sensor Fusion Techniques (Kalman Filter, Complementary Filter)

### Video tutorial
- STM32F4 GPIO/Timer configuration
- PID Control Basics
- Sensor Fusion with IMU + Odometry

---

## 👥 Thông tin dự án

| Thông tin | Chi tiết |
|-----------|---------|
| **Tên dự án** | Maze Solver Robot (Micromouse) |
| **Môn học** | Nhập môn Nhúng (Embedded Systems) |
| **Khóa** | 2024-2025 |
| **Trường** | Đại học Công nghệ - ĐHQG Hà Nội |
| **Ngôn ngữ** | C/C++ (embedded) |
| **Platform** | STM32F4 |

---

## 📝 Ghi chú

### Hướng phát triển tương lai
- [ ] Thêm LCD display để hiển thị trạng thái
- [ ] Tích hợp Wi-Fi module (ESP8266) để điều khiển từ xa
- [ ] Kalman Filter cho fusion algorithm tốt hơn
- [ ] Pathfinding algorithm (A*, Dijkstra) cho maze solving
- [ ] Lưu trữ map mê cung và re-planning

### Bảo trì
- Kiểm tra battery mỗi tuần
- Calibrate sensor mỗi tháng
- Update firmware khi có fix/feature mới

---

**Cập nhật lần cuối: 25/05/2026**

---

## 📞 Hỗ trợ

Nếu gặp vấn đề, hãy:
1. Kiểm tra phần **Troubleshooting** ở trên
2. Xem file **CODE_EXPLANATION.txt** để hiểu thêm chi tiết
3. Liên hệ giảng viên hướng dẫn

**Happy coding! 🚀**
