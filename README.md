# tutrc_stm32_template

TUTRC向け STM32G474RE (NUCLEO-G474RE) テンプレートプロジェクト。[halx](https://github.com/eyr1n/halx) ライブラリを使い、DCモーター3基をPS3コントローラで駆動する。

## ハードウェア構成

- MCU: STM32G474RET6 (NUCLEO-G474RE)
- DCモーター x3 (エンコーダ付き、`Motor1`〜`Motor3`)
  - motor1: PWM=TIM15_CH1, エンコーダ=TIM1
  - motor2: PWM=TIM5_CH1, エンコーダ=TIM3
  - motor3: PWM=TIM2_CH4, エンコーダ=TIM4
- PS3コントローラ受信機 (UART5経由)
- ST-LINK仮想COMポート (LPUART1)

## 構成

- `app/main.cpp` — エントリーポイント(`app_main`)。`TIM6`の100Hz周期割り込み内でPI速度制御を行い、メインループでPS3スティック入力をキネマティクス変換して目標速度をセットする。
- `app/motor.hpp` — PWM出力・enableピン・エンコーダをまとめたモーター1個分のラッパー(`Motor`)。
- `app/pid_controller.hpp` — 位置型PIDコントローラ(`PIDController`/`PIDParameters`)。
- `app/ps3.hpp` — PS3コントローラ受信機のUARTプロトコル実装(`PS3`)。
- `app/bno055.hpp` — Bosch BNO055 9軸IMUのI2Cドライバ(`BNO055`)。
- `app/feetech_servo.hpp` — Feetech製シリアルサーボのドライバ(`FeetechServo`)。

各クラスの詳しい使い方はドキュメント内のサンプルコードを参照。
