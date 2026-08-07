#include <atomic>
#include <cmath>
#include <cstdio>
#include <numbers>

#include <halx/core.hpp>
#include <halx/driver/gpio.hpp>
#include <halx/driver/uart_dma.hpp>
#include <halx/driver/uart_it.hpp>
#include <halx/peripheral.hpp>

#include "main.h"
#include "motor.hpp"
#include "pid_controller.hpp"
#include "ps3.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart5;

using halx::driver::GPIO;
using halx::driver::UART_DMA;
using halx::driver::UART_IT;
using halx::peripheral::ST_TIM;

// TIM6周期割り込みの周期(100Hz)。速度制御・エンコーダ更新はこの周期で行う。
constexpr float CONTROL_DT = 0.01f;

// 3輪オムニホイールの取り付け角度(ロボット正面基準)。
constexpr float WHEEL_THETA_1 = std::numbers::pi / 2.0f;
constexpr float WHEEL_THETA_2 = std::numbers::pi * 11.0f / 6.0f;
constexpr float WHEEL_THETA_3 = std::numbers::pi * 7.0f / 6.0f;

// motor1で調整したゲイン。motor2/3も同じ個体差の範囲とみなして流用している。
constexpr PIDParameters WHEEL_PID_PARAMS{
    .kp = 0.01f,
    .ki = 0.7f,
    .kd = 0.0f,
    .output_upper_limit = 0.7f,
    .integral_upper_limit = 1.0f,
};

GPIO motor1_pin(Motor1_GPIO_Port, Motor1_Pin);
GPIO motor2_pin(Motor2_GPIO_Port, Motor2_Pin);
GPIO motor3_pin(Motor3_GPIO_Port, Motor3_Pin);

UART_IT<&hlpuart1> lpuart1;

static uint8_t tx_buf[64];
static uint8_t rx_buf[64];
UART_DMA<&huart5> uart5(tx_buf, sizeof(tx_buf), rx_buf, sizeof(rx_buf));

Motor<&htim15, &htim1> motor1(TIM_CHANNEL_1, motor1_pin, 2048, 2.0f,
                              CONTROL_DT);
Motor<&htim5, &htim3> motor2(TIM_CHANNEL_1, motor2_pin, 2048, 2.0f, CONTROL_DT);
Motor<&htim2, &htim4> motor3(TIM_CHANNEL_4, motor3_pin, 2048, 2.0f, CONTROL_DT);

PS3 ps3(uart5);

PIDController motor1_pid(WHEEL_PID_PARAMS, CONTROL_DT);
PIDController motor2_pid(WHEEL_PID_PARAMS, CONTROL_DT);
PIDController motor3_pid(WHEEL_PID_PARAMS, CONTROL_DT);

// メインループ(PS3読み取り)からセットし、timer_callback(割り込み)から読む
// 目標回転速度。
std::atomic<float> motor1_target_rps = 0.0f;
std::atomic<float> motor2_target_rps = 0.0f;
std::atomic<float> motor3_target_rps = 0.0f;

void timer_callback(void *);

extern "C" void app_main() {
  halx::driver::enable_stdout(lpuart1);
  lpuart1.start();
  uart5.start();

  motor1.start();
  motor2.start();
  motor3.start();

  ST_TIM<&htim6>::register_period_elapsed_callback(timer_callback, nullptr);
  ST_TIM<&htim6>::start_base_it();

  while (true) {
    ps3.update();

    // 左スティックの並進速度を、各ホイールの接線方向速度に変換する。
    float vx = 2.0f * ps3.get_axis(PS3Axis::LEFT_X);
    float vy = -2.0f * ps3.get_axis(PS3Axis::LEFT_Y);

    motor1_target_rps =
        -vx * std::sin(WHEEL_THETA_1) + vy * std::cos(WHEEL_THETA_1);
    motor2_target_rps =
        -vx * std::sin(WHEEL_THETA_2) + vy * std::cos(WHEEL_THETA_2);
    motor3_target_rps =
        -vx * std::sin(WHEEL_THETA_3) + vy * std::cos(WHEEL_THETA_3);

    halx::core::delay(10);
  }
}

// TIM6周期割り込み(100Hz)。エンコーダ更新とPI速度制御をここで行う。
void timer_callback(void *) {
  motor1.update();
  motor2.update();
  motor3.update();

  float motor1_output = motor1_pid.solve(motor1_target_rps - motor1.get_rps());
  float motor2_output = motor2_pid.solve(motor2_target_rps - motor2.get_rps());
  float motor3_output = motor3_pid.solve(motor3_target_rps - motor3.get_rps());

  motor1.set_output(motor1_output);
  motor2.set_output(motor2_output);
  motor3.set_output(motor3_output);
}
