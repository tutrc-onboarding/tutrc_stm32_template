#pragma once

#include <algorithm>
#include <cstdint>

#include "halx/driver/gpio.hpp"
#include "halx/peripheral.hpp"

/**
 * @brief PWM出力・enableピン・エンコーダをまとめたDCモーター1個分のラッパー。
 * `update()` を一定周期`dt`で呼び出して速度・位置を更新する。
 * @tparam PWMHandle PWM出力に使うタイマーのハンドル
 * @tparam EncoderHandle エンコーダ(2逓倍/4逓倍カウント)に使うタイマーのハンドル
 *
 * @code{.cpp}
 * #include "motor.hpp"
 *
 * extern TIM_HandleTypeDef htim15; // PWM
 * extern TIM_HandleTypeDef htim1;  // エンコーダ
 *
 * halx::driver::GPIO motor1_pin(Motor1_GPIO_Port, Motor1_Pin);
 * constexpr float DT = 0.01f; // 100 Hz
 * Motor<&htim15, &htim1> motor1(TIM_CHANNEL_1, motor1_pin, 2048, 2.0f, DT);
 *
 * extern "C" void app_main() {
 *   motor1.start();
 *
 *   // 100 Hzの周期割り込み等から呼び出す
 *   motor1.update();
 *   float rps = motor1.get_rps();
 *   motor1.set_output(0.2f); // -1.0〜1.0
 * }
 * @endcode
 */
template <TIM_HandleTypeDef *PWMHandle, TIM_HandleTypeDef *EncoderHandle>
class Motor {
public:
  /**
   * @brief コンストラクタ。
   * @param pwm_channel PWM出力チャンネル(`TIM_CHANNEL_1`など)
   * @param motor_pin モータードライバのenable/directionピン
   * @param ppr エンコーダの1回転あたりパルス数
   * @param gear_ratio ギア比(出力軸1回転あたりのモーター軸回転数)
   * @param dt `update()` を呼び出す周期[秒]
   */
  Motor(uint32_t pwm_channel, halx::driver::GPIO &motor_pin, uint32_t ppr,
        float gear_ratio, float dt)
      : pwm_channel_{pwm_channel}, motor_pin_{motor_pin}, ppr_{ppr},
        gear_ratio_{gear_ratio}, dt_{dt} {}

  /**
   * @brief 出力0でPWM・エンコーダを開始し、モータードライバを有効化する。
   */
  void start() {
    set_output(0.0f);
    halx::peripheral::ST_TIM<PWMHandle>::start_pwm(pwm_channel_);
    motor_pin_.write(halx::driver::GPIOState::HIGH);
    halx::peripheral::ST_TIM<EncoderHandle>::start_encoder();
  }

  /**
   * @brief PWM出力を設定する。
   * @param output 出力(-1.0〜1.0にクランプされ、0〜100%duty比に線形変換される)
   */
  void set_output(float output) {
    float duty =
        std::clamp((std::clamp(output, -1.0f, 1.0f) + 1.0f) * 0.5f, 0.0f, 1.0f);
    uint32_t period = halx::peripheral::ST_TIM<PWMHandle>::get_autoreload();
    halx::peripheral::ST_TIM<PWMHandle>::set_compare(
        pwm_channel_, static_cast<uint32_t>(period * duty));
  }

  /**
   * @brief エンコーダカウンタを読み取ってリセットし、出力軸の回転速度・位置を更新する。
   * コンストラクタに渡した周期`dt`ごとに呼び出すこと。
   */
  void update() {
    int16_t count = halx::peripheral::ST_TIM<EncoderHandle>::get_counter();
    halx::peripheral::ST_TIM<EncoderHandle>::set_counter(0);

    float delta = count / (ppr_ * 4.0f) / gear_ratio_;
    rps_ = delta / dt_;
    position_ += delta;
  }

  /**
   * @brief 直近の `update()` で計算した出力軸の回転速度を取得する。
   * @return 回転速度[rps]
   */
  float get_rps() const { return rps_; }

  /**
   * @brief `update()` 呼び出しごとに積算された出力軸の位置を取得する。
   * @return 位置[回転数]
   */
  float get_position() const { return position_; }

private:
  uint32_t pwm_channel_;
  halx::driver::GPIO &motor_pin_;
  uint32_t ppr_;
  float gear_ratio_;
  float dt_;
  float rps_ = 0.0f;
  float position_ = 0.0f;
};
