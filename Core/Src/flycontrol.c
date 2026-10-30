#include "flycontrol.h"
#include "filter.h"
#include "param.h"
#include <string.h>

#define RAD_TO_DEG (57.29577951308232f)

/* 降落 flare 剖面的斜率（每秒下降率 = 离地高度 × 这个系数）。
 * 0.8 表示 2m 高时降 1.6 m/s、1m 时 0.8 m/s、0.4m 时 0.32 m/s，再低由
 * alt.land_flare 兜底。想更"贴地"就加大到 1.0，想更柔和就降到 0.5。 */
#define LAND_FLARE_GAIN          0.8f

static float PID_Update(PID *pid, float error, float dt)
{
	pid->integral += error * dt;

	if (pid->integral_max > 0.0f)
	{
		if (pid->integral > pid->integral_max)
		{
			pid->integral = pid->integral_max;
		}
		else if (pid->integral < -pid->integral_max)
		{
			pid->integral = -pid->integral_max;
		}
	}

	float derivative = (error - pid->prev_error) / dt;

	float alpha = pid->dterm_lpf;
	if (alpha <= 0.0f || alpha > 1.0f)
	{
		alpha = 1.0f;
	}
	pid->dterm_filtered = alpha * derivative + (1.0f - alpha) * pid->dterm_filtered;
	pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * pid->dterm_filtered;
	pid->prev_error = error;
	return pid->output;
}

/* 微分先行 PID：微分项在测量值上计算，避免设定值突变带来的微分冲击 */
static float PID_Update_DerivOnMeas(PID *pid, float error, float measurement, float dt)
{
	pid->integral += error * dt;

	if (pid->integral_max > 0.0f)
	{
		if (pid->integral > pid->integral_max)
		{
			pid->integral = pid->integral_max;
		}
		else if (pid->integral < -pid->integral_max)
		{
			pid->integral = -pid->integral_max;
		}
	}

	float derivative = (-measurement + pid->prev_error) / dt;

	float alpha = pid->dterm_lpf;
	if (alpha <= 0.0f || alpha > 1.0f)
	{
		alpha = 1.0f;
	}
	pid->dterm_filtered = alpha * derivative + (1.0f - alpha) * pid->dterm_filtered;
	pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * pid->dterm_filtered;
	pid->prev_error = measurement;
	return pid->output;
}

static void PID_Reset(PID *pid)
{
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->output = 0.0f;
	pid->dterm_filtered = 0.0f;
}

void FlyController_Init(FlyController *fc)
{
	memset(fc, 0, sizeof(FlyController));

	/* ── 外环（角度环）参数：它的输出是**角速度指令(deg/s)**，喂给内环──
	 *
	 * 实测反馈"飞机自己回正的动作太软、太慢"。
	 * 原 kp=7.0 的含义：角度差 10° 只产生 70 deg/s 的角速度指令。
	 * 外环穿越频率 ≈ kp = 7 rad/s ≈ **1.1 Hz** —— 对四轴而言偏低（典型 3~5Hz），
	 * 所以回正慢、手感发软。
	 *
	 * kp 历史：7.0（太软，越频频率仅 1.1Hz）-> 12.0（太灵敏且振荡）-> **9.0**。
	 *   9.0 对应穿越频率 ≈ 1.4 Hz，回正比原值快约 30%，又不会过冲。
	 * ki 0.6 -> 1.0 （保持积分时间常数一致，否则 kp 变大后积分会显得过强）。
	 * kd 保持 0.12：它在角度差上做微分，主要是抑制回正时的过冲。
	 *
	 * 若回正快了但开始出现**低频摆动**（约 1~2Hz，像摇晃），说明外环过强，
	 * 往回降到 10。若仍觉得软，可再升到 15。 */
	fc->roll_outer.kp = 9.0f;
	fc->roll_outer.ki = 1.0f;
	fc->roll_outer.kd = 0.12f;
	fc->roll_outer.integral_max = 20.0f;
	fc->roll_outer.dterm_lpf = 0.16f;

	fc->pitch_outer.kp = 9.0f;
	fc->pitch_outer.ki = 1.0f;
	fc->pitch_outer.kd = 0.12f;
	fc->pitch_outer.integral_max = 20.0f;
	fc->pitch_outer.dterm_lpf = 0.16f;
 
	fc->yaw_outer.kp = 2.0f;
	fc->yaw_outer.ki = 0.1f;
	fc->yaw_outer.kd = 0.02f;
	fc->yaw_outer.integral_max = 20.0f;
	fc->yaw_outer.dterm_lpf = 0.16f;

	/* ── 内环（角速率环）参数 ──
	 * 黑匣子实测：内环在 10~11.5Hz 自激（修振动前 RMS 11、修后一度到 28）。
	 *
	 * 根因之一是 D 项低通过狠：dterm_lpf=0.12 @1kHz 等效 fc≈20Hz，
	 * 在 11.5Hz 处带来 ~30° 相位滞后 —— 而 D 项本来要提供 +90° 超前，
	 * 被吃掉 30° 后只剩 +60°，阻尼不足以压住高频振荡。
	 * 现在陀螺已经干净（振动残留 >100Hz 仅 3.3 deg/s），
	 * 放宽到 0.30（fc≈57Hz），11.5Hz 处滞后降到 ~11°，相位超前回到 ~79°。
	 *
	 * kp 10.2 -> 9.0：环路增益降 12%，给相位裕度留余量（响应略软一点）。
	 *
	 * 若仍自激：kp 再降到 7~8，或 dterm_lpf 提到 0.4。
	 * 若感觉太肉：kp 回到 10，dterm_lpf 保持 0.30。 */
	fc->roll_inner.kp = 10.0f;
	fc->roll_inner.ki = 0.25f;
	fc->roll_inner.kd = 0.16f;
	fc->roll_inner.integral_max = 8.0f;   /* 与 ki=8.0 配套，保持 I 项上限 ~240 PWM */
	fc->roll_inner.dterm_lpf = 0.20f;

	fc->pitch_inner.kp = 10.0f;
	fc->pitch_inner.ki = 0.25f;   /* 同 roll：原 0.6 对应 Ti=17s，实测每次都抑制不住稳态误差 */
	fc->pitch_inner.kd = 0.16f;  
	fc->pitch_inner.integral_max = 8.0f;  /* 同 roll */
	fc->pitch_inner.dterm_lpf = 0.20f;

	fc->yaw_inner.kp = 6.0f;
	fc->yaw_inner.ki = 0.3f;
	fc->yaw_inner.kd = 0.06f;
	fc->yaw_inner.integral_max = 8.0f;
	fc->yaw_inner.dterm_lpf = 0.16f;
	
	fc->altitude.kp = 30.0f;//180
	fc->altitude.ki = 3.0f;//3
	fc->altitude.kd = 3.0f;//5200
	fc->altitude.integral_max = 300.0f;
	fc->altitude.dterm_lpf = 1.0f;

	fc->altitude_velocity.kp = 4.0f;
	fc->altitude_velocity.ki = 0.0f;
	fc->altitude_velocity.kd = 2.0f;
	fc->altitude_velocity.integral_max = 100.0f;
	fc->altitude_velocity.dterm_lpf = 1.0f;

	fc->target_roll = 0.0f;
	fc->target_pitch = 0.0f;
	fc->target_yaw = 0.0f;
	fc->throttle = 0.0f;
	fc->target_altitude = 0.0f;
	fc->current_altitude = 0.0f;
	fc->prev_altitude = 0.0f;
	fc->current_vertical_velocity = 0.0f;
	fc->alt_dt = 0.05f;              /* 气压 20Hz 时的默认间隔，任务会持续刷新 */
	fc->mix_scale = 1.0f;
	fc->land_cut_reason = 0U;
	fc->altitude_output = 0.0f;
	fc->altitude_ready = 0;
	fc->alt_source_ok = 0;
	/* 悬停油门学习：从"还没学到"开始（上电时任务会用 alt.hover_throttle 灌历史值）。
	 * 学到之后才允许当基准用。 */
	fc->hover_throttle = 0.0f;
	fc->hover_learn_s = 0.0f;
	fc->hover_valid = 0;
	fc->bat_percent = 0;
	fc->bat_filt = 0.0f;
	
	fc->motor1 = MOTOR_MIN;
	fc->motor2 = MOTOR_MIN;
	fc->motor3 = MOTOR_MIN;
	fc->motor4 = MOTOR_MIN;
}

/* ── 用参数表覆盖编译期默认值 ──
 *
 * 只在任务启动时调一次。飞行代码之后只读 fc->xxx 这些字段，
 * 所以 CFG:SET 改的值不会在飞行中突然生效（要 SAVE + 重启）。
 *
 * ⚠ APPLY 依赖参数 ID 的排布：每个 PID 环占 5 个连续 ID
 *   base+0=kp, +1=ki, +2=kd, +3=integral_max, +4=dterm_lpf（见 param.h）。
 *   改 ID 时两边必须一起改，否则会把增益写到别的环上去。
 */
#define APPLY_PID(pid, base)                                              \
	do {                                                                  \
		(pid).kp           = Param_GetApplied((uint16_t)((base) + 0));    \
		(pid).ki           = Param_GetApplied((uint16_t)((base) + 1));    \
		(pid).kd           = Param_GetApplied((uint16_t)((base) + 2));    \
		(pid).integral_max = Param_GetApplied((uint16_t)((base) + 3));    \
		(pid).dterm_lpf    = Param_GetApplied((uint16_t)((base) + 4));    \
	} while (0)

void FlyController_ApplyGains(FlyController *fc)
{
	if (fc == NULL)
	{
		return;
	}

	APPLY_PID(fc->roll_outer,     P_ROLL_OUTER_KP);
	APPLY_PID(fc->pitch_outer,    P_PITCH_OUTER_KP);
	APPLY_PID(fc->yaw_outer,      P_YAW_OUTER_KP);
	APPLY_PID(fc->roll_inner,     P_ROLL_INNER_KP);
	APPLY_PID(fc->pitch_inner,    P_PITCH_INNER_KP);
	APPLY_PID(fc->yaw_inner,      P_YAW_INNER_KP);
	APPLY_PID(fc->altitude,       P_ALT_KP);
	APPLY_PID(fc->altitude_velocity, P_ALT_VEL_KP);

	/* 目标垂速限幅：默认 1.0 m/s。参数非法就退回 1.0，绝不允许 0 或负数
	 * （那会把定高环彻底锁死）。 */
	fc->alt_rate_max = Param_GetApplied(P_ALT_RATE_MAX);
	if ((fc->alt_rate_max < 0.2f) || (fc->alt_rate_max > 3.0f))
	{
		fc->alt_rate_max = 1.0f;
	}

	/* 降落近地减速下限：必须在 [0.15, 0.5] 之间。
	 * 下界 0.15：触地判据用"平均垂速 < 0.12"判断已停住，如果 flare 底线本身
	 *          就低于 0.12，正常下降就会被误判成触地 → 空中断电。
	 * 上界 0.5 ：太大则落地砸，且垂速迟迟降不下来。 */
	fc->land_flare_rate = Param_GetApplied(P_ALT_LAND_FLARE);
	if ((fc->land_flare_rate < 0.15f) || (fc->land_flare_rate > 0.5f))
	{
		fc->land_flare_rate = 0.25f;
	}
}

void FlyController_Update(FlyController *fc, ICM42605_Data *imu_data)
{
	float dt = imu_data->sample_period;
	if (dt <= 0.0f) {
			dt = 0.001f;
	}
	
	float current_roll = imu_data->angle.roll;
	float current_pitch = imu_data->angle.pitch;
	float current_yaw = imu_data->angle.yaw;
	
	float gyro_x = imu_data->gyro_x * RAD_TO_DEG;
	float gyro_y = imu_data->gyro_y * RAD_TO_DEG;
	float gyro_z = imu_data->gyro_z * RAD_TO_DEG;
	
	float roll_error = fc->target_roll - current_roll;
	float pitch_error = fc->target_pitch - current_pitch;
	float yaw_error = fc->target_yaw - current_yaw;
	
	if (yaw_error > 180.0f) {
			yaw_error -= 360.0f;
	} else if (yaw_error < -180.0f) {
			yaw_error += 360.0f;
	}
	
	/* ★ 外环改用“微分先行”版本：微分作用在**测量角度**上，
	 * 而不是作用在“目标-实际”的误差上。
	 *
	 * 原来用 PID_Update：一打杆目标瞬间变化 -> error 突变 ->
	 * D 项产生很大的瞬态冲击 -> 手感“太灵敏、打杆有震荡”。
	 * 改成 DerivOnMeas 后：打杆变化不再引起 D 冲击，
	 * 而 D 仍然对飞机真实的角速度起阻尼作用（帮助抑制回正过冲）。
	 *
	 * 注意第 3 个参数传的是**当前角度**（测量值），不是误差。 */
	float roll_outer_output = PID_Update_DerivOnMeas(&fc->roll_outer, roll_error, current_roll, dt);
	float pitch_outer_output = PID_Update_DerivOnMeas(&fc->pitch_outer, pitch_error, current_pitch, dt);
	float yaw_outer_output = PID_Update_DerivOnMeas(&fc->yaw_outer, yaw_error, current_yaw, dt);

	/* ── 外环输出限幅（角速度指令，deg/s）──
	 * 原来没有限幅：打杆瞬间角度误差突变，外环 D 项会给出
	 * 非常大的瞬态角速度指令，内环根不上反而形成冲击。
	 * 250 deg/s 对这类小四轴已经是很快的滚转，做为上限合理。 */
	#define FLY_OUTER_RATE_LIMIT   250.0f
	if (roll_outer_output >  FLY_OUTER_RATE_LIMIT) { roll_outer_output =  FLY_OUTER_RATE_LIMIT; }
	if (roll_outer_output < -FLY_OUTER_RATE_LIMIT) { roll_outer_output = -FLY_OUTER_RATE_LIMIT; }
	if (pitch_outer_output >  FLY_OUTER_RATE_LIMIT) { pitch_outer_output =  FLY_OUTER_RATE_LIMIT; }
	if (pitch_outer_output < -FLY_OUTER_RATE_LIMIT) { pitch_outer_output = -FLY_OUTER_RATE_LIMIT; }
	
	float roll_inner_error = roll_outer_output - gyro_y;
	float pitch_inner_error = pitch_outer_output + gyro_x;
	float yaw_inner_error = yaw_outer_output + gyro_z;
	
	float roll_output = PID_Update(&fc->roll_inner, roll_inner_error, dt);
	float pitch_output = PID_Update(&fc->pitch_inner, pitch_inner_error, dt);
	float yaw_output = PID_Update(&fc->yaw_inner, yaw_inner_error, dt);
	
	/* ── 定高串级环：位置 → 目标垂速 → 油门修正 ──
	 *
	 * fc->altitude_output 原来放在这里的 static。改成结构体字段之后：
	 *   · 黑匣子能记录当前修正量（判断"定高到底在不在用力"）
	 *   · 数据源失效时可以被明确清零，而不是永远保持最后一个值
	 *
	 * ★ 关键：HORIZON 或**高度数据源不健康**时，把修正量清零并复位两个环。
	 *   原来只判断 HORIZON_FLYING，气压计中途坏掉(或从来没起来)时
	 *   altitude_output 会一直保持上一个值 → final_throttle 卡在固定偏置上，
	 *   而定高模式下 fc->throttle 是电压补偿给的一个**常数**，飞手打杆也没用，
	 *   只能 disarm。现在会立刻把油门交回飞手（见 freertos.c 的 MODE_ALT_HOLD 分支）。 */
	if ((fc->flying_status == HORIZON_FLYING) || (fc->alt_source_ok == 0))
	{
		/* 只清修正量，**不**复位 PID：复位会让 prev_error 归零，
		 * 恢复那一拍的 D 项 = (err-0)/0.02 直接顶到几千油门（切模式窜一下）。
		 * 保持环路状态、只把本次修正量归零更温和，恢复时也不会有额外的微分冲击。 */
		fc->altitude_output = 0.0f;
	}
	else if (fc->altitude_ready == 1)
	{
		/* ══════════════════════════════════════════════════════════════════
		 *  定高串级环的调参关系（别再凭感觉单改某一个增益）
		 *
		 *  被控对象：竖直加速度 = K × 油门修正量，K = g / 悬停油门
		 *            （悬停 1650 PWM 时 K ≈ 0.0059 m/s²/PWM，纯积分器）
		 *  外环：目标垂速 = kp_a·err + ki_a·∫err + kd_a·d(err)/dt
		 *  内环：修正量   = kp_v·(目标垂速 − 垂速) + kd_v·d/dt
		 *
		 *  线性化后（d(err)/dt = −垂速）：
		 *      accel = K·kp_v·kp_a·err − K·kp_v·(kd_a+1)·垂速
		 *  于是：
		 *      ωn²  = K·kp_v·kp_a            （环路带宽）
		 *      2ζωn = K·kp_v·(kd_a+1)        （阻尼）
		 *  ⇒   ζ = (kd_a+1)/2 · sqrt(K·kp_v/kp_a)
		 *
		 *  ── 从这里能看出两件事 ──
		 *  ① ζ 只取决于 kp_v/kp_a 的**比值**。单改 kp_v 会把带宽和阻尼
		 *     一起提高，ζ 只按 √kp_v 改善 —— 表现为"越快越荡"。
		 *     （2026-10-23 曾把 kp_v 4→15 单独改，结果过冲更大，就是这个原因。）
		 *  ② 这条机臂上 K≈0.0059 很小，所以 ωn 对增益不敏感、
		 *     ζ 却很敏感：kp_a=30 配 kp_v=4 时 ζ≈0.06，
		 *     即"几乎没有阻尼，全靠位置过冲来回拉"。
		 *
		 *  ── 推荐结构（也是当前默认思路）：内环快、外环慢 ──
		 *      kp_v ≈ 150（速度环时间常数 1/(K·kp_v) ≈ 1.1s）
		 *      kp_a ≈ 1.5、kd_a ≈ 1、ki_a ≈ 0.1（imax 小，防积分灌满）
		 *      → ωn≈1.1 rad/s、ζ≈0.77：起飞超调 0.12m、悬停高度 RMS 8mm
		 *  注意 kp_a 的量纲是 1/s，1.5 这种"小数值"才是正常的；
		 *  kp_a=30 相当于给飞机下"误差 1m 就爬 30m/s"的命令，靠内外环
		 *  的饱和才没炸，属于病态配置。
		 * ══════════════════════════════════════════════════════════════════ */
		/* ★ dt 用"高度样本的实际间隔"，不能写死。
		 * 原来写死 0.02（那时气压 50Hz）。后来读取周期改成 50ms(20Hz)，
		 * 写死 0.02 会把 D 项放大 2.5 倍：气压的 ±15cm 波动 → 目标垂速 ±75m/s
		 * → 内环顶到 ±600 → 电机在 1050~2265 PWM 之间 20Hz 抖
		 * （实测 corr 饱和 22%、RMS 329）。改成真实 dt 后饱和降到 1%。 */
		float alt_dt = fc->alt_dt;

		if ((alt_dt < 0.005f) || (alt_dt > 0.2f))
		{
			alt_dt = 0.05f;                 /* 兜底，正常由任务每拍更新 */
		}

		// Outer loop: position → target vertical velocity
		float altitude_error = fc->target_altitude - fc->current_altitude;

		/* 反积分饱和：把位置误差限到"刚好让目标垂速打满"的大小（rate_max/kp_a）。
		 * 爬升过程中误差一直很大（目标 3m 就一直是 3m），外环积分 ku 一路灌满，
		 * 到目标后还在推油门 → 超调。限误差之后爬升率不变（本来就顶在 rate_max），
		 * 但 3m 爬升的超调从 0.45m 降到 0.20m。 */
		{
			float err_lim = fc->alt_rate_max /
			                ((fc->altitude.kp > 0.01f) ? fc->altitude.kp : 0.01f);

			if (altitude_error >  err_lim) { altitude_error =  err_lim; }
			if (altitude_error < -err_lim) { altitude_error = -err_lim; }
		}

		float target_vertical_velocity = PID_Update(&fc->altitude, altitude_error, alt_dt);

		/* ── 目标垂速限幅（双向，alt.rate_max）──
		 * 这一条同时管三件事：
		 *  ① 定高起飞/切定高那一瞬间"窜"：位置误差 × kp_a 会给出几 m/s 的
		 *     爬升命令，内环 kp_v 再乘一下就顶到满舵。限到 1m/s 后，
		 *     爬 1m 用 1 秒、爬 3m 用 3 秒，油门突跳只有约 150 PWM。
		 *  ② 降落模式把目标设成 -3m 时的硬下坠（原来误差 4m → 目标垂速
		 *     -120m/s → 内环 -600 满舵 → 3m/s 下坠，把估计器都逼发散过）。
		 *  ③ 切模式瞬间 D 项的冲击（PID 的 prev_error 是上次的旧值）。
		 * 只限下降一侧的做法试过：飞机冲过头后最多只能命令"下降 1m/s"，
		 * 救不回来 —— 所以必须双向。 */
		if (fc->flying_status == LANDING)
		{
			/* ── 降落剖面（flare）──
			 * 位置环在降落时只会把目标垂速一路顶在 -rate_max 上（2 m/s 砸到地面），
			 * 而且下降率会随"基准差多少"漂移。这里改成按离地高度的线性剖面：
			 *     2m→-1.6   1m→-0.8   0.4m→-0.32   更低→-land_flare
			 * 落地速度因此是可控的 0.25 m/s，触地判据的"|垂速|<0.25"也才成立。
			 * 位置环仍在跑，但目标高度由任务设成"当前高度"（误差≈0，积分不 windup），
			 * 所以它的 P/I/D 输出在本分支里不参与。 */
			float v = fc->current_altitude * LAND_FLARE_GAIN;

			if (v > fc->alt_rate_max)      { v = fc->alt_rate_max; }
			if (v < fc->land_flare_rate)   { v = fc->land_flare_rate; }

			target_vertical_velocity = -v;
		}
		else
		{
			if (target_vertical_velocity >  fc->alt_rate_max) { target_vertical_velocity =  fc->alt_rate_max; }
			if (target_vertical_velocity < -fc->alt_rate_max) { target_vertical_velocity = -fc->alt_rate_max; }
		}

		// Inner loop: vertical velocity → throttle correction
		float vertical_velocity_error = target_vertical_velocity - fc->current_vertical_velocity;
		fc->altitude_output = PID_Update(&fc->altitude_velocity, vertical_velocity_error, alt_dt);
		fc->prev_altitude = fc->current_altitude;
	}
	else
	{
		/* 这一拍没有新高度样本：保持上一拍的修正量（20 拍里 19 拍是这种情况） */
	}

	/* 定高修正量限幅。没有它的话，刚切进定高/高度数据刚恢复的那一拍，
	 * 外环 D 项 (kd·err/dt，dt 只有 0.02) 能把修正量顶到几千，
	 * 表现为"一开定高就窜一下"。±600 PWM 已经远大于正常修正范围
	 * （1 米高度误差 ≈ 120）。 */
	#define ALT_OUTPUT_LIMIT   600.0f
	if (fc->altitude_output >  ALT_OUTPUT_LIMIT) { fc->altitude_output =  ALT_OUTPUT_LIMIT; }
	if (fc->altitude_output < -ALT_OUTPUT_LIMIT) { fc->altitude_output = -ALT_OUTPUT_LIMIT; }

	float final_throttle = fc->throttle + fc->altitude_output;

	/* ── 悬停油门学习 ──
	 * 样本用**倾角补偿之前**的总油门（= 水平悬停该给的油门），
	 * 所以放在除法之前。判据自己会挡住"没在平稳悬停"的时刻。 */
	FlyController_UpdateHoverLearn(fc, final_throttle,
	                               imu_data->filtered_vert_accel,
	                               imu_data->tilt_total * RAD_TO_DEG,
	                               imu_data->sample_period);

	if(imu_data->tilt_total < 60.0f)
	{
		final_throttle = 1/cosf(imu_data->tilt_total/RAD_TO_DEG) * final_throttle;
	}

	
	float m1 = final_throttle + pitch_output - roll_output + yaw_output;
	float m2 = final_throttle - pitch_output - roll_output - yaw_output;
	float m3 = final_throttle + pitch_output + roll_output - yaw_output;
	float m4 = final_throttle - pitch_output + roll_output + yaw_output;	

	/* ══════════════════════════════════════════════════════════════════════
	 *  混控抗饱和（mixer desaturation）
	 *
	 *  四路是「总量 ± 差动」线性相加，而**推力 ∝ 油门²**。
	 *  一旦差动超出可用行程，逐个通道硬限幅会**改变四路的总和**：
	 *  高的一侧顶到 4200、低的一侧钳到 0，等效推力 Σm²/(4·mean²) 能到 1.3~1.9，
	 *  也就是同样的"平均油门"凭空多出 30%~90% 的升力。
	 *
	 *  实测（Height_v4 那架次）：剧烈摇杆时两路在 0、两路在 4000+，
	 *  baro 在 2 秒内从 0.58 爬到 1.49 m，而定高环只有 ±600 PWM 的权限，
	 *  根本追不回来 —— 这就是"猛打杆飞机自己上升"的真正原因。
	 *
	 *  处理办法：限幅**之前**把差动按比例缩到刚好能放进 [MOTOR_MIN, MOTOR_MAX]，
	 *  **保持总量 final_throttle 不变** → 推力不跳变。
	 *  代价：极端机动时滚转/俯仰角速率会被削一点（宁可转慢，也不能窜）。
	 *
	 *  ⚠ 不要用"整组平移"的办法：平移会改变总量，照样造升力。
	 * ══════════════════════════════════════════════════════════════════════ */
	if (final_throttle < (float)MOTOR_MIN) { final_throttle = (float)MOTOR_MIN; }
	if (final_throttle > (float)MOTOR_MAX) { final_throttle = (float)MOTOR_MAX; }
	{
		float mx = m1 > m2 ? m1 : m2;
		float mn = m1 < m2 ? m1 : m2;
		float low_room  = final_throttle - (float)MOTOR_MIN;
		float high_room = (float)MOTOR_MAX - final_throttle;
		float head = 2.0f * ((low_room < high_room) ? low_room : high_room);

		if (m3 > mx) { mx = m3; }
		if (m4 > mx) { mx = m4; }
		if (m3 < mn) { mn = m3; }
		if (m4 < mn) { mn = m4; }

		if ((mx - mn) > head)
		{
			/* head 可能为 0（总量已经贴着限位）→ 差动只能全部放弃 */
			float kk = (head > 1.0f) ? (head / (mx - mn)) : 0.0f;

			m1 = final_throttle + (m1 - final_throttle) * kk;
			m2 = final_throttle + (m2 - final_throttle) * kk;
			m3 = final_throttle + (m3 - final_throttle) * kk;
			m4 = final_throttle + (m4 - final_throttle) * kk;

			fc->mix_scale = kk;      /* <1 表示这一拍被缩放过（记进黑匣子看比例） */
		}
		else
		{
			fc->mix_scale = 1.0f;
	fc->land_cut_reason = 0U;
		}
	}

	if (m1 < MOTOR_MIN) m1 = MOTOR_MIN;
	if (m1 > MOTOR_MAX) m1 = MOTOR_MAX;
	if (m2 < MOTOR_MIN) m2 = MOTOR_MIN;
	if (m2 > MOTOR_MAX) m2 = MOTOR_MAX;
	if (m3 < MOTOR_MIN) m3 = MOTOR_MIN;
	if (m3 > MOTOR_MAX) m3 = MOTOR_MAX;
	if (m4 < MOTOR_MIN) m4 = MOTOR_MIN;
	if (m4 > MOTOR_MAX) m4 = MOTOR_MAX;		
	
	fc->motor1 = (uint16_t)m1;
	fc->motor2 = (uint16_t)m2;
	fc->motor3 = (uint16_t)m3;
	fc->motor4 = (uint16_t)m4;		

}

void FlyController_SetTarget(FlyController *fc, float roll, float pitch, float yaw, float throttle)
{
	fc->target_roll = roll;
	fc->target_pitch = pitch;
	fc->target_yaw = yaw;
	
	if (throttle < MOTOR_MIN)
	{
		fc->throttle = MOTOR_MIN;
	}
	else if (throttle > MOTOR_MAX)
	{
		fc->throttle = MOTOR_MAX;
	}
	else
	{
		fc->throttle = throttle;
	}
}

void FlyController_Reset(FlyController *fc)
{
	PID_Reset(&fc->roll_outer);
	PID_Reset(&fc->roll_inner);
	PID_Reset(&fc->pitch_outer);
	PID_Reset(&fc->pitch_inner);
	PID_Reset(&fc->yaw_outer);
	PID_Reset(&fc->yaw_inner);
	PID_Reset(&fc->altitude);
	PID_Reset(&fc->altitude_velocity);

	fc->target_roll = 0.0f;
	fc->target_pitch = 0.0f;
	fc->target_yaw = 0.0f;
	fc->target_altitude = 0.0f;
	fc->altitude_output = 0.0f;     /* 定高修正量必须跟着复位，否则解锁瞬间会带一个旧偏置 */
	
	fc->motor1 = MOTOR_MIN;
	fc->motor2 = MOTOR_MIN;
	fc->motor3 = MOTOR_MIN;
	fc->motor4 = MOTOR_MIN;
}

void FlyController_SetTargetAltitude(FlyController *fc, float target_altitude)
{
   fc->target_altitude = target_altitude;
}

void FlyController_SetCurrentAltitude(FlyController *fc, float current_altitude)
{
   fc->current_altitude = current_altitude;
}

void FlyController_SetCurrentVerticalVelocity(FlyController *fc, float current_vertical_velocity)
{
	fc->current_vertical_velocity = current_vertical_velocity;
}

void FlyController_SetBatteryPercent(FlyController *fc, uint16_t bat_percentage)
{
	/* 电量百分比低通（τ≈1.4s，1kHz 调用）。
	 * 遥控端报的电量是"带载电压换算出来的"，电机一加速电压就跌，
	 * 实测同一架次能从 83% 跳到 50% —— 而电压补偿会把这种抖动放大成
	 * ±170 PWM 的基准油门抖动，等于给定高环塞了一个 ±1 m/s² 的假扰动。
	 * 真实放电是分钟级的，1.4s 的低通完全够用。 */
	const float a = 0.0007f;

	if (fc->bat_percent == 0U)
	{
		fc->bat_filt = (float)bat_percentage;   /* 首个样本直接采纳 */
	}
	else
	{
		fc->bat_filt += a * ((float)bat_percentage - fc->bat_filt);
	}
	fc->bat_percent = bat_percentage;
}

/* 电压补偿表用的电量（低通后，四舍五入成整数百分比） */
static uint16_t fc_battery_pct(FlyController *fc)
{
	float v = fc->bat_filt;

	if (v < 0.0f)   { v = 0.0f; }
	if (v > 100.0f) { v = 100.0f; }
	return (uint16_t)(v + 0.5f);
}

/* ============================================================================
 *  悬停油门学习
 *
 *  ── 旧实现的问题（全部换掉了）──
 *   1. 学出来的值**根本没进控制回路**：fc->hover_throttle 只被降落判据用过一次，
 *      而定高基准油门仍然固定用 Throttle_Voltage_Compensation()。
 *      "学习"等于白学。
 *   2. 判据 `fabsf(accel_z-0.995f)<0.1f` 是**瞬时值**判据，不是"平稳"判据：
 *      振动、瞬时机动都能蒙混过去；而且 0.995 是这台机加计标定出来的魔法数，
 *      换机/重新标定就不成立。
 *   3. 只在 |tilt|<2° 时学 —— 有风时悬停倾角常 3~8°，基本上永远学不到。
 *   4. 低通 alpha=0.08 且 1kHz 调用 → 时间常数约 12ms，太快，
 *      等于"最后一次满足条件的值"，没有任何平均意义。
 *   5. 滤波器是函数内 static：FlyController_Reset() 复位不掉，跨架次带脏状态。
 *   6. 学的是绝对值，电池电压一掉就失配。
 *
 *  ── 现在的做法 ──
 *   · 学**绝对油门 (PWM)**。⚠ 第一版学的是"总油门 / 电压补偿值"的比例，
 *     结果被遥控电量的剧烈跳动（同一架次 88%→37%）放大成几百 PWM 的基准抖动，
 *     改回绝对量：电压补偿只在"还没学会"时当起点，学会之后与电量无关。
 *   · 判据用"真的没在升降"：|天向加速度|<0.03g（融合滤波后的垂向加计）
 *     且 |垂速|<0.2m/s（融合器估计），再加倾角/油门区间/修正量未饱和；
 *   · 在**任何飞行模式**下都学（手动悬停时就学会了，切定高时已经有效）；
 *   · 先快后慢两段低通：前 1 秒 alpha=0.01（τ≈0.1s）快速收敛，
 *     之后 alpha=0.0005（τ≈2s）精修；
 *   · 学到 disarm 时由 Logger 任务落盘到 alt.hover_throttle，下次上电直接恢复。
 * ========================================================================== */

#define HOVER_LEARN_ACC_GATE     0.03f    /* |天向加速度| 门限 (g)   */
#define HOVER_LEARN_VVEL_GATE    0.20f    /* |垂速| 门限 (m/s)       */
#define HOVER_LEARN_TILT_GATE    8.0f     /* 倾角门限 (deg)          */
#define HOVER_LEARN_TRIM_GATE  540.0f     /* 修正量门限 (0.9×600 限幅) */
#define HOVER_LEARN_MIN_THR      0.20f    /* 总油门下限 (×MOTOR_MAX) */
#define HOVER_LEARN_MAX_THR      0.90f    /* 总油门上限 (×MOTOR_MAX) */
#define HOVER_LEARN_FAST_S       1.0f     /* 前 1 秒用快速收敛       */
#define HOVER_LEARN_VALID_S      1.5f     /* 累计有效 1.5 秒才算可用 */
/* 悬停油门对电量的斜率（PWM / 1% 电量）。实测：满电 1500、快没电 1700，
 * 跨约 80% 电量 → 2.5。偏大只是"多修一点"，偏小就是"修不够"，
 * 想再准就飞两个电量点（比如 100% 和 30% 各记一次悬停油门）反算。 */
#define HOVER_BATT_SLOPE         2.5f

void FlyController_SeedHoverThrottle(FlyController *fc, float pwm)
{
    if ((pwm > HOVER_LEARN_MIN_THR * (float)MOTOR_MAX) &&
        (pwm < HOVER_LEARN_MAX_THR * (float)MOTOR_MAX))
    {
        fc->hover_throttle = pwm;
        fc->hover_valid    = 1U;      /* 存过的值可以直接用，不用等重新学 */
        fc->hover_learn_s  = HOVER_LEARN_VALID_S;
    }
}

void FlyController_SetHoverReference(FlyController *fc, float ref_pwm, float ref_pct)
{
    /* 只要 ref_pwm 落在"像悬停油门"的区间、ref_pct 是有效百分比，就认这个参考点。
     * alt.ref_throttle 的出厂默认是 537（老预留值），远低于任何真实悬停油门，
     * 所以没存过参考点的板子会自然走到"只按绝对油门"的老路径，不会有兼容问题。 */
    if ((ref_pwm > 800.0f) && (ref_pwm < (float)MOTOR_MAX) &&
        (ref_pct > 0.5f) && (ref_pct <= 100.0f))
    {
        fc->hover_ref_pwm = ref_pwm;
        fc->hover_ref_pct = ref_pct;
    }
    else
    {
        fc->hover_ref_pwm = 0.0f;
        fc->hover_ref_pct = 0.0f;
    }
}

void FlyController_UpdateHoverForBattery(FlyController *fc)
{
    float bat;
    float pwm;

    if (fc->hover_ref_pwm <= 0.0f)
    {
        return;                       /* 没有参考点：保持绝对油门（老行为） */
    }

    bat = fc->bat_filt;               /* 低通后的电量，避免带载抖动带偏 */
    if (bat < 5.0f)
    {
        return;                       /* 还没收到有效电量 */
    }

    /* 电量越高，同样推力需要的油门越小 —— 所以是减。
     * 斜率来自实测：满电悬停 1500、快没电 1700 → 约 2.5 PWM/%。 */
    pwm = fc->hover_ref_pwm - HOVER_BATT_SLOPE * (bat - fc->hover_ref_pct);

    if (pwm < HOVER_LEARN_MIN_THR * (float)MOTOR_MAX)
    {
        pwm = HOVER_LEARN_MIN_THR * (float)MOTOR_MAX;
    }
    if (pwm > HOVER_LEARN_MAX_THR * (float)MOTOR_MAX)
    {
        pwm = HOVER_LEARN_MAX_THR * (float)MOTOR_MAX;
    }

    fc->hover_throttle = pwm;
    fc->hover_valid    = 1U;
    fc->hover_learn_s  = HOVER_LEARN_VALID_S;
}

float FlyController_GetBatteryPercent(FlyController *fc)
{
    return fc->bat_filt;
}

void FlyController_UpdateHoverLearn(FlyController *fc, float throttle_total,
                                    float a_up_g, float tilt_deg, float dt)
{
    float alpha;
    float sample;
    uint8_t gate = 0U;

    if ((dt <= 0.0f) || (dt > 0.05f))
    {
        return;
    }

    /* ---- 判据：只在"真的平稳悬停"时学 ---- */
    if ((fc->flying_status != LANDED) &&
        (fc->alt_source_ok != 0U) &&
        (fabsf(a_up_g) < HOVER_LEARN_ACC_GATE) &&
        (fabsf(fc->current_vertical_velocity) < HOVER_LEARN_VVEL_GATE) &&
        (tilt_deg < HOVER_LEARN_TILT_GATE) &&
        /* 修正量没被 ±600 限幅：贴着限幅说明还没到平衡点，
         * 此时"总油门"不是悬停油门，学进去会带偏 */
        (fabsf(fc->altitude_output) < HOVER_LEARN_TRIM_GATE) &&
        (throttle_total > HOVER_LEARN_MIN_THR * (float)MOTOR_MAX) &&
        (throttle_total < HOVER_LEARN_MAX_THR * (float)MOTOR_MAX))
    {
        gate = 1U;
    }

    if (gate == 0U)
    {
        return;
    }

    fc->hover_learn_s += dt;
    sample = throttle_total;          /* 绝对油门 (PWM) */

    if (fc->hover_valid == 0U)
    {
        /* 还没学会：直接采纳第一个合格样本，避免从一个离谱的起点慢慢爬 */
        fc->hover_throttle = sample;
    }
    else
    {
        alpha = (fc->hover_learn_s < HOVER_LEARN_FAST_S) ? 0.01f : 0.0005f;
        fc->hover_throttle += alpha * (sample - fc->hover_throttle);
    }

    if (fc->hover_throttle < HOVER_LEARN_MIN_THR * (float)MOTOR_MAX)
    {
        fc->hover_throttle = HOVER_LEARN_MIN_THR * (float)MOTOR_MAX;
    }
    if (fc->hover_throttle > HOVER_LEARN_MAX_THR * (float)MOTOR_MAX)
    {
        fc->hover_throttle = HOVER_LEARN_MAX_THR * (float)MOTOR_MAX;
    }

    if (fc->hover_learn_s >= HOVER_LEARN_VALID_S)
    {
        fc->hover_valid = 1U;
    }
}

float FlyController_GetHoverBase(FlyController *fc)
{
    if (fc->hover_valid != 0U)
    {
        return fc->hover_throttle;    /* 学到/存过的绝对油门，与电量无关 */
    }
    /* 还没学会：退回电压补偿（旧行为）。电量用低通值，见 SetBatteryPercent。 */
    return Throttle_Voltage_Compensation(fc_battery_pct(fc));
}

float Throttle_Voltage_Compensation(uint16_t bat_percentage)
{
	/* 原来是四个编译期宏，现在走参数表（重启生效）。
	 * 注意这里读的是"生效值"，所以飞行中改参数不会影响它。 */
	float tho_h = Param_GetApplied(P_THR_THO_H);
	float tho_l = Param_GetApplied(P_THR_THO_L);
	float vol_h = Param_GetApplied(P_THR_VOL_H);
	float vol_l = Param_GetApplied(P_THR_VOL_L);
	float scale = Param_GetApplied(P_THR_SCALE);
	float dv    = vol_h - vol_l;

	/* 分母保护：vol_h 与 vol_l 相等时公式会除零。
	 * 两个参数各自的取值范围允许相等，所以必须在这里挡一下，退回编译期默认值。 */
	if (fabsf(dv) < 1.0f)
	{
		tho_h = 380.0f;
		tho_l = 440.0f;
		vol_h = 60.0f;
		vol_l = 26.0f;
		dv    = vol_h - vol_l;
		scale = 3.0f;
	}

	return (((tho_h + tho_l) / 2.0f) +
	        ((tho_h - tho_l) * (2.0f * (float)bat_percentage - vol_h - vol_l)) / (2.0f * dv)) * scale;
}
