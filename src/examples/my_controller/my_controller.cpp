#include "my_controller.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

ModuleBase::Descriptor MyController::desc{task_spawn, custom_command, print_usage};

MyController::MyController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

bool MyController::init()
{
	// 用 vehicle_angular_velocity 驱动控制循环
	if (!_ang_vel_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void MyController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	// 1) 读角速度反馈
	if (_ang_vel_sub.updated()) {
		_ang_vel_sub.copy(&_ang_vel);
	}

	// 2) 读姿态反馈
	if (_att_sub.updated()) {
		_att_sub.copy(&_att);
	}

	// 3) 读遥控器指令
	if (_manual_sub.updated()) {
		_manual_sub.copy(&_manual);
	}

	// 4) 读转速（rpm优先）
	if (_rpm_sub.updated()) {
		_rpm_sub.copy(&_rpm);

		if (PX4_ISFINITE(_rpm.rpm_estimate) && _rpm.rpm_estimate > 1.f) {
			_last_measured_rpm = _rpm.rpm_estimate;
		}
	}

	// 5) rpm无效时，用esc_report兜底
	if ((!PX4_ISFINITE(_last_measured_rpm) || _last_measured_rpm < 1.f) && _esc_report_sub.updated()) {
		_esc_report_sub.copy(&_esc_report);

		if (_esc_report.esc_rpm > 0) {
			_last_measured_rpm = static_cast<float>(_esc_report.esc_rpm);
		}
	}

	// 6) 姿态四元数 -> yaw角
	matrix::Quatf q(_att.q);
	const matrix::Eulerf euler(q);
	const float yaw = euler.psi();

	// 7) 当前角速度反馈
	const float roll_rate_feedback = _ang_vel.xyz[0];
	const float pitch_rate_feedback = _ang_vel.xyz[1];
	const float yaw_rate_feedback = _ang_vel.xyz[2];

	// 8) 默认输出
	float roll_rate_sp = 0.f;
	float pitch_rate_sp = 0.f;
	float yaw_rate_sp = 0.f;
	float throttle = -1.f;

	// 9) 遥控器映射：杆量[-1,1] -> 角速度设定
	if (_manual.valid) {
		roll_rate_sp = math::constrain(_manual.roll, -1.f, 1.f) * _max_roll_rate;
		pitch_rate_sp = math::constrain(_manual.pitch, -1.f, 1.f) * _max_pitch_rate;
		yaw_rate_sp = math::constrain(_manual.yaw, -1.f, 1.f) * _max_yaw_rate;
		throttle = math::constrain(_manual.throttle, -1.f, 1.f);
	}

	// 10) 简单偏航保持（可先保留）
	const float yaw_hold_gain = 0.05f;
	yaw_rate_sp += yaw_hold_gain * (-yaw);

	// 11) 简单阻尼项（抑制角速度）
	const float damping_gain = 0.08f;
	roll_rate_sp -= damping_gain * roll_rate_feedback;
	pitch_rate_sp -= damping_gain * pitch_rate_feedback;
	yaw_rate_sp -= damping_gain * yaw_rate_feedback;

	// 12) 转速保护：低转速时抑制姿态控制
	if (PX4_ISFINITE(_last_measured_rpm) && _last_measured_rpm < _min_rpm_for_control) {
		roll_rate_sp = 0.f;
		pitch_rate_sp = 0.f;
		yaw_rate_sp = 0.f;
	}

	// 13) 发布控制输出
	vehicle_rates_setpoint_s rates_sp{};
	rates_sp.timestamp = hrt_absolute_time();
	rates_sp.roll = roll_rate_sp;
	rates_sp.pitch = pitch_rate_sp;
	rates_sp.yaw = yaw_rate_sp;
	rates_sp.thrust_body[0] = 0.f;
	rates_sp.thrust_body[1] = 0.f;
	rates_sp.thrust_body[2] = -0.5f * (throttle + 1.f); // [-1,1] -> [0,-1]
	rates_sp.reset_integral = false;
	_rates_sp_pub.publish(rates_sp);
}

int MyController::task_spawn(int argc, char *argv[])
{
	MyController *instance = new MyController();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int MyController::print_status()
{
	PX4_INFO("my_controller running, last rpm: %.1f", (double)_last_measured_rpm);
	return 0;
}

int MyController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MyController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
my_controller:
- subscribe manual_control_setpoint
- subscribe vehicle_attitude / vehicle_angular_velocity / rpm / esc_report
- publish vehicle_rates_setpoint
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("my_controller", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int my_controller_main(int argc, char *argv[])
{
	return ModuleBase::main(MyController::desc, argc, argv);
}
