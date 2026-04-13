#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_attitude.h>          // 姿态四元数
#include <uORB/topics/manual_control_setpoint.h>   // 遥控器输入
#include <uORB/topics/vehicle_angular_velocity.h>  // 角速度
#include <uORB/topics/rpm.h>                       // RPM
#include <uORB/topics/esc_report.h>                // ESC 报告（含转速）
#include <uORB/topics/vehicle_rates_setpoint.h>    // 控制输出（角速度设定）

class MyController : public ModuleBase<MyController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MyController();
	~MyController() override = default;

	/** ModuleBase API */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static ModuleBase<MyController>::Descriptor desc;//补充声明定义

	bool init();
	int print_status() override;

private:
	void Run() override;

	// ---------- 订阅 ----------
	// 用角速度topic触发Run（控制环常用高频触发）
	uORB::SubscriptionCallbackWorkItem _ang_vel_sub{this, ORB_ID(vehicle_angular_velocity)};

	// 读取姿态四元数（角度信息来源）
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};

	// 读取rpm消息（转速首选来源）
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};

	// 读取esc_report（转速兜底来源）
	uORB::Subscription _esc_report_sub{ORB_ID(esc_report)};

	// 读取遥控器输入（备用）
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	// ---------- 发布 ----------
	// 这里输出控制：发布机体系角速度设定
	uORB::Publication<vehicle_rates_setpoint_s> _rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};



	// ---------- 缓存 ----------
	vehicle_attitude_s _att{};
	vehicle_angular_velocity_s _ang_vel{};
	rpm_s _rpm{};
	esc_report_s _esc_report{};
	manual_control_setpoint_s _manual{};

	// ---------- 简单控制参数（先写死，后续可改成参数） ----------
	float _kp_yaw{1.2f};       // 偏航角P增益
	float _kd_yaw_rate{0.15f}; // 偏航角速度D增益
	float _min_rpm_for_ctrl{300.f}; // 小于这个转速就不输出控制（保护）
	//遥控器缓存区

	float _last_measured_rpm{NAN};// 上次测量的转速（rpm或esc_report里选一个），用于转速保护
	float _max_roll_rate{1.2f};// 把杆量【-1.1】映射到角速度目标
	float _max_pitch_rate{1.2f};
	float _max_yaw_rate{1.2f};
	// 目标偏航角（示例设为0，表示朝北/初始方向）
	float _yaw_sp_rad{0.f};
};
