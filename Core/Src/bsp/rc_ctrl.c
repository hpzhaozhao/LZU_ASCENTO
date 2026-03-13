#include "rc_ctrl.h"
#include "chassis_task.h"
#include "elrs.h"
//遥控器LX:横滚  LY:变腿长 RX:左右旋转 RY:前进后退 左按键A：start 左三相按键B：  右三相按键C：  右按键D：prejump 左后按键E：起跳
float vel_ratio=0.005f;
float turn_ratio=0.005f;

float leg_ratio=0.0016f;
float roll_ratio=0.003f;

void rcdata_ctrl(void)
{
	static uint8_t last_A = 0;
	static uint8_t last_D = 0;
	static uint8_t last_E = 0;

	// 获取当前按键值
	uint8_t cur_A = elrs_data.A;
	uint8_t cur_D = elrs_data.D;
	uint8_t cur_E = elrs_data.E;

	// ----- 按键 A（start）-----
	if (cur_A == 2 && last_A != 2 && chassis_move.start_flag==0) {          // 上升沿：从非2变为2
		chassis_move.start_flag = 1;
	}
	else if (cur_A == 0 && last_A == 2 &&chassis_move.start_flag==1) {     // 下降沿：从2变为0
		chassis_move.start_flag = 0;
		chassis_move.recover_flag = 0;
	}

	// ----- 按键 D（prejump）-----
	if (cur_D == 2 && last_D != 2 && chassis_move.prejump_flag==0) {          // 上升沿
		chassis_move.prejump_flag = 1;
	}
	else if (cur_D == 0 && last_D == 2 && chassis_move.prejump_flag==1) {     // 下降沿
		chassis_move.prejump_flag = 0;
	}

	// ----- 按键 E（jump）-----
	// 跳跃键仅在预跳跃标志为1时有效，且仅检测上升沿
	if (cur_E == 1 && last_E != 1 && chassis_move.prejump_flag == 1) {
		chassis_move.jump_flag = 1;
	}

	// 更新上一次状态
	last_A = cur_A;
	last_D = cur_D;
	last_E = cur_E;

	if(elrs_data.D==2&&chassis_move.prejump_flag==0){
		//prejump按键被按下
		chassis_move.prejump_flag=1;
	}
	else if(elrs_data.D==0&&chassis_move.prejump_flag==2){
		//prejump按键弹起
		chassis_move.prejump_flag=0;
	}

	if(elrs_data.E==1&&chassis_move.prejump_flag==1&&chassis_move.jump_flag==0){
		//只有当预跳跃标志置1，按下这个键才能开启跳跃
		chassis_move.jump_flag=1;
	}

	// if(rev_data.last_key!=128&&rev_data.key==128&&chassis_move.movejump_flag==0){
	// 	chassis_move.movejump_flag=1;
	// }
	// else if(rev_data.last_key!=128&&rev_data.key==128&&chassis_move.movejump_flag==1){
	// 	chassis_move.movejump_flag=0;
	// }

        //前进后退,转向
	if(chassis_move.start_flag==1) {
		if (elrs_data.Right_Y!=0) {
			chassis_move.front_flag=1;
			chassis_move.v_set=((float)((int8_t)elrs_data.Right_Y))*vel_ratio;
			chassis_move.x_set=chassis_move.x_set+chassis_move.v_set*0.004f;//遥控器数据包速率250hz
		}
		else{
			chassis_move.front_flag=0;
			chassis_move.v_set=0.0f;
		}
		chassis_move.last_front_flag=chassis_move.front_flag;

		if(elrs_data.Right_X!=0){
			chassis_move.turn_flag=1;
			chassis_move.turn_set=((float)((int8_t)elrs_data.Right_X))*turn_ratio;
		}
		else{
			chassis_move.turn_flag=0;
		}
		if(chassis_move.last_turn_flag==1&&chassis_move.turn_flag==0){
			chassis_move.turn_set=chassis_move.total_yaw;
		}
		chassis_move.last_turn_flag=chassis_move.turn_flag;

		chassis_move.leg_set= 0.12+((float)((int8_t)elrs_data.Left_Y))*leg_ratio;

		chassis_move.roll_set=((float)((int8_t)elrs_data.Left_X))*roll_ratio;
		mySaturate(&chassis_move.roll_set,-0.1f,0.1f);
		mySaturate(&chassis_move.leg_set,0.12f,0.28f);

		if(fabsf(chassis_move.last_leg_set-chassis_move.leg_set)>0.0001f){//遥控器控制腿长在变化
			chassis_move.leg_flag=1;//为1标志着遥控器在控制腿长伸缩，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判为离地了
		}
		else{
			chassis_move.leg_flag=0;
		}
		chassis_move.last_leg_set=chassis_move.leg_set;
		// if(chassis_move.movejump_flag==1)
		// {
		// 	chassis_move.v_set=100.0f*vel_ratio;
		// 	chassis_move.x_set=chassis_move.x_set+chassis_move.v_set*0.0024f;
		// 	movetime++;
		// 	if(movetime>50)
		// 	{
		// 		movetime=0;
		// 		chassis_move.jump_flag=1;
		// 		//chassis_move.v_set=0.0f;
		// 		chassis_move.movejump_flag=0;
		// 	}
		// }
	}
// 	if(rev_data.key==8){
// 		chassis_move.roll_set=0.0f;
// 	}
}
