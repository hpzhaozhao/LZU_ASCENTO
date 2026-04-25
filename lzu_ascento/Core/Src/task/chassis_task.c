#include "chassis_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "tjc_usart_hmi.h"
#include "elrs.h"
#include "stdio.h"
#include "usart.h"
chassis_t chassis_move;

extern INS_t INS;

vmc_leg_t vmc;

uint32_t CHASSR_TIME=1;

float Turn_Kp=0.2f;
float Turn_Kd=0.02f;
float Turn_Kp_1=-0.01f;

static uint8_t ground_lost_cnt = 0;

 float Turn(float Angle,float Gyro)
 {
 	float tor3;

 	if(chassis_move.turn_flag==1)
 	{
 		tor3=(chassis_move.turn_set-Gyro)*Turn_Kp_1;
 	}
 	else if(chassis_move.turn_flag==0)
 	{
 		tor3=Turn_Kp*(chassis_move.turn_set-Angle)-Gyro*Turn_Kd;
 	}
 	return tor3;
 }

float turn_T=0.0f;

float leg_kp= 130.0f;
float leg_kd= -5.0f;

float mg_L=5.5f;
float mg_R=5.0f;//前馈

float roll_kp=10.0f;
float roll_kd=0.6f;

float joint_zero_angle ;

int jump_time =0;
int finish_time=0;

void Chassis_init(chassis_t *chassis)
{
	//左边
	joint_motor_init(&chassis->joint_motor[0],0x02,SF_MODE);//发送id为2
	wheel_motor_init(&chassis->wheel_motor[0],0x01,MIT_MODE);//发送id为1
	//右边
	joint_motor_init(&chassis->joint_motor[1],0x02,SF_MODE);//发送id为2
	wheel_motor_init(&chassis->wheel_motor[1],0x01,MIT_MODE);//发送id为1

	vmc.right_l1=0.1535f;//第一杆的长度，单位m
	vmc.left_l1=0.1535f;//第一杆的长度，单位m

	chassis_move.phi_set=0.21f;
	chassis_move.leg_set=0.12f;
	chassis_move.roll_set=0.0f;
	joint_zero_angle = acosf(0.038f / (2.0f * 0.1535f));

	for(int j=0;j<10;j++)
	{//左边
		enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
		save_motor_zero(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);//左边关节电机
		osDelay(1);
	}
	for(int j=0;j<10;j++)
	{//左边
		enable_motor_mode(&hfdcan1,chassis->wheel_motor[0].para.id,chassis->wheel_motor[0].mode);//左边轮毂电机
		osDelay(1);
	}

	for(int j=0;j<10;j++)
	{//右边
		enable_motor_mode(&hfdcan2,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
		save_motor_zero(&hfdcan2,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);//右边关节电机
		osDelay(1);
	}
	for(int j=0;j<10;j++)
	{//右边
		enable_motor_mode(&hfdcan2,chassis->wheel_motor[1].para.id,chassis->wheel_motor[1].mode);//右边轮毂电机
		osDelay(1);
	}
}
int64_t time=0;
float averr[10]={0.0f};

//sf关节电机逆时针为正，顺时针为负
float get_left_actual_angle(float encoder_pos)//左encoder_pos为负值，腿变长，值变小
{
	return joint_zero_angle + encoder_pos;
}

float get_right_actual_angle(float encoder_pos)//右encoder_pos为正值，腿变长，值变大
{
	return joint_zero_angle - encoder_pos;
}

void chassisR_feedback_update(chassis_t *chassis,INS_t *ins,vmc_leg_t *vmc_leg)
{
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[1];

	chassis->total_yaw=ins->YawTotalAngle;
	chassis->v_act=((chassis->wheel_motor[0].para.vel-chassis->wheel_motor[1].para.vel)/2.0f)*0.06f;//0.06m是轮子半径
	//chassis->v_act=(-chassis->wheel_motor[0].para.vel)*0.0425f;//0.0425m是轮子半径

	averr[0]=averr[1];
	averr[1]=averr[2];
	averr[2]=averr[3];
	averr[3]=averr[4];
	averr[4]=averr[5];
	averr[5]=averr[6];
	averr[6]=averr[7];
	averr[7]=averr[8];
	averr[8]=averr[9];
	averr[9]=chassis->v_act;

	chassis->v=0.1f*averr[0]+0.1f*averr[1]+0.1f*averr[2]+0.1f*averr[3]
						+0.1f*averr[4]+0.1f*averr[5]+0.1f*averr[6]+0.1f*averr[7]
						+0.1f*averr[8]+0.1f*averr[9];//对速度进行均值滤波

	chassis->x=chassis->x+chassis->v*0.001f;//控制周期为1ms

	float left_actual_angle =get_left_actual_angle(chassis->joint_motor[0].para.pos);
	float right_actual_angle = get_right_actual_angle(chassis->joint_motor[1].para.pos);

	//连杆1和连杆2近似等长
	//轮子在竖直方向运动近似为一条直线
	vmc_leg->left_len= 2.0f * vmc_leg->left_l1 * arm_cos_f32(left_actual_angle);//余弦定理
	vmc_leg->left_len_dot = -2.0f * vmc_leg->left_l1 * arm_sin_f32(left_actual_angle) * (chassis->joint_motor[0].para.vel);//直接求导

	vmc_leg->right_len= 2.0f * vmc_leg->right_l1 * arm_cos_f32(right_actual_angle);//余弦定理
	vmc_leg->right_len_dot = -2.0f * vmc_leg->right_l1*arm_sin_f32(right_actual_angle) * (chassis->joint_motor[1].para.vel);//直接求导
	chassis->myRoll=INS.Roll;

}


void mySaturate(float *in,float min,float max)
{
	if(*in < min)
	{
		*in = min;
	}
	else if(*in > max)
	{
		*in = max;
	}
}

float LQR_K_calc(float *coe,float len)
{
	return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}


float lqr_k[8]={
	 -0.1054 ,  -0.2837  , -2.4054 ,  -0.3720,
   -0.1054 ,  -0.2837 ,  -2.4054  , -0.3720
};

float Poly_Coefficient[8][4]={
	{1.7157e-12, -9.9069e-13, 1.8819e-13, -0.10541},
	{5.8329, -4.5708, 1.2674, -0.37987},
	{-30.1102, 27.0942, -10.3735, -1.4992},
	{2.227, -1.8923, -0.084791, -0.33836},
	{8.3232e-13, -5.0142e-13, 1.0162e-13, -0.10541},
	{5.8329, -4.5708, 1.2674, -0.37987},
	{-30.1102, 27.0942, -10.3735, -1.4992},
	{2.227, -1.8923, -0.084791, -0.33836}
};
void Chassis_task(void) {
	while(INS.ins_flag==0)
	{//等待加速度收敛
		osDelay(1);
	}

	Chassis_init(&chassis_move);

	while(1) {
		//  printf("leg = %.2f\n",chassis_move.leg_set);
		// printf("roll = %.2f\n",chassis_move.roll_set);
		//printf("roll = %.2f\n",INS.Roll);

		 chassisR_feedback_update(&chassis_move,&INS,&vmc);//更新数据
		 for(int i=0;i<4;i++)
		 {
		 	lqr_k[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmc.left_len);
		 }
		 for(int j=4;j<8;j++)
		 {
		 	lqr_k[j]=LQR_K_calc(&Poly_Coefficient[j][0],vmc.right_len);
		 }

		  chassis_move.phi_set=-0.7833f*((vmc.left_len+vmc.right_len)/2.0f)+0.3011f;//腿长变化导致机械中值发生变化
//          printf("phi = %.2f\n",chassis_move.phi_set);
		  chassis_move.wheel_motor[0].wheel_T=lqr_k[0]*(chassis_move.x_set-chassis_move.x)
		  																+lqr_k[1]*(chassis_move.v_set-chassis_move.v)
		  																+lqr_k[2]*(chassis_move.phi_set-chassis_move.myPithR)
		  																+lqr_k[3]*(chassis_move.d_phi_set-chassis_move.myPithGyroR);

		  chassis_move.wheel_motor[1].wheel_T=lqr_k[4]*(chassis_move.x_set-chassis_move.x)
		  																+lqr_k[5]*(chassis_move.v_set-chassis_move.v)
		  																+lqr_k[6]*(chassis_move.phi_set-chassis_move.myPithR)
		  																+lqr_k[7]*(chassis_move.d_phi_set-chassis_move.myPithGyroR);


		  chassis_move.wheel_motor[1].wheel_T=0.0f-chassis_move.wheel_motor[1].wheel_T;

		 turn_T= Turn(chassis_move.total_yaw,INS.Gyro[2]);
		 chassis_move.wheel_motor[0].wheel_T=chassis_move.wheel_motor[0].wheel_T - turn_T;
		 chassis_move.wheel_motor[1].wheel_T=chassis_move.wheel_motor[1].wheel_T - turn_T;


		  mySaturate(&chassis_move.wheel_motor[0].wheel_T,-1.2f,1.2f);
		  mySaturate(&chassis_move.wheel_motor[1].wheel_T,-1.2f,1.2f);

		float leg_set_base = chassis_move.leg_set;
		float roll_error = chassis_move.roll_set - chassis_move.myRoll;
		float delta_leg = 0.2f * roll_error;
		mySaturate(&delta_leg, -0.15f, 0.15f);

		// 分别计算左右腿的期望长度
		float left_leg_set  = leg_set_base + delta_leg;
		float right_leg_set = leg_set_base - delta_leg;

		// 限制总腿长范围，防止结构超限
		mySaturate(&left_leg_set,  0.07f, 0.30f);
		mySaturate(&right_leg_set, 0.07f, 0.30f);

		vmc.roll_F0=roll_kp*(chassis_move.roll_set-chassis_move.myRoll)+roll_kd*INS.Gyro[0];

		 if(chassis_move.jump_flag==1||chassis_move.jump_flag==2||chassis_move.jump_flag==3)
		 {
		 	if(chassis_move.jump_flag==1)
		 	{//压缩阶段
		 		vmc.left_F0=mg_L+leg_kp*(0.08f-vmc.left_len) + leg_kd*vmc.left_len_dot;
		 		vmc.right_F0=mg_R+leg_kp*(0.08f-vmc.right_len) - leg_kd*vmc.right_len_dot;
		 		if(vmc.left_len<0.11f&&vmc.right_len<0.11f)
		 		{
		 			jump_time++;
		 		}
		 		if(jump_time>=4)
		 		{
		 			jump_time=0;
		 			chassis_move.jump_flag=2;//压缩完毕进入上升加速阶段
		 		}
		 	}
		 	else if(chassis_move.jump_flag==2)
		 	{//上升加速阶段
		 		vmc.left_F0=mg_L+leg_kp*(0.6f-vmc.left_len) + leg_kd*vmc.left_len_dot;
		 		vmc.right_F0=mg_R+leg_kp*(0.6f-vmc.right_len) - leg_kd*vmc.right_len_dot;

		 		if(vmc.left_len>0.27f&&vmc.right_len>0.27f)
		 		{
		 			jump_time++;
		 		}
		 		if(jump_time>=3)
		 		{
		 			jump_time=0;
		 			chassis_move.jump_flag=3;//上升完毕进入缩腿阶段
		 		}
		 	}
		 	else if(chassis_move.jump_flag==3)
		 	{//缩腿阶段
		 		vmc.left_F0=leg_kp*(-0.1f-vmc.left_len) + leg_kd*vmc.left_len_dot;
		 		vmc.right_F0=leg_kp*(-0.1f-vmc.right_len) - leg_kd*vmc.right_len_dot;
		 		chassis_move.x_set=chassis_move.x;
		 		if(vmc.left_len<0.08f&&vmc.right_len<0.08f)
		 		{
		 			jump_time++;
		 		}
		 		if(jump_time>=2)
		 		{
		 			jump_time=0;
		 			chassis_move.leg_set=0.12f;
		 			chassis_move.last_leg_set=0.12f;
		 			chassis_move.jump_flag=4;//缩腿完毕
		 		}
		 	}
		 }
		 else
		 {
		 	//vmc.left_F0=mg_L+leg_kp*(chassis_move.leg_set-vmc.left_len) + leg_kd*vmc.left_len_dot;
		 	//vmc.right_F0=mg_R+leg_kp*(chassis_move.leg_set-vmc.right_len) - leg_kd*vmc.right_len_dot;
		 	vmc.left_F0  = mg_L + leg_kp * (left_leg_set  - vmc.left_len)  + leg_kd * vmc.left_len_dot;
		 	vmc.right_F0 = mg_R + leg_kp * (right_leg_set - vmc.right_len) - leg_kd * vmc.right_len_dot;
		 }

		 if(vmc.left_len>0.12f&&vmc.right_len>0.12f&&chassis_move.jump_flag==4)
		 {
		 	chassis_move.jump_flag=0;
		 }
//离地检测
		if(chassis_move.recover_flag==0)
		{//倒地自起不需要检测是否离地
			//离地检测
			if((vmc.left_F0<0.1f&&vmc.right_F0<0.1f&&chassis_move.leg_flag==0&&chassis_move.jump_flag!=1&&chassis_move.jump_flag!=2)
					||chassis_move.jump_flag==3||chassis_move.jump_flag==4)
			{//离地了
				chassis_move.wheel_motor[0].wheel_T=0.0f;
				chassis_move.wheel_motor[1].wheel_T=0.0f;
				chassis_move.x=0.0f;
				chassis_move.x_set=chassis_move.x;
				chassis_move.turn_set=chassis_move.total_yaw;
			}
			else
			{
				vmc.left_F0=vmc.left_F0+vmc.roll_F0;
				vmc.right_F0=vmc.right_F0-vmc.roll_F0;
			}
		}
		else if(chassis_move.recover_flag==1)
		{
			vmc.left_F0=0.0f;
			vmc.right_F0=0.0f;
		}

		 //关节电机
		 float left_actual_angle =get_left_actual_angle(chassis_move.joint_motor[0].para.pos);
		 float right_actual_angle =get_right_actual_angle(chassis_move.joint_motor[1].para.pos);

		 vmc.left_T1=-2.0f*vmc.left_l1*arm_sin_f32(left_actual_angle)*vmc.left_F0;
		 vmc.right_T1=2.0f*vmc.right_l1*arm_sin_f32(right_actual_angle)*vmc.right_F0;

		 if(chassis_move.jump_flag==1||chassis_move.jump_flag==2||chassis_move.jump_flag==3)
		 {//跳跃的时候需要更大扭矩
		 	mySaturate(&vmc.left_T1,-16.0f,16.0f);
		 	mySaturate(&vmc.right_T1,-16.0f,16.0f);
		 }
		 else
		 {//不跳跃的时候
		 	mySaturate(&vmc.left_T1,-10.0f,10.0f);
		 	mySaturate(&vmc.right_T1,-10.0f,10.0f);
		 }

		 if(chassis_move.start_flag==1)
		 {
		 	mit_ctrl2(&hfdcan1,0x02,  0.0f, 0.0f, 0.0f,  0.0f,vmc.left_T1);//左边关节电机
		 	mit_ctrl2(&hfdcan2,0x02,  0.0f, 0.0f, 0.0f,  0.0f,vmc.right_T1);//右边关节电机

		 	mit_ctrl1(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[0].wheel_T);//左边轮毂电机
		 	mit_ctrl1(&hfdcan2,0x01, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[1].wheel_T);//右边轮毂电机
		 	osDelay(CHASSR_TIME);
		 }
		 else if(chassis_move.start_flag==0)
		 {
		 	chassis_move.turn_set=chassis_move.total_yaw;
		 	chassis_move.roll_set=0.0f;
		 	chassis_move.leg_set=0.12f;//原始腿长
		 	chassis_move.x=0.0f;
		 	chassis_move.x_set=chassis_move.x;

		 	mit_ctrl2(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//左边关节电机
		 	mit_ctrl2(&hfdcan2,0x02, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//右边关节电机

		 	mit_ctrl1(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);//左边轮毂电机
		 	mit_ctrl1(&hfdcan2,0x01, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);//右边轮毂电机
		 	osDelay(CHASSR_TIME);
	     }
	}
}