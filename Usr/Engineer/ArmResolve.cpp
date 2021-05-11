/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    ArmResolve.cpp
  * @author  EnnisKoh (8762322@qq.com) & Hehe
  * @brief   Code for resolve of armmodel like puma650
  * @date    2021-05-01
  * @version 0.4.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    				 <th>Description
  * <tr><td>2021-05-01  <td> 0.4.1   <td>EnnisKoh&Hehe     <td>Creator
  * </table>
  *
  ==============================================================================
                            How to use this driver     
  ==============================================================================
    @note 
		
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */



/* Includes ------------------------------------------------------------------*/ 
#include "ArmResolve.h"
#include <iostream>
using namespace std;
/* function prototypes -------------------------------------------------------*/
//MechanicalArm::MechanicalArm(int i) :T0_6(4, 4), Tc_g(4, 4), Tw_c(4, 4), T6_g(4, 4), Tw_g(4, 4) {};

double rad2deg(double rad)
{
	return rad / 3.14 * 180;
}

double deg2rad(double deg)
{
	return deg / 180 * 3.14;
}

Matrix MechanicalArm::convert_DHModel_to_Matrix(DH_MODEL_Typedef& axis)
{
	Matrix Matrix_ans(4,4);
	/*Matrix_ans = { (float)cos(axis.theta),(float)-sin(axis.theta),0,(float)axis.a ,
		(float)sin(axis.theta) * (float)cos(axis.alpha),(float)cos(axis.theta) * (float)cos(axis.alpha),(float)-sin(axis.alpha),(float)-sin(axis.alpha) * (float)axis.d,
		(float)sin(axis.theta) * (float)sin(axis.alpha),(float)cos(axis.theta) * (float)sin(axis.alpha), (float)cos(axis.alpha) ,(float)cos(axis.alpha) * (float)axis.d,
		0,0,0,1 };*/
	Matrix_ans(1, 1) = 0;
	Matrix_ans(1, 2) = -sin(axis.theta);
	Matrix_ans(1, 3) = 0;
	Matrix_ans(1, 4) = axis.a;
	Matrix_ans(2, 1) = sin(axis.theta) * cos(axis.alpha);
	Matrix_ans(2, 2) = cos(axis.theta) * cos(axis.alpha);
	Matrix_ans(2, 3) = -sin(axis.alpha);
	Matrix_ans(2, 4) = -sin(axis.alpha) * axis.d;
	Matrix_ans(3, 1) = sin(axis.theta) * sin(axis.alpha);
	Matrix_ans(3, 2) = cos(axis.theta) * sin(axis.alpha);
	Matrix_ans(3, 3) = cos(axis.alpha);
	Matrix_ans(3, 4) = cos(axis.alpha) * axis.d;
	Matrix_ans(4, 1) = 0;
	Matrix_ans(4, 2) = 0;
	Matrix_ans(4, 3) = 0;
	Matrix_ans(4, 4) = 1;
	return Matrix_ans;
}

bool MechanicalArm::FK_cal()
{
	/*Matrix T0_1(4,4), T1_2(4, 4), T2_3(4, 4), T3_4(4, 4), T4_5(4, 4), T5_6(4, 4);
	Matrix T(4, 4);
	for (int i = 0; i < 6; i++)
		this->dh_model[i].theta = current_deg.deg[i];
	T0_1 = convert_DHModel_to_Matrix(dh_model[0]);
	T1_2 = convert_DHModel_to_Matrix(dh_model[1]);
	T2_3 = convert_DHModel_to_Matrix(dh_model[2]);
	T3_4 = convert_DHModel_to_Matrix(dh_model[3]);
	T4_5 = convert_DHModel_to_Matrix(dh_model[4]);
	T5_6 = convert_DHModel_to_Matrix(dh_model[5]);
	T = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6;
	world_x = T(1,4);
	world_y = T(2,4);
	world_z = T(3,4);*/
	double l1 = 0.21f, l2 = 0.21f, l3 = 0.29f;
	double c1 = cos(deg2rad(fabs(current_deg.deg[1])));
	double st = sin(deg2rad(90.0f + fabs(current_deg.deg[1]) - current_deg.deg[2]));
	//std::cout << c1 << " " << st << " " << cos(3.14) << " ";
	double l = l2 * c1 + l3 * st;
	//std::cout << l2 * cos(current_deg.deg[1]) << "     " << l3 * sin(90 + current_deg.deg[1] - current_deg.deg[2]) << "\n";
	//delete(&l);
	world_x = l * cos(deg2rad(current_deg.deg[0]))+x_offset;
	world_y = l * sin(deg2rad(current_deg.deg[0]))+y_offset;
	world_z = l1 + l2 * sin(deg2rad(fabs(current_deg.deg[1]))) - l3 * cos(deg2rad(90 + fabs(current_deg.deg[1]) - current_deg.deg[2])) + z_offset;
	roll = current_deg.deg[3];
	pitch = current_deg.deg[4];
	yaw = current_deg.deg[5];
	return true;
}

/**
    * @brief  IK Matrit init
    * @param  Tw_c,T6_g
    * @retval None
    */
int MechanicalArm::Init(Matrix Tw2c, Matrix T62g)
{
	if (goal_input_mode==Vision_Input) this->Tw_c = Tw2c;
	this->T6_g = T62g;
	return 1;
}

/**
    * @brief  Set Vision to goal Matrix
    * @param  Tc_g
    * @retval None
    */
int MechanicalArm::SetVision(Matrix Tc_g)
{
	this->Tc_g = Tc_g;
	return 1;
}

/**
    * @brief  Set World to Goal Matrix
    * @param  Tw_g
    * @retval None
    */
int MechanicalArm::SetWorldGoal(Matrix Tw_g)
{
	this->Tw_g = Tw_g;
	return 1;
}

/**
    * @brief  input DH-model of arm
		* @param  a[6],d[6],interval[6][2]:config of DH-model
    * @retval None
    */
bool MechanicalArm::Set_DHModel_Config(double a[6], double d[6],double interval[6][2])
{
	for (int i = 0; i < 6; i++)
	{
		dh_model[i].a = a[i];
		dh_model[i].d = d[i];
		dh_model[i].min_deg = interval[i][0];
		dh_model[i].max_deg = interval[i][1];
	}
	return true;
}

/**
    * @brief  cal T0_6
    * @param  Node
    * @retval None
    */
int MechanicalArm::solveT0_6()
{
	if (goal_input_mode==Vision_Input) Tw_g = Tw_c*Tc_g;
	if (CP62g)
	{
		Matrix T6_g_I(4, 4);//T6_g inverse
		T6_g_I = T6_g.Inverse();
		Matrix Tw_6(4, 4);
		Tw_6 = Tw_g * T6_g_I;
		T0_6 = Tw_6;//if define axis0 == axisw
	}
	else
	{
		T0_6 = Tw_g;
	}
	return 1;
}

/**
    * @brief  IK cal of puma650 armmodel
    * @param  None
    * @retval None
    */
bool MechanicalArm::IK_cal()
{
	//double p_x = this->T0_6[3], p_y = this->T0_6[7], p_z = this->T0_6[11];
	//double r13 = this->T0_6[2], r23 = this->T0_6[6], r33 = this->T0_6[10];
	//double r11 = this->T0_6[0], r21 = this->T0_6[4], r31 = this->T0_6[8];
	/*double p_x = this->T0_6(1,4), p_y = this->T0_6(2,4), p_z = this->T0_6(3,4);
	double r13 = this->T0_6(1,3), r23 = this->T0_6(2,3), r33 = this->T0_6(3,3);
	double r11 = this->T0_6(1,1), r21 = this->T0_6(2,1), r31 = this->T0_6(3,1);*/
	double p_x = fabs(this->GetTargetx()) < 1e-5 ? this->GetWorldx() : this->GetTargetx();
	double p_y = fabs(this->GetTargety()) < 1e-5 ? this->GetWorldy() : this->GetTargety();
	double p_z = fabs(this->GetTargetz()) < 1e-5 ? this->GetWorldz() : this->GetTargetz();
	double theta1, theta2, theta3, theta4, theta5, theta6;
	double l1 = 0.21f, l2 = 0.21f, l3 = 0.29f;
	if (p_x-x_offset != 0) theta1 = atan2(p_y-y_offset,p_x-x_offset);
	else theta1 = 1.57f;
	this->target_deg.deg[0] = rad2deg(theta1);
	if (this->target_deg.deg[0]<dh_model[0].min_deg || this->target_deg.deg[0]>dh_model[0].max_deg)
	{
		SysLog.Record(_INFO_, "TigerArm", "Theta1 out of range.TigerArm may not reach this target point...");
		return false;
	}
	double k = (p_x-x_offset) * (p_x-x_offset) + (p_y-y_offset) * (p_y-y_offset) + (p_z - z_offset - l1) * (p_z - z_offset - l1) - l2 * l2 - l3 * l3;
	double c3 = k / (2 * l2 * l3);
	if (c3>1)
	{
		SysLog.Record(_INFO_, "TigerArm", "Theta3 error.TigerArm may not reach this target point...");
		return false;
	}
	theta3 = acos((k / (2 * l2 * l3)));
	this->target_deg.deg[2] = rad2deg(theta3);
	if (this->target_deg.deg[2]<dh_model[2].min_deg || this->target_deg.deg[2]>dh_model[2].max_deg) this->target_deg.deg[2] = -this->target_deg.deg[2];
	if (this->target_deg.deg[2]<dh_model[2].min_deg || this->target_deg.deg[2]>dh_model[2].max_deg)
	{
		SysLog.Record(_INFO_, "TigerArm", "Theta3 out of range.TigerArm may not reach this target point...");
		return false;
	}
	theta2 = atan2(p_z-z_offset - l1, sqrt((p_x-x_offset) * (p_x-x_offset) + (p_y-y_offset) * (p_y-y_offset))) + atan2(l3 * sin(theta3), l2 + l3 * cos(theta3));
	this->target_deg.deg[1] = -rad2deg(theta2);
	if (this->target_deg.deg[1]<dh_model[1].min_deg || this->target_deg.deg[1]>dh_model[1].max_deg)
	{
		SysLog.Record(_INFO_, "TigerArm", "Theta2 out of range.TigerArm may not reach this target point...");
		return false;
	}
	NewTarget = ENABLE;
	return true;
	/*for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			//for (int t = 0; t < 2; t++)
			//{//d2!=0!!!!!!!!!!!!!!!!!!!!!!!!!!!
				//theta 1
				double sqrt1[2], theta1;
				sqrt1[0] = sqrt(p_x * p_x + p_y * p_y - d(2) * d(2));
				sqrt1[1] = -sqrt(p_x * p_x + p_y * p_y - d(2) * d(2));
				theta1 = atan2(p_y, p_x) - atan2(d(2), sqrt1[i]);
				//theta 3
				double k = (p_x * p_x + p_y * p_y + p_z * p_z - a(2) * a(2) - a(3) * a(3) - d(2) * d(2) - d(3) * d(3)) / (2 * a(2));
				double sqrt3[2], theta3;
				sqrt3[0] = sqrt(a(3) * a(3) + d(3) * d(3) - k * k);
				sqrt3[1] = -sqrt(a(3) * a(3) + d(3) * d(3) - k * k);
				theta3 = atan2(a(3), d(3)) - atan2(k, sqrt3[j]);
				//theta 2
				double fs23, sc23, theta23, theta2;
				fs23 = (-a(3) - a(2) * cos(theta3)) * p_z + (cos(theta1) * p_x + sin(theta1) * p_y)*(a(2) * sin(theta3) - d(3));
				sc23 = (a(2) * sin(theta3) - d(3)) * p_z - (a(3) + a(2) * cos(theta3))*(cos(theta(1)) * p_x + sin(theta1) * p_y);
				theta23 = atan2(fs23, sc23);
				theta2 = theta23 - theta3;
				//theta 4
				double theta4 = atan2(-r13 * sin(theta1) + r23 * cos(theta1), -r13 * cos(theta1) * cos(theta23) - r23 * sin(theta1) * cos(theta23) + r33 * sin(theta23));
				//theta 5
				double s5, c5, theta5;
				s5 = -r13 * (cos(theta1) * cos(theta23) * cos(theta4) + sin(theta1) * sin(theta4)) - r23 * (sin(theta1) * cos(theta23) * cos(theta4) - cos(theta1) * sin(theta4)) + r33 * (sin(theta23) * cos(theta4));
				c5 = r13 * (-cos(theta1) * sin(theta23)) + r23 * (-sin(theta1) * sin(theta23)) + r33 * (-cos(theta23));
				theta5 = atan2(s5, c5);
				//theta 6
				double s6, c6, theta6;
				s6 = -r11 * (cos(theta1) * cos(theta23) * sin(theta4) - sin(theta1) * cos(theta4)) - r21 * (sin(theta1) * cos(theta23) * sin(theta4) + cos(theta1) * cos(theta4)) + r31 * (sin(theta23) * sin(theta4));
				c6 = r11 * ((cos(theta1) * cos(theta23) * cos(theta4) + sin(theta1) * sin(theta4)) * cos(theta5) - cos(theta1) * sin(theta23) * sin(theta5)) + r21 * ((sin(theta1) * cos(theta23) * cos(theta4) - cos(theta1) * sin(theta4)) * cos(theta5) - sin(theta1) * sin(theta23) * sin(theta5)) - r31 * (sin(theta23) * cos(theta4) * cos(theta5) + cos(theta23) * sin(theta5));
				theta6 = atan2(s6, c6);
				//convert to degree
				theta[4 * i + 2 * j][0] = theta1 * 180 / PI;
				theta[4 * i + 2 * j + 1][0] = theta1 * 180 / PI;
				
				theta[4 * i + 2 * j][1] = theta2 * 180 / PI;
				theta[4 * i + 2 * j + 1][1] = theta2 * 180 / PI;
				
				theta[4 * i + 2 * j][2] = theta3 * 180 / PI;
				theta[4 * i + 2 * j + 1][2] = theta3 * 180 / PI;
				
				theta[4 * i + 2 * j][3] = theta4 * 180 / PI;
				theta[4 * i + 2 * j + 1][3] = theta4 * 180 / PI + 180;
				
				theta[4 * i + 2 * j][4] = theta5 * 180 / PI;
				theta[4 * i + 2 * j + 1][4] = -theta5 * 180 / PI;
				
				theta[4 * i + 2 * j][5] = theta6 * 180 / PI;
				theta[4 * i + 2 * j + 1][5] = theta6 * 180 / PI + 180;
			//}
		}
	}
	bool flag;
	for (int i = 0; i < 8; i++)
	{
		flag = true;
		for (int j = 0; j < 6; j++)
		{
			if (theta[i][j]<dh_model[j].min_deg || theta[i][j]>dh_model[j].max_deg)
			{
				flag = false;
				break;
			}
		}
		if (flag)
		{
			for (int j = 0; j < 6; j++)
			{
				target_deg.deg[j] = theta[i][j];
			}
			return true;
		}
	}
	return false;*/

}

/**
    * @brief  get_IK_ans
    * @param  Node
    * @retval Ik_ans
    */
theta_deg_pack MechanicalArm::get_IK_ans()
{
	return target_deg;
}

/**
    * @brief  cal current target of arm
    * @param  uint32_t now_time
    * @retval curtarget_deg
    */
theta_deg_pack MechanicalArm::get_curtarget_deg(uint32_t now_time)
{
	double t=now_time-last_IP_time;
	if (t > tf) t = tf;
	for (int i=0;i<6;i++)
	{
		curtarget_deg.deg[i]=joint_IP_data[i].a[0]+joint_IP_data[i].a[1]*t+joint_IP_data[i].a[2]*t*t+joint_IP_data[i].a[3]*t*t*t;
	}
	return curtarget_deg;
}

bool MechanicalArm::ReachTargetDeg()
{
	for (int i = 0; i < 6; i++)
	{
		if (fabs(current_deg.deg[i] - target_deg.deg[i]) > 1e-5) return false;
	}
	return true;
}

/**
    * @brief  set and cal cubic IP config
    * @param  cur deg & uint32_t now_time
    * @retval None
    */
void MechanicalArm::Set_Cubic_IP_Config(uint32_t now_time)
{
	last_IP_time=now_time;
	for (int i=0;i<6;i++)
	{
		joint_IP_data[i].a[0]=current_deg.deg[i];
		joint_IP_data[i].a[1]=0.0f;
		joint_IP_data[i].a[2]=3.0f*(target_deg.deg[i]-current_deg.deg[i])/(tf*tf);
		joint_IP_data[i].a[3]=-2.0f*(target_deg.deg[i]-current_deg.deg[i])/(tf*tf*tf);
	}
}

/**
    * @brief  update current deg
    * @param  cur deg
    * @retval None
    */
void MechanicalArm::update(theta_deg_pack* cur)
{
	for (int i=0;i<6;i++)
		current_deg.deg[i]=cur->deg[i];
	this->FK_cal();
	return;
}

