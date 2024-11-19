/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2024 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\algorithm\ekf.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "arm_math.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

arm_matrix_instance_f32 q;//姿态四元数
float32_t p_q_data[4][1]={1,0,0,0};
arm_matrix_instance_f32 q_new;//姿态四元数
float32_t p_q_new_data[4][1]={0};

arm_matrix_instance_f32 delta_q;//姿态四元数
float32_t p_delta_q_data[4][4]={
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1
};

arm_matrix_instance_f32 q_type_body2nav_rotation_matrix;
float32_t p_q_type_body2nav_rotation_matrix_data[3][3]={
    1,0,0,
    0,1,0,
    0,0,1
};//默认w=1 三个虚部为0

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

float Q_rsqrt( float number );

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

void q_init(void)
{
    arm_mat_init_f32(&q,4,1,p_q_data);
    arm_mat_init_f32(&q_new,4,1,p_q_new_data);
    arm_mat_init_f32(&delta_q,4,4,p_delta_q_data);
    arm_mat_init_f32(&q_type_body2nav_rotation_matrix,3,3,p_q_type_body2nav_rotation_matrix_data);

}

/*
┌  ┐   ┌                   ┐┌  ┐
│q0│   │+∆q0 -∆q1 -∆q2 -∆q3││q0│
│q1│ = │+∆q1 +∆q0 +∆q3 -∆q2││q1│
│q2│   │+∆q2 -∆q3 +∆q0 +∆q1││q2│
│q3│   │+∆q3 +∆q2 -∆q1 +∆q0││q3│
└  ┘k+1└                   ┘└  ┘k
∆q0=1 ∆q1=∆θx/2 ∆q2=∆θy/2 ∆q3=∆θz/2
∆θx=ωx*∆t
*/

/**
 * @param   omega_x: 单位md/s，角度制
 * @param   delta_usec: 单位us
 * @note    保证传入的角速度已经减去零飘
 **/
void q_update(int32_t omega_x,int32_t omega_y,
              int32_t omega_z,uint32_t delta_usec)
{
    // 小角度正弦值近似等于角的弧度值
    // (md/s)*us*(π/180°)/2
    float32_t delta_q1=omega_x*(PI/180)*delta_usec/1000/1000000/2;
    float32_t delta_q2=omega_y*(PI/180)*delta_usec/1000/1000000/2;
    float32_t delta_q3=omega_z*(PI/180)*delta_usec/1000/1000000/2;

    p_delta_q_data[0][1]=-delta_q1;
    p_delta_q_data[0][2]=-delta_q2;
    p_delta_q_data[0][3]=-delta_q3;

    p_delta_q_data[1][0]=delta_q1;
    p_delta_q_data[1][2]=delta_q3;
    p_delta_q_data[1][3]=-delta_q2;

    p_delta_q_data[2][0]=delta_q2;
    p_delta_q_data[2][1]=-delta_q3;
    p_delta_q_data[2][3]=delta_q1;

    p_delta_q_data[3][0]=delta_q3;
    p_delta_q_data[3][1]=delta_q2;
    p_delta_q_data[3][2]=-delta_q1;

    arm_mat_mult_f32(&delta_q,&q,&q_new);
    //归一化

    // 格拉斯曼积≈数学多项式乘法≈乘积
    // 点积：元素一对一乘积的和，返回一个标量
    // q/√(q·q)
    arm_mat_scale_f32(&q_new,
                      Q_rsqrt(p_q_new_data[0][0]*p_q_new_data[0][0]+
                              p_q_new_data[1][0]*p_q_new_data[1][0]+
                              p_q_new_data[2][0]*p_q_new_data[2][0]+
                              p_q_new_data[3][0]*p_q_new_data[3][0]),
                      &q);
    
}

/*
┌  ┐   ┌  ┐        ┌  ┐      ┌ ┐
│VN│   │VN│   ┌ ┐N │ax│      │0│
│VE│ = │VE│ + │T│ *│ay│*∆t + │0│*∆t
│VD│   │VD│   └ ┘B │az│      │g│
└  ┘k+1└  ┘k       └  ┘      └ ┘
并减去重力，但是不知道为啥公式是加重力
//对于自由落体设备，IMU获取值为0，但实际上往下掉，所以加g来测速度
//对于静置桌面设备，IMU获取值为向上一个g，综合计算就为0
*/

/**
 * @note    保证传入的加速度已经减去零飘
 **/
void v_update(int32_t a_x,int32_t a_y,int32_t a_z,uint32_t delta_usec)
{
    
}

/*
                 ╭             ╮
┌  ┐   ┌  ┐      │ ┌  ┐   ┌  ┐ │
│PN│   │PN│      │ │VN│   │VN│ │
│PE│ = │PE│ + 1/2│ │VE│ + │VE│ │*∆t
│PD│   │PD│      │ │VD│   │VD│ │
└  ┘k+1└  ┘k     │ └  ┘k+1└  ┘k│
                 ╰             ╯
*/

/**
 * @note    其实可以和上面一个函数揉成一坨
 */
void p_update(int32_t a_x,int32_t a_y,int32_t a_z,uint32_t delta_usec)
{

}

/**
 * @brief   四元数形式的旋转矩阵
 * @note    zyx旋转次序，北东地，前右下坐标系，从集体坐标系到导航坐标系的旋转
 * @note    传入的是姿态的四元数
 **/
void rotation_matrix_body2nav(float32_t q0,float32_t q1,
                              float32_t q2,float32_t q3,
                              float32_t bx,float32_t by,float32_t bz,
                              float32_t nx,float32_t ny,float32_t nz)
{
    float32_t q0q0=q0*q0;
    float32_t q0q1=q0*q1;
    float32_t q0q2=q0*q2;
    float32_t q0q3=q0*q3;
    float32_t q1q1=q1*q1;
    float32_t q1q2=q1*q2;
    float32_t q1q3=q1*q3;
    float32_t q2q2=q2*q2;
    float32_t q2q3=q2*q3;
    float32_t q3q3=q3*q3;

    p_q_type_body2nav_rotation_matrix_data[0][0]=2*(q0q0+q1q1)-1;
    p_q_type_body2nav_rotation_matrix_data[0][1]=2*(q1q2+q0q3);
    p_q_type_body2nav_rotation_matrix_data[0][2]=2*(q1q3-q0q2);
    p_q_type_body2nav_rotation_matrix_data[1][0]=2*(q1q2-q0q3);
    p_q_type_body2nav_rotation_matrix_data[1][1]=2*(q0q0+q2q2)-1;
    p_q_type_body2nav_rotation_matrix_data[1][2]=2*(q2q3+q0q1);
    p_q_type_body2nav_rotation_matrix_data[2][0]=2*(q1q3+q0q2);
    p_q_type_body2nav_rotation_matrix_data[2][1]=2*(q2q3-q0q1);
    p_q_type_body2nav_rotation_matrix_data[2][2]=2*(q0q0+q3q3)-1;

    //需要构造输入输出的3x1矩阵，可能是加速度，速度，距离或者别的需要旋转的东西

    arm_matrix_instance_f32 body;
    float32_t p_body_data[3][1]={bx,by,bz};
    arm_mat_init_f32(&body,3,1,p_body_data);

    arm_matrix_instance_f32 nav;
    float32_t p_nav_data[3][1]={nx,ny,nz};
    arm_mat_init_f32(&nav,3,1,p_nav_data);

    /*
    ┌  ┐         ┌  ┐
    │Nx│   ┌ ┐N  │Bx│
    │Ny│ = │T│ * │By│
    │Nz│   └ ┘B  │Bz│
    └  ┘N        └  ┘B
    */
    arm_mat_mult_f32(&q_type_body2nav_rotation_matrix,&body,&nav);

}

/******************************************************************************
 * private functions definition
 *****************************************************************************/

float Q_rsqrt( float number )
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;                        // evil floating point bit level hacking
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//    y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

#ifndef Q3_VM
#ifdef __linux__
    assert( !isnan(y) ); // bk010122 - FPE?
#endif
#endif
    return y;
}