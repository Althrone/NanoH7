/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2026 Althrone <mail>
 * 
 * @file    rt-thread\components\drivers\lin\iso17987_5_cfg.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_components_drivers_lin_iso17987_5_cfg_h_
#define NANOH7_rt_thread_components_drivers_lin_iso17987_5_cfg_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum {

   /* Interface_name = LI0 */

   LI0_Motor1ErrorCode

   , LI0_Motor1ErrorValue
  
   , LI0_Motor1LinError
  
   , LI0_Motor1Selection
  
   , LI0_Motor1Temp
  
   , LI0_MotorDirection
  
   , LI0_MotorSpeed
  
   , LI0_Motor1Position
  
} l_signal_handle;//应自动生成

typedef enum {
    LI0_Motor1Selection_flag,
} l_flag_handle;//应自动生成

typedef enum {
   LI0 = 0x00U,
   INVALID_IFC = 0xFFU
}l_ifc_handle;

typedef enum
{
  LIN_TAB_schTabConfig,     /*   0 */
  LIN_TAB_schTab1,     /*   1 */
  LIN_TAB_evCollisionResolving,     /*   2 */
  LIN_TAB_Diagnostic,     /*   3 */
  LIN_TAB_MasterOnly,     /*   4 */
  LIN_TAB_SlaveOnly,     /*   5 */
  LIN_TAB_GOTO_SLEEP,     /*   6 */
  L_NULL_SCHEDULE,
  LIN_SCHED_TAB_NR_MAX
} l_schedule_handle;

typedef enum
{
    LIN_IOCTL_DRIVER_STATE,
    LIN_IOCTL_READ_FRAME_ID,
    LIN_IOCTL_READ_MESSAGE_ID,
    LIN_IOCTL_READ_FRAME_ID_BY_INDEX,
    LIN_IOCTL_SET_FRAME_ID,
    LIN_IOCTL_FORCE_BUSSLEEP,
    LIN_IOCTL_SET_VARIANT_ID,
    LIN_IOCTL_READ_VARIANT_ID,
    LIN_IOCTL_READ_CONFIG_FLAGS,
    LIN_IOCTL_READ_NAD,
    LIN_IOCTL_WRITE_NAD,
    LIN_IOCTL_WRITE_INITIAL_NAD,
} l_ioctl_op;

typedef enum
{

}l_irqmask;

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_components_drivers_lin_iso17987_5_cfg_h_ */