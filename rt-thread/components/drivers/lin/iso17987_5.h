/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2026 Althrone <mail>
 * 
 * @file    rt-thread\components\drivers\lin\iso17987_5.h
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_components_drivers_lin_iso17987_5_h_
#define NANOH7_rt_thread_components_drivers_lin_iso17987_5_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

/*!
 * @brief 0 is false, and non-zero (>0) is true
 */
typedef bool l_bool;

// l_ioctl_op implementation dependent
// l_irqmask implementation dependent

/*!
 * @brief Unsigned 8 bit integer
 */
typedef uint8_t l_u8;

/*!
 * @brief Unsigned 16 bit integer
 */
typedef uint16_t l_u16;

typedef union
{
    l_u16 u16;
    struct
    {
        l_u16 Error_in_response:1;
        l_u16 Successful_transfer:1;
        l_u16 Overrun:1;
        l_u16 Go_to_sleep:1;
        l_u16 Bus_activity:1;
        l_u16 Event_triggered_frame_collision:1;
        l_u16 Save_configuration :1;
        l_u16 :1;
        l_u16 Last_frame_PRID :8;
    };
}ISO17987_5StatusUnion;

/****SIGNAL INTERACTION*****/

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description Reads and returns the current value of the signal.
 * @Reference See signal types in ISO 17987-3.
 * 
 * @param sss 
 * @return l_bool 
 */
#define l_bool_rd(sss)      l_bool_rd_##sss()
#define l_u8_rd(sss)        l_u8_rd_##sss()
#define l_u16_rd(sss)       l_u16_rd_##sss()

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description Sets the current value of the signal to v.
 * @Reference See signal types in ISO 17987-3.
 * 
 * @param v 
 */
#define l_bool_wr(sss,v)    l_bool_wr_##sss(v)
#define l_u8_wr(sss,v)      l_u8_wr_##sss(v)
#define l_u16_wr(sss,v)     l_u16_wr_##sss(v)

// void l_bytes_rd (l_signal_handle sss,
//                  l_u8 start, /* first byte to read from */
//                  l_u8 count, /* number of bytes to read */
//                  l_u8* const data) /* where data will be written */
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description Reads and returns the current values of the selected bytes in the signal. The sum of start and count shall never be greater than the length of the byte array.
 * @example Assume that a byte array is 6 bytes long, numbered 0 to 5. Reading 
 *          byte 2 and 3 from this array requires the start to be 2 (skipping 
 *          byte 0 and 1) and count to be 2 (reading byte 2 and 3). In this 
 *          case byte 2 is written to data [0] and byte 3 is written to data 
 *          [1].
 * @Reference See signal types in ISO 17987-3.
 * 
 * @param start 
 * @param count 
 * @param data 
 */
#define l_bytes_rd(sss,start,count,data) l_bytes_rd_##sss(start,count,data)


// where sss is the name of the signal, e.g. l_bytes_wr_EngineSpeed ().
// void l_bytes_wr (l_signal_handle sss,
//                  l_u8 start, /* first byte to write to */
//                  l_u8 count, /* number of bytes to write */
//                  const l_u8* const data) /* where data is read from */
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description Sets the current value of the selected bytes in the signal specified by the name sss to the value specified.
 *              The sum of start and count shall never be greater than the length of the byte array, although the device driver may choose not to enforce this in runtime.
 * @example Assume that a byte array is 7 bytes long, numbered 0 to 6. Writing 
 *          byte 3 and 4 from this array requires the start to be 3 (skipping 
 *          byte 0, 1 and 2) and count to be 2 (writing byte 3 and 4). In this 
 *          case byte 3 is read from data [0] and byte 4 is read from data [1].
 * @Reference See signal types in ISO 17987-3.
 * 
 * @param start 
 * @param count 
 * @param data 
 */
#define l_bytes_wr(sss,start,count,data) l_bytes_wr_##sss(start,count,data)

/****NOTIFICATION*****/
// Where fff is the name of the flag, e.g. l_flg_tst_RxEngineSpeed ().
// l_bool l_flg_tst (l_flag_handle fff)
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description Returns a C boolean indicating the current state of the flag specified by the name fff, i.e. returns zero if the flag is cleared, non-zero otherwise.
 * @example A flag, named tx confirmation, is attached to a published signal 
 *          valve position stored in the IO_1 frame. The static implementation 
 *          of the l_flg_tst will be:
 *          l_bool l_flg_tst_txconfirmation (void);
 *          The flag will be set when the IO_1 frame (containing the signal 
 *          valve position) is successfully transmitted from the node.
 * @Reference No reference, flags are API specific and not described anywhere else.
 * 
 * @return l_bool 
 */
#define l_flg_tst(fff) l_flg_tst_##fff()

// Where fff is the name of the flag, e.g. l_flg_clr_RxEngineSpeed ().
// void l_flg_clr (l_flag_handle fff)
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description Sets the current value of the flag specified by the name fff to zero.
 * @Reference No reference, flags are API specific and not described anywhere else.
 * 
 */
#define l_flg_clr(fff) l_flg_clr_##fff()

/****SCHEDULE MANAGEMENT*****/

// where iii is the name of the interface, e.g. l_sch_tick_MyLinIfc ().
// l_u16 l_sch_tick (l_ifc_handle iii)
/**
 * @brief 
 * @Applicability Master nodes only.
 * @Description The l_sch_tick function follows a schedule. When a frame becomes due, its transmission is initiated. When the end of the current schedule is reached, l_sch_tick starts again at the beginning of the schedule.
 *              The l_sch_tick shall be called periodically and individually for each interface within the node. The period is the time base, see schedule tables in ISO 17987-3, set in the LDF, see participating nodes in ISO 17987-2. The period of the l_sch_tick call effectively sets the time base tick, see schedule tables in ISO 17987-3. Therefore it is essential that the time base period is uphold with minimum jitter.
 *              The call to l_sch_tick will not only start the transition of the next frame due, it will also update the signal values for those signals received since the previous call to l_sch_tick, see reception and transmission in ISO 17987-3.
 *  @return value Zero, if the next call of l_sch_tick will not start 
 *          transmission of a frame.
 *          Non-zero,if the next call of l_sch_tick will start the transmission 
 *          of the frame in the next schedule table entry. The return value 
 *          will in this case be the next schedule table entry's number 
 *          (counted from the beginning of the schedule table) in the schedule 
 *          table. The return value will be in range 1 to N if the schedule 
 *          table has N entries.
 * @Reference See schedule tables in ISO 17987-3.
 */
#define l_sch_tick(iii) l_sch_tick_##iii()

// where iii is the name of the interface, e.g. l_sch_set_MyLinIfc (MySchedule1, 0).
// void l_sch_set (l_ifc_handle iii,l_schedule_handle schedule,l_u16 entry)
/**
 * @brief 
 * @Applicability Master node only.
 * @Description Sets up the next schedule to be followed by the l_sch_tick function for a certain interface iii. The new schedule will be activated as soon as the current schedule reaches its next schedule entry point. The extension “_iii“ is the interface name. It is optional and the intention is to solve naming conflicts when the node is a master on more than one LIN cluster.
 *              The entry defines the starting entry point in the new schedule table. The value should be in the range 0 to N if the schedule table has N entries, and if entry is 0 or 1 the new schedule table will be started from the beginning.
 *              A predefined schedule table, L_NULL_SCHEDULE, shall exist and may be used to stop all transfers on the LIN cluster.
 * @example A possible use of the entry value is in combination with the 
 *          l_sch_tick return value to temporarily interrupt one schedule with 
 *          another schedule table, and still be able to switch back to the 
 *          interrupted schedule table at the point where this was interrupted.
 * @Reference See schedule tables in ISO 17987-3.
 * @param schedule 
 * @param entry 
 */
#define l_sch_set(iii,schedule,entry) l_sch_set_##iii(schedule,entry)

/****INTERFACE MANAGEMENT*****/
// Where iii is the name of the interface, e.g. l_ifc_init_MyLinIfc ().
// l_bool l_ifc_init (l_ifc_handle iii)
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description l_ifc_init initializes the controller specified by the name iii, i.e. sets up internal functions such as the baud rate. The default schedule set by the l_ifc_init call will be the L_NULL_SCHEDULE where no frames will be sent and received.
 *              This is the first call a user shall perform before using any other interface related LIN API functions.
 *              The function returns zero if the initialisation was successful and non-zero if failed.
 * @Reference A general description of the interface concept is found in concept of operation in ISO 17987-3.
 * @return l_bool 
 */
#define l_ifc_init(iii) l_ifc_init_##iii()

// Where iii is the name of the interface, e.g. l_ifc_goto_sleep_MyLinIfc ().
// void l_ifc_goto_sleep (l_ifc_handle iii)
/**
 * @brief 
 * @Applicability Master node only.
 * @Description This call requests slave nodes on the cluster connected to the interface to enter bus sleep mode by issuing one go to sleep command, see go to sleep in ISO 17987-2.
 *              The go to sleep command will be scheduled latest when the next schedule entry is due.
 *              The l_ifc_goto_sleep will not affect the power mode. It is up to the application to do this.
 *              If the go to sleep command was successfully transmitted on the cluster the go to sleep bit will be set in the status register, see go to sleep in ISO 17987-2.
 * @Reference See go to sleep in ISO 17987-2.
 */
#define l_ifc_goto_sleep(iii) l_ifc_goto_sleep_#iii()

// where iii is the name of the interface, e.g. l_ifc_wake_up_MyLinIfc ().
// void l_ifc_wake_up (l_ifc_handle iii)
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The function will transmit one wake up signal. The wake up signal will be transmitted directly when this function is called. It is the responsibility of the application to retransmit the wake up signal according to the wake up sequence defined in ISO 17987-2
 * @Reference See wake up in ISO 17987-2.
 */
#define l_ifc_wake_up(iii) l_ifc_wake_up_##iii()

// where iii is the name of the interface, e.g. l_ifc_ioctl_MyLinIfc (MyOp, &MyPars).
// l_u16 l_ifc_ioctl (l_ifc_handle iii, l_ioctl_op op, void* pv)
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description This function controls functionality that is not covered by the other API calls. It is used for protocol specific parameters or hardware specific functionality. Example of such functionality can be to switch on/off the wake up signal detection.
 *              The iii is the name of the interface to which the operation defined in op should be applied. The pointer pv points to an optional parameter that may be provided to the function.
 *              Exactly which operations that are supported is implementation dependent.
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 * @param op 
 * @param pv 
 * @return l_u16 
 */
#define l_ifc_ioctl(iii,op,pv) l_ifc_ioctl_##iii(op,pv)

// where iii is the name of the interface, e.g. l_ifc_rx_MyLinIfc ().
// void l_ifc_rx (l_ifc_handle iii)
/**
 * @brief 
 * @Applicability Master and slave nodes. 
 * @Description The application program is responsible for binding the interrupt and for setting the correct interface handle (if interrupt is used).
 *              For UART based implementations it may be called from a user-defined interrupt handler triggered by a UART when it receives one character of data. In this case the function will perform necessary operations on the UART control registers.
 *              For more complex LIN hardware it may be used to indicate the reception of a complete frame. 
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 */
#define l_ifc_rx(iii) l_ifc_rx_##iii()

// where iii is the name of the interface, e.g. l_ifc_tx_MyLinIfc ().
// void l_ifc_tx (l_ifc_handle iii)
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The application program is responsible for binding the interrupt and for setting the correct interface handle (if interrupt is used).
 *              For UART based implementations it may be called from a user-defined interrupt handler triggered by a UART when it has transmitted one character of data. In this case the function will perform necessary operations on the UART control registers.
 *              For more complex LIN hardware it may be used to indicate the transmission of a complete frame.
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 */
#define l_ifc_tx(iii) l_ifc_tx_##iii()

// Where iii is the name of the interface, e.g. l_ifc_aux_MyLinIfc ().
// void l_ifc_aux (l_ifc_handle iii)
/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description This function may be used in the slave nodes to synchronize to the break/sync field sequence transmitted by the master node on the interface specified by iii.
 *              It may, for example, be called from a user-defined interrupt handler raised upon a flank detection on a hardware pin connected to the interface iii.
 *              l_ifc_aux may only be used in a slave node.
 *              This function is strongly hardware connected and the exact implementation and usage is implementation dependent.
 *              This function might even be empty in cases where the break/sync field sequence detection is implemented in the l_ifc_rx function.
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 * 
 */
#define l_ifc_aux(iii) l_ifc_aux_##iii()

// where iii is the name of the interface, e.g. l_ifc_read_status_MyLinIfc ().
// l_u16 l_ifc_read_status (l_ifc_handle iii)
/**
 * @brief 
 * @Applicability Master and slave nodes. The behaviour is different for master and slave nodes, see description below. 
 * @Description This function will return the status of the previous communication. The call returns the status word (16 bit value), as shown in Table 20. 
 * @Reference See status management in ISO 17987-3.
 * 
 * @return l_u16 
 */
#define l_ifc_read_status(iii) l_ifc_read_status_##iii()

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_components_drivers_lin_iso17987_5_h_ */