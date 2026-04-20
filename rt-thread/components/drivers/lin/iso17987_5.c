/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2026 Althrone <mail>
 * 
 * @file    rt-thread\components\drivers\lin\iso17987_5.c
 * 
 * ref: Specification of <some UM RM or Datasheet>
 *****************************************************************************/

/******************************************************************************
 * includes
 *****************************************************************************/

#include "iso17987_5.h"
#include "iso17987_5_cfg.h"

/******************************************************************************
 * private macros
 *****************************************************************************/

/******************************************************************************
 * pubilc variables
 *****************************************************************************/

/******************************************************************************
 * private types
 *****************************************************************************/

/******************************************************************************
 * private variables
 *****************************************************************************/

/******************************************************************************
 * private functions declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions definition
 *****************************************************************************/

/****DRIVER AND CLUSTER MANAGEMENT*****/
/**
 * @brief l_sys_init performs the initialization of the LIN core.
 * @Applicability Master and slave nodes.
 * @Description l_sys_init performs the initialization of the LIN core. The scope of the initialization is the physical node i.e., the complete node (see node composition definition in ISO 17987-2).
 *              The call to the l_sys_init is the first call a user shall use in the LIN core before using any other API functions.
 * 
 * @return Zero if the initialization succeeded. 
 *         Non-zero if the initialization failed.
 */
l_bool l_sys_init (void)
{
    return 0;
}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The user implementation of this function shall achieve a state in which no interrupts from the LIN communication can occur.
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 * 
 * @return l_irqmask 
 */
l_irqmask l_sys_irq_disable (void)
{

}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The user implementation of this function shall restore the interrupt level identified by the provided l_irqmask previous.
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 * 
 * @param previous 
 */
void l_sys_irq_restore (l_irqmask previous)
{

}

// Prototype
/**
 * @brief 
 * @Applicability Master node only.
 * @Description This call returns the status of the last requested configuration service.
 * 
 * @param iii 
 * @return LD_SERVICE_BUSY: Service is ongoing.
 *         LD_REQUEST_FINISHED: The configuration request has been completed. This is a intermediate status between the configuration request and configuration response.
 *         LD_SERVICE_IDLE: The configuration request/response combination has been completed, i.e. the response is valid and may be analyzed. Also, this value is returned if no request has yet been called.
 *         LD_SERVICE_ERROR: The configuration request or response experienced an error.Error here means error on the bus, and not a negative configuration response from the slave node.
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 */
l_u8 ld_is_ready (l_ifc_handle iii)
{

}

// Prototype
/**
 * @brief 
 * @Applicability Master node only.
 * @Description This call returns the result of the last node configuration service, in the parameters RSID and error_code. A value in RSID is always returned but not always in the error_code. Default values for RSID and error_code is 0 (zero).
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 * 
 * @param iii 
 * @param RSID 
 * @param error_code 
 */
void ld_check_response (l_ifc_handle iii,
                        l_u8* const RSID,
                        l_u8* const error_code)
{

}

// Prototype
/**
 * @brief 
 * @Applicability Master node only.
 * @Description This call assigns the protected identifier of up to four frames in the slave node with the addressed NAD. The PRIDs parameter shall be four bytes long, each byte shall contain a PRID, do not care or unassign value.
 * @Reference See service assign frame id range in ISO 17987-3.
 * 
 * @param iii 
 * @param NAD 
 * @param start_index 
 * @param PRIDs 
 */
void ld_assign_frame_id_range (l_ifc_handle iii,
                               l_u8 NAD,
                               l_u8 start_index,
                               const l_u8* const PRIDs)
{
    
}

// Prototype
/**
 * @brief 
 * @Applicability Master node only.
 * @Description This call assigns the NAD (node diagnostic address) of all slave nodes that matches the initial_NAD, the supplier ID and the function ID. The new NAD of the slave node will be new_NAD.
 * @Reference See the definition of the service assign NAD, see ISO 17987-3.
 */
void ld_assign_NAD (l_ifc_handle iii,
                    l_u8 initial_NAD,
                    l_u16 supplier_id,
                    l_u16 function_id,
                    l_u8 new_NAD)
{

}

// Prototype
/**
 * @brief 
 * @Applicability Master node only.
 * @Description This call will make a save configuration request to a specific slave node with the given NAD, or to all slave nodes if NAD is set to broadcast.
 * @Reference See the definition of the save configuration service in ISO 17987-3. API call l_ifc_read_status see 7.3.7.9 and example in 7.6.
 * 
 * @param iii 
 * @param NAD 
 */
void ld_save_configuration (l_ifc_handle iii,
                            l_u8 NAD)
{
    
}

/**
 * @brief 
 * @Applicability Slave node only.
 * @Description This function will not transport anything on the bus.
 *              This call will serialize the current configuration and copy it to the area (data pointer) provided by the application. The intention is to call this function when the save configuration request flag is set in the status register, see 7.3.7.9. After the call is finished the application is responsible to store the data in appropriate memory.
 *              The caller shall reserve bytes in the data area equal to length, before calling this function. The function will set the length parameter to the actual size of the configuration. In case the data area is too short the function will return with no action.
 *              In case the NAD has not been set by a previous call to ld_set_configuration or the master node has used the configuration services, the returned NAD will be the initial NAD.
 *              The data contains the NAD and the PRIDs and occupies one byte each. The structure of the data is: NAD and then all PRIDs for the frames. The order of the PRIDs are the same as the frame list in the LDF and the frame definition in the NCF, both in ISO 17987-2.
 * @Reference See the definition of the save configuration service in ISO 17987-3.
 *            Function l_ifc_read_status see 7.3.7.9 and example in 7.6.
 * 
 * @param iii 
 * @param data 
 * @param length 
 * @return LD_READ_OK: If the service was successful.
 *         LD_LENGTH_TOO_SHORT: If the configuration size is greater than the length. It means that the data area does not contain a valid configuration.
 */
l_u8 ld_read_configuration (l_ifc_handle iii,
                            l_u8* const data,
                            l_u8* const length)
{

}

/**
 * @brief 
 * @Applicability Slave node only.
 * @Description This call will not transport anything on the bus.
 *              The function will configure the NAD and the PRIDs according to the configuration given by data. The intended usage is to restore a saved configuration or set an initial configuration (e.g. coded by I/O pins). The function shall be called after calling l_ifc_init.
 *              The caller shall set the size of the data area before calling the function.
 *              The data contains the NAD and the PRIDs and occupies one byte each. The structure of the data is: NAD and then all PRIDs for the frames. The order of the PRIDs are the same as the frame list in the LDF and the frame definition in the NCF, both in ISO 17987-2.
 * @Reference See the definition of the save configuration service in ISO 17987-3.
 *            Function l_ifc_read_status see 7.3.7.9 and example in 7.6.
 * @param iii 
 * @param data 
 * @param length 
 * @return LD_SET_OK: If the service was successful.
 *         LD_LENGTH_NOT_CORRECT: If the required size of the configuration is not equal to the given length.
 *         LD_DATA_ERROR: The set of configuration could not be made.
 */
l_u8 ld_set_configuration (l_ifc_handle iii, const l_u8* const data, l_u16 length)
{
    
}

/**
 * @brief 
 * @Applicability Master node only.
 * @Description This call changes the NAD if the node properties fulfil the test specified by id, byte, mask and invert.
 *              Id shall be in the range 0 to 31, see ISO 17987-3 Table "Possible ReadByIdentifier positive response message", and byte in the range 1 to 5 (specifying the byte to use in the id). Mask and Invert shall have values between 0 and 255.
 * @Reference See the definition of the conditional change NAD service in ISO 17987-3.
 * @param iii 
 * @param NAD 
 * @param id 
 * @param byte 
 * @param mask 
 * @param invert 
 * @param new_NAD 
 */
void ld_conditional_change_NAD (l_ifc_handle iii,
                                l_u8 NAD,
                                l_u8 id,
                                l_u8 byte,
                                l_u8 mask,
                                l_u8 invert,
                                l_u8 new_NAD)
{

}

/**
 * @brief 
 * @Applicability Master node only.
 * @Description The call requests the slave node selected with the NAD to return the property associated with the id parameter, see ISO 17987-3 Table "ReadByIdentifier negative response message" for interpretation of the id. When the next call to ld_is_ready returns LD_SERVICE_IDLE (after the ld_read_by_id is called), the RAM area specified by data contains between one and five bytes data according to the request.
 *              The result is returned in a big endian style. It is up to little endian CPUs to swap the bytes, not the LIN diagnostic driver. The reason for using big endian data is to simplify message routing to a (e.g. CAN) backbone network.
 * @Reference See definition of the ReadByIdentifier service in ISO 17987-3.
 * 
 * @param iii 
 * @param NAD 
 * @param supplier_id 
 * @param function_id 
 * @param id 
 * @param data 
 */
void ld_read_by_id (l_ifc_handle iii,
                    l_u8 NAD,
                    l_u16 supplier_id,
                    l_u16 function_id,
                    l_u8 id,
                    l_u8* const data)
{
    
}

/**
 * @brief 
 * @Applicability This callout is optional and is available in slave node only. In case the user defined read by identifier request is used, the slave node application shall implement this call-out.
 * @Description This callout is used when the master node transmits a ReadByIdentifier request with an identifier in the user defined area. The slave node application will be called from the driver when such request is received.
 *              The id parameter is the identifier in the user defined area (32 to 63), see ISO 17987-3 Table "Supported identifiers using the ReadByIdentifier request message" from the ReadByIdentifier configuration request.
 *              The data pointer points to a data area with 5 bytes. This area will be used by the application to set up the positive response, see the user defined area in ISO 17987-3 Table "Possible ReadByIdentifier positive response message".
 * @Reference Node configuration and Identification, see ISO 17987-3.
 * @param iii 
 * @param id 
 * @param data 
 * @return LD_NEGATIVE_RESPONSE: The slave node will respond with a negative response as defined in ISO 17987-3 Table "ReadByIdentifier negative response message". In this case the data area is not considered.
           LD_POSTIVE_RESPONSE: The slave node will setup a positive response using the data provided by the application.
           LD_NO_RESPONSE: The slave node will not answer. 
 */
l_u8 ld_read_by_id_callout(l_ifc_handle iii,
                           l_u8 id,
                           l_u8* data)
{
    
}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description This call will (re)initialize the raw and the messaged-based layers on the interface iii. All transport layer buffers will be initialized. If there is an ongoing diagnostic frame transporting a messaged-based or raw message on the bus, it will not be aborted.
 * @Reference No reference, the behaviour is API specific and not described anywhere else.
 * 
 */
void ld_init (l_ifc_handle iii)
{

}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The call queues the transmission of 8 bytes of data in one frame. The data is sent in the next suitable frame (master request frame for master nodes and slave response frame for slave nodes).
 *              The data area will be copied in the call, the pointer will not be memorized.
 *              If no more queue resources are available, the data may be jettisoned and the appropriate error status will be set.
 * @Reference The raw and messaged-based is not differentiated outside the API. A general description of the transport layer can be found in ISO 17987-2.
 * 
 * @param iii 
 * @param data 
 */
void ld_put_raw (l_ifc_handle iii, const l_u8* const data)
{

}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The call copies the oldest received diagnostic frame data to the memory specified by data.
 *              The data returned is received from master request frame for slave nodes and slave response frame for master nodes.
 *              If the receive queue is empty no data will be copied.
 * @Reference The raw and messaged-based is not differentiated outside the API. A general description of the transport layer can be found in ISO 17987-2.
 * 
 * @param iii 
 * @param data 
 */
void ld_get_raw (l_ifc_handle iii, l_u8* const data)
{

}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The call returns the status of the raw frame transmission function
 * @Reference The raw and messaged-based is not differentiated outside the API. A general description of the transport layer can be found in ISO 17987-2.
 * 
 * @param iii 
 * @return LD_QUEUE_EMPTY: The transmit queue is empty. In case previous calls to ld_put_raw, all frames in the queue have been transmitted.
           LD_QUEUE_AVAILABLE: The transmit queue contains entries, but is not full.
           LD_QUEUE_FULL: The transmit queue is full and cannot accept further frames.
           LD_TRANSMIT_ERROR: LIN protocol errors occurred during the transfer; initialize and redo the transfer. 
 */
l_u8 ld_raw_tx_status (l_ifc_handle iii)
{

}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The call returns the status of the raw frame receive function
 * @Reference The raw and messaged-based is not differentiated outside the API. A general description of the transport layer can be found in ISO 17987-2.
 * 
 * @param iii 
 * @return LD_NO_DATA: The receive queue is empty.
           LD_DATA_AVAILABLE: The receive queue contains data that can be read.
           LD_RECEIVE_ERROR: LIN protocol errors occurred during the transfer; initialize and redo the transfer. 
 */
l_u8 ld_raw_rx_status (l_ifc_handle iii)
{

}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The call packs the information specified by data and DataLength into one or multiple diagnostic frames. If the call is made in a master node application the frames are transmitted to the slave node with the address NAD. If the call is made in a slave node application the frames are transmitted to the master node with the address NAD. The parameter NAD is not used in slave nodes.
 *              The value of the SID (or RSID) shall be the first byte in the data area.
 *              DataLength shall be in the range of 1 to 4 095 bytes. The DataLength shall also include the SID (or RSID) value, i.e. message length plus one.
 *              The call is asynchronous, i.e. not suspended until the message has been sent, and the buffer may not be changed by the application as long as calls to ld_tx_status returns LD_IN_PROGRESS.
 *              The data is transmitted in suitable frames (master request frame for master nodes and slave response frame for slave nodes).
 *              If there is a message in progress, the call will return with no action.
 * @Reference The raw and messaged-based is not differentiated outside the API. A general description of the transport layer can be found in ISO 17987-2.
 * 
 */
void ld_send_message (l_ifc_handle iii,
                      l_u16 length,
                      l_u8 NAD,
                      const l_u8* const data)
{
    
}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The call prepares the LIN diagnostic module to receive one message and store it in the buffer pointed to by data. At the call, DataLength shall specify the maximum length allowed. When the reception has completed, DataLength is changed to the actual length and NAD to the NAD in the message.
 *              SID (or RSID) will be the first byte in the data area.
 *              DataLength will be in the range of 1 to 4 095 bytes, but never more than the value originally set in the call. SID (or RSID) is included in the DataLength.
 *              The parameter NAD is not used in slave nodes.
 *              The call is asynchronous, i.e. not suspended until the message has been received, and the buffer may not be changed by the application as long as calls to ld_rx_status returns LD_IN_PROGRESS. If the call is made after the message transmission has commenced on the bus (i.e. the SF or FF is already transmitted), this message will not be received. Instead the function will wait until next message commence.
 *              The data is received from the succeeding suitable frames (master request frame for slave nodes and slave response frame for master nodes).
 *              The application shall monitor the ld_rx_status and shall not call this function until the status is LD_COMPLETED. Otherwise this function may return inconsistent data in the parameters.
 * @Reference The raw and messaged-based is not differentiated outside the API. A general description of the transport layer can be found in ISO 17987-2.
 * 
 * @param iii 
 * @param DataLength 
 * @param NAD 
 * @param data 
 */
void ld_receive_message (l_ifc_handle iii,
                         l_u16* const DataLength,
                         l_u8* const NAD,
                         l_u8* const data)
{
    
}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The call returns the status of the last made call to ld_send_message. The following values can be returned.
 * @Reference The raw and messaged-based is not differentiated outside the API. A general description of the transport layer can be found in ISO 17987-2.
 * 
 * @param iii 
 * @return LD_IN_PROGRESS: The transmission is not yet completed.
 *         LD_COMPLETED: The transmission has completed successfully (and you can issue a new ld_send_message call). This value is also returned after initialization of the transport layer.
 *         LD_FAILED: The transmission ended in an error. The data was only partially sent. The transport layer shall be reinitialized before processing further messages. To find out why a transmission has failed, check the status management function l_ifc_read_status, see 7.3.7.9.
 *         LD_N_AS_TIMEOUT: The transmission failed because of a N_As timeout, see ISO 17987-2. 
 */
l_u8 ld_tx_status (l_ifc_handle iii)
{

}

/**
 * @brief 
 * @Applicability Master and slave nodes.
 * @Description The call returns the status of the last made call to ld_receive_message. The following values can be returned.
 * @Reference The raw and messaged-based is not differentiated outside the API. A general description of the transport layer can be found in ISO 17987-2.
 * 
 * @param iii 
 * @return LD_IN_PROGRESS: The reception is not yet completed.
 *         LD_COMPLETED: The reception has completed successfully and all information (DataLength, NAD, data) is available. (You can also issue a new ld_receive_message call). This value is also returned after initialization of the transport layer.
 *         LD_FAILED: The reception ended in an error. The data was only partially received and should not be trusted. Initialize before processing further transport layer messages. To find out why a reception has failed, check the status management function l_ifc_read_status, see 7.3.7.9.
 *         LD_N_CR_: The reception failed because of a N_Cr timeout, see ISO 17987-2.
 *         LD_WRONG_SN: The reception failed because of an unexpected sequence number. 
 */
l_u8 ld_rx_status (l_ifc_handle iii)
{

}

/******************************************************************************
 * private functions definition
 *****************************************************************************/
