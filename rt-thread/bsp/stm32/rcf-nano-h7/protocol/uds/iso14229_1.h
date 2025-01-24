/******************************************************************************
 * NanoH7 - UAV firmware base on RT-Thread
 * Copyright (C) 2023 - 2025 Althrone <mail>
 * 
 * @file    rt-thread\bsp\stm32\rcf-nano-h7\protocol\uds\iso14229_1.h
 * 
 * ref: Specification of <ISO 14229-1:2020>
 *****************************************************************************/

//不知道正则表达式怎么支持嵌套，暂时没法按google style风格命名define保护,请手动大写
#ifndef NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso14229_1_h_
#define NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso14229_1_h_

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * includes
 *****************************************************************************/

#include "rtthread.h"

/******************************************************************************
 * macros
 *****************************************************************************/

/******************************************************************************
 * pubilc types
 *****************************************************************************/

typedef enum
{
    //ISO14229-1
    kUdsAppDscSi    =0x10, //DiagnosticSessionControl
    kUdsAppErSi     =0x11, //ECUReset
    kUdsAppCdtciSi  =0x14, //ClearDiagnosticInformation
    kUdsAppRdtciSi  =0x19, //ReadDTCInformation
    kUdsAppRdbiSi   =0x22, //ReadDataByIdentifier
    kUdsAppRmbaSi   =0x23, //ReadMemoryByAddress
    kUdsAppRsdbiSi  =0x24, //ReadScalingDataByIdentifier
    kUdsAppSaSi     =0x27, //SecurityAccess
    kUdsAppCcSi     =0x28, //CommunicationControl
    kUdsAppArsSi    =0x29, //Authentication
    kUdsAppRdbpiSi  =0x2A, //ReadDataByPeriodicIdentifier
    kUdsAppDddiSi   =0x2C, //DynamicallyDefineDataIdentifier
    kUdsAppWdbiSi   =0x2E, //WriteDataByIdentifier
    kUdsAppIocbiSi  =0x2F, //InputOutputControlByIdentifier
    kUdsAppRcSi     =0x31, //RoutineControl
    kUdsAppRdSi     =0x34, //RequestDownload
    kUdsAppRuSi     =0x35, //RequestUpload
    kUdsAppTdSi     =0x36, //TransferData
    kUdsAppRteSi    =0x37, //RequestTransferExit
    kUdsAppRftSi    =0x38, //RequestFileTransfer
    kUdsAppWmbaSi   =0x3D, //WriteMemoryByAddress
    kUdsAppTpSi     =0x3E, //TesterPresent
    //ISO14229-1
    kUdsAppSdtSi    =0x84, //SecuredDataTransmission
    kUdsAppCdtcsSi  =0x85, //ControlDTCSetting
    kUdsAppRoeSi    =0x86, //ResponseOnEvent
    kUdsAppLcSi     =0x87, //LinkControl
}UdsAppSiEnum;//"SI" is the parameter Service identifier.

typedef enum
{
    kUdsAppNrcPR        =0x00, //positiveResponse
    //0x01 to 0x0F ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcGR        =0x10, //generalReject
    kUdsAppNrcSNS       =0x11, //serviceNotSupported
    kUdsAppNrcSFNS      =0x12, //SubFunctionNotSupported
    kUdsAppNrcIMLOIF    =0x13, //incorrectMessageLengthOrInvalidFormat
    kUdsAppNrcRTL       =0x14, //responseTooLong
    //0x15 to 0x20 ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcBRR       =0x21, //busyRepeatRequest
    kUdsAppNrcCNC       =0x22, //conditionsNotCorrect
    //0x23 ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcRSE       =0x24, //requestSequenceError
    kUdsAppNrcNRFSC     =0x25, //noResponseFromSubnetComponent
    kUdsAppNrcFPEORA    =0x26, //FailurePreventsExecutionOfRequestedAction
    //0x27 to 0x30 ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcROOR      =0x31, //requestOutOfRange
    //0x32 ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcSAD       =0x33, //securityAccessDenied
    kUdsAppNrcAR        =0x34, //authenticationRequired
    kUdsAppNrcIK        =0x35, //invalidKey
    kUdsAppNrcENOA      =0x36, //exceedNumberOfAttempts
    kUdsAppNrcRTDNE     =0x37, //requiredTimeDelayNotExpired
    kUdsAppNrcSDTR      =0x38, //secureDataTransmissionRequired
    kUdsAppNrcSDTNA     =0x39, //secureDataTransmissionNotAllowed
    kUdsAppNrcSDVF      =0x3A, //secureDataVerificationFailed
    //0x3B to 0x4F ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcCVFITP    =0x50, //Certificate verification failed - Invalid Time Period
    kUdsAppNrcCVFIS     =0x51, //Certificate verification failed - Invalid Signature
    kUdsAppNrcCVFICOT   =0x52, //Certificate verification failed - Invalid Chain of Trust
    kUdsAppNrcCVFIT     =0x53, //Certificate verification failed - Invalid Type
    kUdsAppNrcCVFIF     =0x54, //Certificate verification failed - Invalid Format
    kUdsAppNrcCVFIC     =0x55, //Certificate verification failed - Invalid Content
    kUdsAppNrcCVFISC     =0x56, //Certificate verification failed - Invalid Scope
    kUdsAppNrcCVFICE     =0x57, //Certificate verification failed – Invalid Certificate (revoked)
    kUdsAppNrcOVF       =0x58, //Ownership verification failed
    kUdsAppNrcCCF       =0x59, //Challenge calculation failed
    kUdsAppNrcSARF      =0x5A, //Setting Access Rights failed
    kUdsAppNrcSKCDF     =0x5B, //Session key creation/derivation failed
    kUdsAppNrcCDUF      =0x5C, //Configuration data usage failed
    kUdsAppNrcDAF       =0x5D, //DeAuthentication failed
    //0x5E to 0x6F ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcUDNA      =0x70, //uploadDownloadNotAccepted
    kUdsAppNrcTDS       =0x71, //transferDataSuspended
    kUdsAppNrcGPF       =0x72, //generalProgrammingFailure
    kUdsAppNrcWBSC      =0x73, //wrongBlockSequenceCounter
    //0x74 to 0x77 ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcRCRRP     =0x78, //requestCorrectlyReceived-ResponsePending
    //0x79 to 0x7D ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcSFNSIAS   =0x7E, //SubFunctionNotSupportedInActiveSession
    kUdsAppNrcSNSIAS    =0x7F, //serviceNotSupportedInActiveSession
    //0x80 ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcRPMTH     =0x81, //rpmTooHigh
    kUdsAppNrcRPMTL     =0x82, //rpmTooLow
    kUdsAppNrcEIR       =0x83, //engineIsRunning
    kUdsAppNrcEINR      =0x84, //engineIsNotRunning
    kUdsAppNrcERTTL     =0x85, //engineRunTimeTooLow
    kUdsAppNrcTEMPTH    =0x86, //temperatureTooHigh
    kUdsAppNrcTEMPTL    =0x87, //temperatureTooLow
    kUdsAppNrcVSTH      =0x88, //vehicleSpeedTooHigh
    kUdsAppNrcVSTL      =0x89, //vehicleSpeedTooLow
    kUdsAppNrcTPTH      =0x8A, //throttle/PedalTooHigh
    kUdsAppNrcTPTL      =0x8B, //throttle/PedalTooLow
    kUdsAppNrcTRNIN     =0x8C, //transmissionRangeNotInNeutral
    kUdsAppNrcTRNIG     =0x8D, //transmissionRangeNotInGear
    //0x8E ISOSAEReserved ISOSAERESRVD
    kUdsAppNrcBSNC      =0x8F, //brakeSwitch(es)NotClosed (Brake Pedal not pressed or not applied)
    kUdsAppNrcSLNIP     =0x90, //shifterLeverNotInPark
    kUdsAppNrcTCCL      =0x91, //torqueConverterClutchLocked
    kUdsAppNrcVTH       =0x92, //voltageTooHigh
    kUdsAppNrcVTL       =0x93, //voltageTooLow
    kUdsAppNrcRTNA      =0x94, //ResourceTemporarilyNotAvailable
    //0x95 to 0xEF reservedForSpecificConditionsNotCorrect RFSCNC
    //0xF0 to 0xFE vehicleManufacturerSpecificConditionsNotCorrect VMSCNC
    //0xFF ISOSAEReserved ISOSAERESRVD
}UdsAppNrcEnum;//Table A.1 — Negative Response Code (NRC) definition and values

typedef enum
{
    //0x00 ISOSAEReserved                                   M ISOSAERESRVD
    kUdsAppDS       =0x01, //defaultSession                 M
    kUdsAppPRGS     =0x02, //ProgrammingSession             U
    kUdsAppEXTDS    =0x03, //extendedDiagnosticSession      U
    kUdsAppSSDS     =0x04, //safetySystemDiagnosticSession  U
    //0x05 to 0x3F ISOSAEReserved                           M ISOSAERESRVD
    //0x40 to 0x5F vehicleManufacturerSpecific              U VMS
    //0x60 to 0x7E systemSupplierSpecific                   U SSS
    //0x7F ISOSAEReserved                                   M ISOSAERESRVD
}UdsAppDiagnosticSessionType;//Table 25 — Request message SubFunction parameter definition

typedef enum
{
    //0x0000 to 0x00FF ISOSAEReserved                                                                               M ISOSAERESRVD
    //0x0100 to 0xA5FF VehicleManufacturerSpecific                                                                  U VMS
    //0xA600 to 0xA7FF ReservedForLegislativeUse                                                                    M RFLU
    //0xA800 to 0xACFF VehicleManufacturerSpecific                                                                  U VMS
    //0xAD00 to 0xAFFF ReservedForLegislativeUse                                                                    M RFLU
    //0xB000 to 0xB1FF VehicleManufacturerSpecific                                                                  U VMS
    //0xB200 to 0xBFFF ReservedForLegislativeUse                                                                    M RFLU
    //0xC000 to 0xC2FF VehicleManufacturerSpecific                                                                  U VMS
    //0xC300 to 0xCEFF ReservedForLegislativeUse                                                                    M RFLU
    //0xCF00 to 0xEFFF VehicleManufacturerSpecific                                                                  U VMS
    //0xF000 to 0xF00F networkConfigurationDataForTractorTrailerApplicationDataIdentifier                           U NCDFTTADID
    //0xF010 to 0xF0FF vehicleManufacturerSpecific                                                                  U VMS
    //0xF100 to 0xF17F identificationOptionVehicleManufacturerSpecificDataIdentifier                                U IDOPTVMSDID
    kUdsAppDidBSIDID        =0xF180, //BootSoftwareIdentificationDataIdentifier                                     U
    kUdsAppDidASIDID        =0xF181, //applicationSoftwareIdentificationDataIdentifier                              U
    kUdsAppDidADIDID        =0xF182, //applicationDataIdentificationDataIdentifier                                  U
    kUdsAppDidBSFPDID       =0xF183, //bootSoftwareFingerprintDataIdentifier                                        U
    kUdsAppDidASFPDID       =0xF184, //applicationSoftwareFingerprintDataIdentifier                                 U
    kUdsAppDidADFPDID       =0xF185, //applicationDataFingerprintDataIdentifier                                     U
    kUdsAppDidADSDID        =0xF186, //ActiveDiagnosticSessionDataIdentifier                                        U
    kUdsAppDidVMSPNDID      =0xF187, //vehicleManufacturerSparePartNumberDataIdentifier                             U
    kUdsAppDidVMECUSNDID    =0xF188, //vehicleManufacturerECUSoftwareNumberDataIdentifier                           U
    kUdsAppDidVMECUSVNDID   =0xF189, //vehicleManufacturerECUSoftwareVersionNumberDataIdentifier                    U
    kUdsAppDidSSIDDID       =0xF18A, //systemSupplierIdentifierDataIdentifier                                       U
    kUdsAppDidECUMDDID      =0xF18B, //ECUManufacturingDateDataIdentifier                                           U
    kUdsAppDidECUSNDID      =0xF18C, //ECUSerialNumberDataIdentifier                                                U
    kUdsAppDidSFUDID        =0xF18D, //supportedFunctionalUnitsDataIdentifier                                       U
    kUdsAppDidVMKAPNDID     =0xF18E, //VehicleManufacturerKitAssemblyPartNumberDataIdentifier                       U
    kUdsAppDidRXSWIN        =0xF18F, //RegulationXSoftwareIdentificationNumbers (RxSWIN)                            U
    kUdsAppDidVINDID        =0xF190, //VINDataIdentifier                                                            U
    kUdsAppDidVMECUHNDID    =0xF191, //vehicleManufacturerECUHardwareNumberDataIdentifier                           U
    kUdsAppDidSSECUHWNDID   =0xF192, //systemSupplierECUHardwareNumberDataIdentifier                                U
    kUdsAppDidSSECUHWVNDID  =0xF193, //systemSupplierECUHardwareVersionNumberDataIdentifier                         U
    kUdsAppDidSSECUSWNDID   =0xF194, //systemSupplierECUSoftwareNumberDataIdentifier                                U
    kUdsAppDidSSECUSWVNDID  =0xF195, //systemSupplierECUSoftwareVersionNumberDataIdentifier                         U
    kUdsAppDidEROTANDID     =0xF196, //exhaustRegulationOrTypeApprovalNumberDataIdentifier                          U
    kUdsAppDidSNOETDID      =0xF197, //systemNameOrEngineTypeDataIdentifier                                         U
    kUdsAppDidRSCOTSNDID    =0xF198, //repairShopCodeOrTesterSerialNumberDataIdentifier                             U
    kUdsAppDidPDDID         =0xF199, //programmingDateDataIdentifier                                                U
    kUdsAppDidCRSCOCESNDID  =0xF19A, //calibrationRepairShopCodeOrCalibrationEquipmentSerialNumberDataIdentifier    U
    kUdsAppDidCDDID         =0xF19B, //calibrationDateDataIdentifier                                                U
    kUdsAppDidCESWNDID      =0xF19C, //calibrationEquipmentSoftwareNumberDataIdentifier                             U
    kUdsAppDidEIDDID        =0xF19D, //ECUInstallationDateDataIdentifier                                            U
    kUdsAppDidODXFDID       =0xF19E, //ODXFileDataIdentifier                                                        U
    kUdsAppDidEDID          =0xF19F, //EntityDataIdentifier                                                         U
    //0xF1A0 to 0xF1EF identificationOptionVehicleManufacturerSpecific                                              U IDOPTVMS
    //0xF1F0 to 0xF1FF identificationOptionSystemSupplierSpecific                                                   U IDOPTSSS
    //0xF200 to 0xF2FF periodicDataIdentifier                                                                       U PDID
    //0xF300 to 0xF3FF DynamicallyDefinedDataIdentifier                                                             U DDDDI
    //0xF400 to 0xF5FF OBDDataIdentifier                                                                            M OBDDID
    //0xF600 to 0xF6FF OBDMonitorDataIdentifier                                                                     M OBDMDID
    //0xF700 to 0xF7FF OBDDataIdentifier                                                                            M OBDDID
    //0xF800 to 0xF8FF OBDInfoTypeDataIdentifier                                                                    M OBDINFTYPDID
    //0xF900 to 0xF9FF TachographDataIdentifier                                                                     M TACHODID
    //0xFA00 to 0xFA0F AirbagDeploymentDataIdentifier                                                               M ADDID
    kUdsAppDidNOEDRD        =0xFA10, //NumberOfEDRDevices                                                           U
    kUdsAppDidEDRI          =0xFA11, //EDRIdentification                                                            U
    kUdsAppDidEDRDAI        =0xFA12, //EDRDeviceAddressInformation                                                  U
    //0xFA13 to 0xFA18 EDREntries                                                                                   U EDRES
    //0xFA19 to 0xFAFF SafetySystemDataIdentifier                                                                   M SSDID
    //0xFB00 to 0xFCFF ReservedForLegislativeUse                                                                    M RFLU
    //0xFD00 to 0xFEFF SystemSupplierSpecific                                                                       U SSS
    kUdsAppDidUDSVDID       =0xFF00, //UDSVersionDataIdentifier                                                     U
    kUdsAppDidRESRVDCPADLC  =0xFF01, //ReservedForISO15765-5                                                        U
    //0xFF02 to 0xFFFF ISOSAEReserved                                                                               M ISOSAERESRVD
}UdsAppDidEnum;//Table C.1 — DID data-parameter definitions

/******************************************************************************
 * pubilc variables declaration
 *****************************************************************************/

/******************************************************************************
 * pubilc functions declaration
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NANOH7_rt_thread_bsp_stm32_rcf_nano_h7_protocol_uds_iso14229_1_h_ */