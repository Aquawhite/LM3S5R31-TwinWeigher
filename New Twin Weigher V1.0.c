//V1.4		Point no 1 only been implemented for TR Module only, will continue for other modules
//			Optimizing all modules so the machine can be faster. Based on GT Timings (GTModuleStepTimer)
//			Have an idea to have GTRequestFilling so the timing can be adjusted, also OFRequesttoo
//			Need to add timing settings for SPVibEngage
//V1.3		1.Add additional check for Init Mode, 1 sensor should be checked on the OFF and ON STATE
//			not just the ON State
//			2.SPPusher 0 State is changed to FALSE
//			3. VFVibEngage now active when already start filling (start later now)
//			   VFVibEngageDelay can now be adjusted
//V1.2		Add TRModuleWaitingPick to make the TR Flow more naturally
//V1.1      17 March 2016 , trying to build initialization program
//V1.0		FEB 2016
// 			Every mm is 0.01mm per unit (eg. RTServoPosmm = 100 means 1mm)

#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"

#define TRUE	1
#define FALSE	0

#define FORWARD 1
#define BACKWARD 0

#define UNUSED 0
#define FASTBUFFER 7

//*****************************************************************************
// Defines for the pins that are used in the device
//*****************************************************************************

// Button PIN Mapping

#define PIN_RX_BASE					GPIO_PORTA_BASE
#define PIN_RX_PIN					GPIO_PIN_0
#define PORT_TX_BASE				GPIO_PORTA_BASE
#define PORT_TX_PIN					GPIO_PIN_1
#define PORT_SSI_CLK_BASE			GPIO_PORTA_BASE
#define PORT_SSI_CLK_PIN			GPIO_PIN_2
#define PORT_SSI_FSS_BASE			GPIO_PORTA_BASE
#define PORT_SSI_FSS_PIN			GPIO_PIN_3
#define PIN_SSI_RX_BASE				GPIO_PORTA_BASE
#define PIN_SSI_RX_PIN				GPIO_PIN_4
#define PORT_SSI_TX_BASE			GPIO_PORTA_BASE
#define PORT_SSI_TX_PIN				GPIO_PIN_5
#define PORT_W_LSMGATEPNEU_VALVE_BASE	GPIO_PORTA_BASE
#define PORT_W_LSMGATEPNEU_VALVE_PIN	GPIO_PIN_6
#define PORT_W_LLRGATEPNEU_VALVE_BASE	GPIO_PORTA_BASE
#define PORT_W_LLRGATEPNEU_VALVE_PIN	GPIO_PIN_7

#define PIN_W_RHOPPER_SENS_BASE			GPIO_PORTB_BASE			// Not 5V Tolerant , Max 3.6 V
#define PIN_W_RHOPPER_SENS_PIN			GPIO_PIN_0
#define PIN_W_LHOPPER_SENS_BASE			GPIO_PORTB_BASE			// Not 5V Tolerant , Max 3.6 V
#define PIN_W_LHOPPER_SENS_PIN			GPIO_PIN_1
#define PORT_EX1_BASE					GPIO_PORTB_BASE
#define PORT_EX1_PIN					GPIO_PIN_2
#define PIN_EXSENS3_BASE				GPIO_PORTB_BASE
#define PIN_EXSENS3_PIN					GPIO_PIN_3
#define PIN_HEATER_ERROR_BASE			GPIO_PORTB_BASE
#define PIN_HEATER_ERROR_PIN			GPIO_PIN_4
#define PIN_HOPPER_SENSOR_BASE			GPIO_PORTB_BASE
#define PIN_HOPPER_SENSOR_PIN			GPIO_PIN_5
#define PIN_AIR_ERROR_BASE				GPIO_PORTB_BASE
#define PIN_AIR_ERROR_PIN				GPIO_PIN_6
#define PIN_BTN_RUN_BASE				GPIO_PORTB_BASE
#define PIN_BTN_RUN_PIN					GPIO_PIN_7

#define PORT_TCK_BASE					GPIO_PORTC_BASE
#define PORT_TCK_PIN					GPIO_PIN_0
#define PORT_TMS_BASE					GPIO_PORTC_BASE
#define PORT_TMS_PIN					GPIO_PIN_1
#define PIN_TDI_BASE					GPIO_PORTC_BASE
#define PIN_TDI_PIN						GPIO_PIN_2
#define PORT_TDO_BASE					GPIO_PORTC_BASE
#define PORT_TDO_PIN					GPIO_PIN_3
#define PORT_OF_VACUUM_BASE				GPIO_PORTC_BASE
#define PORT_OF_VACUUM_PIN				GPIO_PIN_4
#define PORT_OF_OPENPNEU_VALVE_BASE				GPIO_PORTC_BASE
#define PORT_OF_OPENPNEU_VALVE_PIN				GPIO_PIN_5
#define PORT_OF_ENGAGEPNEU_VALVE_BASE			GPIO_PORTC_BASE
#define PORT_OF_ENGAGEPNEU_VALVE_PIN			GPIO_PIN_6
#define PORT_OF_GATEPNEU_VALVE_BASE			GPIO_PORTC_BASE
#define PORT_OF_GATEPNEU_VALVE_PIN			GPIO_PIN_7

#define PORT_W_LCR_SCL_BASE			GPIO_PORTD_BASE
#define PORT_W_LCR_SCL_PIN			GPIO_PIN_0
#define PIN_W_LCR_DATA_BASE			GPIO_PORTD_BASE
#define PIN_W_LCR_DATA_PIN			GPIO_PIN_1
#define PIN_EXSENS4_BASE			GPIO_PORTD_BASE
#define PIN_EXSENS4_PIN				GPIO_PIN_2
#define PIN_TR_VACUUM_SENS_BASE			GPIO_PORTD_BASE
#define PIN_TR_VACUUM_SENS_PIN			GPIO_PIN_3
#define PIN_OF_ROPENPNEU_EXTSENS_BASE		GPIO_PORTD_BASE
#define PIN_OF_ROPENPNEU_EXTSENS_PIN		GPIO_PIN_4
#define PIN_OF_ROPENPNEU_RETSENS_BASE			GPIO_PORTD_BASE
#define PIN_OF_ROPENPNEU_RETSENS_PIN			GPIO_PIN_5
#define PIN_OF_ENGAGEPNEU_RETSENS_BASE			GPIO_PORTD_BASE					// WARNING MOVED TEMPORARY
#define PIN_OF_ENGAGEPNEU_RETSENS_PIN			GPIO_PIN_6
#define PIN_OF_ENGAGEPNEU_EXTSENS_BASE			GPIO_PORTD_BASE
#define PIN_OF_ENGAGEPNEU_EXTSENS_PIN			GPIO_PIN_7

#define PORT_EX2_BASE						GPIO_PORTE_BASE
#define PORT_EX2_PIN						GPIO_PIN_0
#define PORT_DATECODE_BASE					GPIO_PORTE_BASE
#define PORT_DATECODE_PIN					GPIO_PIN_1
#define PIN_OF_LOPENPNEU_EXTSENS_BASE		GPIO_PORTE_BASE
#define PIN_OF_LOPENPNEU_EXTSENS_PIN			GPIO_PIN_2
#define PIN_OF_LOPENPNEU_RETSENS_BASE		GPIO_PORTE_BASE
#define	PIN_OF_LOPENPNEU_RETSENS_PIN			GPIO_PIN_3
#define PIN_W_LCL_DATA_BASE					GPIO_PORTE_BASE
#define PIN_W_LCL_DATA_PIN					GPIO_PIN_4
#define PORT_W_LCL_SCL_BASE					GPIO_PORTE_BASE
#define	PORT_W_LCL_SCL_PIN					GPIO_PIN_5
#define PIN_OF_GATEPNEU_EXTSENS_BASE			GPIO_PORTE_BASE
#define PIN_OF_GATEPNEU_EXTSENS_PIN			GPIO_PIN_6
#define PIN_OF_GATEPNEU_RETSENS_BASE			GPIO_PORTE_BASE
#define	PIN_OF_GATEPNEU_RETSENS_PIN			GPIO_PIN_7

#define PIN_OF_VACUUM_SENSOR_BASE			GPIO_PORTF_BASE
#define PIN_OF_VACUUM_SENSOR_PIN				GPIO_PIN_0
#define	PIN_SP_SEALERPNEU_EXTSENS_BASE		GPIO_PORTF_BASE				//VLVCUTTER
#define PIN_SP_SEALERPNEU_EXTSENS_PIN		GPIO_PIN_1
#define PIN_SP_SEALERPNEU_RETSENS_BASE		GPIO_PORTF_BASE				// Gusset Module
#define PIN_SP_SEALERPNEU_RETSENS_PIN		GPIO_PIN_2
#define PORT_SP_SEALERPNEU_VALVE_BASE		GPIO_PORTF_BASE
#define PORT_SP_SEALERPNEU_VALVE_PIN			GPIO_PIN_3
#define PORT_W_RLRGATEPNEU_VALVE_BASE		GPIO_PORTF_BASE				// WARNING, PROBLEM IN 5R31
#define PORT_W_RLRGATEPNEU_VALVE_PIN		GPIO_PIN_4					// Solved! Changed to PF7 for 5R31
#define PORT_W_LOPGATEPNEU_VALVE_BASE		GPIO_PORTF_BASE				// WARNING, PROBLEM IN 5R31
#define PORT_W_LOPGATEPNEU_VALVE_PIN		GPIO_PIN_5					// Solved! Changed to PG4 for 5R31

#define PORT_TR_EXTPNEU_VALVE_BASE			GPIO_PORTG_BASE
#define PORT_TR_EXTPNEU_VALVE_PIN			GPIO_PIN_0
#define PORT_TR_REOPNEU_VALVE_BASE		GPIO_PORTG_BASE
#define PORT_TR_REOPNEU_VALVE_PIN		GPIO_PIN_1
#define PORT_W_RSMGATEPNEU_VALVE_BASE		GPIO_PORTG_BASE
#define PORT_W_RSMGATEPNEU_VALVE_PIN		GPIO_PIN_7

#define PIN_TR_EXTPNEU_EXTSENS_BASE		GPIO_PORTH_BASE
#define PIN_TR_EXTPNEU_EXTSENS_PIN		GPIO_PIN_0
#define PIN_TR_EXTPNEU_RETSENS_BASE		GPIO_PORTH_BASE
#define	PIN_TR_EXTPNEU_RETSENS_PIN  	GPIO_PIN_1
#define PIN_TR_REOPNEU_RETSENS_BASE		GPIO_PORTH_BASE
#define PIN_TR_REOPNEU_RETSENS_PIN		GPIO_PIN_2
#define PIN_TR_REOPNEU_EXTSENS_BASE		GPIO_PORTH_BASE
#define PIN_TR_REOPNEU_EXTSENS_PIN		GPIO_PIN_3
#define PIN_IT_RMATEMPTY_SENS_BASE			GPIO_PORTH_BASE
#define PIN_IT_RMATEMPTY_SENS_PIN			GPIO_PIN_4
#define PIN_EXSENS2_BASE					GPIO_PORTH_BASE
#define PIN_EXSENS2_PIN						GPIO_PIN_5
#define PIN_EXSENS1_BASE					GPIO_PORTH_BASE
#define PIN_EXSENS1_PIN						GPIO_PIN_6
#define PORT_TR_VACUUM_BASE		GPIO_PORTH_BASE
#define PORT_TR_VACUUM_PIN		GPIO_PIN_7

#define PORT_GT_INFPUSHPNEU_VALVE_BASE	GPIO_PORTJ_BASE
#define PORT_GT_INFPUSHPNEU_VALVE_PIN	GPIO_PIN_0
#define PIN_BTN_STOP_BASE				GPIO_PORTJ_BASE
#define	PIN_BTN_STOP_PIN				GPIO_PIN_1
#define PORT_W_ROPGATEPNEU_VALVE_BASE	GPIO_PORTJ_BASE
#define PORT_W_ROPGATEPNEU_VALVE_PIN	GPIO_PIN_2

#define UARTDISABLEDCOUNT			1000

/* Communication : TouchScreen */
// Operation Page
/* System Variable */
unsigned char MachineRunning, ServiceMode, LastServiceMode, MachineStopping, Initting;
unsigned char IgnoreAirError, NoAirErrorMsg, NoAirErrorMsgDuration, NoAirError;
unsigned char FoilJam, FoilJamMsgDuration, FoilJamMsg;
unsigned int FoilJamCounter, NoAirErrorCounter;
unsigned char ProductCountBit, ProductCountBitCounter;
unsigned char EyemarkLampCounter, EyemarkON, EyemarkEnable;
unsigned int CounterEyemarkIgnore;
unsigned char RunBtn, StopBtn, LastRunBtn, LastStopBtn, StopCommand;
unsigned char Error, FatalError;
unsigned char DoorOpen, EyemarkError, FoilEmpty, DatesensError;
unsigned char HMICurrentWindow, HMILastWindow;

unsigned char TRSbsMode, GTSbsMode, OFSbsMode, SPSbsMode;
unsigned char TRModMode, GTModMode, OFModMode, SPModMode;
unsigned char InitMON, LastInitMON;

unsigned char TRInitting, GTInitting, OFInitting, SPInitting;
unsigned char TRInittingDone, GTInittingDone, OFInittingDone, SPInittingDone;
unsigned short TRInitErrorResult, TRInitError, GTInitErrorResult, GTInitError;
unsigned short OFInitErrorResult, OFInitError, SPInitErrorResult, SPInitError;

unsigned short TRInittingTOCounter, GTInittingTOCounter, OFInittingTOCounter, SPInittingTOCounter;
unsigned short InittingTO = 1200;

unsigned long int LifeDuration, longintdelay;
unsigned int BufferSensor;
/* Communication Data Declaration */
unsigned int  ReadACKCounter = 1000;
unsigned char dataIndex = 0, dataRXIndex = 0, dataTXIndex = 0;
unsigned char  ComAddress = 0, BytestoRead = 0;
unsigned char ComOUT[150], ComIN[110];
tBoolean ComDirection = FALSE, ComEnable = FALSE, DoubleSpeed = FALSE, UARTDisabled = FALSE;
unsigned char DataTimerTrigger = FALSE, DataLock = FALSE, DataReady = FALSE, ReadACK = FALSE;
unsigned char TransferIdle = FALSE, ReceiveIdle = FALSE;
unsigned int  IntTXCounter = 0, IntRXCounter = 0;
unsigned long int DataTimerCounter = 0;
unsigned int  UARTTimeOut = UARTDISABLEDCOUNT, UARTDisabledCounter = 0;
unsigned int SPIFirstCycleDurationCounter;

unsigned char SlavePortA, SlavePortB, SlavePortC, SlavePortD;
unsigned char SlavePortE, SlavePortF, SlavePortG, SlavePortH;
unsigned char SlavePortI, SlavePortJ;

unsigned char BlinkFlag = FALSE, ComFlag;
unsigned char BlinkCom;
unsigned int blinkON, blinkOFF;

/* Saved Data */
unsigned long int ProductCounterepr;
unsigned long int Eepromsave = 1000000;
unsigned char autosaveflag;
// Communication SPI
unsigned long SPIOut[26], SPIIn[26];
unsigned char SPICounter, SPIReiterationCounterIN, SPIReiterationCounterOUT, SPICounterSkip;
unsigned char SPIDataLock, SPIDataReady, SPIReiterationMode;                            // bit
unsigned char SPIHeaterON1, SPIHeaterON2, SPIHeaterON3, SPIHeaterON4;
unsigned char SPIInTEST, SPIOutTEST, SPIStartTimer = 250;
unsigned int  SPITimerCounter = 0;
unsigned char SPIComErrorFlag = TRUE, SPIComError;
unsigned int  SPIComErrorCounter;
unsigned char SPITransmitted = FALSE;
/* Timer Variables */
unsigned char ms10counter, ms100counter;

/* Loop Analyze */
unsigned int LoopCycle, LoopTime;

/* Infeed Table Module */
unsigned int ITTrayPosLeftSensor, ITTrayPosRightSensor, ITTrayElevationMaxSensor, ITTrayElevationMinSensor;
unsigned int ITLeftMaterialAvailableSensor, ITRightMaterialAvailableSensor;
unsigned int ITLeftMaterialNotAvailableSensor = 100, ITRightMaterialNotAvailableSensor = 100;

unsigned char ITMotorTrayElevator, ITMotorTrayElevatorMON, ITMotorTrayElevatorDir, ITMotorTrayElevatorDirMON;
unsigned char ITValveTrayPos, ITValveTrayPosMON;

unsigned char ITEnable, ITMotorTrayElevatorDirSet, ITReady, ITSwitchTrayStep;
unsigned char DirUP, DirDOWN;
unsigned char ITOverride;

/* Transfer & Reorient Module */
unsigned int TRExtenderPneuExtSensor, TRExtenderPneuRetSensor, TRReorientPneuExtSensor;
unsigned int TRReorientPneuRetSensor, TRVacuumSensor;

unsigned char TRExtenderPneuValve, TRExtenderPneuValveMON, TRReorientPneuValve,	TRReorientPneuValveMON;
unsigned char TRVacuumValve, TRVacuumValveMON, TRFoilPusherValve, TRFoilPusherValveMON;
unsigned char TRModuleStep, TRModuleWaiting, TRModuleWaitingPick;
unsigned int TRVacuumValveDelay, TRVacuumValveDelayCounter;
unsigned char TRSbsMON, LastTRSbsMON, TRModMON, LastTRModMON;
unsigned char IgnoreTRVacuumSensor, TRVacuumSensorErrorCounter;

/* Grip & Transfer */
unsigned int GTInFillGripPneuLRetSensor, GTInFillGripPneuRRetSensor, GTOutFillOpenPneuLRetSensor;
unsigned int GTOutFillOpenPneuRRetSensor, GTOutFillOpenPneuLExtSensor, GTOutFillOpenPneuRExtSensor;
unsigned int GTOutFillGripPneuLRetSensor, GTOutFillGripPneuRRetSensor, GTTransferPneuRetSensor;
unsigned int GTTransferPneuExtSensor;

unsigned char GTInFillGripPneuValve, GTInFillGripPneuValveMON, GTOutFillOpenPneuValve, GTOutFillOpenPneuValveMON;
unsigned char GTOutFillGripPneuValve, GTOutFillGripPneuValveMON, GTTransferPneuValve, GTTransferPneuValveMON;

unsigned char GTModuleStep, GTSA1ModuleWaiting, GTSA2ModuleWaiting, GTRequestFilling;
unsigned int GTCloseBagDelay, GTCloseBagDelayCounter, GTBothOpenGripDelay, GTBothOpenGripDelayCounter;
unsigned char GTFirstCycle = TRUE;

unsigned char GTFPushPneuValve;
unsigned int GTFPushPneuValveDelay, GTFPushPneuValveDelayCounter;

unsigned char GTSbsMON, LastGTSbsMON, GTModMON, LastGTModMON;
unsigned int GTModuleStepTimer[10], GTModuleStepTimerCounter[10];
unsigned int GTRequestFillingDelayCounter, GTRequestFillingDelay;

/* Filling Module & Vibrator Filling Module */
unsigned int OFGatePneuRetSensor, OFGatePneuExtSensor, OFEngagePneuRetSensor, OFEngagePneuExtSensor;
unsigned int OFOpenPneuRRetSensor, OFOpenPneuLRetSensor, OFOpenPneuRExtSensor, OFOpenPneuLExtSensor;
unsigned int OFVacuumSensor;
unsigned int VFVibEngagePneuRetSensor, VFVibEngagePneuExtSensor;

unsigned char OFGatePneuValve, OFGatePneuValveMON, OFEngagePneuValve, OFEngagePneuValveMON;
unsigned char OFOpenPneuValve, OFOpenPneuValveMON, OFVacuumValve, OFVacuumValveMON;
unsigned char VFVibEngagePneuValve, VFVibEngagePneuValveMON, VFVibrator, VFVibratorMON;

unsigned int OFGatePneuValveDuration, OFVacuumValveDelay, OFGatePneuValveDurationCounter, OFVacuumValveDelayCounter;
unsigned char OFGatePneuFilled, OFGatePneuFillingProcess;

unsigned char OFModuleStep, OFModuleWaiting;
unsigned char OFSbsMON, LastOFSbsMON, OFModMON, LastOFModMON;

unsigned char OFRequestFilling;

unsigned char OFGatePneuValveVibratorMode;
unsigned long int OFGatePneuValveVibratorPeriod, OFGatePneuValveVibratorPeriodCounter;
unsigned long int OFGatePneuValveVibratorFrequency, OFGatePneuValveVibratorDutyCycle;
unsigned int OFGatePneuValveDelay, OFGatePneuValveDelayCounter;
unsigned int OFGatePneuValveVibDelay, OFGatePneuValveVibDelayCounter;

unsigned char VFVibEngagePneuValveVibratorMode;
unsigned long int VFVibEngageVibratorPeriod, VFVibEngageVibratorPeriodCounter;
unsigned long int VFVibEngageVibratorFrequency, VFVibEngageVibratorDutyCycle;
unsigned int VFVibratorEngageDelay, VFVibratorEngageDelayCounter;
unsigned int VFVibratorEngageVibDelay, VFVibratorEngageVibDelayCounter;

unsigned int OFRequestFillingDelayCounter, OFRequestFillingDelay;
unsigned char OFFastRequestFilling;

/* Sealing Module & Vibrator Sealer Module */
unsigned int SPSealerPneuRetSensor, SPSealerPneuExtSensor, SPVibEngagePneuRetSensor, SPVibEngagePneuExtSensor;
unsigned int SPPusherPneuRetSensor, SPPusherPneuExtSensor;

unsigned char SPSealerPneuValve, SPSealerPneuValveMON, SPVibEngagePneuValve, SPVibEngagePneuValveMON;
unsigned char SPPusherValve, SPPusherValveMON, SPVibrator, SPVibratorMON;

unsigned int SPSealerPneuValveDuration,	SPSealerPneuValveDelay, SPSealerPneuValveDurationCounter, SPSealerPneuValveDelayCounter;
unsigned char SPModuleStep;

unsigned char SPSbsMON, LastSPSbsMON, SPModMON, LastSPModMON;

/* Weigher Module */
unsigned int LHopperSensor, RHopperSensor;

unsigned char WLLrGatePneuValve, WLLrGatePneuValveMON, WLSmGatePneuValve, WLSmGatePneuValveMON;
unsigned char WLOpGatePneuValve, WLOpGatePneuValveMON, WRLrGatePneuValve, WRLrGatePneuValveMON;
unsigned char WRSmGatePneuValve, WRSmGatePneuValveMON, WROpGatePneuValve, WROpGatePneuValveMON;

unsigned int WLOpGatePneuDuration, WROpGatePneuDuration, WNextOpGateDelay;
unsigned int WLOpGatePneuDurationCounter, WROpGatePneuDurationCounter, WNextOpGateDelayCounter;
signed long int WLCLSDARatioCustomKg, WLCRSDARatioCustomKg, WLCLSDAReadingOffset, WLCRSDAReadingOffset;
signed long int WLCLSDARatioCustomKgCO, WLCRSDARatioCustomKgCO, WLCLSDAReadingOffsetCO, WLCRSDAReadingOffsetCO;
signed long int WLCLDispWeight, WLCRDispWeight, WLCLSDAReading, WLCRSDAReading;

signed long int WLCLSDAReadingCustomKg, WLCLSDAReadingAverage, WLCLSDAReadingZero, WLCLSDAReadingTemp;
signed long int WLCLSDAReadingBuffer[32];
unsigned char WLCLCalibrationMode, WLCLSDAAverageCounter, WLCLLoadSampling = 1, WLCLWaitingMode;
unsigned char WLCLSCLCounter;

signed long int WLCRSDAReadingCustomKg, WLCRSDAReadingAverage, WLCRSDAReadingZero, WLCRSDAReadingTemp;
signed long int WLCRSDAReadingBuffer[32];
unsigned char WLCRCalibrationMode, WLCRSDAAverageCounter, WLCRLoadSampling = 1, WLCRWaitingMode;
unsigned char WLCRSCLCounter;

signed int WLCLTargetWeight, WLCRTargetWeight, WLCLCorrectionWeight, WLCRCorrectionWeight;
signed int WLCLSlowAt, WLCRSlowAt;
unsigned long int WCustomKg;
unsigned char WLCLOnZeroingBtn, WLCROnZeroingBtn, LastWLCLOnZeroingBtn, LastWLCROnZeroingBtn;
unsigned char WLCLCalibrationFlag, WLCRCalibrationFlag, WLCLCalibrationFlagCounter, WLCRCalibrationFlagCounter;

unsigned char WLCLOnZeroingBtn, WLCROnZeroingBtn, WLCLWeightOK, WLCRWeightOK;
unsigned char WLCLWeightOKFlag, WLCRWeightOKFlag, WLCLWeightOKCounter, WLCRWeightOKCounter;

signed long int WLCLTempReading, WLCLSDAReadingNormalised, WLCRTempReading, WLCRSDAReadingNormalised;
unsigned char WZeroMON, WLCLZeroMONFlag, WLCRZeroMONFlag;
unsigned int WLCLZeroDelayCounter, WLCRZeroDelayCounter;
unsigned char i;

unsigned char WSoloEnable, LastWSoloEnable, WSoloOpenGate;
unsigned char WLCNextGatePriority;

/* External Heater Module */
unsigned int HeaterErrorCounter, HeaterErrorMsgDuration;
unsigned char HeaterError, HeaterErrorMsg;

/* Auxiliaries */
unsigned char TowerLightGreen, TowerLightYellow, TowerLightRed;
unsigned char LastTRModuleStep, LastGTModuleStep, LastOFModuleStep, LastSPModuleStep;
unsigned int TRModuleStepCounter, GTModuleStepCounter, OFModuleStepCounter, SPModuleStepCounter;
//*****************************************************************************
// Interrupt handler for the SSI0 interrupt
//*****************************************************************************
void IntSSI0(void)
{
	SSIIntClear(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );
	SSIDataGetNonBlocking(SSI0_BASE, &SPIIn[SPICounter]);//SPIIn[SPICounter] = SPDR;
	//PORTSS = TRUE;
	switch(SPICounter)
	{
		case 0:
			if(SPIIn[0] != 0xF0)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
			}
		break;
		case 1:
			if(SPIIn[1] != 0x64)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
			}
			break;
		case 2:
			if(SPIIn[2] != 0x04)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
			}
			break;
		case 24:
			if(SPIIn[24] != 0x0F)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
			}
			break;
		case 25:
			if(SPIIn[25] != 0x0F)
			{
				SPICounter = 0;
				SPICounterSkip = TRUE;
				SPIComErrorFlag = TRUE;
			}
	}

	if(SPICounter >= 25)                              // SPI Communication has read all the frame successfully
	{
		if(SPIComErrorFlag)		// First Cycle After SPI Transmission Succeeded
		{
			SPIFirstCycleDurationCounter = 1000;
		}
		SPIComErrorFlag = FALSE;

		ITTrayPosLeftSensor =        	SPIIn[3] % 2;
		ITTrayPosRightSensor =       	(SPIIn[3] >> 1) % 2;
		ITTrayElevationMaxSensor =      (SPIIn[3] >> 2) % 2;
		ITTrayElevationMinSensor =      (SPIIn[3] >> 3) % 2;
		ITLeftMaterialAvailableSensor = (SPIIn[3] >> 4) % 2;	// Active High From Slave
		ITLeftMaterialNotAvailableSensor = (SPIIn[3] >> 5) % 2;

		GTInFillGripPneuLRetSensor 	=   SPIIn[4] % 2;
		GTInFillGripPneuRRetSensor 	=   (SPIIn[4] >> 1) % 2;
		GTOutFillOpenPneuLRetSensor	=   (SPIIn[4] >> 2) % 2;
		GTOutFillOpenPneuRRetSensor	=   (SPIIn[4] >> 3) % 2;
		GTOutFillOpenPneuLExtSensor	=   (SPIIn[4] >> 4) % 2;
		GTOutFillOpenPneuRExtSensor	=   (SPIIn[4] >> 5) % 2;
		GTOutFillGripPneuLRetSensor	=   (SPIIn[4] >> 6) % 2;
		GTOutFillGripPneuRRetSensor	=   (SPIIn[4] >> 7) % 2;
		GTTransferPneuRetSensor		=   SPIIn[5] % 2;
		GTTransferPneuExtSensor		=   (SPIIn[5] >> 1) % 2;

		VFVibEngagePneuRetSensor	=   SPIIn[6] % 2;
		VFVibEngagePneuExtSensor	=   (SPIIn[6] >> 1) % 2;

		SPVibEngagePneuRetSensor	=   SPIIn[7] % 2;
		SPVibEngagePneuExtSensor	=   (SPIIn[7] >> 1) % 2;
		SPPusherPneuRetSensor		=   (SPIIn[7] >> 2) % 2;
		SPPusherPneuExtSensor		=   (SPIIn[7] >> 3) % 2;

		SlavePortA = SPIIn[8];
		SlavePortB = SPIIn[9];
		SlavePortC = SPIIn[10];
		SlavePortD = SPIIn[11];
		SlavePortE = SPIIn[12];
		SlavePortF = SPIIn[13];
		SlavePortG = SPIIn[14];
		SlavePortH = SPIIn[15];
		SlavePortI = SPIIn[16];
		SlavePortJ = SPIIn[17];

		SPICounter = 0;
		SPICounterSkip = TRUE;
		SPIDataLock = FALSE;
	}

	if(SPICounterSkip)
		SPICounterSkip = FALSE;
	else
		SPICounter++;

	SPITransmitted = FALSE;
}

//*****************************************************************************
// Interrupt handler for the UART0 interrupt
//*****************************************************************************
void UART0IntHandler(void)
{
    unsigned long ulStatus;

    // Get the interrupt status.
    ulStatus = ROM_UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART0_BASE, ulStatus);

    // Loop while there are characters in the receive FIFO.
    if( (ulStatus & UART_INT_RX) == UART_INT_RX )
    {
		if(ROM_UARTCharsAvail(UART0_BASE))
		{
			if(!ReadACK)
			{
				ComIN[dataRXIndex] = ROM_UARTCharGetNonBlocking(UART0_BASE);
	            switch(dataRXIndex)
	            {
	                case 0:                                         // 0x0F
	                    break;
	                case 1:             // station number
	                    if(ComIN[1] != 0x01)
	                        dataRXIndex = 0;
	                    break;
	                case 2:             // function number
	                    if(ComIN[2] != 0x03)
	                        dataRXIndex = 0;
	                    break;
	                case 3:
	                    ComAddress = ComIN[3];
	                    break;
	                case 4:
	                    BytestoRead = ComIN[4];
	                    break;
	                case 5:             // Vibrator Counter Setting
	                    if(ComAddress == 1)
	                    {
	                    	if(!MachineRunning)
	                    	{
								ServiceMode = 				ComIN[5] % 2;
								ITMotorTrayElevatorMON =    (ComIN[5] >> 1) % 2;
								ITMotorTrayElevatorDirMON = (ComIN[5] >> 2) % 2;                                          // Vibrator ON/OFF
								ITValveTrayPosMON =   		(ComIN[5] >> 3) % 2;
								TRExtenderPneuValveMON =    (ComIN[5] >> 4) % 2;
								TRReorientPneuValveMON =    (ComIN[5] >> 5) % 2;
								TRVacuumValveMON = 			(ComIN[5] >> 6) % 2;
								TRFoilPusherValveMON = 		(ComIN[5] >> 7) % 2;
	                    	}
	                    }
	                    break;
	                case 6:
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                    	{
	                        	GTInFillGripPneuValveMON =	ComIN[6] % 2;
	                        	GTOutFillOpenPneuValveMON = (ComIN[6] >> 1) % 2;
	                        	GTOutFillGripPneuValveMON = (ComIN[6] >> 2) % 2;
	                        	GTTransferPneuValveMON =    (ComIN[6] >> 3) % 2;
	                        	OFGatePneuValveMON =     	(ComIN[6] >> 4) % 2;
	                        	OFEngagePneuValveMON =     	(ComIN[6] >> 5) % 2;
	                        	OFOpenPneuValveMON = 		(ComIN[6] >> 6) % 2;
	                        	OFVacuumValveMON  = 		(ComIN[6] >> 7) % 2;
	                        }
	                    }
	                    break;
	                case 7:
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                    	{
	                        	VFVibEngagePneuValveMON =  	ComIN[7] % 2;
	                        	VFVibratorMON = 			(ComIN[7] >> 1) % 2;
	                        	SPSealerPneuValveMON =   	(ComIN[7] >> 2) % 2;
	                        	SPVibEngagePneuValveMON = 	(ComIN[7] >> 3) % 2;
	                        	SPPusherValveMON = 			(ComIN[7] >> 4) % 2;
	                        	SPVibratorMON = 			(ComIN[7] >> 5) % 2;
	                        	WLLrGatePneuValveMON = 		(ComIN[7] >> 6) % 2;
	                        	WLSmGatePneuValveMON = 		(ComIN[7] >> 7) % 2;
	                        }
	                    }
	                    break;
	                case 8:
	                    if(ComAddress == 1)
	                    {
	                    	WLOpGatePneuValveMON = 	ComIN[8] % 2;
	                    	WRLrGatePneuValveMON =	(ComIN[8] >> 1) % 2;                                          // Vibrator ON/OFF
	                    	WRSmGatePneuValveMON = 	(ComIN[8] >> 2) % 2;
	                    	WROpGatePneuValveMON = 	(ComIN[8] >> 3) % 2;
	                    	//RunBtn = 				(ComIN[8] >> 4) % 2;
	                    	//StopBtn =				(ComIN[8] >> 5) % 2;
	                    	if( LastWLCLOnZeroingBtn != ((ComIN[8] >> 6) % 2) )
	                    		WLCLOnZeroingBtn =		(ComIN[8] >> 6) % 2;
	                    	LastWLCLOnZeroingBtn = (ComIN[8] >> 6) % 2;

	                    	if( LastWLCROnZeroingBtn != ((ComIN[8] >> 7) % 2) )
	                    		WLCROnZeroingBtn =		(ComIN[8] >> 7) % 2;
	                    	LastWLCROnZeroingBtn = (ComIN[8] >> 7) % 2;
	                    }
	                    break;
	                case 9:
	                    if(ComAddress == 1)
	                    {
	                    	ITEnable = 					ComIN[9] % 2;
	                    	ITMotorTrayElevatorDirSet =	(ComIN[9] >> 1) % 2;
	                    	WSoloEnable =				(ComIN[9] >> 2) % 2;
	                    	WZeroMON =					(ComIN[9] >> 3) % 2;
	                    	IgnoreAirError =			(ComIN[9] >> 4) % 2;
	                    	IgnoreTRVacuumSensor = 		(ComIN[9] >> 5) % 2;
	                    	ITOverride = 				(ComIN[9] >> 6) % 2;
	                    	if( LastInitMON != ((ComIN[9] >> 7) % 2) )
								InitMON =		(ComIN[9] >> 7) % 2;
							LastInitMON = (ComIN[9] >> 7) % 2;
	                    }
	                    break;
	                case 10:
	                    if(ComAddress == 1)
	                    {
	                    	// Step by Step
	                    	if( LastTRSbsMON != (ComIN[10] % 2) )
	                    		TRSbsMON =		ComIN[10] % 2;
	                    	LastTRSbsMON = ComIN[10] % 2;
	                    	if( LastGTSbsMON != ((ComIN[10] >> 1) % 2) )
								GTSbsMON =		(ComIN[10] >> 1) % 2;
							LastGTSbsMON = (ComIN[10] >> 1) % 2;
	                    	if( LastOFSbsMON != ((ComIN[10] >> 2) % 2) )
								OFSbsMON =		(ComIN[10] >> 2) % 2;
							LastOFSbsMON = (ComIN[10] >> 2) % 2;
	                    	if( LastSPSbsMON != ((ComIN[10] >> 3) % 2) )
								SPSbsMON =		(ComIN[10] >> 3) % 2;
							LastSPSbsMON = (ComIN[10] >> 3) % 2;

							// Module by module
	                    	if( LastTRModMON != ((ComIN[10] >> 4) % 2) )
	                    		TRModMON =		(ComIN[10] >> 4) % 2;
	                    	LastTRModMON = (ComIN[10] >> 4) % 2;
	                    	if( LastGTModMON != ((ComIN[10] >> 5) % 2) )
								GTModMON =		(ComIN[10] >> 5) % 2;
							LastGTModMON = (ComIN[10] >> 5) % 2;
	                    	if( LastOFModMON != ((ComIN[10] >> 6) % 2) )
								OFModMON =		(ComIN[10] >> 6) % 2;
							LastOFModMON = (ComIN[10] >> 6) % 2;
	                    	if( LastSPModMON != ((ComIN[10] >> 7) % 2) )
								SPModMON =		(ComIN[10] >> 7) % 2;
							LastSPModMON = (ComIN[10] >> 7) % 2;
	                    }
	                    break;
	                case 11:
	                    if(ComAddress == 1)
	                    {
	                    	TRVacuumValveDelay = ComIN[11] * 10;
	                    }
	                    break;
	                case 12:
	                    if(ComAddress == 1)
	                    {
	                    	GTCloseBagDelay = ComIN[12] * 10;
	                    }
	                    break;
	                case 13:
	                    if(ComAddress == 1)
	                    {
	                    	GTBothOpenGripDelay = ComIN[13] * 10;
	                    }
	                    break;
	                case 14:
	                    if(ComAddress == 1)
	                    {
	                    	OFGatePneuValveDuration = ComIN[14] * 10;
	                    }
	                    break;
	                case 15:
	                    if(ComAddress == 1)
	                    {
	                    	OFVacuumValveDelay = ComIN[15] * 10;
	                    }
	                    break;
	                case 16:
	                    if(ComAddress == 1)
	                    {
	                    	SPSealerPneuValveDuration = ComIN[16] * 10;
	                    }
	                    break;
	                case 17:
	                    if(ComAddress == 1)
	                    {
	                    	SPSealerPneuValveDelay = ComIN[17] * 10;
	                    }
	                    break;
	                case 18:
	                    if(ComAddress == 1)
	                    {
	                    	WLOpGatePneuDuration = ComIN[18] * 100;
	                    }
	                    break;
	                case 19:
	                    if(ComAddress == 1)
	                    {
	                    	WROpGatePneuDuration = ComIN[19] * 100;
	                    }
	                    break;
	                case 20:
	                    if(ComAddress == 1)
	                    {
	                    	WNextOpGateDelay = ComIN[20] * 10;
	                    }
	                    break;
	                case 24:
	                    if(ComAddress == 1)
	                    {
	                    	WLCLSDARatioCustomKg = (ComIN[21] << 24) + (ComIN[22] << 16) + (ComIN[23] << 8) + ComIN[24];
	                    }
	                    break;
	                case 28:
	                    if(ComAddress == 1)
	                    {
	                    	WLCRSDARatioCustomKg = (ComIN[25] << 24) + (ComIN[26] << 16) + (ComIN[27] << 8) + ComIN[28];
	                    }
	                    break;
	                case 32:
	                    if(ComAddress == 1)
	                    {
	                    	WLCLSDAReadingOffset = (ComIN[29] << 24) + (ComIN[30] << 16) + (ComIN[31] << 8) + ComIN[32];
	                    }
	                    break;
	                case 36:
	                    if(ComAddress == 1)
	                    {
	                    	WLCRSDAReadingOffset = (ComIN[33] << 24) + (ComIN[34] << 16) + (ComIN[35] << 8) + ComIN[36];
	                    }
	                    break;
	                case 38:
	                    if(ComAddress == 1)
	                    {
	                    	WLCLTargetWeight = (ComIN[37] << 8) + ComIN[38];
	                    }
	                    break;
	                case 40:
	                    if(ComAddress == 1)
	                    {
	                    	WLCRTargetWeight = (ComIN[39] << 8) + ComIN[40];
	                    }
	                    break;
	                case 42:
	                    if(ComAddress == 1)
	                    {
	                    	WLCLCorrectionWeight = (ComIN[41] << 8) + ComIN[42];
	                    }
	                    break;
	                case 44:
	                    if(ComAddress == 1)
	                    {
	                    	WLCRCorrectionWeight = (ComIN[43] << 8) + ComIN[44];
	                    }
	                    break;
	                case 46:
	                    if(ComAddress == 1)
	                    {
	                    	WLCLSlowAt = (ComIN[45] << 8) + ComIN[46];
	                    }
	                    break;
	                case 48:
	                    if(ComAddress == 1)
	                    {
	                    	WLCRSlowAt = (ComIN[47] << 8) + ComIN[48];
	                    }
	                    break;
	                case 50:
	                    if(ComAddress == 1)
	                    {
	                    	WCustomKg = ((ComIN[49] << 8) + ComIN[50]) * 10;
	                    }
	                    break;
	                case 51:
	                    if(ComAddress == 1)
	                    {
	                    	HMICurrentWindow = ComIN[51];
	                    	if(HMILastWindow != HMICurrentWindow)
	                    	{
	                    		// LEFT LOADCELL
	                    		// If going from Calibration page 1 -> page 2
	                    		if((HMICurrentWindow == 37) && (HMILastWindow == 36) )
	                    			WLCLCalibrationMode = 1;
	                    		// If going from Calibration page 2 -> Main Calibration Page
	                    		if((HMICurrentWindow == 40) && (HMILastWindow == 37) )
	                    			WLCLCalibrationMode = 2;

	                    		// RIGHT LOADCELL
	                    		// If going from Calibration page 1 -> page 2
	                    		if((HMICurrentWindow == 39) && (HMILastWindow == 38) )
	                    			WLCRCalibrationMode = 1;
	                    		// If going from Calibration page 2 -> Main Calibration Page
	                    		if((HMICurrentWindow == 40) && (HMILastWindow == 39) )
	                    			WLCRCalibrationMode = 2;
	                    	}
	                    	HMILastWindow = HMICurrentWindow;
	                    }
	                    break;
	                case 52:
	                    if(ComAddress == 1)
	                    {
	                    	GTFPushPneuValveDelay = ComIN[52] * 10;
	                    }
	                    break;
	                case 54:
	                    if(ComAddress == 1)
	                    {
	                    	if(!Initting)
	                    		BufferSensor = (ComIN[53] << 8) + ComIN[54];
	                    	else
	                    		BufferSensor = 120;
	                    }
	                    break;
	                case 55:
	                    if(ComAddress == 1)
	                    {
	                    	VFVibEngageVibratorFrequency = ComIN[55];
	                    	if(VFVibEngageVibratorFrequency)
	                    		VFVibEngageVibratorPeriod = 10000 / VFVibEngageVibratorFrequency; // 1 (0.1hz) = 10000 ms
	                    	else
	                    		VFVibEngageVibratorPeriod = 0;		// Means DISACTIVATE VIBRATION MODE
	                    }
	                    break;
	                case 56:
	                    if(ComAddress == 1)
	                    {
	                    	if(ComIN[56])
	                    		VFVibEngageVibratorDutyCycle = ComIN[56];
	                    	else
	                    		VFVibEngageVibratorDutyCycle = 50;
	                    }
	                    break;
	                case 57:
	                    if(ComAddress == 1)
	                    {
	                    	if( (!WLOpGatePneuDurationCounter) && (!WROpGatePneuDurationCounter) )
	                    		WSoloOpenGate = ComIN[57] % 2;
	                    	else
	                    		WSoloOpenGate = FALSE;
	                    	//ITMotorTrayElevatorDirSet =	(ComIN[9] >> 1) % 2;
	                    }
	                    break;
	                case 58:
	                    if(ComAddress == 1)
	                    {
	                    	VFVibratorEngageDelay = ComIN[58] * 10;
	                    }
	                    break;
	                case 59:
	                    if(ComAddress == 1)
	                    {
	                    	VFVibratorEngageVibDelay = ComIN[59] * 10;
	                    }
	                    break;
	                case 60:
	                    if(ComAddress == 1)
	                    {
	                    	OFGatePneuValveVibratorFrequency = ComIN[60];
	                    	if(OFGatePneuValveVibratorFrequency)
	                    		OFGatePneuValveVibratorPeriod = 10000 / OFGatePneuValveVibratorFrequency; // 1 (0.1hz) = 10000 ms
	                    	else
	                    		OFGatePneuValveVibratorPeriod = 0;		// Means DISACTIVATE VIBRATION MODE
	                    }
	                    break;
	                case 61:
	                    if(ComAddress == 1)
	                    {
	                    	if(ComIN[61])
	                    		OFGatePneuValveVibratorDutyCycle = ComIN[61];
	                    	else
	                    		OFGatePneuValveVibratorDutyCycle = 50;
	                    }
	                    break;
	                case 62:
	                    if(ComAddress == 1)
	                    {
	                    	//OFGatePneuValveDelay = ComIN[62] * 10;
	                    	GTRequestFillingDelay = ComIN[62] * 10;		// WARNING BORROW VARIABLE
	                    }
	                    break;
	                case 63:
	                    if(ComAddress == 1)
	                    {
	                    	OFGatePneuValveVibDelay = ComIN[63] * 10;
	                    }
	                    break;
	                case 64:
	                    if(ComAddress == 1)
	                    {
	                    	OFRequestFillingDelay = ComIN[64] * 10;
	                    }
	                    break;
	                case 90:	// case 90 - 98 is for Direct PORTOUT  PORTA = 90, PORTJ = 98
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {

	                        }
	                    }
	                    break;
	                case 91:	// PORTB
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {

	                        }
	                    }
	                    break;
	                case 92:	// PORTC
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {

	                        }
	                    }
	                    break;
	                case 93:	// PORTD
	                    if(ComAddress == 1)
	                    {

	                    }
	                    break;
	                case 96:	// PORTG
	                    if(ComAddress == 1)
	                    {

	                    }
	                    break;
	                case 98:	// PORTJ
	                    if(ComAddress == 1)
	                    {
	                        if(!MachineRunning)
	                        {

	                        }
	                    }
	                    break;
	                case 99:
	                    if(ComAddress == 1)
	                    {
	                    }
	                    break;
	                case 101:
	                    if(ComAddress == 1)
	                    {
	                    }
	                    break;

	            }
				if(dataRXIndex == 109)
				{
					UARTTimeOut = UARTDISABLEDCOUNT;
                    ComAddress = ComIN[3];
                    BytestoRead = ComIN[4];
					ReadACK = TRUE;
					dataTXIndex = ComAddress * 20;
					dataRXIndex = 0;
	                //ROM_UARTCharPutNonBlocking(UART0_BASE, ComOUT[dataTXIndex]);
	                //dataTXIndex++;
					IntRXCounter++;
					ComFlag = TRUE;                         // Have communicated with TouchScreen Successfully
				}

				if( (ComIN[0] != 0xF0) || (ReadACK == TRUE) )
				{
					dataRXIndex = 0;
					//debugcounter1++;
				}
				else
					dataRXIndex++;
			}
		}
    }


    // Interrupt Transmit Handler
    if( (ulStatus & UART_INT_TX) == UART_INT_TX )
    {
    }
}

//*****************************************************************************
// Interrupt handler for the LD_DATA interrupt
//*****************************************************************************
void IntGPIOd(void)
{
	ROM_GPIOPinIntClear(PIN_W_LCR_DATA_BASE, PIN_W_LCR_DATA_PIN);
    if(WLCRWaitingMode)
    	WLCRWaitingMode = FALSE;
}
void IntGPIOe(void)
{
	ROM_GPIOPinIntClear(PIN_W_LCL_DATA_BASE, PIN_W_LCL_DATA_PIN);
    if(WLCLWaitingMode)
    	WLCLWaitingMode = FALSE;
}

//*****************************************************************************
// Interrupt handler for the Timer0 interrupt
//*****************************************************************************
void Timer0IntHandler(void)
{

    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    LoopTime = LoopCycle;
    LoopCycle = 0;

    // ITValvePos Initialization to avoid crash
    if(SPIFirstCycleDurationCounter)
    {
    	SPIFirstCycleDurationCounter--;
    	if(!ITTrayPosLeftSensor)
    		ITValveTrayPos = TRUE;
    	if(!ITTrayPosRightSensor)
    		ITValveTrayPos = FALSE;
    }
    // ZEROING WEIGHER
    if( WZeroMON && (!MachineRunning) && (!Initting) )
    {
    	if( (!WLCLZeroMONFlag) && (!WLCRZeroMONFlag) )
    	{
    		WLCLZeroMONFlag = TRUE;
    		WLCRZeroMONFlag = TRUE;
			WLOpGatePneuDurationCounter = WLOpGatePneuDuration;
			WROpGatePneuDurationCounter = WROpGatePneuDuration;
    	}
    }
    // Tower Light Subroutine
    // Machine Running :
    // Green if machine active and steps keep running
    // Blinking Yellow if Machine Stopping OR the Steps are stuck somewhere
    // Machine Not Running :
    // Yellow if machine is okay, just waiting to be run
    // Blinking Red if there are errors+
    if(MachineRunning)
    {
    	if( (!TRModuleStepCounter) && (!GTModuleStepCounter) &&
    			(!OFModuleStepCounter) && (!SPModuleStepCounter) )
    	{
    		TowerLightGreen = FALSE;
    		TowerLightYellow = BlinkFlag;
    	}
    	else if( MachineStopping )
    	{
    		TowerLightGreen = FALSE;
    		TowerLightYellow = BlinkFlag;
    	}
    	else
    	{
    		TowerLightGreen = TRUE;
    		TowerLightYellow = FALSE;
    	}
    }
    else
    {
    	TowerLightGreen = FALSE;
    	if(Error || FatalError)
    	{
    		TowerLightYellow = FALSE;
    		TowerLightRed = BlinkFlag;
    	}
    	else
    	{
    		TowerLightYellow = TRUE;
    		TowerLightRed = FALSE;
    	}
    }

    // if EVERY module step got stuck for 5 seconds, then the yellow light will start blinking
    // Meaning 1. out of product 2. out of foil 3. Sensors not responding
    if(LastTRModuleStep == TRModuleStep)
    {
    	if(TRModuleStepCounter)
    		TRModuleStepCounter--;
    }
    else
    	TRModuleStepCounter = 5000;
    LastTRModuleStep = TRModuleStep;

    if(LastGTModuleStep == GTModuleStep)
    {
    	if(GTModuleStepCounter)
    		GTModuleStepCounter--;
    }
    else
    	GTModuleStepCounter = 5000;
    LastGTModuleStep = GTModuleStep;

    if(LastOFModuleStep == OFModuleStep)
    {
    	if(OFModuleStepCounter)
    		OFModuleStepCounter--;
    }
    else
    	OFModuleStepCounter = 5000;
    LastOFModuleStep = OFModuleStep;

    if(LastSPModuleStep == SPModuleStep)
    {
    	if(SPModuleStepCounter)
    		SPModuleStepCounter--;
    }
    else
    	SPModuleStepCounter = 5000;
    LastSPModuleStep = SPModuleStep;

    // STEP BY STEP MODE
	//StepbyStepMode SBSMode
	if(TRSbsMON)
	{
		if(TRModuleStep == 7)
			TRModuleStep = 0;
		else
			TRModuleStep++;

		TRSbsMode = TRUE;
		TRSbsMON = FALSE;
	}
	if(GTSbsMON)
	{
		if(GTModuleStep == 9)
			GTModuleStep = 0;
		else
			GTModuleStep++;

		GTSbsMode = TRUE;
		GTSbsMON = FALSE;
	}
	if(OFSbsMON)
	{
		if(OFModuleStep == 9)
			OFModuleStep = 0;
		else
			OFModuleStep++;

		OFSbsMode = TRUE;
		OFSbsMON = FALSE;
	}
	if(SPSbsMON)
	{
		if(SPModuleStep == 7)
			SPModuleStep = 0;
		else
			SPModuleStep++;

		SPSbsMode = TRUE;
		SPSbsMON = FALSE;
	}

	// Modular Mode
	if(TRModMON)
	{
		if(TRModuleStep == 7)
			TRModuleStep = 0;
		else
			TRModuleStep++;

		TRModMode = TRUE;
		TRModMON = FALSE;
	}
	if(GTModMON)
	{
		if(GTModuleStep == 9)
			GTModuleStep = 0;
		else
			GTModuleStep++;

		GTModMode = TRUE;
		GTModMON = FALSE;
	}
	if(OFModMON)
	{
		if(OFModuleStep == 8)
			OFModuleStep = 0;
		else
			OFModuleStep++;

		OFModMode = TRUE;
		OFModMON = FALSE;
	}
	if(SPModMON)
	{
		if(SPModuleStep == 7)
			SPModuleStep = 0;
		else
			SPModuleStep++;

		SPModMode = TRUE;
		SPModMON = FALSE;
	}

    /* Coiling to PORT */
    if(!MachineRunning && ServiceMode)
    {	// IT Module is controlled by SLAVE
    	ITMotorTrayElevator = ITMotorTrayElevatorMON;
    	ITMotorTrayElevatorDir = ITMotorTrayElevatorDirMON;
    	if(!ITTrayElevationMinSensor)
    		ITValveTrayPos = ITValveTrayPosMON;

    	if(TRExtenderPneuValveMON)
    		ROM_GPIOPinWrite(PORT_TR_EXTPNEU_VALVE_BASE, PORT_TR_EXTPNEU_VALVE_PIN, PORT_TR_EXTPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_TR_EXTPNEU_VALVE_BASE, PORT_TR_EXTPNEU_VALVE_PIN, 0);
    	if(TRReorientPneuValveMON)
    		ROM_GPIOPinWrite(PORT_TR_REOPNEU_VALVE_BASE, PORT_TR_REOPNEU_VALVE_PIN, PORT_TR_REOPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_TR_REOPNEU_VALVE_BASE, PORT_TR_REOPNEU_VALVE_PIN, 0);
    	if(TRVacuumValveMON)
    		ROM_GPIOPinWrite(PORT_TR_VACUUM_BASE, PORT_TR_VACUUM_PIN, PORT_TR_VACUUM_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_TR_VACUUM_BASE, PORT_TR_VACUUM_PIN, 0);
    	if(TRFoilPusherValveMON)
    		ROM_GPIOPinWrite(PORT_GT_INFPUSHPNEU_VALVE_BASE, PORT_GT_INFPUSHPNEU_VALVE_PIN, PORT_GT_INFPUSHPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_GT_INFPUSHPNEU_VALVE_BASE, PORT_GT_INFPUSHPNEU_VALVE_PIN, 0);

    	// GT Module is controlled by SLAVE
    	GTInFillGripPneuValve = GTInFillGripPneuValveMON;
    	GTOutFillOpenPneuValve = GTOutFillOpenPneuValveMON;
    	GTOutFillGripPneuValve = GTOutFillGripPneuValveMON;
    	GTTransferPneuValve = GTTransferPneuValveMON;

    	if(OFGatePneuValveMON)
    		ROM_GPIOPinWrite(PORT_OF_GATEPNEU_VALVE_BASE, PORT_OF_GATEPNEU_VALVE_PIN, PORT_OF_GATEPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_OF_GATEPNEU_VALVE_BASE, PORT_OF_GATEPNEU_VALVE_PIN, 0);
    	if(OFEngagePneuValveMON)
    		ROM_GPIOPinWrite(PORT_OF_ENGAGEPNEU_VALVE_BASE, PORT_OF_ENGAGEPNEU_VALVE_PIN, PORT_OF_ENGAGEPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_OF_ENGAGEPNEU_VALVE_BASE, PORT_OF_ENGAGEPNEU_VALVE_PIN, 0);
    	if(OFOpenPneuValveMON)
    		ROM_GPIOPinWrite(PORT_OF_OPENPNEU_VALVE_BASE, PORT_OF_OPENPNEU_VALVE_PIN, PORT_OF_OPENPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_OF_OPENPNEU_VALVE_BASE, PORT_OF_OPENPNEU_VALVE_PIN, 0);
    	if(OFVacuumValveMON)
    		ROM_GPIOPinWrite(PORT_OF_VACUUM_BASE, PORT_OF_VACUUM_PIN, PORT_OF_VACUUM_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_OF_VACUUM_BASE, PORT_OF_VACUUM_PIN, 0);

    	// Vibrator Filling Module is controlled by SLAVE
    	VFVibEngagePneuValve = VFVibEngagePneuValveMON;
    	VFVibrator = VFVibratorMON;

    	if(SPSealerPneuValveMON)
    		ROM_GPIOPinWrite(PORT_SP_SEALERPNEU_VALVE_BASE, PORT_SP_SEALERPNEU_VALVE_PIN, PORT_SP_SEALERPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_SP_SEALERPNEU_VALVE_BASE, PORT_SP_SEALERPNEU_VALVE_PIN, 0);

    	// Vibrator Sealer Module is controller by SLAVE
    	SPVibEngagePneuValve = SPVibEngagePneuValveMON;
    	SPPusherValve = SPPusherValveMON;
    	SPVibrator = SPVibratorMON;

    	if(WLLrGatePneuValveMON)
			ROM_GPIOPinWrite(PORT_W_LLRGATEPNEU_VALVE_BASE, PORT_W_LLRGATEPNEU_VALVE_PIN, PORT_W_LLRGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_LLRGATEPNEU_VALVE_BASE, PORT_W_LLRGATEPNEU_VALVE_PIN, 0);
		if(WLSmGatePneuValveMON)
			ROM_GPIOPinWrite(PORT_W_LSMGATEPNEU_VALVE_BASE, PORT_W_LSMGATEPNEU_VALVE_PIN, PORT_W_LSMGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_LSMGATEPNEU_VALVE_BASE, PORT_W_LSMGATEPNEU_VALVE_PIN, 0);
		if(WLOpGatePneuValveMON)
			ROM_GPIOPinWrite(PORT_W_LOPGATEPNEU_VALVE_BASE, PORT_W_LOPGATEPNEU_VALVE_PIN, PORT_W_LOPGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_LOPGATEPNEU_VALVE_BASE, PORT_W_LOPGATEPNEU_VALVE_PIN, 0);

		if(WRLrGatePneuValveMON)
			ROM_GPIOPinWrite(PORT_W_RLRGATEPNEU_VALVE_BASE, PORT_W_RLRGATEPNEU_VALVE_PIN, PORT_W_RLRGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_RLRGATEPNEU_VALVE_BASE, PORT_W_RLRGATEPNEU_VALVE_PIN, 0);
		if(WRSmGatePneuValveMON)
			ROM_GPIOPinWrite(PORT_W_RSMGATEPNEU_VALVE_BASE, PORT_W_RSMGATEPNEU_VALVE_PIN, PORT_W_RSMGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_RSMGATEPNEU_VALVE_BASE, PORT_W_RSMGATEPNEU_VALVE_PIN, 0);
		if(WROpGatePneuValveMON)
			ROM_GPIOPinWrite(PORT_W_ROPGATEPNEU_VALVE_BASE, PORT_W_ROPGATEPNEU_VALVE_PIN, PORT_W_ROPGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_ROPGATEPNEU_VALVE_BASE, PORT_W_ROPGATEPNEU_VALVE_PIN, 0);
    }
    if(LastServiceMode != ServiceMode)		// if servicemode just turned off, back to standby position
    {
    	if(!ServiceMode)
    	{
    		// IT Module
    		ITMotorTrayElevator = FALSE;
			//ITMotorTrayElevatorDir = ITMotorTrayElevatorDirMON;
			//ITValveTrayPos = ITValveTrayPosMON;

    		// TR Module
			ROM_GPIOPinWrite(PORT_TR_EXTPNEU_VALVE_BASE, PORT_TR_EXTPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PORT_TR_REOPNEU_VALVE_BASE, PORT_TR_REOPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PORT_TR_VACUUM_BASE, PORT_TR_VACUUM_PIN, 0);

			ROM_GPIOPinWrite(PORT_GT_INFPUSHPNEU_VALVE_BASE, PORT_GT_INFPUSHPNEU_VALVE_PIN, 0);

			// GT Module
			GTInFillGripPneuValve = TRUE;	// Grip Open
			GTTransferPneuValve = FALSE;
			GTOutFillOpenPneuValve = FALSE;
			GTOutFillGripPneuValve = TRUE;	// Grip Open

			// OF Module
			ROM_GPIOPinWrite(PORT_OF_GATEPNEU_VALVE_BASE, PORT_OF_GATEPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PORT_OF_ENGAGEPNEU_VALVE_BASE, PORT_OF_ENGAGEPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PORT_OF_OPENPNEU_VALVE_BASE, PORT_OF_OPENPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PIN_OF_VACUUM_SENSOR_BASE, PIN_OF_VACUUM_SENSOR_PIN, 0);
	    	VFVibEngagePneuValve = FALSE;
	    	VFVibrator = FALSE;

	    	// SP Module
			ROM_GPIOPinWrite(PORT_SP_SEALERPNEU_VALVE_BASE, PORT_SP_SEALERPNEU_VALVE_PIN, 0);
			SPVibEngagePneuValve = FALSE;
			SPPusherValve = FALSE;
			SPVibrator = FALSE;

			// W Module
			ROM_GPIOPinWrite(PORT_W_LLRGATEPNEU_VALVE_BASE, PORT_W_LLRGATEPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PORT_W_LSMGATEPNEU_VALVE_BASE, PORT_W_LSMGATEPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PORT_W_LOPGATEPNEU_VALVE_BASE, PORT_W_LOPGATEPNEU_VALVE_PIN, 0);

			ROM_GPIOPinWrite(PORT_W_RLRGATEPNEU_VALVE_BASE, PORT_W_RLRGATEPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PORT_W_RSMGATEPNEU_VALVE_BASE, PORT_W_RSMGATEPNEU_VALVE_PIN, 0);
			ROM_GPIOPinWrite(PORT_W_ROPGATEPNEU_VALVE_BASE, PORT_W_ROPGATEPNEU_VALVE_PIN, 0);
    	}
    }
    LastServiceMode = ServiceMode;

    /* Outputs that need to be controlled indepently
    if(!MachineRunning)
    {
    	if(PRRollerValveON)
    		ROM_GPIOPinWrite(PORT_PULLROLL_VALVE_BASE, PORT_PULLROLL_VALVE_PIN, PORT_PULLROLL_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_PULLROLL_VALVE_BASE, PORT_PULLROLL_VALVE_PIN, 0);
    }
	  Outputs that need to be controlled indepently */

    // When Machine Running
    if(!ServiceMode)
    {
    	// Skip ALL SLAVES I/O
    	if(TRExtenderPneuValve)
    		ROM_GPIOPinWrite(PORT_TR_EXTPNEU_VALVE_BASE, PORT_TR_EXTPNEU_VALVE_PIN, PORT_TR_EXTPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_TR_EXTPNEU_VALVE_BASE, PORT_TR_EXTPNEU_VALVE_PIN, 0);
    	if(TRReorientPneuValve)
    		ROM_GPIOPinWrite(PORT_TR_REOPNEU_VALVE_BASE, PORT_TR_REOPNEU_VALVE_PIN, PORT_TR_REOPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_TR_REOPNEU_VALVE_BASE, PORT_TR_REOPNEU_VALVE_PIN, 0);
    	if(TRVacuumValve)
    		ROM_GPIOPinWrite(PORT_TR_VACUUM_BASE, PORT_TR_VACUUM_PIN, PORT_TR_VACUUM_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_TR_VACUUM_BASE, PORT_TR_VACUUM_PIN, 0);
    	if(GTFPushPneuValve)
    		ROM_GPIOPinWrite(PORT_GT_INFPUSHPNEU_VALVE_BASE, PORT_GT_INFPUSHPNEU_VALVE_PIN, PORT_GT_INFPUSHPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_GT_INFPUSHPNEU_VALVE_BASE, PORT_GT_INFPUSHPNEU_VALVE_PIN, 0);

    	// Filling
    	if(OFGatePneuValve)
    		ROM_GPIOPinWrite(PORT_OF_GATEPNEU_VALVE_BASE, PORT_OF_GATEPNEU_VALVE_PIN, PORT_OF_GATEPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_OF_GATEPNEU_VALVE_BASE, PORT_OF_GATEPNEU_VALVE_PIN, 0);
    	if(OFEngagePneuValve)
    		ROM_GPIOPinWrite(PORT_OF_ENGAGEPNEU_VALVE_BASE, PORT_OF_ENGAGEPNEU_VALVE_PIN, PORT_OF_ENGAGEPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_OF_ENGAGEPNEU_VALVE_BASE, PORT_OF_ENGAGEPNEU_VALVE_PIN, 0);
    	if(OFOpenPneuValve)
    		ROM_GPIOPinWrite(PORT_OF_OPENPNEU_VALVE_BASE, PORT_OF_OPENPNEU_VALVE_PIN, PORT_OF_OPENPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_OF_OPENPNEU_VALVE_BASE, PORT_OF_OPENPNEU_VALVE_PIN, 0);
    	if(OFVacuumValve)
    		ROM_GPIOPinWrite(PORT_OF_VACUUM_BASE, PORT_OF_VACUUM_PIN, PORT_OF_VACUUM_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_OF_VACUUM_BASE, PORT_OF_VACUUM_PIN, 0);

    	// Sealing
    	if(SPSealerPneuValve)
    		ROM_GPIOPinWrite(PORT_SP_SEALERPNEU_VALVE_BASE, PORT_SP_SEALERPNEU_VALVE_PIN, PORT_SP_SEALERPNEU_VALVE_PIN);
    	else
    		ROM_GPIOPinWrite(PORT_SP_SEALERPNEU_VALVE_BASE, PORT_SP_SEALERPNEU_VALVE_PIN, 0);

    	//Weigher
    	if(WRLrGatePneuValve)
			ROM_GPIOPinWrite(PORT_W_RLRGATEPNEU_VALVE_BASE, PORT_W_RLRGATEPNEU_VALVE_PIN, PORT_W_RLRGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_RLRGATEPNEU_VALVE_BASE, PORT_W_RLRGATEPNEU_VALVE_PIN, 0);
		if(WRSmGatePneuValve)
			ROM_GPIOPinWrite(PORT_W_RSMGATEPNEU_VALVE_BASE, PORT_W_RSMGATEPNEU_VALVE_PIN, PORT_W_RSMGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_RSMGATEPNEU_VALVE_BASE, PORT_W_RSMGATEPNEU_VALVE_PIN, 0);
		if(WROpGatePneuValve)
			ROM_GPIOPinWrite(PORT_W_ROPGATEPNEU_VALVE_BASE, PORT_W_ROPGATEPNEU_VALVE_PIN, PORT_W_ROPGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_ROPGATEPNEU_VALVE_BASE, PORT_W_ROPGATEPNEU_VALVE_PIN, 0);

		if(WLLrGatePneuValve)
			ROM_GPIOPinWrite(PORT_W_LLRGATEPNEU_VALVE_BASE, PORT_W_LLRGATEPNEU_VALVE_PIN, PORT_W_LLRGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_LLRGATEPNEU_VALVE_BASE, PORT_W_LLRGATEPNEU_VALVE_PIN, 0);
		if(WLSmGatePneuValve)
			ROM_GPIOPinWrite(PORT_W_LSMGATEPNEU_VALVE_BASE, PORT_W_LSMGATEPNEU_VALVE_PIN, PORT_W_LSMGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_LSMGATEPNEU_VALVE_BASE, PORT_W_LSMGATEPNEU_VALVE_PIN, 0);
		if(WLOpGatePneuValve)
			ROM_GPIOPinWrite(PORT_W_LOPGATEPNEU_VALVE_BASE, PORT_W_LOPGATEPNEU_VALVE_PIN, PORT_W_LOPGATEPNEU_VALVE_PIN);
		else
			ROM_GPIOPinWrite(PORT_W_LOPGATEPNEU_VALVE_BASE, PORT_W_LOPGATEPNEU_VALVE_PIN, 0);

    }
    /* Coiling to PORT */

    /* Coiling to PIN */
	//	INPUT WITH BUFFER
    // Infeed Table **IN SLAVE**

    // Buttons
    if(LastRunBtn != !ROM_GPIOPinRead(PIN_BTN_RUN_BASE, PIN_BTN_RUN_PIN))
    	RunBtn = !ROM_GPIOPinRead(PIN_BTN_RUN_BASE, PIN_BTN_RUN_PIN);
    LastRunBtn = !ROM_GPIOPinRead(PIN_BTN_RUN_BASE, PIN_BTN_RUN_PIN);

    if(LastStopBtn != !ROM_GPIOPinRead(PIN_BTN_STOP_BASE, PIN_BTN_STOP_PIN))
    	StopBtn = !ROM_GPIOPinRead(PIN_BTN_STOP_BASE, PIN_BTN_STOP_PIN);
    LastStopBtn = !ROM_GPIOPinRead(PIN_BTN_STOP_BASE, PIN_BTN_STOP_PIN);
    // Infeed Table
	if(ROM_GPIOPinRead(PIN_IT_RMATEMPTY_SENS_BASE, PIN_IT_RMATEMPTY_SENS_PIN))
	{
		if(ITRightMaterialAvailableSensor)
			ITRightMaterialAvailableSensor--;
		ITRightMaterialNotAvailableSensor = 100;
	}
	else
	{
		if(ITRightMaterialNotAvailableSensor)
			ITRightMaterialNotAvailableSensor--;
		ITRightMaterialAvailableSensor = FASTBUFFER;
	}
	// Transfer & Reorient
	if(!ROM_GPIOPinRead(PIN_TR_EXTPNEU_EXTSENS_BASE, PIN_TR_EXTPNEU_EXTSENS_PIN))
	{
		if(TRExtenderPneuExtSensor)
			TRExtenderPneuExtSensor--;
	}
	else
		TRExtenderPneuExtSensor = BufferSensor;
    if(!ROM_GPIOPinRead(PIN_TR_EXTPNEU_RETSENS_BASE, PIN_TR_EXTPNEU_RETSENS_PIN))
	{
		if(TRExtenderPneuRetSensor)
			TRExtenderPneuRetSensor--;
	}
	else
		TRExtenderPneuRetSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_TR_REOPNEU_EXTSENS_BASE, PIN_TR_REOPNEU_EXTSENS_PIN))
	{
		if(TRReorientPneuExtSensor)
			TRReorientPneuExtSensor--;
	}
	else
		TRReorientPneuExtSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_TR_REOPNEU_RETSENS_BASE, PIN_TR_REOPNEU_RETSENS_PIN))
	{
		if(TRReorientPneuRetSensor)
			TRReorientPneuRetSensor--;
	}
	else
		TRReorientPneuRetSensor = BufferSensor;
	if(ROM_GPIOPinRead(PIN_TR_VACUUM_SENS_BASE, PIN_TR_VACUUM_SENS_PIN))	// Active high
	{
		if(TRVacuumSensor)
			TRVacuumSensor--;
	}
	else
		TRVacuumSensor = BufferSensor;

	// Grip & Transfer ** IN SLAVE

	// Filling , Vibrator Filling IN SLAVE
	if(!ROM_GPIOPinRead(PIN_OF_GATEPNEU_EXTSENS_BASE, PIN_OF_GATEPNEU_EXTSENS_PIN))
	{
		if(OFGatePneuExtSensor)
			OFGatePneuExtSensor--;
	}
	else
		OFGatePneuExtSensor = BufferSensor;
    if(!ROM_GPIOPinRead(PIN_OF_GATEPNEU_RETSENS_BASE, PIN_OF_GATEPNEU_RETSENS_PIN))
	{
		if(OFGatePneuRetSensor)
			OFGatePneuRetSensor--;
	}
	else
		OFGatePneuRetSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_OF_ENGAGEPNEU_EXTSENS_BASE, PIN_OF_ENGAGEPNEU_EXTSENS_PIN))
	{
		if(OFEngagePneuExtSensor)
			OFEngagePneuExtSensor--;
	}
	else
		OFEngagePneuExtSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_OF_ENGAGEPNEU_RETSENS_BASE, PIN_OF_ENGAGEPNEU_RETSENS_PIN))
	{
		if(OFEngagePneuRetSensor)
			OFEngagePneuRetSensor--;
	}
	else
		OFEngagePneuRetSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_OF_LOPENPNEU_EXTSENS_BASE, PIN_OF_LOPENPNEU_EXTSENS_PIN))
	{
		if(OFOpenPneuLExtSensor)
			OFOpenPneuLExtSensor--;
	}
	else
		OFOpenPneuLExtSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_OF_ROPENPNEU_EXTSENS_BASE, PIN_OF_ROPENPNEU_EXTSENS_PIN))
	{
		if(OFOpenPneuRExtSensor)
			OFOpenPneuRExtSensor--;
	}
	else
		OFOpenPneuRExtSensor = BufferSensor;
    if(!ROM_GPIOPinRead(PIN_OF_LOPENPNEU_RETSENS_BASE, PIN_OF_LOPENPNEU_RETSENS_PIN))
	{
		if(OFOpenPneuLRetSensor)
			OFOpenPneuLRetSensor--;
	}
	else
		OFOpenPneuLRetSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_OF_ROPENPNEU_RETSENS_BASE, PIN_OF_ROPENPNEU_RETSENS_PIN))
	{
		if(OFOpenPneuRRetSensor)
			OFOpenPneuRRetSensor--;
	}
	else
		OFOpenPneuRRetSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_OF_VACUUM_SENSOR_BASE, PIN_OF_VACUUM_SENSOR_PIN))
	{
		if(OFVacuumSensor)
			OFVacuumSensor--;
	}
	else
		OFVacuumSensor = BufferSensor;

	// Sealing, Vibrator Sealer IN SLAVE
	if(!ROM_GPIOPinRead(PIN_SP_SEALERPNEU_EXTSENS_BASE, PIN_SP_SEALERPNEU_EXTSENS_PIN))
	{
		if(SPSealerPneuExtSensor)
			SPSealerPneuExtSensor--;
	}
	else
		SPSealerPneuExtSensor = BufferSensor;
    if(!ROM_GPIOPinRead(PIN_SP_SEALERPNEU_RETSENS_BASE, PIN_SP_SEALERPNEU_RETSENS_PIN))
	{
		if(SPSealerPneuRetSensor)
			SPSealerPneuRetSensor--;
	}
	else
		SPSealerPneuRetSensor = BufferSensor;

	// Weigher
	if(!ROM_GPIOPinRead(PIN_W_LHOPPER_SENS_BASE, PIN_W_LHOPPER_SENS_PIN))
	{
		if(LHopperSensor)
			LHopperSensor--;
	}
	else
		LHopperSensor = BufferSensor;
	if(!ROM_GPIOPinRead(PIN_W_RHOPPER_SENS_BASE, PIN_W_RHOPPER_SENS_PIN))
	{
		if(RHopperSensor)
			RHopperSensor--;
	}
	else
		RHopperSensor = BufferSensor;

    /* Coiling to PIN */
    if(BlinkFlag)
    {
        if(blinkON)
            blinkON--;
        else
        {
            blinkOFF = 700;
            BlinkFlag = FALSE;
        }

    }
    else
    {
        if(blinkOFF)
            blinkOFF--;
        else
        {
            blinkON = 700;
            BlinkFlag = TRUE;
        }
    }

	/* Infeed Table Module v */
    // IT = Infeed Table
    if(ITEnable && (!SPIComErrorFlag))
    {
    	if(!ITSwitchTrayStep)
    	{
    		// Check if there are any foil at all at any of the tray
    		if( (!ITLeftMaterialNotAvailableSensor) && (!ITRightMaterialNotAvailableSensor) )
    		{
    			ITReady = FALSE;
    			ITMotorTrayElevator = FALSE;
    		}
    		// Check if there are any foil at all at any of the tray
    		else if( (ITTrayPosLeftSensor) && (ITTrayPosRightSensor) )
    		{
    			ITReady = FALSE;
    			ITMotorTrayElevator = FALSE;
    		}
    		else
    		{
    			// Checkposition of the active tray, Left or Right
				if(!ITTrayPosLeftSensor)
				{
					// Check if the foil is available and on the correct place
					if( (!ITLeftMaterialAvailableSensor) && (!ITTrayElevationMaxSensor) )
					{
						// if material is sensed
						ITReady = TRUE;
						ITMotorTrayElevator = FALSE;
					}
					else
					{
						ITReady = FALSE;
						if(!ITLeftMaterialNotAvailableSensor)	// if foil is empty, Switch Tray
							ITSwitchTrayStep = 1;
						else
						{
							if(ITTrayElevationMaxSensor)// if foil is available but not at the position
							{
								ITMotorTrayElevatorDir = DirUP;
								ITMotorTrayElevator = TRUE;
							}
						}
					}
				}
				else if(!ITTrayPosRightSensor)
				{
					// Check if the foil is available and on the correct place
					if( (!ITRightMaterialAvailableSensor) && (!ITTrayElevationMaxSensor) )
					{
						// if material is sensed
						ITReady = TRUE;
						ITMotorTrayElevator = FALSE;
					}
					else
					{
						ITReady = FALSE;
						if(!ITRightMaterialNotAvailableSensor)	// if foil is empty, Switch Tray
							ITSwitchTrayStep = 1;
						else
						{
							if(ITTrayElevationMaxSensor)// if foil is available but not at the position
							{
								ITMotorTrayElevatorDir = DirUP;
								ITMotorTrayElevator = TRUE;
							}
						}
					}
				}
    		}
    	}
    	else
    	{
    		switch(ITSwitchTrayStep)
    		{
    			case 1:
    				ITMotorTrayElevatorDir = DirDOWN;
    				ITMotorTrayElevator = TRUE;
    				if(!ITTrayElevationMinSensor)
    				{
    					ITSwitchTrayStep = 2;
    				}
    				break;
    			case 2:
    				ITMotorTrayElevator = FALSE;
    				if(!ITTrayPosLeftSensor)
    				{
    					if(!ITTrayElevationMinSensor)
    						ITValveTrayPos = FALSE;
    					ITSwitchTrayStep = 3;
    				}
    				else if(!ITTrayPosRightSensor)
    				{
    					if(!ITTrayElevationMinSensor)
    						ITValveTrayPos = TRUE;
    					ITSwitchTrayStep = 3;
    				}
    				break;
    			case 3:
    				if(!ITValveTrayPos)
    					if(!ITTrayPosRightSensor)
    						ITSwitchTrayStep = 0;
    				if(ITValveTrayPos)
    					if(!ITTrayPosLeftSensor)
    						ITSwitchTrayStep = 0;
    				break;
    		}
    	}
    }
    // Turn Off motor when ITEnable OFF
    if( (!ITEnable) && (!ServiceMode) )
    	ITMotorTrayElevator = FALSE;

	/* Transfer & Reorient Module v */
    if(MachineRunning || TRSbsMode || TRModMode || TRInitting)
    {
		switch(TRModuleStep)
		{
			case 0:  // Standby
				TRExtenderPneuValve = FALSE;
				TRReorientPneuValve = FALSE;
				TRVacuumValve		= FALSE;
				//TRFoilPusherValve	= TRUE;

				if( (MachineRunning) && (!MachineStopping) || TRInitting )
				{
					if( (!TRExtenderPneuRetSensor) && (!TRReorientPneuRetSensor)
							&& (TRExtenderPneuExtSensor) && (TRReorientPneuExtSensor) )		// V1.3
					{
						if(!TRSbsMode)
							TRModuleStep = 1;
						else
							TRSbsMode = FALSE;

						if(TRInittingTOCounter)
						{
							TRInittingTOCounter = 0;
							//TRInitError -= 1;
						}
					}
					else
					{
						if(TRInitting)
						{
							if(!TRInittingTOCounter)
							{
								TRInittingTOCounter = InittingTO;
								//TRInitError += 1;
							}
						}
					}
				}
				break;
			case 1:	// Extend the Extender Pneu , about to get foil
				if(MachineStopping)
					TRModuleStep = 0;
				else
				{
					if( ((!TRModuleWaitingPick) && (ITReady || ITOverride)) || TRInitting)
					{
						TRExtenderPneuValve = TRUE;		// get vacuum to touch foil
						TRReorientPneuValve = FALSE;
						TRVacuumValve		= FALSE;
						//TRFoilPusherValve	= TRUE;
						if( (!TRExtenderPneuExtSensor)
								&& TRExtenderPneuRetSensor)
						{
							if(!TRSbsMode)
								TRModuleStep = 2;
							else
								TRSbsMode = FALSE;

							if(TRInittingTOCounter)
							{
								TRInittingTOCounter = 0;
								//TRInitError -= 2;
							}
						}
						else
						{
							if(TRInitting)
							{
								if(!TRInittingTOCounter)
								{
									TRInittingTOCounter = InittingTO;
									//TRInitError += 2;
								}
							}
						}
					}
				}
				break;
			case 2:	// Vacuum ON and TRFoilPusher retracts, vacuum the foil
				TRExtenderPneuValve = TRUE;
				TRReorientPneuValve = FALSE;
				if(!TRInitting)
					TRVacuumValve		= TRUE;
				//TRFoilPusherValve	= FALSE;
				if(!TRVacuumValveDelayCounter)
				{
					GTFPushPneuValveDelayCounter = GTFPushPneuValveDelay;
					if(!TRInitting)
						TRVacuumValveDelayCounter = TRVacuumValveDelay + 1;

					if(!TRSbsMode)
						TRModuleStep = 3;
					else
						TRSbsMode = FALSE;
				}
				break;
			case 3: // Retract the extender, raise the foil
				if(!TRVacuumValveDelayCounter)
				{
					TRExtenderPneuValve = FALSE;
					TRReorientPneuValve = FALSE;
					//GTFPushPneuValve 	= TRUE;
					if(!TRInitting)
						TRVacuumValve		= TRUE;
					//TRFoilPusherValve	= FALSE;
					if(!TRExtenderPneuRetSensor)
					{
						// Check if vacuumed correctly
						if((!TRVacuumSensor) || IgnoreTRVacuumSensor || TRInitting || MachineStopping)
						{
							TRVacuumSensorErrorCounter = 0;
							if(!TRSbsMode)
								TRModuleStep = 4;
							else
								TRSbsMode = FALSE;
						}
						else	// if vacuum not correctly, repeat from TRModuleStep 0
						{
							TRModuleStep = 0;
							TRVacuumSensorErrorCounter += 1;
						}
						if(TRInittingTOCounter)
						{
							TRInittingTOCounter = 0;
							//TRInitError -= 4;
						}
					}
					else
					{
						if(TRInitting)
						{
							if(!TRInittingTOCounter)
							{
								TRInittingTOCounter = InittingTO;
								//TRInitError += 4;
							}
						}
					}
				}
				break;
			case 4: // Transfer & Reorient, go to in-fill gripper
				//if((!TRModuleWaitingPick) || TRInitting )
				//{
					TRExtenderPneuValve = FALSE;
					TRReorientPneuValve = TRUE;
					TRVacuumValve		= TRUE;
					//GTFPushPneuValve 	= TRUE;
					//TRFoilPusherValve	= FALSE;
					if( (!TRReorientPneuExtSensor)
						&& TRReorientPneuRetSensor)
					{
						if(MachineStopping)
						{
							GTFPushPneuValve = FALSE;
							GTFPushPneuValveDelayCounter = 0;
						}

						if(!TRSbsMode)
							TRModuleStep = 5;
						else
							TRSbsMode = FALSE;

						TRModuleWaiting = TRUE;

						if(TRInittingTOCounter)
						{
							TRInittingTOCounter = 0;
							//TRInitError -= 8;
						}
					}
					else
					{
						if(TRInitting)
						{
							if(!TRInittingTOCounter)
							{
								TRInittingTOCounter = InittingTO;
								//TRInitError += 8;
							}
						}
					}
				//}
				break;
			case 5: //	Waiting for in-fill gripper to take over foil
				if( (!TRModuleWaiting) || MachineStopping || TRInitting)
				{
					TRModuleStep = 6;
					TRModuleWaitingPick = TRUE;
					GTFPushPneuValveDelayCounter = 0;
					GTFPushPneuValve 	= FALSE;
				}
				break;
			case 6:	// Shut down vacuum before moving
				TRExtenderPneuValve = FALSE;
				TRReorientPneuValve = TRUE;
				TRVacuumValve		= FALSE;
				//TRFoilPusherValve	= FALSE;
				if( (!TRVacuumValveDelayCounter) && (!TRInitting) )
					TRVacuumValveDelayCounter = TRVacuumValveDelay + 1;

				if(!TRSbsMode)
					TRModuleStep = 7;
				else
					TRSbsMode = FALSE;

				break;
			case 7: // After delay
				if(!TRVacuumValveDelayCounter)
				{
					TRModuleStep = 0;
					// Step 0
					TRExtenderPneuValve = FALSE;
					TRReorientPneuValve = FALSE;
					TRVacuumValve		= FALSE;
					TRModMode = FALSE;
					TRInitting = FALSE;
				}
				break;
		}
    }

	if(TRVacuumValveDelayCounter)
		TRVacuumValveDelayCounter--;
	if(TRInittingTOCounter)
	{
		TRInittingTOCounter--;
		if(!TRInittingTOCounter)
		{
			// TR Init Error Identifier Subroutine
			switch(TRModuleStep)
			{
				case 0:
					if(TRExtenderPneuRetSensor)
						TRInitError |= 1;
					if(TRReorientPneuRetSensor)
						TRInitError |= 2;

					if(!TRExtenderPneuExtSensor)
						TRInitError |= 4;
					if(!TRReorientPneuExtSensor)
						TRInitError |= 8;
					break;
				case 1:
					if(TRExtenderPneuExtSensor)
						TRInitError |= 4;
					if(!TRExtenderPneuRetSensor)
						TRInitError |= 1;
					break;
				case 4:
					if(TRReorientPneuExtSensor)
						TRInitError |= 8;
					if(!TRReorientPneuRetSensor)
						TRInitError |= 2;
			}
			TRModuleStep++;
		}
	}
	/* Transfer & Reorient Module ^ */

	/* Grip & Transfer Module v */
	if(MachineRunning || GTSbsMode || GTModMode || GTInitting)
	{
		switch(GTModuleStep)
		{
			case 0:  // Standby
				//GTFPushPneuValve = FALSE;
				GTInFillGripPneuValve = TRUE;	// Grip Open
				GTTransferPneuValve = FALSE;

				GTOutFillOpenPneuValve = FALSE;
				GTOutFillGripPneuValve = TRUE;	// Grip Open

				if( (MachineRunning && (!MachineStopping)) || GTInitting )
				{	// no Open Gripper Delay.. maybe need later after trial
					if( (!GTOutFillOpenPneuLRetSensor) && (!GTOutFillOpenPneuRRetSensor) && (!GTTransferPneuRetSensor)
							&& GTOutFillOpenPneuLExtSensor && GTOutFillOpenPneuRExtSensor && GTTransferPneuExtSensor
							&& GTOutFillGripPneuLRetSensor && GTOutFillGripPneuRRetSensor
							&& GTInFillGripPneuLRetSensor && GTInFillGripPneuRRetSensor )
					{
						if(!GTSbsMode)
							GTModuleStep = 1;
						else
							GTSbsMode = FALSE;

						if(GTInittingTOCounter)
						{
							GTInittingTOCounter = 0;
							//GTInitError -= 1;
						}
					}
					else
					{
						if(GTInitting)
						{
							if(!GTInittingTOCounter)
							{
								GTInittingTOCounter = InittingTO;
								//GTInitError += 1;
							}
						}
					}
				}
				break;
			case 1:  // SA1 : Receive the foil from TR (Activate foil pusher), SA2 Initiate Filling Process
				// SA 1 Close grip because the foil is already in position by TR Module
				GTTransferPneuValve = FALSE;
				/*if( TRModuleWaiting && (!GTInitting) )
				{
					GTFPushPneuValve = TRUE;
					GTFPushPneuValveDelayCounter = GTFPushPneuValveDelay;
				}*/
				// SA 2 Close grip because the foil is already in position by F Module
				if(OFModuleWaiting || GTFirstCycle || GTInitting)
				{
					GTOutFillOpenPneuValve = FALSE;
					GTOutFillGripPneuValve = FALSE;	// Grip Closed
					if( (!GTFirstCycle) && (!GTRequestFilling) )					// WARNING NEW
						if(!GTRequestFillingDelayCounter)
							GTRequestFillingDelayCounter = GTRequestFillingDelay;
				}

				if(((!GTOutFillGripPneuLRetSensor) && (!GTOutFillGripPneuRRetSensor) && TRModuleWaiting/*&& (GTFPushPneuValve || GTInitting)*/) || MachineStopping)
				{
					if(MachineStopping)
					{
						GTFPushPneuValve = FALSE;
						GTFPushPneuValveDelayCounter = 0;
					}
					if(!GTSbsMode)
						GTModuleStep = 2;
					else
						GTSbsMode = FALSE;

					if(GTInittingTOCounter)
					{
						GTInittingTOCounter = 0;
						//GTInitError -= 2;
					}
				}
				else
				{
					if(GTInitting)
					{
						if(!GTInittingTOCounter)
						{
							GTInittingTOCounter = InittingTO;
							//GTInitError += 2;
						}
					}
				}
				break;

			case 2:	// SA 1 After PushPneuDelay, Activate Gripper
				//SA 2  Open the bag

				GTTransferPneuValve = FALSE;

				// SA 1 After PushPneuDelay, Activate Gripper
				if(!GTFPushPneuValveDelayCounter)
				GTInFillGripPneuValve = FALSE;	// Grip Closed

				if((!GTInFillGripPneuLRetSensor) && (!GTInFillGripPneuRRetSensor))
				{
					//GTFPushPneuValve = FALSE;
					// Move TR Module Waiting after Transfer Pneu been done
					if(TRModuleWaiting)
					{
						TRVacuumValve	= FALSE;
						TRModuleWaiting = FALSE;
						TRModuleWaitingPick = TRUE;			// TR will wait until TransferPneu move before starting again
					}
				}

				// SA 2 Initiating opening the bag
				GTOutFillOpenPneuValve = TRUE;	// Open the bag
				GTOutFillGripPneuValve = FALSE;	// Grip Still Closed
				if((!GTOutFillOpenPneuLExtSensor) && (!GTOutFillOpenPneuRExtSensor) && TRModuleWaitingPick// && (!GTFPushPneuValve)
						&& GTOutFillOpenPneuLRetSensor && GTOutFillOpenPneuRRetSensor )
				{
					OFModuleWaiting = FALSE;
					if(!GTSbsMode)
						GTModuleStep = 3;
					else
						GTSbsMode = FALSE;

					if(GTInittingTOCounter)
					{
						GTInittingTOCounter = 0;
						//GTInitError -= 4;
					}
				}
				else
				{
					if(GTInitting)
					{
						if(!GTInittingTOCounter)
						{
							GTInittingTOCounter = InittingTO;
							//GTInitError += 4;
						}
					}
				}
				break;
			case 3:  // SA1 : Receive the foil from TR, SA2 Filling Process
				// SA 1
				// Waiting for SA2 to finish

				// SA 2 // Open the bag
				GTOutFillOpenPneuValve = TRUE;	// Open the bag
				GTOutFillGripPneuValve = FALSE;	// Grip Still Closed
				if((!GTOutFillOpenPneuLExtSensor) && (!GTOutFillOpenPneuRExtSensor))
				{
					if(!GTSbsMode)
						GTModuleStep = 4;
					else
						GTSbsMode = FALSE;

					if(GTInittingTOCounter)
					{
						GTInittingTOCounter = 0;
						//GTInitError -= 8;
					}
				}
				else
				{
					if(GTInitting)
					{
						if(!GTInittingTOCounter)
						{
							GTInittingTOCounter = InittingTO;
							//GTInitError += 8;
						}
					}
				}
				break;
			case 4:  // SA1 : Receive the foil from TR, SA2 Filling Process
				// SA 1
				// Waiting for SA2 to finish

				// SA 2 // Request Filling and wait till Filling finished
				// Skip Filling if First Cycle
				//if(!GTFirstCycle)
				//	GTRequestFilling = TRUE;

				if(!GTSbsMode)
					GTModuleStep = 5;
				else
					GTSbsMode = FALSE;
				break;
			case 5:  // SA1 : Receive the foil from TR, SA2 Filling Process
				// SA 1
				// Waiting for SA2 to finish

				// SA2 Close the bag after filling
				GTOutFillGripPneuValve = FALSE;	// Grip Still Closed
				if( (!GTRequestFilling) || GTInitting )
				{
					//GTOutFillOpenPneuValve = FALSE;	// Close the bag after filling.. need delay
					if(!GTCloseBagDelayCounter)
						GTCloseBagDelayCounter = GTCloseBagDelay + 1;
				}
				if((!GTOutFillOpenPneuLRetSensor) && (!GTOutFillOpenPneuRRetSensor))
				{
					if(!GTSbsMode)
						GTModuleStep = 6;
					else
						GTSbsMode = FALSE;

					if(GTInittingTOCounter)
					{
						GTInittingTOCounter = 0;
						//GTInitError -= 16;
					}
				}
				else
				{
					if(GTInitting)
					{
						if(!GTInittingTOCounter)
						{
							GTInittingTOCounter = InittingTO;
							//GTInitError += 16;
						}
					}
				}
				break;
			case 6:	// SA1 & SA2 change position.. TransferPneu Activated
				GTInFillGripPneuValve = FALSE;	// Grip Closed
				GTOutFillOpenPneuValve = FALSE; // Bag Closed
				GTOutFillGripPneuValve = FALSE;	// Grip Closed
				GTTransferPneuValve = TRUE;		// Shift Arm 1 go to filling position
				if( (!GTTransferPneuExtSensor) && GTTransferPneuRetSensor )
				{
					GTSA1ModuleWaiting = TRUE;
					GTSA2ModuleWaiting = TRUE;
					// Moved from GTModule = 2 v
					if(TRModuleWaitingPick)
					{
						TRModuleWaitingPick = FALSE;
					}
					// Moved from GTModule = 2 ^
					if(!GTSbsMode)
						GTModuleStep = 7;
					else
						GTSbsMode = FALSE;

					if(GTInittingTOCounter)
					{
						GTInittingTOCounter = 0;
						//GTInitError -= 32;
					}
				}
				else
				{
					if(GTInitting)
					{
						if(!GTInittingTOCounter)
						{
							GTInittingTOCounter = InittingTO;
							//GTInitError += 32;
						}
					}
				}
				break;
			case 7:
				// SA1 : Wait til FModule grab the foil with vacuum

				GTInFillGripPneuValve = FALSE;	// Grip still  Closed

				// SA2 : Wait till SModule Seal the bag..

				GTOutFillOpenPneuValve = FALSE; // Bag Closed
				GTOutFillGripPneuValve = FALSE;	// Grip still  Closed

				if( ((!GTSA1ModuleWaiting) && (!GTSA2ModuleWaiting)) || MachineStopping || GTInitting )
				{
					if(!GTSbsMode)
						GTModuleStep = 8;
					else
						GTSbsMode = FALSE;
				}
				break;
			case 8:
				// SA1 : Open the gripper
				GTInFillGripPneuValve = TRUE;	// Grip Open

				// SA2 : Open the gripper
				GTOutFillOpenPneuValve = FALSE; // Bag Closed
				GTOutFillGripPneuValve = TRUE;	// Grip Open

				GTBothOpenGripDelayCounter = GTBothOpenGripDelay + 1;

				if(!GTSbsMode)
					GTModuleStep = 9;
				else
					GTSbsMode = FALSE;
				break;
			case 9:
				if((!GTSbsMode) || GTModMode)
					GTFirstCycle = FALSE;
				if(!GTBothOpenGripDelayCounter)
				{
					GTModuleStep = 0;
					// Step 0
					//GTFPushPneuValve = FALSE;
					GTInFillGripPneuValve = TRUE;	// Grip Open
					GTTransferPneuValve = FALSE;

					GTOutFillOpenPneuValve = FALSE;
					GTOutFillGripPneuValve = TRUE;	// Grip Open

					GTModMode = FALSE;
					GTInitting = FALSE;
				}
				break;
		}
	}

	if(GTFPushPneuValveDelayCounter)
	{
		GTFPushPneuValveDelayCounter--;
		if(!GTFPushPneuValveDelayCounter)
			GTFPushPneuValve = TRUE;
	}

	if(GTCloseBagDelayCounter)
	{
		GTCloseBagDelayCounter--;
		if(!GTCloseBagDelayCounter)
			GTOutFillOpenPneuValve = FALSE;  // Close Bag
	}

	if(GTBothOpenGripDelayCounter)
		GTBothOpenGripDelayCounter--;

	if(GTRequestFillingDelayCounter)
	{
		GTRequestFillingDelayCounter--;
		if(!GTRequestFillingDelayCounter)
			GTRequestFilling = TRUE;
	}

	if(GTInittingTOCounter)
	{
		GTInittingTOCounter--;
		if(!GTInittingTOCounter)
		{
			// GT Init Error Identifier Subroutine
			switch(GTModuleStep)
			{
				case 0:
					if(GTOutFillOpenPneuLRetSensor)
						GTInitError |= 1;
					if(GTOutFillOpenPneuRRetSensor)
						GTInitError |= 2;
					if(GTTransferPneuRetSensor)
						GTInitError |= 4;

					if(!GTOutFillOpenPneuLExtSensor)
						GTInitError |= 128;
					if(!GTOutFillOpenPneuRExtSensor)
						GTInitError |= 256;
					if(!GTTransferPneuExtSensor)
						GTInitError |= 512;
					if(!GTOutFillGripPneuLRetSensor)
						GTInitError |= 8;
					if(!GTOutFillGripPneuRRetSensor)
						GTInitError |= 16;
					if(!GTInFillGripPneuLRetSensor)
						GTInitError |= 32;
					if(!GTInFillGripPneuRRetSensor)
						GTInitError |= 64;
					break;
				case 1:
					if(GTOutFillGripPneuLRetSensor)
						GTInitError |= 8;
					if(GTOutFillGripPneuRRetSensor)
						GTInitError |= 16;
					break;
				case 2:
					if(GTInFillGripPneuLRetSensor)
						GTInitError |= 32;
					if(GTInFillGripPneuRRetSensor)
						GTInitError |= 64;
					if(GTOutFillOpenPneuLExtSensor)
						GTInitError |= 128;
					if(GTOutFillOpenPneuRExtSensor)
						GTInitError |= 256;

					if(!GTOutFillOpenPneuLRetSensor)
						GTInitError |= 1;
					if(!GTOutFillOpenPneuRRetSensor)
						GTInitError |= 2;
					break;
				case 6:
					if(GTTransferPneuExtSensor)
						GTInitError |= 512;

					if(!GTTransferPneuRetSensor)
						GTInitError |= 4;
					break;
			}
			GTModuleStep++;
		}
	}

	// Module Timer for Analytical Purpose
	if(MachineRunning)
	{
		GTModuleStepTimerCounter[GTModuleStep]++;
		if(GTModuleStep < 9)
			GTModuleStepTimerCounter[GTModuleStep + 1] = 0;
		else
			GTModuleStepTimerCounter[0] = 0;

		if(GTModuleStep)
			GTModuleStepTimer[GTModuleStep - 1] = GTModuleStepTimerCounter[GTModuleStep - 1];
		else
			GTModuleStepTimer[9] = GTModuleStepTimerCounter[9];
	}
	/* Grip & Transfer Module ^ */

	/* Filling Module v */
	if(MachineRunning || OFSbsMode || OFModMode || OFInitting)
	{
		switch(OFModuleStep)
		{
			case 0: // Standby
				OFGatePneuValve	= FALSE;
				OFEngagePneuValve = FALSE;
				OFOpenPneuValve = FALSE;
				OFVacuumValve = FALSE;
				VFVibEngagePneuValve = FALSE;
				VFVibrator = FALSE;

				if((MachineRunning && (!MachineStopping)) || OFInitting )
				{
					if( (!OFGatePneuRetSensor) && (!OFEngagePneuRetSensor) && (!OFOpenPneuRRetSensor) && (!OFOpenPneuLRetSensor) && (!VFVibEngagePneuRetSensor)
							&& OFGatePneuExtSensor && OFEngagePneuExtSensor && OFOpenPneuRExtSensor && OFOpenPneuLExtSensor && VFVibEngagePneuExtSensor )
					{
						if(!OFSbsMode)
							OFModuleStep = 1;
						else
							OFSbsMode = FALSE;

						if(OFInittingTOCounter)
						{
							OFInittingTOCounter = 0;
							//OFInitError -= 1;
						}
					}
					else
					{
						if(OFInitting)
						{
							if(!OFInittingTOCounter)
							{
								OFInittingTOCounter = InittingTO;
								//OFInitError += 1;
							}
						}
					}
				}
				break;
			case 1: // Receiving from GTModule, Extend the engage and open pneu to locate vacuum into position
				if(GTSA1ModuleWaiting || MachineStopping || OFInitting)
				{
					OFGatePneuValve	= FALSE;
					OFEngagePneuValve = TRUE;
					OFOpenPneuValve = TRUE;
					OFVacuumValve = FALSE;
					VFVibEngagePneuValve = FALSE;
					VFVibrator = FALSE;
					if((!OFEngagePneuExtSensor) && (!OFOpenPneuRExtSensor) && (!OFOpenPneuLExtSensor)
							&& OFEngagePneuRetSensor && OFOpenPneuRRetSensor && OFOpenPneuLRetSensor )
					{
						if(!OFSbsMode)
							OFModuleStep = 2;
						else
							OFSbsMode = FALSE;

						if(OFInittingTOCounter)
						{
							OFInittingTOCounter = 0;
							//OFInitError -= 2;
						}
					}
					else
					{
						if(OFInitting)
						{
							if(!OFInittingTOCounter)
							{
								OFInittingTOCounter = InittingTO;
								//OFInitError += 2;
							}
						}
					}
				}
				break;
			case 2:	// Activate Vacuum
				OFGatePneuValve	= FALSE;
				OFEngagePneuValve = TRUE;
				OFOpenPneuValve = TRUE;
				OFVacuumValve = TRUE;
				VFVibEngagePneuValve = FALSE;
				VFVibrator = FALSE;
				if(!OFVacuumValveDelayCounter)
				{
					OFVacuumValveDelayCounter = OFVacuumValveDelay + 1;
					if(!OFSbsMode)
						OFModuleStep = 3;
					else
						OFSbsMode = FALSE;
				}
				break;
			case 3:	// Vacuum is sure in effect now
				if(!OFVacuumValveDelayCounter)
				{
					GTSA1ModuleWaiting = FALSE;
					OFModuleWaiting = TRUE;

					if(!OFSbsMode)
						OFModuleStep = 4;
					else
					OFSbsMode = FALSE;
					// GTSA2 GRIP HERE
				}
				break;
			case 4:	// Open the bag (Step 1)& Ready the vibrator
				if((!OFModuleWaiting) || MachineStopping || OFInitting)
				{
					OFGatePneuValve	= FALSE;
					OFEngagePneuValve = TRUE;
					OFOpenPneuValve = FALSE;		// Open the bag
					OFVacuumValve = TRUE;
					if(!OFInitting)
						VFVibEngagePneuValve = FALSE;	// Delayed : Engage the Vibrator
					else
						VFVibEngagePneuValve = TRUE;	// Used for Initting purpose
					VFVibrator = FALSE;
					if( (!OFOpenPneuRRetSensor) && (!OFOpenPneuLRetSensor))
					{
						if(!OFSbsMode)
							OFModuleStep = 5;
						else
							OFSbsMode = FALSE;

						if(OFInittingTOCounter)
						{
							OFInittingTOCounter = 0;
							//OFInitError -= 4;
						}
					}
					else
					{
						if(OFInitting)
						{
							if(!OFInittingTOCounter)
							{
								OFInittingTOCounter = InittingTO;
								//OFInitError += 4;
							}
						}
					}
				}
				break;
			case 5:	// Open the bag (Step 2)& Ready the vibrator
				OFGatePneuValve	= FALSE;
				OFEngagePneuValve = FALSE;		// Open the bag further
				OFOpenPneuValve = FALSE;
				if(!OFInitting)
					VFVibEngagePneuValve = FALSE;	// Delayed : Engage the Vibrator
				else
					VFVibEngagePneuValve = TRUE;	// Used for Initting purpose
				VFVibrator = FALSE;
				// If not Initting, just OFEngagePneuRetSensor, if Initting, check both OFEngage & VfVib
				if( (!OFEngagePneuRetSensor) && ((!OFInitting) || ((!VFVibEngagePneuExtSensor) && VFVibEngagePneuRetSensor)) )
				{
					OFVacuumValve = FALSE;
					if(!OFSbsMode)
						OFModuleStep = 6;
					else
						OFSbsMode = FALSE;

					if(OFInittingTOCounter)
					{
						OFInittingTOCounter = 0;
						//OFInitError -= 8;
					}
				}
				else
				{
					if(OFInitting)
					{
						if(!OFInittingTOCounter)
						{
							OFInittingTOCounter = InittingTO;
							//OFInitError += 8;
						}
					}
				}
				break;
			case 6:	// Open the BRIDGE to initiate FILLING PROCESS
				if(GTRequestFilling || MachineStopping || OFInitting)
				{
					OFGatePneuValve	= TRUE;			// Thrust the Gate Tongue (V1.4 Moved into GT0)
					OFEngagePneuValve = FALSE;
					OFOpenPneuValve = FALSE;
					OFVacuumValve = FALSE;
					if(!OFInitting)
						VFVibEngagePneuValve = FALSE;	// Delayed : Engage the Vibrator
					else
						VFVibEngagePneuValve = TRUE;	// Used for Initting purpose
					VFVibrator = FALSE;

					if( (!OFRequestFilling) && (!OFRequestFillingDelayCounter) )
					{
						// if weight is already OK before checking, then OPEN faster
						if(WLCLWeightOK || WLCRWeightOK)
						{
							OFRequestFillingDelayCounter = OFRequestFillingDelay;
							VFVibrator = TRUE;				// Start Vibrating
							VFVibratorEngageDelayCounter = VFVibratorEngageDelay + 1;
							OFFastRequestFilling = TRUE;
						}
					}
					if( (!OFGatePneuExtSensor) && OFGatePneuRetSensor )
					{
						if(OFGatePneuValveVibratorFrequency)	// Set Frequency = 0 for make it inactive
							OFGatePneuValveVibDelayCounter = OFGatePneuValveVibDelay + 1;

						if(!OFSbsMode)
							OFModuleStep = 7;
						else
							OFSbsMode = FALSE;

						if(OFInittingTOCounter)
						{
							OFInittingTOCounter = 0;
							//OFInitError -= 16;
						}
					}
					else
					{
						if(OFInitting)
						{
							if(!OFInittingTOCounter)
							{
								OFInittingTOCounter = InittingTO;
								//OFInitError += 16;
							}
						}
					}
				}
				break;
			case 7:	// Start FILLING when weigher module READY
				if(MachineStopping || OFInitting)
				{
					OFModuleStep = 8;
					OFRequestFilling = FALSE;
				}
				// if the weigher was ready sooner, then OFRequestSooner
				// BUT if the weight was not OK, then wait till weight OK
				if(OFFastRequestFilling)
				{
					OFFastRequestFilling = FALSE;
					if(!OFSbsMode)
						OFModuleStep = 8;
					else
						OFSbsMode = FALSE;

				}
				else
				{
					if(WLCLWeightOK || WLCRWeightOK)
					{
						OFRequestFilling = TRUE;
						VFVibrator = TRUE;				// Start Vibrating
						VFVibratorEngageDelayCounter = VFVibratorEngageDelay + 1;
						VFVibEngagePneuValveVibratorMode = TRUE;	// Start SUPER Vibrating
						if(!OFSbsMode)
							OFModuleStep = 8;
						else
							OFSbsMode = FALSE;
					}
				}
				//if(WLCLWeightOK || WLCRWeightOK)
				//{
				//	OFRequestFilling = TRUE;
				//	VFVibrator = TRUE;				// Start Vibrating
				//	VFVibratorEngageDelayCounter = VFVibratorEngageDelay + 1;
					//VFVibEngagePneuValveVibratorMode = TRUE;	// Start SUPER Vibrating
					//if(!OFSbsMode)
					//	OFModuleStep = 8;
					//else
					//	OFSbsMode = FALSE;
				//}
				break;
			case 8:	// Close the gate after finished & Stop Vibrating, tell GT to continue
				if( (!OFRequestFilling) && (!OFGatePneuValveDurationCounter) && (!WLOpGatePneuDurationCounter) && (!WROpGatePneuDurationCounter) )
				{
					GTRequestFilling = FALSE;			// Finished Filling, tell GT to continue

					OFGatePneuValveDelayCounter = 0;			// Cancel the order to vibrate if delay too long
					OFGatePneuValveVibDelayCounter = 0;		// Cancel the order to vibrate if delay too long
					OFGatePneuValveVibratorMode = FALSE;	// Stop SUPER Vibrating
					OFGatePneuValve	= FALSE;			// Close the gate
					OFEngagePneuValve = FALSE;
					OFOpenPneuValve = FALSE;
					OFVacuumValve = FALSE;
					// OFF Vibrate Mode Moved to case 9
					//VFVibrator = FALSE;					// Stop Vibrating
					if( (!OFGatePneuRetSensor) )// && (!VFVibEngagePneuRetSensor) )
					{
						//OFGatePneuFilled = FALSE;
						if(!OFSbsMode)
							OFModuleStep = 9;
						else
							OFSbsMode = FALSE;

						if(OFInittingTOCounter)
						{
							OFInittingTOCounter = 0;
							//OFInitError -= 32;
						}
					}
					else
					{
						if(OFInitting)
						{
							if(!OFInittingTOCounter)
							{
								OFInittingTOCounter = InittingTO;
								//OFInitError += 32;
							}
						}
					}
				}
				break;
			case 9:	// back to module step 0
				OFGatePneuValve	= FALSE;
				OFEngagePneuValve = FALSE;
				OFOpenPneuValve = FALSE;
				OFVacuumValve = FALSE;
				VFVibratorEngageDelayCounter = 0;			// Cancel the order to vibrate if delay too long
				VFVibratorEngageVibDelayCounter = 0;		// Cancel the order to vibrate if delay too long
				VFVibEngagePneuValveVibratorMode = FALSE;	// Stop SUPER Vibrating
				VFVibEngagePneuValve = FALSE;
				VFVibrator = FALSE;
				OFModMode = FALSE;
				if(!VFVibEngagePneuRetSensor)
				{
					OFInitting = FALSE;
					//OFGatePneuFilled = FALSE;
					if(!OFSbsMode)
						OFModuleStep = 0;
					else
						OFSbsMode = FALSE;

					if(OFInittingTOCounter)
					{
						OFInittingTOCounter = 0;
						//OFInitError -= 32;
					}
				}
				else
				{
					if(OFInitting)
					{
						if(!OFInittingTOCounter)
						{
							OFInittingTOCounter = InittingTO;
							//OFInitError += 32;
						}
					}
				}
				/*
				if(!OFSbsMode)
					OFModuleStep = 0;
				else
					OFSbsMode = FALSE;*/
				break;
		}
	}

	if(OFVacuumValveDelayCounter)
		OFVacuumValveDelayCounter--;

	if(OFRequestFillingDelayCounter)
	{
		OFRequestFillingDelayCounter--;
		if(!OFRequestFillingDelayCounter)
			OFRequestFilling = TRUE;
	}

	if(OFInittingTOCounter)
	{
		OFInittingTOCounter--;
		if(!OFInittingTOCounter)
		{
			//OF Init Error Identifier Subroutine
			switch(OFModuleStep)
			{
				case 0:
					if(OFGatePneuRetSensor)
						OFInitError |= 1;
					if(OFEngagePneuRetSensor)
						OFInitError |= 2;
					if(OFOpenPneuRRetSensor)
						OFInitError |= 4;
					if(OFOpenPneuLRetSensor)
						OFInitError |= 8;
					if(VFVibEngagePneuRetSensor)
						OFInitError |= 16;

					if(!OFGatePneuExtSensor)
						OFInitError |= 512;
					if(!OFEngagePneuExtSensor)
						OFInitError |= 32;
					if(!OFOpenPneuRExtSensor)
						OFInitError |= 64;
					if(!OFOpenPneuLExtSensor)
						OFInitError |= 128;
					if(!VFVibEngagePneuExtSensor)
						OFInitError |= 256;
					break;
				case 1:
					if(OFEngagePneuExtSensor)
						OFInitError |= 32;
					if(OFOpenPneuRExtSensor)
						OFInitError |= 64;
					if(OFOpenPneuLExtSensor)
						OFInitError |= 128;

					if(!OFEngagePneuRetSensor)
						OFInitError |= 2;
					if(!OFOpenPneuRRetSensor)
						OFInitError |= 4;
					if(!OFOpenPneuLRetSensor)
						OFInitError |= 8;
					break;
				case 5:
					if(VFVibEngagePneuExtSensor)
						OFInitError |= 256;

					if(!VFVibEngagePneuRetSensor)
						OFInitError |= 16;
					break;
				case 6:
					if(OFGatePneuExtSensor)
						OFInitError |= 512;

					if(!OFGatePneuRetSensor)
						OFInitError |= 1;
					break;
			}
			if(OFModuleStep >= 9)
				OFModuleStep++;
			else
			{
				OFModuleStep = 0;
				OFInitting = FALSE;
			}
		}
	}
	// OF Gate Valve
	if(OFGatePneuValveDelayCounter)
	{
		OFGatePneuValveDelayCounter--;
		if(!OFGatePneuValveDelayCounter)
		{
			OFGatePneuValve = TRUE;	// Engaging
			OFGatePneuValveVibDelayCounter = OFGatePneuValveVibDelay + 1;
		}
	}
	if(OFGatePneuValveVibDelayCounter)
	{
		OFGatePneuValveVibDelayCounter--;
		if(!OFGatePneuValveVibDelayCounter)
			OFGatePneuValveVibratorMode = TRUE;	// Start SUPER Vibrating
	}
	if(OFGatePneuValveVibratorMode)
	{
		if(OFGatePneuValveVibratorPeriodCounter)
		{
			OFGatePneuValveVibratorPeriodCounter--;
			if(!OFGatePneuValveVibratorPeriodCounter)
			{
				if(OFGatePneuValve)
				{
					OFGatePneuValve = FALSE;
					OFGatePneuValveVibratorPeriodCounter = (OFGatePneuValveVibratorPeriod * (100 - OFGatePneuValveVibratorDutyCycle)) / 100;
				}
				else
				{
					OFGatePneuValve = TRUE;
					OFGatePneuValveVibratorPeriodCounter = (OFGatePneuValveVibratorPeriod * OFGatePneuValveVibratorDutyCycle) / 100;
				}
			}
		}
		else
			OFGatePneuValveVibratorPeriodCounter = (OFGatePneuValveVibratorPeriod * (100 - OFGatePneuValveVibratorDutyCycle)) / 100; // Low Duty Cycle = 100% - DutyCycle
	}
	else
		OFGatePneuValveVibratorPeriodCounter = (OFGatePneuValveVibratorPeriod * (100 - OFGatePneuValveVibratorDutyCycle)) / 100; // Low Duty Cycle = 100% - DutyCycle
	// Vibrator Engage
	if(VFVibratorEngageDelayCounter)
	{
		VFVibratorEngageDelayCounter--;
		if(!VFVibratorEngageDelayCounter)
		{
			VFVibEngagePneuValve = TRUE;	// Engaging
			VFVibratorEngageVibDelayCounter = VFVibratorEngageVibDelay + 1;
		}
	}
	if(VFVibratorEngageVibDelayCounter)
	{
		VFVibratorEngageVibDelayCounter--;
		if(!VFVibratorEngageVibDelayCounter)
			VFVibEngagePneuValveVibratorMode = TRUE;	// Start SUPER Vibrating
	}
	if(VFVibEngagePneuValveVibratorMode)
	{
		if(VFVibEngageVibratorPeriodCounter)
		{
			VFVibEngageVibratorPeriodCounter--;
			if(!VFVibEngageVibratorPeriodCounter)
			{
				if(VFVibEngagePneuValve)
				{
					VFVibEngagePneuValve = FALSE;
					VFVibEngageVibratorPeriodCounter = (VFVibEngageVibratorPeriod * (100 - VFVibEngageVibratorDutyCycle)) / 100;
				}
				else
				{
					VFVibEngagePneuValve = TRUE;
					VFVibEngageVibratorPeriodCounter = (VFVibEngageVibratorPeriod * VFVibEngageVibratorDutyCycle) / 100;
				}
			}
		}
		else
			VFVibEngageVibratorPeriodCounter = (VFVibEngageVibratorPeriod * (100 - VFVibEngageVibratorDutyCycle)) / 100; // Low Duty Cycle = 100% - DutyCycle
	}
	else
		VFVibEngageVibratorPeriodCounter = (VFVibEngageVibratorPeriod * (100 - VFVibEngageVibratorDutyCycle)) / 100; // Low Duty Cycle = 100% - DutyCycle
	/* Filling Module ^ */

	/* Sealing Module v */
	if(MachineRunning || SPSbsMode || SPModMode || SPInitting)
	{
		switch(SPModuleStep)
		{
			case 0: // Standby
				SPSealerPneuValve		= FALSE; 	// Sealer Opened
				SPVibEngagePneuValve	= FALSE;	// Vibrator Disengaged
				SPPusherValve			= FALSE;		// Pusher Retended
				SPVibrator				= FALSE;	// Vibrator OFF

				if( (MachineRunning && (!MachineStopping)) || SPInitting )
				{
					if( (!SPVibEngagePneuRetSensor) && (!SPSealerPneuRetSensor) && (!SPPusherPneuRetSensor)
							&& SPVibEngagePneuExtSensor && SPSealerPneuExtSensor && SPPusherPneuExtSensor )
					{
						if(!SPSbsMode)
							SPModuleStep = 1;
						else
							SPSbsMode = FALSE;

						if(SPInittingTOCounter)
						{
							SPInittingTOCounter = 0;
							//SPInitError -= 1;
						}
					}
					else
					{
						if(SPInitting)
						{
							if(!SPInittingTOCounter)
							{
								SPInittingTOCounter = InittingTO;
								//SPInitError += 1;
							}
						}
					}
				}
				break;
			case 1: // Bag from GT is in Place
				if(GTSA2ModuleWaiting || MachineStopping || SPInitting)
				{
					SPSealerPneuValveDelayCounter = SPSealerPneuValveDelay + 1;	// Sealer controlled by timing
					//SPSealerPneuValve		= TRUE; 	// Sealer Closed
					SPVibEngagePneuValve	= FALSE;
					SPPusherValve			= FALSE;	// Pusher Retracted so VibEngage can be ext
					SPVibrator				= FALSE;
					if(!SPPusherPneuRetSensor)
					{
						GTSA2ModuleWaiting 		= FALSE;			// Tell GT Module SA 2 to continue
						if(!SPSbsMode)
							SPModuleStep = 2;
						else
							SPSbsMode = FALSE;

						if(SPInittingTOCounter)
						{
							SPInittingTOCounter = 0;
							//SPInitError -= 2;
						}
					}
					else
					{
						if(SPInitting)
						{
							if(!SPInittingTOCounter)
							{
								SPInittingTOCounter = InittingTO;
								//SPInitError += 2;
							}
						}
					}
				}
				break;
			case 2:	// Engage the vibrator
				//SPSealerPneuValve		= TRUE; 	// Sealer still Closed
				SPVibEngagePneuValve	= TRUE;		// Start engaging
				SPPusherValve			= FALSE;
				SPVibrator				= FALSE;
				if( (!SPVibEngagePneuExtSensor) && SPVibEngagePneuRetSensor )
				{
					if(!SPSbsMode)
						SPModuleStep = 3;
					else
						SPSbsMode = FALSE;

					if(SPInittingTOCounter)
					{
						SPInittingTOCounter = 0;
						//SPInitError -= 4;
					}
				}
				else
				{
					if(SPInitting)
					{
						if(!SPInittingTOCounter)
						{
							SPInittingTOCounter = InittingTO;
							//SPInitError += 4;
						}
					}
				}
				break;
			case 3: // Vibrate
				//SPSealerPneuValve		= TRUE; 	// Sealer still Closed
				SPVibEngagePneuValve	= TRUE;
				SPPusherValve			= FALSE;
				SPVibrator				= TRUE;		// Vibrator ON
				if(!SPSbsMode)
					SPModuleStep = 4;
				else
					SPSbsMode = FALSE;
				break;
			case 4: // Open Sealer & Turn Off Vibrator
				//SPSealerPneuValve		= FALSE; 	// Start Opening
				SPVibEngagePneuValve	= TRUE;
				SPPusherValve			= FALSE;
				SPVibrator				= FALSE;	// Vibrator OFF
				if(!SPSealerPneuRetSensor)
				{
					if(!SPSbsMode)
						SPModuleStep = 5;
					else
						SPSbsMode = FALSE;

					if(SPInittingTOCounter)
					{
						SPInittingTOCounter = 0;
						//SPInitError -= 8;
					}
				}
				else
				{
					if(SPInitting)
					{
						if(!SPInittingTOCounter)
						{
							SPInittingTOCounter = InittingTO;
							//SPInitError += 8;
						}
					}
				}
				break;
			case 5: // Bring the VibEngagePneu down with the bag
				//SPSealerPneuValve		= FALSE; 	// Sealer Opened
				SPVibEngagePneuValve	= FALSE;	// Start disengaging
				SPPusherValve			= FALSE;
				SPVibrator				= FALSE;	// Vibrator OFF
				if(!SPVibEngagePneuRetSensor)
				{
					if(!SPSbsMode)
						SPModuleStep = 6;
					else
						SPSbsMode = FALSE;

					if(SPInittingTOCounter)
					{
						SPInittingTOCounter = 0;
						//SPInitError -= 16;
					}
				}
				else
				{
					if(SPInitting)
					{
						if(!SPInittingTOCounter)
						{
							SPInittingTOCounter = InittingTO;
							//SPInitError += 16;
						}
					}
				}
				break;
			case 6: // Push the bag out to conveyor
				//SPSealerPneuValve		= FALSE;
				SPVibEngagePneuValve	= FALSE;	// Vib is disengaged
				SPPusherValve			= TRUE;		// Pusher start pushing
				SPVibrator				= FALSE;	// Vibrator OFF
				if( (!SPPusherPneuExtSensor) && SPPusherPneuRetSensor )
				{
					if(!SPSbsMode)
						SPModuleStep = 7;
					else
						SPSbsMode = FALSE;

					if(SPInittingTOCounter)
					{
						SPInittingTOCounter = 0;
						//SPInitError -= 32;
					}
				}
				else
				{
					if(SPInitting)
					{
						if(!SPInittingTOCounter)
						{
							SPInittingTOCounter = InittingTO;
							//SPInitError += 32;
						}
					}
				}
				break;
			case 7:  // back to standby position
				SPModuleStep = 0;
				// Step 0
				SPVibEngagePneuValve	= FALSE;	// Vibrator Disengaged
				SPPusherValve			= FALSE;		// Pusher Extended
				SPVibrator				= FALSE;	// Vibrator OFF
				SPModMode = FALSE;
				SPInitting = FALSE;
				break;
		}
	}

	if(SPSealerPneuValveDelayCounter)
	{
		SPSealerPneuValveDelayCounter--;
		if(!SPSealerPneuValveDelayCounter)
		{
			SPSealerPneuValveDurationCounter = SPSealerPneuValveDuration;
		}
	}
	if(SPSealerPneuValveDurationCounter)
	{
		SPSealerPneuValveDurationCounter--;
		SPSealerPneuValve = TRUE;
		if(!SPSealerPneuValveDurationCounter)
		{

		}
	}
	else
		SPSealerPneuValve = FALSE;

	if(SPInittingTOCounter)
	{
		SPInittingTOCounter--;
		if(!SPInittingTOCounter)
		{
			//OF Init Error Identifier Subroutine
			switch(SPModuleStep)
			{
				case 0:
					if(SPVibEngagePneuRetSensor)
						SPInitError |= 1;
					if(SPSealerPneuRetSensor)
						SPInitError |= 2;
					if(SPPusherPneuRetSensor)
						SPInitError |= 8;

					if(!SPVibEngagePneuExtSensor)
						SPInitError |= 16;
					if(!SPSealerPneuExtSensor)
						SPInitError |= 32;
					if(!SPPusherPneuExtSensor)
						SPInitError |= 4;
					break;
				case 2:
					if(SPVibEngagePneuExtSensor)
						SPInitError |= 16;

					if(!SPVibEngagePneuRetSensor)
						SPInitError |= 1;
					break;
				case 6:
					if(SPPusherPneuExtSensor)
						SPInitError |= 4;

					if(!SPPusherPneuRetSensor)
						SPInitError |= 8;
					break;
			}
			SPModuleStep++;
		}
	}
	/* Sealing Module ^ */

	/* Weigher Module v */
    // Loadcell Calibration v
    if(WLCLCalibrationMode == 1)
    {
        WLCLSDAReadingZero = WLCLSDAReadingAverage;
        WLCLCalibrationMode = 0;
    }
    else if(WLCLCalibrationMode == 2)
    {
    	WLCLSDAReadingCustomKg = WLCLSDAReadingAverage;
    	WLCLSDARatioCustomKgCO = ((WLCLSDAReadingCustomKg - WLCLSDAReadingZero) * 250) / WCustomKg;
    	WLCLSDAReadingOffsetCO = ((WLCLSDAReadingZero * 250) / WLCLSDARatioCustomKgCO);
    	WLCLCalibrationMode = 0;
    	WLCLCalibrationFlagCounter = 20;
    }

    if(WLCRCalibrationMode == 1)
    {
        WLCRSDAReadingZero = WLCRSDAReadingAverage;
        WLCRCalibrationMode = 0;
    }
    else if(WLCRCalibrationMode == 2)
    {
    	WLCRSDAReadingCustomKg = WLCRSDAReadingAverage;
    	WLCRSDARatioCustomKgCO = ((WLCRSDAReadingCustomKg - WLCRSDAReadingZero) * 250) / WCustomKg;
    	WLCRSDAReadingOffsetCO = ((WLCRSDAReadingZero * 250) / WLCRSDARatioCustomKgCO);
    	WLCRCalibrationMode = 0;
    	WLCRCalibrationFlagCounter = 20;
    }
    // Loadcell Calibration ^

    // Loadcell Zeroing v
    if(WLCLOnZeroingBtn)
    {
    	WLCLSDARatioCustomKgCO = WLCLSDARatioCustomKg;
    	WLCLSDAReadingOffsetCO = ((WLCLSDAReadingAverage * 250) / WLCLSDARatioCustomKg);
    	WLCLOnZeroingBtn = FALSE;
    	WLCLCalibrationFlagCounter = 20;
    }

    if(WLCROnZeroingBtn)
    {
    	WLCRSDARatioCustomKgCO = WLCRSDARatioCustomKg;
    	WLCRSDAReadingOffsetCO = ((WLCRSDAReadingAverage * 250) / WLCRSDARatioCustomKg);
    	WLCROnZeroingBtn = FALSE;
    	WLCRCalibrationFlagCounter = 20;
    }
    // Loadcell Zeroing ^

    // Filtering Weight OK for 200ms v
    if(WLCLWeightOKFlag)
    {
        if(WLCLWeightOKCounter)
        {
        	WLCLWeightOKCounter--;
            if(!WLCLWeightOKCounter)
            {
            	WLCLWeightOK = TRUE;
            }
        }
    }
    else
    	WLCLWeightOKCounter = 200;

    if(WLCRWeightOKFlag)
      {
          if(WLCRWeightOKCounter)
          {
        	  WLCRWeightOKCounter--;
              if(!WLCRWeightOKCounter)
              {
            	  WLCRWeightOK = TRUE;
              }
          }
      }
      else
    	  WLCRWeightOKCounter = 200;
    // Filtering Weight OK for 200ms ^

    if(WLOpGatePneuDurationCounter)
    {
    	WLOpGatePneuDurationCounter--;
		WLOpGatePneuValve = TRUE;
    	if(!WLOpGatePneuDurationCounter)
    	{
    		WNextOpGateDelayCounter = WNextOpGateDelay + 1;
    		WLCLWeightOKFlag = FALSE;
    		WLCLWeightOK = FALSE;
    		if(WLCLZeroMONFlag)
    			WLCLZeroDelayCounter = 1000;

    		//OFGatePneuFilled = TRUE;
    		//OFGatePneuFillingProcess = FALSE;
    	}
    }
    else
    	WLOpGatePneuValve = FALSE;

    if(WLCLZeroDelayCounter)
    {
    	WLCLZeroDelayCounter--;
    	if(!WLCLZeroDelayCounter)
    	{
			WLCLOnZeroingBtn = TRUE;
			WLCLZeroMONFlag = FALSE;
    	}
    }

    if(WROpGatePneuDurationCounter)
    {
    	WROpGatePneuDurationCounter--;
		WROpGatePneuValve = TRUE;
    	if(!WROpGatePneuDurationCounter)
    	{
    		WNextOpGateDelayCounter = WNextOpGateDelay + 1;
    		WLCRWeightOKFlag = FALSE;
    		WLCRWeightOK = FALSE;
			if(WLCRZeroMONFlag)
				WLCRZeroDelayCounter = 1000;
			//OFGatePneuFilled = TRUE;
    		//OFGatePneuFillingProcess = FALSE;
    	}
    }
    else
    	WROpGatePneuValve = FALSE;

    if(WLCRZeroDelayCounter)
    {
    	WLCRZeroDelayCounter--;
    	if(!WLCRZeroDelayCounter)
    	{
			WLCROnZeroingBtn = TRUE;
			WLCRZeroMONFlag = FALSE;
    	}
    }

    if(WNextOpGateDelayCounter)
    	WNextOpGateDelayCounter--;
	/* Weigher Module ^ */

	if(ProductCountBitCounter)
	{
		ProductCountBit = TRUE;
		ProductCountBitCounter--;
	}
	else
		ProductCountBit = FALSE;


	/* Input Page : PROX 2 (Must use SVE-MBTIu)
	if(DatesensEN)
	{
		if(ROM_GPIOPinRead(PIN_DATECODE_SENSOR_BASE, PIN_DATECODE_SENSOR_PIN))            // PINDATESENS if the sensor doesnt pick the film
		{
			if(DatesensCounter)
				DatesensCounter--;
			else
				DatesensError = TRUE;
		}
		else
		{
			DatesensCounter = 500;
			DatesensError = FALSE;
		}
	}
	else
	{
		DatesensCounter = 500;
		DatesensError = FALSE;
	}
	 Input Page ^ */

	/* Communication Module v */
	if(ReadACK)                                                         // ReadACK time out 1500 ms
	{                                                                   // if doesnt transfer during 1500 ms then terminate transfer
		if(ReadACKCounter)
			ReadACKCounter--;
		else
		{
			DataLock = FALSE;
			ReadACK = FALSE;
			dataTXIndex = 0;
		}
	}
	else
		ReadACKCounter = 1000;

	if(!ReadACK)     				// if reading mode
	{
		UARTTimeOut--;
		if(!UARTTimeOut)
		{
		    // Disable the Uart to be re-enabled
		    UARTDisable(UART0_BASE);
		    UARTDisabled = TRUE;
		    UARTTimeOut = UARTDISABLEDCOUNT;
		}
	}
	else
		UARTTimeOut = UARTDISABLEDCOUNT;

    if(!SPIStartTimer)
    {
		if(SPIDataReady && (!SPITransmitted))
		{
			//PORTSS = FALSE;
			SSIDataPutNonBlocking(SSI0_BASE, SPIOut[SPICounter]);//SPDR = SPIOut[SPICounter];
			SPITransmitted = TRUE;
		}
    }
    else
        SPIStartTimer--;
	/* Communication Module ^ */

    /* External Heater Module v */
	if(ROM_GPIOPinRead(PIN_HEATER_ERROR_BASE, PIN_HEATER_ERROR_PIN))                                                  // If heater not ready pin is ON
	{
		HeaterErrorCounter = 3000;
		HeaterError = FALSE;
	}
	else
	{
		if(HeaterErrorCounter)
			HeaterErrorCounter--;
		else
		{
			HeaterErrorMsgDuration = 80;
			HeaterError = TRUE;
		}
	}
	/* External Heater Module ^ */

	// 10 ms timer Zone
	if(!ms10counter)
	{
		/* Air Check Subroutine v */
        if(!IgnoreAirError)
        {
        	if(ROM_GPIOPinRead(PIN_AIR_ERROR_BASE, PIN_AIR_ERROR_PIN))
        	{
        		if(NoAirErrorCounter)
        			NoAirErrorCounter--;
        		else
                {
                	NoAirError = TRUE;
                	NoAirErrorMsgDuration = 80;
                }
        	}
            else
            {
            	NoAirError = FALSE;
                NoAirErrorCounter = 20;
            }
        }
        else
        {
        	NoAirErrorCounter = 20;
        	NoAirError = FALSE;
        }
        /* Air Check Subroutine ^ */

        /* Weigher Calibration Subroutine v */
        if(WLCLCalibrationFlagCounter)
        {
        	WLCLCalibrationFlagCounter--;
        	WLCLCalibrationFlag = TRUE;
        }
        else
        	WLCLCalibrationFlag = FALSE;

        if(WLCRCalibrationFlagCounter)
        {
        	WLCRCalibrationFlagCounter--;
        	WLCRCalibrationFlag = TRUE;
        }
        else
        	WLCRCalibrationFlag = FALSE;
        /* Weigher Calibration Subroutine ^ */

		ms10counter = 9;
	}
	else
		ms10counter--;

	// 10 ms timer Zone
	if(!ms100counter)
	{
		// To detect if it resets (Counting how long since turned ON)
		LifeDuration++;

		if(UARTDisabled)
		{
		    // Enable the UART.
		    UARTEnable(UART0_BASE);

		    // Unfortunately, in StellarisWare, UARTEnable() silently re-enables the FIFO,
		    // so to get rid of that, move UARTFIFODisable() call after UARTEnable()
		    UARTFIFODisable(UART0_BASE);
			dataTXIndex = ComAddress * 20;
			dataRXIndex = 0;
			UARTDisabled = FALSE;
			UARTDisabledCounter++;
			ReadACK = FALSE;
		}

		/* Error Message for HMI Subroutine v */
		if(HeaterErrorMsgDuration)
		{
			HeaterErrorMsgDuration--;
			HeaterErrorMsg = TRUE;
		}
		else
			HeaterErrorMsg = FALSE;

		if(FoilJamMsgDuration)
		{
			FoilJamMsgDuration--;
			FoilJamMsg = TRUE;
		}
		else
			FoilJamMsg = FALSE;

		if(NoAirErrorMsgDuration)
		{
			NoAirErrorMsgDuration--;
			NoAirErrorMsg = TRUE;
		}
		else
			NoAirErrorMsg = FALSE;
		/* Error Message for HMI Subroutine ^ */

		ms100counter = 99;

	}
	else
		ms100counter--;
}

//*****************************************************************************
// Interrupt handler for the Timer0 interrupt
//*****************************************************************************
void Timer1IntHandler(void)
{
    //unsigned long ulPinStaRaya Kupang Baru No.62, Suko Manunggal, Kota Surabaya, Jawa Timur 60189tus;

    // Clear the timer interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    if(ReadACK)
    {
		if(DataTimerTrigger)
		{
			if(DataReady)
			{
				DataLock = TRUE;
				/*while( dataTXIndex <= ((ComAddress * 20) + (BytestoRead) + 5 - 1) )
				{
					ROM_UARTCharPut(UART0_BASE, ComOUT[dataTXIndex]);
					dataTXIndex++;
				}
				DataTimerCounter++;
				DataLock = FALSE;
				ReadACK = FALSE;
				dataTXIndex = ComAddress * 20;*/
				if(ROM_UARTSpaceAvail(UART0_BASE))
				{
					ROM_UARTCharPutNonBlocking(UART0_BASE, ComOUT[dataTXIndex]);
					if( dataTXIndex == ((ComAddress * 20) + (BytestoRead) + 5 - 1) )
					{
						DataLock = FALSE;
						ReadACK = FALSE;
						dataTXIndex = 0;
					}
					else
						dataTXIndex++;

					DataTimerTrigger = FALSE;
				}
				else
					DataTimerTrigger = TRUE;


			}
			DataTimerCounter++;
		}
    }

    // Loadcell Communication v
	if(!WLCLWaitingMode)
	{
		if( ROM_GPIOPinRead(PORT_W_LCL_SCL_BASE, PORT_W_LCL_SCL_PIN) )
		{
			if(WLCLSCLCounter < 24)
				if( ROM_GPIOPinRead(PIN_W_LCL_DATA_BASE, PIN_W_LCL_DATA_PIN) )
					WLCLSDAReadingTemp = (WLCLSDAReadingTemp << 1) + 1;
				else
					WLCLSDAReadingTemp = WLCLSDAReadingTemp << 1;

			ROM_GPIOPinWrite(PORT_W_LCL_SCL_BASE, PORT_W_LCL_SCL_PIN, 0);//PORTSCL = FALSE;

			WLCLSCLCounter++;
		}
		else
			ROM_GPIOPinWrite(PORT_W_LCL_SCL_BASE, PORT_W_LCL_SCL_PIN, PORT_W_LCL_SCL_PIN);

		if(WLCLSCLCounter > 23)
		{
			WLCLSCLCounter = 0;
			WLCLSDAReading = WLCLSDAReadingTemp;
			WLCLSDAReadingBuffer[WLCLSDAAverageCounter] = WLCLSDAReading;
			WLCLSDAAverageCounter++;
			if(WLCLSDAAverageCounter >= WLCLLoadSampling)
				WLCLSDAAverageCounter = 0;

			WLCLSDAReadingTemp = 0;                                                 // SDAReadingTemp must be cleared first before receiving new data
			WLCLWaitingMode = TRUE;
		}
	}

	if(!WLCRWaitingMode)
	{
		if( ROM_GPIOPinRead(PORT_W_LCR_SCL_BASE, PORT_W_LCR_SCL_PIN) )
		{
			if(WLCRSCLCounter < 24)
				if( ROM_GPIOPinRead(PIN_W_LCR_DATA_BASE, PIN_W_LCR_DATA_PIN) )
					WLCRSDAReadingTemp = (WLCRSDAReadingTemp << 1) + 1;
				else
					WLCRSDAReadingTemp = WLCRSDAReadingTemp << 1;

			ROM_GPIOPinWrite(PORT_W_LCR_SCL_BASE, PORT_W_LCR_SCL_PIN, 0);//PORTSCL = FALSE;

			WLCRSCLCounter++;
		}
		else
			ROM_GPIOPinWrite(PORT_W_LCR_SCL_BASE, PORT_W_LCR_SCL_PIN, PORT_W_LCR_SCL_PIN);

		if(WLCRSCLCounter > 23)
		{
			WLCRSCLCounter = 0;
			WLCRSDAReading = WLCRSDAReadingTemp;
			WLCRSDAReadingBuffer[WLCRSDAAverageCounter] = WLCRSDAReading;
			WLCRSDAAverageCounter++;
			if(WLCRSDAAverageCounter >= WLCRLoadSampling)
				WLCRSDAAverageCounter = 0;

			WLCRSDAReadingTemp = 0;                                                 // SDAReadingTemp must be cleared first before receiving new data
			WLCRWaitingMode = TRUE;
		}
	}
	// Loadcell Communication ^
}

int
main(void)
{

    // Set the system clock 80MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOG);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOH);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOJ);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_TIMER1);
    //ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_QEI0);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_SSI0);
    ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_UART0);
    //ROM_SysCtlPeripheralEnable (SYSCTL_PERIPH_I2C1);

	// Change PB7 into GPIO outputs. First open the lock and select
	// the bits we want to modify in the GPIO commit register.
	//
	HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTB_BASE + GPIO_O_CR) = 0x80;

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x10;

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x20;

    // Initialize the GPIO
    ROM_GPIOPinTypeGPIOInput (GPIO_PORTA_BASE, 0x11);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, 0xEE);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, 0xFB);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, 0x04);

    ROM_GPIOPinTypeGPIOOutput (GPIO_PORTC_BASE, 0xF0);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTD_BASE, 0xFE);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, 0x01);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTE_BASE, 0xDC);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, 0x23);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTF_BASE, 0x07);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x38);

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, 0x83);

    ROM_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, 0x7F);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, 0x80);

    ROM_GPIOPinTypeGPIOInput (GPIO_PORTJ_BASE, 0x02);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, 0x05);

    //*****************************************************************************
    // Interrupt Settings
    //*****************************************************************************
	ROM_GPIOIntTypeSet(PIN_W_LCL_DATA_BASE, PIN_W_LCL_DATA_PIN, GPIO_FALLING_EDGE);
	ROM_GPIOIntTypeSet(PIN_W_LCR_DATA_BASE, PIN_W_LCR_DATA_PIN, GPIO_FALLING_EDGE);

	ROM_GPIOPinIntEnable(PIN_W_LCL_DATA_BASE, PIN_W_LCL_DATA_PIN);
	ROM_GPIOPinIntEnable(PIN_W_LCR_DATA_BASE, PIN_W_LCR_DATA_PIN);

    //*****************************************************************************
    // Timer 0 Setup
    //*****************************************************************************
    // Configure Timer0 as 32-bit periodic timer
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
    // Set Timer0 period as 1/1000 of the system clock, i.e. 1000 interrupts per second
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1000);
    // Configure the timer to generate an interrupt on time-out
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Enable the TImer Interrupt in the NVIC
    IntEnable(INT_TIMER0A);
    // Enable the timer
    TimerEnable(TIMER0_BASE, TIMER_A);

    //*****************************************************************************
    // Timer 1 Setup
    //*****************************************************************************
    // Configure Timer0 as 32-bit periodic timer
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);
    // Set Timer0 period as 1/10000 of the system clock, i.e. 10000 interrupts per second
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 10000);
    // Configure the timer to generate an interrupt on time-out
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Enable the timer Interrupt in the NVIC
    IntEnable(INT_TIMER1A);
    // Enable the timer
    TimerEnable(TIMER1_BASE, TIMER_A);

    //*****************************************************************************
    // UART Setup
    //*****************************************************************************
 	GPIOPinConfigure(GPIO_PA1_U0TX);
 	GPIOPinConfigure(GPIO_PA0_U0RX);

	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 |
						GPIO_PIN_1);

	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));

    // Configuring the UART interrupt.
	//ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
	UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_EOT);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_TX);
    ROM_IntEnable(INT_UART0);

    // Enable the UART.
    UARTEnable(UART0_BASE);

    // Unfortunately, in StellarisWare, UARTEnable() silently re-enables the FIFO,
    // so to get rid of that, move UARTFIFODisable() call after UARTEnable()
    UARTFIFODisable(UART0_BASE);

    //*****************************************************************************
    // SSI Setup
    //*****************************************************************************

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    ROM_GPIOPinTypeSSI(PORT_SSI_CLK_BASE, PORT_SSI_CLK_PIN | PORT_SSI_FSS_PIN |
                   PIN_SSI_RX_PIN | PORT_SSI_TX_PIN);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 125000, 8);

    SSIEnable(SSI0_BASE);

    // Clear SPI Buffer
    while(SSIDataGetNonBlocking(SSI0_BASE, &SPIIn[0]))
    {
    }

    /* switch to interrupt driven SPI */

    SSIIntDisable(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );
    SSIIntClear(SSI0_BASE, SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR );

    HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT; /* switch tx interrupt to eot int */

    SSIIntEnable(SSI0_BASE, SSI_TXFF ); /* SSI_TXFF | SSI_RXFF | SSI_RXTO | SSI_RXOR  */

    ROM_IntEnable(INT_SSI0);

    // Enable Interrupt
    ROM_IntMasterEnable();

	//ROM_IntEnable(INT_GPIOC);
    ROM_IntEnable(INT_GPIOD);
	ROM_IntEnable(INT_GPIOE);
	//ROM_IntEnable(INT_GPIOH);

	// Init
	GTInFillGripPneuValve = TRUE;	// Grip Open
	GTOutFillGripPneuValve = TRUE;	// Grip Open
    while(1)
    {
    	LoopCycle++;
    	if(ITMotorTrayElevatorDirSet)
    	{
    		DirUP = TRUE;
    		DirDOWN = FALSE;
    	}
    	else
    	{
    		DirUP = FALSE;
    		DirDOWN = TRUE;
    	}

        /* Load Cell Reading v */
        WLCLTempReading = 0;
        for(i = 0; i < WLCLLoadSampling; i++)
        {
        	WLCLTempReading += WLCLSDAReadingBuffer[i];
            if(i == (WLCLLoadSampling - 1))
            {
            	WLCLSDAReadingAverage = WLCLTempReading / WLCLLoadSampling;
            }
        }
        WLCLSDAReadingNormalised = ( (WLCLSDAReadingAverage * 250) / WLCLSDARatioCustomKg) - WLCLSDAReadingOffset;

        if( ((WLCLDispWeight * 10) - WLCLSDAReadingNormalised > 8) || ((WLCLDispWeight * 10) - WLCLSDAReadingNormalised < 0) )
        	WLCLDispWeight = WLCLSDAReadingNormalised / 10;

        if(WLCLDispWeight > 9999)
        	WLCLDispWeight = 9999;

        WLCRTempReading = 0;
        for(i = 0; i < WLCRLoadSampling; i++)
        {
        	WLCRTempReading += WLCRSDAReadingBuffer[i];
            if(i == (WLCRLoadSampling - 1))
            {
            	WLCRSDAReadingAverage = WLCRTempReading / WLCRLoadSampling;
            }
        }
        WLCRSDAReadingNormalised = ( (WLCRSDAReadingAverage * 250) / WLCRSDARatioCustomKg) - WLCRSDAReadingOffset;

        if( ((WLCRDispWeight * 10) - WLCRSDAReadingNormalised > 8) || ((WLCRDispWeight * 10) - WLCRSDAReadingNormalised < 0) )
        	WLCRDispWeight = WLCRSDAReadingNormalised / 10;

        if(WLCRDispWeight > 9999)
        	WLCRDispWeight = 9999;
        /* Load Cell Reading ^ */

        // Weigher Module : Weighing, and passing to filling bucket
        if( (MachineRunning && (!MachineStopping)) || WSoloEnable )
        {
        	// Left Weigher Module
            if(WLCLSDAReadingNormalised < ((WLCLTargetWeight - WLCLCorrectionWeight)* 10))
            {
                if(!WLCLWeightOK)                                // After Weight OK, Always Close Gate)
                {
                	if(WLCLSlowAt)
                	{
                		WLLrGatePneuValve = TRUE;
                		WLSmGatePneuValve = TRUE;
                	}
                	else
                	{
                    	WLLrGatePneuValve = FALSE;
                    	WLSmGatePneuValve = TRUE;
                	}
                    if(WLCLSDAReadingNormalised > (WLCLSlowAt * 10))
                    {
                    	WLLrGatePneuValve = FALSE;
                    	WLSmGatePneuValve = TRUE;
                    }
                    WLCLWeightOKFlag = FALSE;
                }
            }
            else    // Target Weight achieved
            {
            	WLLrGatePneuValve = FALSE;
            	WLSmGatePneuValve = FALSE;
            	// Still buffered for 200ms, then become WLCLWeightOK
                WLCLWeightOKFlag = TRUE;
            }
            // Right Weigher Module
            if(WLCRSDAReadingNormalised < ((WLCRTargetWeight - WLCRCorrectionWeight)* 10))
            {
                if(!WLCRWeightOK)                                // After Weight OK, Always Close Gate)
                {
                	if(WLCRSlowAt)
                	{
                		WRLrGatePneuValve = TRUE;
                		WRSmGatePneuValve = TRUE;
                	}
                	else
                	{
                    	WRLrGatePneuValve = FALSE;
                    	WRSmGatePneuValve = TRUE;
                	}
                    if(WLCRSDAReadingNormalised > (WLCRSlowAt * 10))
                    {
                    	WRLrGatePneuValve = FALSE;
                    	WRSmGatePneuValve = TRUE;
                    }
                    WLCRWeightOKFlag = FALSE;
                }
            }
            else    // Target Weight achieved
            {
            	WRLrGatePneuValve = FALSE;
            	WRSmGatePneuValve = FALSE;
            	// Still buffered for 200ms, then become WLCRWeightOK
                WLCRWeightOKFlag = TRUE;
            }
        }

        // Turn OFF Gates when WSoloEnable just DISABLED
        if(LastWSoloEnable != WSoloEnable)
        {
        	if(!WSoloEnable)
        	{
            	WLLrGatePneuValve = FALSE;
            	WLSmGatePneuValve = FALSE;
            	WRLrGatePneuValve = FALSE;
            	WRSmGatePneuValve = FALSE;
            	WLCLWeightOKFlag = FALSE;
            	WLCRWeightOKFlag = FALSE;
            	WLCLWeightOK = FALSE;
            	WLCRWeightOK = FALSE;
        	}
        }
        LastWSoloEnable = WSoloEnable;

        // Output to OF Module when there is request
        if(WLCRWeightOK || WLCLWeightOK)
        {
        	if(OFRequestFilling || WSoloOpenGate)
        	{
        		// To make more balance, if both okay, then alternate
        		if(WLCLWeightOK && WLCRWeightOK)
        		{
        			if(WLCNextGatePriority == 1)
        			{
        				// Right Gate Opens
        				WROpGatePneuDurationCounter = WROpGatePneuDuration;
        				WLCNextGatePriority = 0;//Next is LEFT;
        			}
        			else
        			{
        				// Left Gate Opens
        				WLOpGatePneuDurationCounter = WLOpGatePneuDuration;
        				WLCNextGatePriority = 1;//Next is RIGHT;
        			}
        		}
        		else if(WLCLWeightOK)
				{
        			// Left Gate Opens
        			WLOpGatePneuDurationCounter = WLOpGatePneuDuration;
        			WLCNextGatePriority = 1;////Next is RIGHT;
				}
				else if(WLCRWeightOK)
				{
					// Right Gate Opens
					WROpGatePneuDurationCounter = WROpGatePneuDuration;
					WLCNextGatePriority = 0;////Next is LEFT;
				}

				OFRequestFilling = FALSE;
				WSoloOpenGate = FALSE;
        	}
        }
        /* Weigher Module Operation Program ^ */

        /* Control Program v */
        if((!ServiceMode) && (ComFlag))                                                                               // Operation Page (Not Service Mode)
        {
            if(!MachineRunning)                                                                          // when machine is not running
            {
            	// If Machine is not busy, RunBtn, InitMON is available
            	if( (!Initting) && (!WLCLZeroMONFlag) && (!WLCRZeroMONFlag) )
            	{
					if( RunBtn ) // (!PINBTNRUN)
					{
						OFFastRequestFilling = FALSE;
						TRSbsMode = FALSE;
						GTSbsMode = FALSE;
						OFSbsMode = FALSE;
						SPSbsMode = FALSE;
						MachineRunning = TRUE;
						WLCLWeightOKCounter = 200;
						WLCRWeightOKCounter = 200;
						if( (!TRModuleStep) && (!GTModuleStep) && (!OFModuleStep) && (!SPModuleStep) )
						{
							TRModuleWaiting = FALSE;
							TRModuleWaitingPick = FALSE;
							GTSA1ModuleWaiting = FALSE;
							GTSA2ModuleWaiting = FALSE;
							OFModuleWaiting = FALSE;
							GTRequestFilling = FALSE;
						}

						RunBtn = FALSE;
					}
					if( InitMON )
					{
						OFFastRequestFilling = FALSE;
						Initting = TRUE;
						TRInitting = TRUE;
						GTInitting = TRUE;
						OFInitting = TRUE;
						SPInitting = TRUE;
						TRInitError = 0;
						GTInitError = 0;
						OFInitError = 0;
						SPInitError = 0;
						InitMON = FALSE;
					}
            	}
            }
            else                                                                                                    // when machine is RUNNING
            {
                if( StopBtn ) //|| Error)         // Motor OFF Btn/Inching pushed or Error Triggered
                {
                	OFFastRequestFilling = FALSE;
                	MachineStopping = TRUE;
                	WLLrGatePneuValve = FALSE;
					WLSmGatePneuValve = FALSE;
					WRLrGatePneuValve = FALSE;
					WRSmGatePneuValve = FALSE;
                	WLCLWeightOK = FALSE;
                	WLCRWeightOK = FALSE;
                	StopBtn = FALSE;
                	/*
                    if(StopCommand == FALSE)                                              // Stop the machine at designated STOP AT position
                    {
                        StopCommand = TRUE;
                    }*/
                }
                if(FatalError)
                {
                    MachineRunning = FALSE;
                }
            }
        }
        if(Initting)
        {
        	if( (!TRInitting) && (!GTInitting) && (!OFInitting) && (!SPInitting) )
        	{
        		OFFastRequestFilling = FALSE;
        		Initting = FALSE;
        		GTSA1ModuleWaiting = FALSE;
        		GTSA2ModuleWaiting = FALSE;
        		OFModuleWaiting = FALSE;
        		TRModuleWaiting = FALSE;
        		TRModuleWaitingPick = FALSE;
        		TRInitErrorResult = TRInitError;
        		GTInitErrorResult = GTInitError;
        		OFInitErrorResult = OFInitError;
        		SPInitErrorResult = SPInitError;
        		GTFirstCycle = TRUE;
        	}
        }
        if(MachineStopping)
        {
        	if( (!TRModuleStep) && (!GTModuleStep) && (!OFModuleStep) && (!SPModuleStep) )
        	{
        		OFFastRequestFilling = FALSE;
        		MachineStopping = FALSE;
        		MachineRunning = FALSE;
            	WLLrGatePneuValve = FALSE;
				WLSmGatePneuValve = FALSE;
				WRLrGatePneuValve = FALSE;
				WRSmGatePneuValve = FALSE;
        		GTSA1ModuleWaiting = FALSE;
        		GTSA2ModuleWaiting = FALSE;
        		OFModuleWaiting = FALSE;
        		TRModuleWaiting = FALSE;
        		TRModuleWaitingPick = FALSE;
        		GTFirstCycle = TRUE;
        	}
        }

        //FoilSub();                                                                        // Motor Foil Subroutine
        /* Control Program ^ */

        /* Error Identifier v */

        if(NoAirError)
            FatalError = TRUE;
        else
            FatalError = FALSE;

        if(HeaterError || DoorOpen || FoilEmpty )
            Error = TRUE;
        else
            Error = FALSE;

        /* Error Identifier ^ */

        /* Error Handler v */

        if(Error || FatalError || HeaterErrorMsg || FoilJamMsg || NoAirErrorMsg)
        {
            if(BlinkFlag)                           // Warning, took DoorOpen place in com
                BlinkCom = TRUE;
            else
                BlinkCom = FALSE;
        }
        else
            BlinkCom = FALSE;

        /* Error Handler ^ */
        if(ReadACK)
        {
            if(DataReady)
            {
                DataLock = TRUE;
				while( dataTXIndex <= ((ComAddress * 20) + (BytestoRead) + 5 - 1) )
				{
					ROM_UARTCharPut(UART0_BASE, ComOUT[dataTXIndex]);
					dataTXIndex++;
				}
				DataLock = FALSE;
				ReadACK = FALSE;
				dataTXIndex = ComAddress * 20;
            }
            else
                DataTimerTrigger = TRUE;                        // Activate the transfer manually
        }

    	// UART Communication Subroutine
        if(DataLock == FALSE)
        {   //Init Page ComOUT[0] - ComOUT[19]
            DataReady = FALSE;
            ComOUT[0]= 0xF0;              // Start
            // OUT: Position: Station No = 0
            ComOUT[1]= 0x01;
            // OUT: Function: Number 3
            ComOUT[2]= 0x03;
            // OUT : END
            ComOUT[3] = 0x0F;
            ComOUT[4] = 0x0F;

            //Main Page ComOUT[20] - ComOUT[39]
            ComOUT[20] = 0xF0;              // Start
            // OUT: Position: Station No = 0
            ComOUT[21] = 0x01;
            // OUT: Function: Number 3
            ComOUT[22] = 0x03;
            // OUT: Data
            ComOUT[23] = MachineRunning + (BlinkCom << 1) + (ProductCountBit << 2) + (GTFirstCycle << 3) + (MachineStopping << 4) + (Initting << 5) + (BlinkFlag << 6);
            ComOUT[24] = HeaterErrorMsg + (EyemarkError << 1) + (DoorOpen << 2) + (SPIComError << 3) + (NoAirErrorMsg << 4)  + (DatesensError << 5);
            ComOUT[25] = ITMotorTrayElevator + (ITMotorTrayElevatorDir << 1) + (ITValveTrayPos << 2) + (ITReady << 3);
            ComOUT[26] = TRExtenderPneuValve + (TRReorientPneuValve << 1) + (TRVacuumValve << 2) + (GTFPushPneuValve << 3) + (TRModuleWaiting << 4);
            ComOUT[27] = GTInFillGripPneuValve + (GTOutFillOpenPneuValve << 1) + (GTOutFillGripPneuValve << 2) + (GTTransferPneuValve << 3) + (GTSA1ModuleWaiting << 4) + (GTSA2ModuleWaiting << 5) + (GTRequestFilling << 6);
            ComOUT[28] = OFGatePneuValve + (OFEngagePneuValve << 1) + (OFOpenPneuValve << 2) + (OFVacuumValve << 3) + (VFVibEngagePneuValve << 4) + (VFVibrator << 5) + (OFGatePneuFilled << 6) + (OFGatePneuFillingProcess << 7);
            ComOUT[29] = OFModuleWaiting;
            ComOUT[30] = SPSealerPneuValve + (SPVibEngagePneuValve << 1) + (SPPusherValve << 2) + (SPVibrator << 3) + (WLCLWeightOK << 6) + (WLCRWeightOK << 7);
            ComOUT[31] = WLLrGatePneuValve + (WLSmGatePneuValve << 1) + (WLOpGatePneuValve << 2) + (WRLrGatePneuValve << 3) + (WRSmGatePneuValve << 4) + (WROpGatePneuValve << 5) + (WLCLCalibrationFlag << 6) + (WLCRCalibrationFlag << 7);
            // All Pins Master
            ComOUT[32] = ROM_GPIOPinRead(GPIO_PORTA_BASE, 0xFF);
            ComOUT[33] = ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xFF);
            ComOUT[34] = ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xFF);
            ComOUT[35] = ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xFF);
            ComOUT[36] = ROM_GPIOPinRead(GPIO_PORTE_BASE, 0xFF);
            ComOUT[37] = ROM_GPIOPinRead(GPIO_PORTF_BASE, 0xFF);
            ComOUT[38] = ROM_GPIOPinRead(GPIO_PORTG_BASE, 0xFF);
            ComOUT[39] = ROM_GPIOPinRead(GPIO_PORTH_BASE, 0xFF);
            ComOUT[41] = ROM_GPIOPinRead(GPIO_PORTJ_BASE, 0xFF);
            // All Pins Slave
            ComOUT[42] = SlavePortA;
            ComOUT[43] = SlavePortB;
            ComOUT[44] = SlavePortC;
            ComOUT[45] = SlavePortD;
            ComOUT[46] = SlavePortE;
            ComOUT[47] = SlavePortF;
            ComOUT[48] = SlavePortG;
            ComOUT[49] = SlavePortH;
            ComOUT[50] = SlavePortI;
            ComOUT[51] = SlavePortJ;

            ComOUT[52] = ITSwitchTrayStep;
            ComOUT[53] = TRModuleStep;
            ComOUT[54] = TRVacuumValveDelayCounter / 10;
            ComOUT[55] = GTModuleStep;
            ComOUT[56] = GTCloseBagDelayCounter / 10;
            ComOUT[57] = GTBothOpenGripDelayCounter / 10;
            ComOUT[58] = OFModuleStep;
            ComOUT[59] = OFGatePneuValveDurationCounter / 10;
            ComOUT[60] = OFVacuumValveDelayCounter / 10;
            ComOUT[61] = SPModuleStep;
            ComOUT[62] = SPSealerPneuValveDelayCounter / 10;
            ComOUT[63] = SPSealerPneuValveDurationCounter / 10;
            ComOUT[64] = WLOpGatePneuDurationCounter / 10;
            ComOUT[65] = WROpGatePneuDurationCounter / 10;

            ComOUT[66] = WNextOpGateDelayCounter / 10;
            ComOUT[67] = WLCLSDARatioCustomKgCO >> 24;			// Com Out
            ComOUT[68] = WLCLSDARatioCustomKgCO >> 16;			// Com Out
            ComOUT[69] = WLCLSDARatioCustomKgCO >> 8;			// Com Out
            ComOUT[70] = WLCLSDARatioCustomKgCO;				// Com Out
            ComOUT[71] = WLCRSDARatioCustomKgCO >> 24;
            ComOUT[72] = WLCRSDARatioCustomKgCO >> 16;
            ComOUT[73] = WLCRSDARatioCustomKgCO >> 8;
			ComOUT[74] = WLCRSDARatioCustomKgCO;

			ComOUT[75] = WLCLSDAReadingOffsetCO >> 24;
			ComOUT[76] = WLCLSDAReadingOffsetCO >> 16;
			ComOUT[77] = WLCLSDAReadingOffsetCO >> 8;
			ComOUT[78] = WLCLSDAReadingOffsetCO;
			ComOUT[79] = WLCRSDAReadingOffsetCO >> 24;
			ComOUT[80] = WLCRSDAReadingOffsetCO >> 16;
			ComOUT[81] = WLCRSDAReadingOffsetCO >> 8;
			ComOUT[82] = WLCRSDAReadingOffsetCO;

			ComOUT[83] = WLCLDispWeight >> 24;
			ComOUT[84] = WLCLDispWeight >> 16;
			ComOUT[85] = WLCLDispWeight >> 8;
			ComOUT[86] = WLCLDispWeight;
			ComOUT[87] = WLCRDispWeight >> 24;
			ComOUT[88] = WLCRDispWeight >> 16;
			ComOUT[89] = WLCRDispWeight >> 8;
			ComOUT[90] = WLCRDispWeight;

			ComOUT[91] = WLCLSDAReading >> 24;
			ComOUT[92] = WLCLSDAReading >> 16;
			ComOUT[93] = WLCLSDAReading >> 8;
			ComOUT[94] = WLCLSDAReading;
			ComOUT[95] = WLCRSDAReading >> 24;
			ComOUT[96] = WLCRSDAReading >> 16;
			ComOUT[97] = WLCRSDAReading >> 8;
			ComOUT[98] = WLCRSDAReading;

			ComOUT[99] = GTFPushPneuValveDelayCounter / 10;
			ComOUT[100] = UNUSED;

			ComOUT[101] = LoopTime >> 8;
			ComOUT[102] = LoopTime;
			ComOUT[103] = LifeDuration >> 24;
			ComOUT[104] = LifeDuration >> 16;
			ComOUT[105] = LifeDuration >> 8;
			ComOUT[106] = LifeDuration;
			ComOUT[107] = 14;				// Program Version

			ComOUT[108] = TRInitErrorResult >> 8;
			ComOUT[109] = TRInitErrorResult;
			ComOUT[110] = GTInitErrorResult >> 8;
			ComOUT[111] = GTInitErrorResult;

			ComOUT[112] = OFInitErrorResult >> 8;
			ComOUT[113] = OFInitErrorResult;
			ComOUT[114] = SPInitErrorResult >> 8;
			ComOUT[115] = SPInitErrorResult;

            ComOUT[116] = VFVibratorEngageDelayCounter / 10;
            ComOUT[117] = VFVibratorEngageVibDelayCounter / 10;

            // New Twin Weigher V1.4 MTP (Touchscreen program changed from v1.0 - v1.4)
            ComOUT[118] = GTModuleStepTimer[0] >> 8;
            ComOUT[119] = GTModuleStepTimer[0];
            ComOUT[120] = GTModuleStepTimer[1] >> 8;
            ComOUT[121] = GTModuleStepTimer[1];
            ComOUT[122] = GTModuleStepTimer[2] >> 8;
            ComOUT[123] = GTModuleStepTimer[2];
            ComOUT[124] = GTModuleStepTimer[3] >> 8;
            ComOUT[125] = GTModuleStepTimer[3];
            ComOUT[126] = GTModuleStepTimer[4] >> 8;
            ComOUT[127] = GTModuleStepTimer[4];
            ComOUT[128] = GTModuleStepTimer[5] >> 8;
            ComOUT[129] = GTModuleStepTimer[5];
            ComOUT[130] = GTModuleStepTimer[6] >> 8;
            ComOUT[131] = GTModuleStepTimer[6];
            ComOUT[132] = GTModuleStepTimer[7] >> 8;
            ComOUT[133] = GTModuleStepTimer[7];
            ComOUT[134] = GTModuleStepTimer[8] >> 8;
            ComOUT[135] = GTModuleStepTimer[8];
            ComOUT[136] = GTModuleStepTimer[9] >> 8;
            ComOUT[137] = GTModuleStepTimer[9];

            ComOUT[138] = (GTModuleStepTimer[0] + GTModuleStepTimer[1] + GTModuleStepTimer[2] + GTModuleStepTimer[3] + GTModuleStepTimer[4] + GTModuleStepTimer[5] + GTModuleStepTimer[6] + GTModuleStepTimer[7] + GTModuleStepTimer[8] + GTModuleStepTimer[9]) >> 8;
            ComOUT[139] = GTModuleStepTimer[0] + GTModuleStepTimer[1] + GTModuleStepTimer[2] + GTModuleStepTimer[3] + GTModuleStepTimer[4] + GTModuleStepTimer[5] + GTModuleStepTimer[6] + GTModuleStepTimer[7] + GTModuleStepTimer[8] + GTModuleStepTimer[9];
            ComOUT[140] = GTRequestFillingDelayCounter / 10;
            ComOUT[141] = OFGatePneuValveVibDelayCounter / 10;
            ComOUT[142] = OFRequestFillingDelayCounter / 10;
            // OUT : END
            ComOUT[148] = 0x0F;
            ComOUT[149] = 0x0F;
            DataReady = TRUE;
        }

    	// SPI Communication Subroutine
        if(!SPIDataLock)
        {
            SPIDataReady = FALSE;
            SPIOut[0] = 0xF0;
            SPIOut[1] = 0x64;
            SPIOut[2] = 0x04;
            SPIOut[3] = ITMotorTrayElevator + ( ITMotorTrayElevatorDir << 1 ) + ( ITValveTrayPos << 2 );
            SPIOut[4] = GTInFillGripPneuValve + ( GTOutFillOpenPneuValve << 1 ) + ( GTOutFillGripPneuValve << 2 ) + ( GTTransferPneuValve << 3 );
            SPIOut[5] = VFVibEngagePneuValve + ( VFVibrator << 1 );
            SPIOut[6] = SPVibEngagePneuValve + ( SPPusherValve << 1 ) + ( SPVibrator << 2 );
            SPIOut[7] = BufferSensor >> 8;
            SPIOut[8] = BufferSensor;
            SPIOut[9] = TowerLightGreen + ( TowerLightYellow << 1 ) + ( TowerLightRed << 2 );
            SPIOut[24] = 0x0F;
            SPIOut[25] = 0x0F;
            SPIDataReady = TRUE;
        }
    }
}
