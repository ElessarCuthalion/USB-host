#include "hal.h"
#include "board.h"
#include "MsgQ.h"
#include "uart.h"
#include "shell.h"
#include "kl_lib.h"
#include "led.h"
#include "Sequences.h"
#include "radio_lvl1.h"
#include "usb_cdc.h"
#include "kl_i2c.h"

#include "VL53L1X.h"
VL53L1X_t VL53L1X;

#define VL53L1_api
#ifdef VL53L1_api
#include "vl53l1_api.h"
#include "vl53l1_platform.h"
#include "vl53l1_platform_user_data.h"
VL53L1_Dev_t Dev;
VL53L1_DetectionConfig_t DetectionConfig;
#endif

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
void OnCmd(Shell_t *PShell);
void ITask();

#define RW_LEN_MAX  108
#define CheckMeasurePeriod_MS 500

const PinOutput_t PillPwr {PILL_PWR_PIN};
LedRGB_t Led { LED_R_PIN, LED_G_PIN, LED_B_PIN };

TmrKL_t MeasTMR {MS2ST(CheckMeasurePeriod_MS), evtIdCheckMeasure, tktPeriodic};
#endif

int main(void) {
    // ==== Init Clock system ====
    Clk.EnablePrefetch();
    Clk.SetupFlashLatency(48000000);
    Clk.SwitchTo(csHSI48);
    Clk.UpdateFreqValues();

    // === Init OS ===
    halInit();
    chSysInit();

    // ==== Init hardware ====
    EvtQMain.Init();
    Uart.Init();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk.PrintFreqs();

    Led.Init();
    Led.StartOrRestart(lsqStart);

    PillPwr.Init();
    PillPwr.SetHi();

    i2c2.Init();

    UsbCDC.Init();
    Clk.EnableCRS();
    Clk.SelectUSBClock_HSI48();
    UsbCDC.Connect();

#ifdef VL53L1_api
//	Dev.I2cHandle = &i2c2;
	Dev.I2cDevAddr = 0x52;
	uint8_t Result = retvOk;

	Result |= VL53L1_WaitDeviceBooted(&Dev);
	Result |= VL53L1_DataInit(&Dev);
	Result |= VL53L1_StaticInit(&Dev);
//	Result |= VL53L1_SetPresetMode(&Dev, VL53L1_PRESETMODE_AUTONOMOUS);
//	Result |= VL53L1_SetDistanceMode(&Dev, VL53L1_DISTANCEMODE_LONG);
//	Result |= VL53L1_SetMeasurementTimingBudgetMicroSeconds(&Dev, 70000);
//	Result |= VL53L1_SetInterMeasurementPeriodMilliSeconds(&Dev, 100);
//	DetectionConfig.DetectionMode = 0;
//	DetectionConfig.Distance.CrossMode = 3;
//	DetectionConfig.IntrNoTarget = 0;
//	DetectionConfig.Distance.High = 4000;
//	DetectionConfig.Distance.Low = 50;
//	Result |= VL53L1_SetThresholdConfig(&Dev, &DetectionConfig);
//	Result |= VL53L1_StartMeasurement(&Dev);

    if(Result == retvOk)
    	Printf("VL53L1X Ok\r");
    else
    	Printf("VL53L1X init Fail\r");

    VL53L1X.StartMeasurement(100);

#else
    if(VL53L1X.InitLiteAndStart() == retvOk) {
//    if(VL53L1X.Init() == retvOk) {
        Printf("VL53L1X Ok\r");
    }
#endif
    MeasTMR.StartOrRestart();

    // Main cycle
    ITask();
}

__noreturn
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdUsbNewCmd:
            case evtIdShellCmd:
                Led.StartOrRestart(lsqUSBCmd); // After that, falling throug is intentional
                OnCmd((Shell_t*)Msg.Ptr);
                ((Shell_t*)Msg.Ptr)->SignalCmdProcessed();
                break;

			case evtIdCheckMeasure:
				static bool temp = true;
				if (temp) {
					temp = false;
					if (VL53L1X.CheckForDataReady()) Printf("DataReady ok\r");
					uint16_t PDistance_MM;
					uint8_t RangeStatus;
					VL53L1X.GetRangeStatus(&RangeStatus);
					Printf("RangeStatus %u\r", RangeStatus);
					VL53L1X.GetDistance(&PDistance_MM);
					Printf("Distance %u\r", PDistance_MM);
					VL53L1X.ClearInterrupt();
					VL53L1X.StopMeasurement();
				}
				else {
					VL53L1X.StartMeasurement();
					temp = true;
				}
			    break;

#if 1 // ======= USB =======
            case evtIdUsbConnect:
                Printf("USB connect\r");
                Clk.EnableCRS();
                Clk.SelectUSBClock_HSI48();
                UsbCDC.Connect();
                break;
            case evtIdUsbDisconnect:
                Printf("USB disconnect\r");
                UsbCDC.Disconnect();
                Clk.DisableCRS();
                break;
            case evtIdUsbReady:
                Printf("USB ready\r");
//                Led.StartOrRestart(lsqUsbReady);
                break;
#endif

            default: break;
        } // switch
    } // while true
} // ITask()

void Standby() {
    i2c2.Standby();
//    PillPwr.SetLo();
//    __NOP(); __NOP(); __NOP(); __NOP(); // Allow power to fade
//    PillPwr.Deinit();
}

void Resume() {
//    PillPwr.Init();
//    PillPwr.SetHi();
//    __NOP(); __NOP(); __NOP(); __NOP(); // Allow power to rise
    i2c2.Resume();
}

#if 1 // ================= Command processing ====================
void OnCmd(Shell_t *PShell) {
	Cmd_t *PCmd = &PShell->Cmd;
    __attribute__((unused)) int32_t dw32 = 0;  // May be unused in some configurations
//    Printf("%S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(retvOk);
    }

    else if(PCmd->NameIs("help")) {
        PShell->Print("\r\n%S %S\r\n"
                "Commands:\r\n"
                "Ping   - returns Ack 0\r\n"
                "Scan   - scans all addresses on i2c bus\r\n"
                "W <Addr> <LenW> <Byte1> [Byte2] ... [Byte_LenW]   - Write bytes. Example: W 0x50 2 45 4 - write to addr 0x50 two bytes: 45 and 4\r\n"
                "WR <Addr> <LenW> <LenR> <Byte1> [Byte2] ... [Byte_LenW]   - Write LenW bytes, then read LenR bytes. Example: WR 0x50 1 4 0 - write one byte 0, then read four bytes.\r\n"
                "W and WR return Ack 1 in case of communication error.\r\n",
                APP_NAME, XSTRINGIFY(BUILD_TIME));
    }

    else if(PCmd->NameIs("Scan")) {
        i2c2.ScanBus(PShell);
    }
#if 0
    // W <Addr> <Len <= 108 > (Data1, Data2, ..., DataLen)
    else if(PCmd->NameIs("W")) {
        uint8_t Addr, Len, Data[RW_LEN_MAX];
        if(PCmd->GetNext<uint8_t>(&Addr) == retvOk) {
//        	PShell->Print("Addr ok\r\n");
            if(PCmd->GetNext<uint8_t>(&Len) == retvOk) {
//            	PShell->Print("Len ok\r\n");
                if(Len > RW_LEN_MAX) Len = RW_LEN_MAX;
                if(PCmd->GetArray<uint8_t>(Data, Len) == retvOk) {
//                	PShell->Print("Data ok\r\n");
                    Resume();
                    uint8_t Rslt = i2c2.Write(Addr, Data, Len);
                    Standby();
                    PShell->Ack(Rslt);
                }
                else PShell->Ack(retvCmdError);
            }
            else PShell->Ack(retvCmdError);
        }
        else PShell->Ack(retvCmdError);
    }

    // WriteRead: WR <Addr> <LenW> <LenR> (Data1, Data2, ..., DataLen)
    else if(PCmd->NameIs("WR")) {
        uint8_t Addr, LenW, LenR, Data[RW_LEN_MAX];
        if(PCmd->GetNext<uint8_t>(&Addr) == retvOk) {
            if(PCmd->GetNext<uint8_t>(&LenW) == retvOk) {
                if(LenW > RW_LEN_MAX) LenW = RW_LEN_MAX;
                if(PCmd->GetNext<uint8_t>(&LenR) == retvOk) {
                    if(LenR > RW_LEN_MAX) LenR = RW_LEN_MAX;
                    if(PCmd->GetArray<uint8_t>(Data, LenW) == retvOk) {
                        Resume();
                        uint8_t Rslt = i2c2.WriteRead(Addr, Data, LenW, Data, LenR);
                        Standby();
                        if(Rslt == retvOk) {
                            PShell->Print("%A\r\n", Data, LenR, ' ');
                        }
                        else PShell->Ack(Rslt);
                    }
                    else PShell->Ack(retvCmdError);
                }
                else PShell->Ack(retvCmdError);
            }
            else PShell->Ack(retvCmdError);
        }
        else PShell->Ack(retvCmdError);
    }
#endif
    else if(PCmd->NameIs("ChangeInterruptPolarity")) {
    	static VLInterruptPolarity_t IntPol = ipLow;
    	if (IntPol == ipLow) {
    		IntPol = ipHigh;
    	} else {
    		IntPol = ipLow;
    	}
		PShell->Ack(VL53L1X.SetInterruptPolarity(IntPol));
		PShell->Ack(VL53L1X.GetInterruptPolarity(&IntPol));
		PShell->Print("InterruptPolarity %u\r", IntPol);
//    	PShell->Ack(VL53L1X.StartMeasurement());
    }

    else if(PCmd->NameIs("GetDistance")) {
//    	uint32_t Data;
		uint16_t Distance_MM = 0;
//		VL53L1X.CheckForDataReady();
		VL53L1X.GetDistance(&Distance_MM);
		VL53L1X.ClearInterrupt();
		PShell->Print("Distance %u\r", Distance_MM);
    }

    else PShell->Ack(retvCmdUnknown);
}
#endif
