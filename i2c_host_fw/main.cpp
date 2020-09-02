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

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
void OnCmd(Shell_t *PShell);
void ITask();

#define RW_LEN_MAX  108
#define CheckMeasurePeriod_MS 10

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


    if(VL53L1X.Init2() == retvOk) {
        Printf("VL53L1X Ok\r");
//        MeasTMR.StartOrRestart();
    }

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
				if (VL53L1X.CheckForDataReady()) {
					uint16_t PDistance_MM;
					VL53L1X.GetDistance(&PDistance_MM);
					VL53L1X.ClearInterrupt();
					Printf("Distance %u\r", PDistance_MM);
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

    else if(PCmd->NameIs("ChangeInterruptPolarity")) {
    	static uint8_t IntPol = 0;
    	if (IntPol == 0) {
    		IntPol = 1;
    		PShell->Ack(VL53L1X.SetInterruptPolarity(ipLow));
    	}
    	else {
    		IntPol = 0;
    		PShell->Ack(VL53L1X.SetInterruptPolarity(ipHigh));
    	}
    	PShell->Ack(VL53L1X.StartMeasurement());
    }

    else if(PCmd->NameIs("GetDistance")) {
//    	uint32_t Data;
		uint16_t PDistance_MM = 0;
//		VL53L1X.CheckForDataReady();
		VL53L1X.GetDistance(&PDistance_MM);
		VL53L1X.ClearInterrupt();
		PShell->Print("Distance %u\r", PDistance_MM);
    }

    else PShell->Ack(retvCmdUnknown);
}
#endif
