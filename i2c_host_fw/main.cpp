#include "hal.h"
#include "board.h"
#include "MsgQ.h"
#include "uart.h"
#include "shell.h"
#include "kl_lib.h"

//#define I2C_EN
#define USB_EN
#define RGB_EN
#define RADIO_EN
#ifdef I2C_EN
#include "kl_i2c.h"
#endif
#ifdef RADIO_EN
#include "radio_lvl1.h"
#endif
#ifdef RGB_EN
#include "led.h"
#include "Sequences.h"
#endif
#ifdef USB_EN
#include "usb_cdc.h"
#endif


#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
void OnCmd(Shell_t *PShell);
void ITask();

#define RW_LEN_MAX  108
#define CheckMeasurePeriod_MS 100

const PinOutput_t PillPwr {PILL_PWR_PIN};
#ifdef RGB_EN
LedRGB_t Led { LED_R_PIN, LED_G_PIN, LED_B_PIN };
#endif
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
#ifdef RGB_EN
    Led.Init();
    Led.StartOrRestart(lsqStart);
#endif
#ifdef I2C_EN
    PillPwr.Init();
    PillPwr.SetHi();
    chThdSleepMilliseconds(300);
    i2c2.Init();
#endif
#ifdef USB_EN
    UsbCDC.Init();
    Clk.EnableCRS();
    Clk.SelectUSBClock_HSI48();
    UsbCDC.Connect();
#endif
#ifdef RADIO_EN
    if(Radio.Init() == retvOk) {
        Printf("Radio Ok\r");
    }
#endif

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
#ifdef RGB_EN
                Led.StartOrRestart(lsqUSBCmd); // After that, falling throug is intentional
#endif
                OnCmd((Shell_t*)Msg.Ptr);
                ((Shell_t*)Msg.Ptr)->SignalCmdProcessed();
                break;

#ifdef USB_EN // ======= USB =======
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

#ifdef I2C_EN
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
#endif

#if 1 // ================= Command processing ====================
void OnCmd(Shell_t *PShell) {
	Cmd_t *PCmd = &PShell->Cmd;
    __attribute__((unused)) int32_t dw32 = 0;  // May be unused in some configurations
//    Printf("%S\r", PCmd->Name);
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(retvOk);
    }
#ifdef I2C_EN
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
#endif

    else PShell->Ack(retvCmdUnknown);
}
#endif
