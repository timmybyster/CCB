/* 
 * File:   main.c
 * Author: Steven Burford
 *
 * Created on 28 January, 2015, 8:23 AM
 *
 * IBC-1 V1.00 Code
 *
 */

#include "ST7540.h"
#include <xc.h>
#include "main.h"
#include "UART.h"
//#include "EEP.h"

///////
//PLM Comms
int ReceivedPLMBufferIndexIG = 0;
int TransmitPLMBufferIG = 0;
char ReceivedPLMBufferCG[PLM_RECEIVE_BUFFER_LENGTH];
char TransmitPLMBufferCG[PLM_TRANSMIT_BUFFER_LENGTH];
char PiDefaultData[2];

unsigned char boosterCommsData[DATA_TYPE_COUNT][60];
unsigned char boosterCommsDataReverse[60];
unsigned char test;

unsigned short ISC_SN_ArrayUIG[MAX_NO_ISC];
unsigned char ISC_SN_Array_ReverseUSG[MAX_NO_ISC * 2];
unsigned short ISC_Switch_TempUSG;
unsigned int ISC_PARENT_SNArrayUIG[MAX_NO_ISC];
unsigned char ISC_SN_ArrayIndexUCG = 0;
unsigned char ISC_SN_Array_SizeUCG = 0;
unsigned int HighSpeedTickUIG;
unsigned int HighSpeedTickPAUIG;
unsigned int BTHighSpeedTickUIG;
unsigned int CFLEDFlashTickUIG = 0;
unsigned int IdleTimerUIG = 0;
unsigned int CF_STAUTS_TimerUIG = 0;
unsigned int CheckISCTimerUIG = 0;
char LowSpeedTickCG;
unsigned int HighSpeedTickPBTUIG;
char DeArmTickCG;
unsigned int DeArmHighTickUIG;
char LowSpeedTickPBTCG;
char PBTEnabled;
char DeArmTimerEnabled;
char LowSpeedTickBTCG;
char BTEnabled;
unsigned char ClearToFire;
unsigned char CFLEDFlashEnable;

char LowSpeedTickPATCG;
char PATEnabled;

char HalfCycleCounterCG;
char MissingPulseCounterCG;
char ModeCG;
char ToggleRelay = 0;
unsigned char RelayStatusUCG;
unsigned char KeySwitchStatusUCG;
unsigned char FireButtonStatusUCG;
unsigned char CableFaultStatusUCG;
unsigned char NewCFStatusUCG;
unsigned char AlarmStatusUCG;
unsigned char CFLEDFlashState;
unsigned char PacketForPiIdentifier = 0b11111111;
unsigned char ISC_Transmit_Packet_Ready = 0;
unsigned char Pi_Status_Update = 0;
unsigned char New_ISC_SN = 0;
unsigned char BroadcastCheckUC = 0;
unsigned char Discover_ISC_ModeUCG = 0;
unsigned char Discovered_New_ISC = 0;

char HighPulseCountCG;

unsigned char KeySwitchStateUCG;

unsigned char earthLeakageProblemUCG;
unsigned char cableFaultProblemUCG;
unsigned short mainsZero_CrossingValueUSG;
short long fireOutValueSLG;
float FireOutFloat;
unsigned int ISC_Packet_Index = 0;
unsigned int ISC_CF_Index = 0;
unsigned int IBC_Default_Data;
unsigned int IBC_Faults_Data;

///////
unsigned short statusFlagsUSG;

unsigned short temp;

unsigned char EEPWrite[5], EEPRead[5];
unsigned char Production_ISC_SN_Counter[2];

void InitSystem(void);
inline void WaitNewTick(void);
void WaitTickCount(unsigned short);
void ReadKeySwitch(void);                                                       //Read the status of the key switch
unsigned short ReadAnalogVoltage(unsigned char);                                //Read either the key or el test voltage
void ReadEarthLeakage(void);
void ReadCableFault(void);
unsigned char ReadMAINS_ZeroCrossing(void);
void ReadFireOut(void);
void TurnRelayOn(void);
void TurnRelayOff(void);
void TurnSSR1On(void);
void TurnSSR1Off(void);
void TurnCableFaultDetectOn(void);
void TurnCableFaultDetectOff(void);
void TurnBuzzer_AlarmOn(void);
void TurnBuzzer_AlarmOff(void);
unsigned char getKeySwitchState(void);
unsigned char getFireButtonState(void);
unsigned char getResetButtonState(void);
void GenerateMissingPulse(void);
void checkCableFaults(void);
void FIRE(void);
void ResetDeArmedTimer(void);
void StartDeArmedTimer(void);
void StartPostFireTimer(void);
void StopPostFireTimer(void);
void StartBlastTimer(void);
void StopBlastTimer(void);
void StartPreArmedTimer(void);
void StopPreArmedTimer(void);
void Construct_ISC_Packet(void);
void Receive_ISC_Packet(void);
void Construct_Pi_Packet(void);
void Receive_Pi_Packet(void);
unsigned short getISC_SN(unsigned char ISC_Index);
void BroadcastGetISC_SNumbers(void);
void GetIB651_SNumbers(unsigned short ISC_SN);
void Add_ISC_SN(unsigned short ISC_SN);
unsigned char Check_Clash_ISC_SN(unsigned short ISC_SN);
void Update_Open_Relay(unsigned short ISC_SN);
void Update_Close_Relay(unsigned short ISC_SN);
void Update_Ping_Command(unsigned short ISC_SN);
void Update_ARM_ISC(unsigned short ISC_SN);
void Update_DISARM_ISC(unsigned short ISC_SN);
void Update_ISC_Cable_Fault(unsigned short ISC_SN);
void Update_Default_Data(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf);
void Update_DC_Data(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf);
void Update_AC_Data(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf);
void Update_Blast_Loop_Data(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf);
void Update_Serial_Number(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf);
unsigned char getISC_Index(unsigned short ISC_SN);
void Inspect_Default_Data_Packet(unsigned short Data_Length);
unsigned char Check_ISC_Cable_Faults(void);
unsigned short getIBCDefaultData(void);
void Transmit_Pi_Default_Data(void);
unsigned short getParentForISC(unsigned short ISC_SN);
void AddParentForISC(unsigned short ISC_SN, unsigned short ISC_PARENT_SN);
void DEBUG_Print_Packet(void);
void Check_ALL_ISC(void);
void EEPROMTest(void);
void EEPROMWrite(void);
void EEPROMRead(void);
void Assign_ISC_New_SN(void);
void CheckBroadcastPacket(unsigned char Command);
void Transmit_NULL_Packet(void);
void Transmit_BLAST_Command_Packet(void);
void Reverse_ISC_SN_List(void);
void Reverse_Data(unsigned char TypeUC);

void Write_b_eep( unsigned int badd,unsigned char bdat );
unsigned char Read_b_eep( unsigned int badd );
void Busy_eep ( void );

extern char UART_Init(const long int baudrate);
extern char UART_Read(void);
extern void UART_Write(char data);
extern void UART_TX_ISRHandler(void);
extern void CreateMessageUART(unsigned short packetSourceUS, unsigned char commandUC, unsigned char dataLenUC, char *dataBuf);
extern void SendUARTPacket(void);
extern short PacketReadParamUART(unsigned char paramName);
extern char CheckCRCUART(void);

extern unsigned char InitST7540(void);
extern void SPIISRHandlerST7540(void);
extern void CreateMessageST7540(unsigned short, unsigned short, unsigned char, unsigned char, char *);
extern void StartTransmitST7540(void);
extern unsigned char TransmitBusyST7540(void);
extern void RXReadyISRHandlerST7540(void);
extern void ReceiveNewDataST7540(void);
extern unsigned char DataReadyST7540(void);
extern unsigned short PacketReadParamST7540(unsigned char);
extern unsigned char CheckCRCST7540(void);
extern unsigned short CRC16(char *, unsigned short);
extern void ClearDataReady(void);

extern void CreateMessageUARTTest(unsigned char Command);
extern void CreatePiToIBCUARTMessage(unsigned char Command, unsigned short packetSourceUS);
extern void CreateMessageUARTDEBUG(unsigned char dataLenUC,unsigned short packetSourceUS, unsigned short packetDestUS,unsigned char packetNOUC,unsigned char commandUC,char *dataBuf);

void interrupt isr(void){

    if(PIR1bits.TMR2IF != 0){
        //LAT_TEST_LED = 1;
	PIR1bits.TMR2IF = 0;
        statusFlagsUSG |= FLAG_TICK;        
        UART_TX_ISRHandler();        

        if (DataReadyST7540()){                                                     //We have a new packet
            
            Receive_ISC_Packet();
            Construct_Pi_Packet();
            
            if (PacketForPiIdentifier != 0b11111111){                               //We have a packet ready for the dispatcher
                SendUARTPacket();
                PacketForPiIdentifier = 0b11111111;
            }
            
            IdleTimerUIG = 0;           
        }

        if (CF_STAUTS_TimerUIG < (20000)){   //10 seconds
            CF_STAUTS_TimerUIG++;
            }

        if (DeArmHighTickUIG < 2000){
            DeArmHighTickUIG++;             
        }
        else{           
            DeArmHighTickUIG = 0;
         if (DeArmTimerEnabled){
                DeArmTickCG++;                
            }
        }

        if (CFLEDFlashTickUIG < 1000)
            CFLEDFlashTickUIG++;
        else{
            CFLEDFlashTickUIG = 0;
            if (CFLEDFlashEnable)
                CFLEDFlashState = !CFLEDFlashState;
        }
        //////////////////////////////////////////////////////////////////////////////////
        if (BTEnabled){
            BTHighSpeedTickUIG++;
        }
        else{
            BTHighSpeedTickUIG = 0;
            if (IdleTimerUIG > 2000){   //1 second
                //Transmit Null Packet down the line
                Transmit_NULL_Packet();
                IdleTimerUIG = 0;
            }
            else
                IdleTimerUIG++;


        if (CheckISCTimerUIG > (2000*10)){   //10 seconds
                Discover_ISC_ModeUCG = 1;
                Check_ALL_ISC();
                CheckISCTimerUIG = 0;
                IdleTimerUIG = 0;
            }
            else
                CheckISCTimerUIG++;
        }

        if (PATEnabled){
            if (HighSpeedTickPAUIG < 2000){ //2000 Ticks is 1 second: 500us per tick.
                HighSpeedTickPAUIG++;
            }
        else{
            HighSpeedTickPAUIG = 0;
            LowSpeedTickPATCG++;
            }
        }        
        //////////////////////////////////////////////////////////////////////////////////
        if (HighSpeedTickUIG < 2000){ //2000 Ticks is 1 second: 500us per tick.
            HighSpeedTickUIG++;
        }
        else{
            HighSpeedTickUIG = 0;
            LowSpeedTickCG++;
            if (PBTEnabled){
                LowSpeedTickPBTCG++;                
            }                        
        }
        
        if (LowSpeedTickPBTCG >= 60){     //Post Blast Timer
            AlarmStatusUCG = FAULT;
            CFLEDFlashEnable = 1;
        }
        if (LowSpeedTickPATCG >= 60){     //Pre Arm Timer
            AlarmStatusUCG = FAULT;
            CFLEDFlashEnable = 1;
        }
        if (DeArmTickCG >= 7){           //Amount of seconds the keyswitch needs to be in the disarmed position before firing is enabled
            ClearToFire = 1;              
        }
        else
            //LAT_DIAG1_LED = ON;
            Nop();

        if (LowSpeedTickCG >= 60)
            LowSpeedTickCG = 0;    
    }
    if(PIR3bits.SSP2IF){
        SPIISRHandlerST7540();        
    }
    if (PIR1bits.RC1IF){
        UART_Read();         
        PIR1bits.RC1IF = 0;
    }
    if (INTCONbits.RBIF){   //else if(neg pin change on SSX){
        INTCONbits.RBIF = 0;
        //LAT_TEST_LED = ON;
        RXReadyISRHandlerST7540();
    }
}

void main(void){

    InitSystem();
    Pi_Status_Update = CLEAR_UPDATE;

Discover_ISC_ModeUCG = 1;       //Now in Discovery mode
BroadcastGetISC_SNumbers();       //Works so far!!!

    
//        CreateMessageST7540(IBC_SN, 0x3130, CMD_OPEN_RELAY, 0, "");
//        StartTransmitST7540();
//        WaitTickCount(4000);
//        CreateMessageST7540(IBC_SN, 0x3130, CMD_CLOSE_RELAY, 0, "");
//        StartTransmitST7540();


LAT_CABLE_FAULT_LED = OFF;
WDTCONbits.SWDTEN = 1; //Enable WDT

while(1){        
        
//        CreateMessageST7540(IBC_SN, 0x3130, CMD_OPEN_RELAY, 0, "");
//            StartTransmitST7540();
//            WaitTickCount(500);
//        CreateMessageST7540(IBC_SN, 0x3130, CMD_CLOSE_RELAY, 0, "");
//            StartTransmitST7540();
    CLRWDT();
    //CONFIG2Hbits.WDTCON = 1000;

    if (Discovered_New_ISC == 1){   //We have discovered a new ISC through our ping
        Discovered_New_ISC = 0;
        Reverse_ISC_SN_List();
        CreateMessageUART(IBC_SN, CMD_PI_SN_ISCS, ISC_SN_Array_SizeUCG * 2, ISC_SN_Array_ReverseUSG);
        PacketForPiIdentifier = CMD_PI_SN_ISCS;
    }
        
        if (PacketForPiIdentifier != 0b11111111){
         SendUARTPacket();
         PacketForPiIdentifier = 0b11111111;
        }

        
        if (DataReadyUART()){ //We have received a packet, now process the packet            
            ClearDataReady();
            if(PacketReadParamUART(UART_CRC_VALID)){
                Receive_Pi_Packet();                
            }
        }

        if (ISC_Transmit_Packet_Ready == 1){
            if (!TransmitBusyST7540() && LineIdleST7540()){  //Only allowed to transmit if the we are not already transmitting and not receiving
                StartTransmitST7540();
                ISC_Transmit_Packet_Ready = 0;
                IdleTimerUIG = 0;
                LAT_DIAG1_LED = !LAT_DIAG1_LED;
            }
        }

        if (Pi_Status_Update == DEFAULT_UPDATE){    //Default Data has updated and we require letting the Pi know            
            Pi_Status_Update = CLEAR_UPDATE;
            getIBCDefaultData();
            Transmit_Pi_Default_Data();
        }
        if (Pi_Status_Update == ERROR_UPDATE){      //An error has occured and we need to let the Pi know of the error

            Pi_Status_Update = CLEAR_UPDATE;
        }

        if (CableFaultStatusUCG == CABLEFAULT || CableFaultStatusUCG == EARTHFAULT){      //We are in a cable fault state, check
        //We are in a fault status
        //Do we have timeout yet? If so, turn on relay and measure
            if (CF_STAUTS_TimerUIG > 19999){
                TurnRelayOn();
                WaitTickCount(100);
                checkCableFaults();
                WaitTickCount(100);
                TurnRelayOff();
                CF_STAUTS_TimerUIG = 0;
            }

        }
        else //We are not in a fault state,
            checkCableFaults();

        if (CableFaultStatusUCG == CABLEFAULT || CableFaultStatusUCG == EARTHFAULT || Check_ISC_Cable_Faults() == FAULT){  //We have a cable fault
        //if (CableFaultStatusUCG == CABLEFAULT ){  //We have a cable fault
            if (NewCFStatusUCG == 1){   //We have a new cable fault
                NewCFStatusUCG = 0;
                Pi_Status_Update = DEFAULT_UPDATE;
                AlarmStatusUCG = FAULT;     //Turn on general alarm
            }
            LAT_CABLE_FAULT_LED = ON;   //Have cable fault so turn LED on
            TurnRelayOff();             //Turn Relay off to stop fault melting things
        }
        else{                           //We dont have a cable fault any more
            if (CFLEDFlashEnable == 0 && AlarmStatusUCG == CLEAR){
                LAT_CABLE_FAULT_LED = OFF;
                TurnRelayOn();          //Fault Clear, turn relay back on
                CF_STAUTS_TimerUIG = 0;
                if (NewCFStatusUCG == 0){   //We no longer have cable fault
                //NewCFStatusUCG = 0;
                NewCFStatusUCG = 1;        //Fault is cleared, hence next time a fault occurs, it is a new fault
                Pi_Status_Update = DEFAULT_UPDATE;
                }
            }                        
        }
        
        if (getResetButtonState() == PRESSED){     //Reset button is pressed    
            if (getKeySwitchState() == DISARMED){
               AlarmStatusUCG = CLEAR;  //Clear the general alarm
               if (CableFaultStatusUCG == CLEAR){ //If the cable fault is clear?
                    LAT_CABLE_FAULT_LED = OFF;  //Turn the LED off
                    TurnRelayOn();          //Fault Clear, turn relay back on
                    CFLEDFlashEnable = 0;
                    CFLEDFlashState = 0;
               }
               else
                   CF_STAUTS_TimerUIG = 20000;
            }
        }        

        if (getKeySwitchState() == DISARMED){
            if(KeySwitchStatusUCG == ARMED)
                Pi_Status_Update = DEFAULT_UPDATE;
                KeySwitchStatusUCG = DISARMED;
                StopPostFireTimer();
                StopPreArmedTimer();
                //if (AlarmStatusUCG == CLEAR && CableFaultStatusUCG == CLEAR)
                    //StartDeArmedTimer();
                
            }
        else { //Keyswitch is in the Armed State
            if(KeySwitchStatusUCG == DISARMED)
                Pi_Status_Update = DEFAULT_UPDATE;
            KeySwitchStatusUCG = ARMED;
            if (PATEnabled == 0)    //Check if the Armed state timer is running yet
                StartPreArmedTimer();   //If not, start the timer
                //ResetDeArmedTimer();
                ClearToFire = 1;
        }
        
        if (getFireButtonState() == PRESSED){             //The fire button has been pressed
            if (FireButtonStatusUCG == DEPRESSED)
                Pi_Status_Update = DEFAULT_UPDATE;
            FireButtonStatusUCG = PRESSED;
            //Check for Cable Fault or if Keyswitch is in the ARMED Position
            if (CableFaultStatusUCG == FAULT){
                 AlarmStatusUCG = FAULT;
            }
            //Check if the KeySwitch is in the DISARMED state
            else if (getKeySwitchState() == DISARMED){
                AlarmStatusUCG = FAULT;
                CFLEDFlashEnable = 1;
            }
            else if (ClearToFire == 0){
                AlarmStatusUCG = FAULT;
                CFLEDFlashEnable = 1;
            }
            //Else all faults are clear and we can fire
            else if (AlarmStatusUCG == CLEAR && ClearToFire == 1){
                ClearToFire = 0;
                //Blasting and therefore stop the prearmed timer
                StopPreArmedTimer();
                //Change the relay over - ie OFF
                TurnRelayOff(); //Patch Mains through
                //Wait long enough for relay to close so we can read the mains voltage before firing
                WaitTickCount(100);
                TurnSSR1On();
                WaitTickCount(100);
                //WaitTickCount(10000);    //Debug
                //WaitTickCount(10000);
                //WaitTickCount(10000);
                FireOutFloat = 0;
                ReadFireOut();
                //if (FireOutFloat > 400)
                    //LAT_TEST_LED = 1;
                FireOutFloat = 1000.0; //debug
                TurnSSR1Off();

                if (FireOutFloat > 815.0){                    
                    //Fire
                    Transmit_BLAST_Command_Packet();
                    //LAT_TEST_LED = ON;
                    CLRWDT();
                    WaitTickCount(2000*4);    //Delay so that the booster comms line goes down
                    CLRWDT();
                    //BTEnabled = 1;
                    FIRE();
                    //LAT_TEST_LED = OFF;
                    //WaitTickCount(1000);
                    //BTEnabled = 0;
                }
                else{
                    AlarmStatusUCG = FAULT;
                }
                //Always Turn SSR OFF as precautionary measure
                TurnSSR1Off();
                //We have fired and required Disarm prior to refiring
                
                //After Firing turn the relay back
                TurnRelayOn(); //Keep Mains off of the output                
            }
        }
        else{
            if (FireButtonStatusUCG == PRESSED)
                Pi_Status_Update = DEFAULT_UPDATE;
            FireButtonStatusUCG = DEPRESSED;
        }
        //if (CFLEDFlashEnable == 1 && CableFaultStatusUCG == CLEAR)
            //LAT_CABLE_FAULT_LED = CFLEDFlashState;

        if (AlarmStatusUCG == 1){
            
            if (CFLEDFlashEnable == 1){
                if (CFLEDFlashState == 1)
                    TurnBuzzer_AlarmOn();
                if (CFLEDFlashState == 0)
                    TurnBuzzer_AlarmOff();
            }
            else
                TurnBuzzer_AlarmOn();
        }
        else{
            TurnBuzzer_AlarmOff();
        }
    } //END WHILE LOOP
}

void Construct_ISC_Packet(void){
    
}

void Receive_ISC_Packet(void){

    LAT_DIAG2_LED = !LAT_DIAG2_LED;
            if(PacketReadParamST7540(ST7540_CRC_VALID)){                            //CRC passes and the data is valid
                //DEBUG_Print_Packet();
                ReceiveNewDataST7540();
                CheckBroadcastPacket(PacketReadParamST7540(ST7540_CMD));
                if (BroadcastCheckUC == 0){
                switch(PacketReadParamST7540(ST7540_CMD)){                          //Extract the command from the Packet
                    case(CMD_SEND_DEFAULT):                                               //Receiving Default Values
                        Update_Default_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_FORCE_DEFAULT):                                               //Receiving Default Values
                        Update_Default_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_SEND_DC):                                                    //Receiving DC Value
                        Update_DC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    //case(CMD_FORCE_DC):                                                    //Receiving DC Value
                        //Update_DC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        //return;
                    case(CMD_FORCE_DC):                                                    //Receiving DC Value
                        Update_DC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_SEND_AC):                                                    //Receiving AC Value
                        Update_AC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_FORCE_AC):                                                    //Receiving AC Value
                        Update_AC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_SEND_BLAST_VALUE):                                           //Receiving Blast Circuit Value
                        Update_Blast_Loop_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_FORCE_BLAST_VALUE):                                           //Receiving Blast Circuit Value
                        Update_Blast_Loop_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    //case(CMD_SEND_TEMPERATURE):                                           //Receiving Temeperatures
                        //return;
                    case(CMD_OPEN_RELAY):                                                 //Receiving open relay ack
                        Update_Open_Relay(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(CMD_CLOSE_RELAY):                                                //Receiving closed relay ack
                        Update_Close_Relay(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(CMD_GET_SN):                                                     //Receiving serial no of ISC/IB651s
                        //if (PacketReadParamST7540(ST7540_DATA_LEN) == 0)
                            //Add_ISC_SN(PacketReadParamST7540(ST7540_SOURCE));
                        //else{                                                             //Then we have some serial numbers of boosters attached to an ISC
                            Update_Serial_Number(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        //}
                        return;
                    case(CMD_SN_LIST_CHANGED):                                            //Receiving serial no list has changed
                        GetIB651_SNumbers(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(CMD_BLAST_COMMAND):                                              //Receiving ACK blast command
                        return;
                    case(CMD_ISC_NEW_SN):
                        Add_ISC_SN(PacketReadParamST7540(ST7540_SOURCE));

                        if (New_ISC_SN == 0){       //Not a new serial number, therefore a PING
                            Update_Ping_Command(PacketReadParamST7540(ST7540_SOURCE));
                        }
                        else{ //It is a new serial number, lets broadcast ping again to get a new one
                            New_ISC_SN = 0;
                            //BroadcastGetISC_SNumbers();
                        }
                        return;
                    case(PING_ISC):                                                       //Receiving ACK for PING
                        if (PacketReadParamST7540(ST7540_SOURCE) == ISC_DEFAULT_SN){                   //An ISC with no SN has replied. We need to give it a serial number
                            Assign_ISC_New_SN();
                            return;
                        }

                        Add_ISC_SN(PacketReadParamST7540(ST7540_SOURCE));
                        if (New_ISC_SN == 0){       //Not a new serial number, therefore a PING
                            if (Discover_ISC_ModeUCG == 0)
                            Update_Ping_Command(PacketReadParamST7540(ST7540_SOURCE));
                        }
                        else{ //It is a new serial number, lets broadcast ping again to get a new one
                            New_ISC_SN = 0;
                                if (Discover_ISC_ModeUCG == 1)
                                    Discovered_New_ISC = 1;
                            //BroadcastGetISC_SNumbers();
                        }
                        //LAT_TEST_LED = !LAT_TEST_LED;
                        return;
                    
                    case(ARM_ISC):                                                        //Receiving ACK for ARMING of ISC
                        Update_ARM_ISC(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(DISARM_ISC):                                                     //Receiving ACK for DISARMING of ISC
                        Update_DISARM_ISC(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(CMD_CABLE_FAULT):                                                //Receiving Cable fault from ISC
                        Update_ISC_Cable_Fault(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                }
                }

                else{//Broadcast Messages

                    if (PacketReadParamST7540(ST7540_DEST) != IBC_SN){  //If packet is not for ISC-1, bounce packet
                        PacketForPiIdentifier = CMD_NULL;
                        return;
                    }

                    switch(PacketReadParamST7540(ST7540_CMD)){                          //Extract the command from the Packet
                    case(CMD_SEND_DEFAULT_B):                                               //Receiving Default Values
                        Update_Default_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_FORCE_DEFAULT_B):                                               //Receiving Default Values
                        Update_Default_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_SEND_DC_B):                                                    //Receiving DC Value
                        Update_DC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    //case(CMD_FORCE_DC):                                                    //Receiving DC Value
                        //Update_DC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        //return;
                    case(CMD_FORCE_DC_B):                                                    //Receiving DC Value
                        Update_DC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_SEND_AC_B):                                                    //Receiving AC Value
                        Update_AC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_FORCE_AC_B):                                                    //Receiving AC Value
                        Update_AC_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_SEND_BLAST_VALUE_B):                                           //Receiving Blast Circuit Value
                        Update_Blast_Loop_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    case(CMD_FORCE_BLAST_VALUE_B):                                           //Receiving Blast Circuit Value
                        Update_Blast_Loop_Data(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        return;
                    //case(CMD_SEND_TEMPERATURE):                                           //Receiving Temeperatures
                        //return;
                    case(CMD_OPEN_RELAY_B):                                                 //Receiving open relay ack
                        Update_Open_Relay(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(CMD_CLOSE_RELAY_B):                                                //Receiving closed relay ack
                        Update_Close_Relay(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(CMD_GET_SN_B):                                                     //Receiving serial no of ISC/IB651s
                        //if (PacketReadParamST7540(ST7540_DATA_LEN) == 0)
                            //Add_ISC_SN(PacketReadParamST7540(ST7540_SOURCE));
                        //else{                                                             //Then we have some serial numbers of boosters attached to an ISC
                            Update_Serial_Number(PacketReadParamST7540(ST7540_SOURCE),PacketReadParamST7540(ST7540_DATA_LEN),PacketDataST7540());
                        //}
                        return;
                    case(CMD_SN_LIST_CHANGED_B):                                            //Receiving serial no list has changed
                        GetIB651_SNumbers(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(CMD_BLAST_COMMAND_B):                                              //Receiving ACK blast command
                        return;
                    case(CMD_ISC_NEW_SN_B):
                        Add_ISC_SN(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(PING_ISC_B):                                                       //Receiving ACK for PING
                        if (PacketReadParamST7540(ST7540_SOURCE) == ISC_DEFAULT_SN){                   //An ISC with no SN has replied. We need to give it a serial number
                            Assign_ISC_New_SN();
                            return;
                        }

                        Add_ISC_SN(PacketReadParamST7540(ST7540_SOURCE));
                        if (New_ISC_SN == 0){       //Not a new serial number, therefore a PING
                            if (Discover_ISC_ModeUCG == 0)
                                Update_Ping_Command(PacketReadParamST7540(ST7540_SOURCE));
                        }
                        else{ //It is a new serial number, lets broadcast ping again to get a new one
                            New_ISC_SN = 0;
                            CreateMessageST7540(IBC_SN, PacketReadParamST7540(ST7540_SOURCE), CMD_CLOSE_RELAY, 0, "");
                            StartTransmitST7540();
                            WaitTickCount(200); //Get Message out before next message
                            BroadcastGetISC_SNumbers();
                        }
                        //LAT_TEST_LED = !LAT_TEST_LED;
                        return;
                    case(ARM_ISC_B):                                                        //Receiving ACK for ARMING of ISC
                        Update_ARM_ISC(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(DISARM_ISC_B):                                                     //Receiving ACK for DISARMING of ISC
                        Update_DISARM_ISC(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                    case(CMD_CABLE_FAULT_B):                                                //Receiving Cable fault from ISC
                        Update_ISC_Cable_Fault(PacketReadParamST7540(ST7540_SOURCE));
                        return;
                }
            }
        }
}

void Receive_Pi_Packet(void){   //We have received a request from the Pi to aquire data from an ISC
    unsigned char BroadcastUC;
//CreateMessageST7540(unsigned short packetSourceUS, unsigned short packetDestUS, unsigned char commandUC, unsigned char dataLenUC, char *dataBuf)
temp = PacketReadParamUART(UART_HEADER);
//We have received a packet from the PI, now we need to extract the info and perform the relevant task
CheckBroadcastPacket(PacketReadParamUART(UART_CMD));

    switch(PacketReadParamUART(UART_CMD)){//Extract the command from the Packet
        case(CMD_PI_DEFAULT_DATA):
            //Get IB651 Default Values
            BroadcastUC = CMD_SEND_DEFAULT;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;         
            return;
        case(CMD_PI_DEFAULT_DATA_B):
            //Get IB651 Default Values
            BroadcastUC = CMD_SEND_DEFAULT;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_FORCE_DEFAULT):
            //Get IB651 Default Values
            BroadcastUC = CMD_FORCE_DEFAULT;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_DC_DATA):                                                    
            //Get IB651 DC Values
            BroadcastUC = CMD_SEND_DC;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_FORCE_DC):
            //Get IB651 DC Values
            BroadcastUC = CMD_FORCE_DC;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_AC_DATA):                                                    
            //Get IB651 AC Values
            BroadcastUC = CMD_SEND_AC;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;            
            return;
        case(CMD_PI_FORCE_AC):
            //Get IB651 AC Values
            BroadcastUC = CMD_FORCE_AC;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_BLAST_VALUE):                                           
            //Get IB651 Blast Circuit Values
            BroadcastUC = CMD_SEND_BLAST_VALUE;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_FORCE_BLAST_VAL):
            //Get IB651 Blast Circuit Values
            BroadcastUC = CMD_FORCE_BLAST_VALUE;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_IBC_DEFAULT):
            //Return own data - Return default data of the IB
            //BroadcastUC = CMD_FORCE_DC;
            //BroadcastUC |= 0b10000000;
            CreateMessageUART(IBC_SN, CMD_PI_IBC_DEFAULT, 1, IBC_Default_Data);
            PacketForPiIdentifier = CMD_PI_IBC_DEFAULT;
            return;
        case(CMD_PI_IBC_ERRORS):
            //Return own data - Return error data of the IBC to the Pi
            CreateMessageUART(IBC_SN, CMD_PI_IBC_ERRORS, 1, IBC_Faults_Data);
            PacketForPiIdentifier = CMD_PI_IBC_ERRORS;
            return;
        case(CMD_PI_SN_ISCS):
            //Ordered list of ISC's, stored on the IBC - No downstream request required
            Reverse_ISC_SN_List();
            CreateMessageUART(IBC_SN, CMD_PI_SN_ISCS, ISC_SN_Array_SizeUCG * 2, ISC_SN_Array_ReverseUSG);
            PacketForPiIdentifier = CMD_PI_SN_ISCS;
            return;
        case(CMD_PI_SN_IB651):
            //Get serial number list of the IB651s connected to particular ISC
            BroadcastUC = CMD_GET_SN;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_BLAST_PERMISSION):
            //Permission to blast granted
            return;
        case(CMD_PI_BLAST_DENIED):
            //Permission to blast denied
            return;
        case(CMD_PI_BLAST_ACK):                                              
            //Blast Command - Probably not going to be used
            return;
        case(CMD_PI_PING_ISC):                                                       
            //Ping the relevant ISC
            BroadcastUC = PING_ISC;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            Discover_ISC_ModeUCG = 0;
            CF_STAUTS_TimerUIG = 0;
            return;
        case(CMD_PI_PING_ISC_B):
            //Ping the relevant ISC
            BroadcastUC = PING_ISC;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_ARM_ISC):                                                        
            //Arm an ISC
            BroadcastUC = ARM_ISC;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_DISARM):                                                     
            //Disarm an ISC
            BroadcastUC = DISARM_ISC;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(IBC_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            ISC_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_ISC_PARENT):
            CreateMessageUART(PacketReadParamUART(UART_SN), CMD_PI_ISC_PARENT, 2, getParentForISC(PacketReadParamUART(UART_SN)));
            return;
    }//END SWITCH
}

void Construct_Pi_Packet(void){

    switch(PacketForPiIdentifier){                          //Extract the command from the Packet
        case(CMD_SEND_DEFAULT):                                               //Receiving Default Values
            Reverse_Data(DATA_STATUS);            
            //CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_DEFAULT_DATA, boosterCommsData[DATA_LENGTH][ISC_Packet_Index], boosterCommsData[DATA_STATUS]);
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_DEFAULT_DATA, boosterCommsData[DATA_LENGTH][ISC_Packet_Index], boosterCommsDataReverse);
            return;
        case(CMD_SEND_DC):                                                    //Receiving DC Value
            Reverse_Data(DATA_DC_VOLT);
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_DC_DATA, boosterCommsData[DATA_LENGTH][ISC_Packet_Index], boosterCommsDataReverse);
            return;
        case(CMD_SEND_AC):                                                    //Receiving AC Value
            Reverse_Data(DATA_AC_VOLT);
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_AC_DATA, boosterCommsData[DATA_LENGTH][ISC_Packet_Index], boosterCommsDataReverse);
            return;
        case(CMD_SEND_BLAST_VALUE):                                           //Receiving Blast Circuit Value
            Reverse_Data(DATA_BL_VOLT);
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_BLAST_VALUE, boosterCommsData[DATA_LENGTH][ISC_Packet_Index], boosterCommsDataReverse);
            return;
        case(CMD_OPEN_RELAY):                                                 //Receiving open relay ack
            //CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_OPEN_RELAY, 0,"");
            return;
        case(CMD_CLOSE_RELAY):                                                //Receiving closed relay ack
            //CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_CLOSE_RELAY, 0,"");
            return;
        case(CMD_GET_SN):            
            //Receiving serial no of ISC/IB651s
            //for(int dataBufLocUC = 0; dataBufLocUC < boosterCommsData[DATA_LENGTH][ISC_Packet_Index]; dataBufLocUC++)             //Packet payload
                //boosterCommsDataTemp[dataBufLocUC] = boosterCommsData[DATA_SERIAL_NUMBER][dataBufLocUC];//TransmitPiBufferCG[dataBufLocUC + 9] = dataBuf[dataBufLocUC];
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_SN_IB651, boosterCommsData[DATA_LENGTH][ISC_Packet_Index], boosterCommsData[DATA_SERIAL_NUMBER]);
            return;
        case(CMD_SN_LIST_CHANGED):                                            //Receiving serial no list has changed
            //Finish
            return;
        case(CMD_BLAST_COMMAND):                                              //Receiving ACK blast command
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_BLAST_ACK, 0,"");
            return;
        case(PING_ISC):                                                       //Receiving ACK for PING
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_PING_ISC, 0,"");
            return;
        case(ARM_ISC):                                                        //Receiving ACK for ARMING of ISC
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_ARM_ISC, 0,"");
            return;
        case(DISARM_ISC):                                                     //Receiving ACK for DISARMING of ISC
            CreateMessageUART(getISC_SN(ISC_Packet_Index), CMD_PI_DISARM, 0,"");
            return;
        case(CMD_CABLE_FAULT):                                                //Receiving Cable fault from ISC

            return;
    }
}

void Transmit_BLAST_Command_Packet(void){
    if (!TransmitBusyST7540() && LineIdleST7540()){  //Only allowed to transmit if the we are not already transmitting and not receiving
        CreateMessageST7540(IBC_SN, ISC_SN_BROADCAST_ADD, CMD_BLAST_COMMAND, 0, "");
                StartTransmitST7540();
                ISC_Transmit_Packet_Ready = 0;

    //CreateMessageST7540(IBC_SN, ISC_NULL_SN, PING_ISC, 0, "");
    //ISC_Transmit_Packet_Ready = 1;
    }
}

void Transmit_NULL_Packet(void){
    CreateMessageST7540(IBC_SN, ISC_NULL_SN, PING_ISC, 0, "");
    ISC_Transmit_Packet_Ready = 1;
}

void Check_ALL_ISC(void){
    CreateMessageST7540(IBC_SN, ISC_SN_BROADCAST_ADD, PING_ISC, 0, "");
    ISC_Transmit_Packet_Ready = 1;
}

void DEBUG_Print_Packet(void){
    LAT_TEST_LED = ON;
 CreateMessageUARTDEBUG(PacketReadParamST7540(ST7540_DATA_LEN),PacketReadParamST7540(ST7540_SOURCE), PacketReadParamST7540(ST7540_DEST),PacketReadParamST7540(ST7540_NUMBER),PacketReadParamST7540(ST7540_CMD),PacketDataST7540());
 SendUARTPacket();
 //PacketForPiIdentifier = 0b11111111;
 LAT_TEST_LED = OFF;
 //WaitTickCount(50);
}

void Transmit_Pi_Default_Data(void){
    CreateMessageUART(IBC_SN, CMD_PI_IBC_DEFAULT, 2, PiDefaultData);
    PacketForPiIdentifier = 0b11111110;
}

void Reverse_ISC_SN_List(void){
    unsigned char TempChar;
    unsigned short TempShort;   

    for (int i = 0; i < ISC_SN_Array_SizeUCG; i++){
        TempShort = ISC_SN_ArrayUIG[i];       
        TempChar = (TempShort >> 8);
        ISC_SN_Array_ReverseUSG[i*2] = TempChar;
        TempChar = TempShort & 0b11111111;
        ISC_SN_Array_ReverseUSG[(i*2)+1] = TempChar;
    }

}

void Reverse_Data(unsigned char TypeUC){
   
   
    for (int i = 0; i < boosterCommsData[DATA_LENGTH][ISC_Packet_Index]; i+=2){

        boosterCommsDataReverse[i] = boosterCommsData[TypeUC][i + 1];
        boosterCommsDataReverse[i + 1] = boosterCommsData[TypeUC][i];

    }
}

void CheckBroadcastPacket(unsigned char Command){

 BroadcastCheckUC = (Command & 0b10000000);
}

void Assign_ISC_New_SN(void){
    unsigned char ISC_NEW_SN[2];
    //Method
    EEPROMRead();
    //ISC_NEW_SN[0] = 0x30;
    //ISC_NEW_SN[1] = 0x31;
    ISC_NEW_SN[0] = EEPRead[0]; //MSB
    ISC_NEW_SN[1] = EEPRead[1]; //LSB
    //Get Last ISC_SN from EEPROM

    if (EEPRead[1] == 0xFF){ //We need to roll over LSB and increment MSB
        ISC_NEW_SN[1] = EEPRead[1] + 1;
        ISC_NEW_SN[0] = 0x00;
    }
    else{
        ISC_NEW_SN[0] = EEPRead[0]; //MSB
        ISC_NEW_SN[1] = EEPRead[1] + 1; //LSB
    }

    EEPWrite[0] = ISC_NEW_SN[0];
    EEPWrite[1] = ISC_NEW_SN[1];
    EEPROMWrite();
    //Increment ISC_SN in EEPROM
    //Send ISC_SN down to ISC without a SN - PING COMMAND WITH SN in DATA Field?
    CreateMessageST7540(IBC_SN, ISC_DEFAULT_SN, CMD_ISC_NEW_SN, 2, ISC_NEW_SN);
    ISC_Transmit_Packet_Ready = 1;
}

unsigned short getParentForISC(unsigned short ISC_SN){
    //return ISC_SN_ArrayUIG[getISC_Index(ISC_SN)][1];                                  //[X][1] is the Parent ISC SN
    return ISC_PARENT_SNArrayUIG[getISC_Index(ISC_SN)];
}

void AddParentForISC(unsigned short ISC_SN, unsigned short ISC_PARENT_SN){
    ISC_PARENT_SNArrayUIG[getISC_Index(ISC_SN)] = ISC_PARENT_SN;
}

unsigned short getIBCDefaultData(void){
    unsigned short returningData = 0b00000000;

    if (KeySwitchStatusUCG)
        returningData |= 0b10000000;        //Keyswitch Status
    if (RelayStatusUCG)                     //Isolation Relay Status
        returningData |= 0b01000000;
    if (FireButtonStatusUCG)                //Fire Button Status - COMPLETE STILL
        returningData |= 0b00100000;
    if (CableFaultStatusUCG == CABLEFAULT)  //Cable Fault
        returningData |= 0b00010000;
    if (CableFaultStatusUCG == EARTHFAULT)  //Earth Leakage Fault
        returningData |= 0b00001000;
        PiDefaultData[0] = 0b01010101;
        PiDefaultData[1] = returningData;
    return returningData;
}

void BroadcastGetISC_SNumbers(void){
    CreateMessageST7540(IBC_SN, ISC_SN_BROADCAST_ADD, PING_ISC, 0, "");
    StartTransmitST7540();
    WaitTickCount(250);
}

void GetIB651_SNumbers(unsigned short ISC_SN){
    CreateMessageST7540(IBC_SN, ISC_SN, CMD_GET_SN, 0, "");
    StartTransmitST7540();
}

unsigned char Check_ISC_Cable_Faults(void){
    unsigned char data;
    unsigned char CF;
    unsigned char EL;
    unsigned char Fault = 0;

    data = boosterCommsData[DATA_STATUS][1];                        //ISC Index is always the first index
    EL = (data & 0b0000100) >> 2;
    CF = (data & 0b0001000) >> 3;
    if (EL == 1 || CF == 1){
        Fault = 1;
        ISC_CF_Index = ISC_Packet_Index;
    }
    else{                                   //No Fault
        if (ISC_CF_Index == ISC_Packet_Index)
            ISC_CF_Index = 0;
    }

    if (ISC_CF_Index > 0)                  //There are no cable faults below
        return 1;
    else
        return 0;
}

void Inspect_Default_Data_Packet(unsigned short Data_Length){
    unsigned char index = 0;   
    unsigned char data;
    unsigned char CF;
    unsigned char EL;
    unsigned char Fault;

    //index = getISC_Index(ISC_SN);

    for(index = 0; index < Data_Length / 2; index++){                     //Packet payload
        data = boosterCommsData[DATA_STATUS][(index * 2) + 1];
        EL = data & 0b0000100;
        EL = EL >> 2;
        CF = data & 0b0001000;
        CF = data >> 3;
        if (EL == 1 || CF == 1)
            Fault = 1;
    }
}

void Update_Open_Relay(unsigned short ISC_SN){
    ISC_Packet_Index = getISC_Index(ISC_SN);
    PacketForPiIdentifier = CMD_OPEN_RELAY;
}

void Update_Close_Relay(unsigned short ISC_SN){
    ISC_Packet_Index = getISC_Index(ISC_SN);
    PacketForPiIdentifier = CMD_CLOSE_RELAY;
}

void Update_Ping_Command(unsigned short ISC_SN){
    ISC_Packet_Index = getISC_Index(ISC_SN);
    PacketForPiIdentifier = PING_ISC;
}

void Update_ARM_ISC(unsigned short ISC_SN){
    ISC_Packet_Index = getISC_Index(ISC_SN);
    PacketForPiIdentifier = ARM_ISC;
}

void Update_DISARM_ISC(unsigned short ISC_SN){
    ISC_Packet_Index = getISC_Index(ISC_SN);
    PacketForPiIdentifier = DISARM_ISC;
}

void Update_ISC_Cable_Fault(unsigned short ISC_SN){
    ISC_Packet_Index = getISC_Index(ISC_SN);
    PacketForPiIdentifier = CMD_CABLE_FAULT;
}

void Update_Default_Data(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf){                                                                 //Function to update the default data set
    unsigned char index = 0;
    unsigned char dataBufLocUC;

    index = getISC_Index(ISC_SN);

    for(dataBufLocUC = 0; dataBufLocUC < Data_Length; dataBufLocUC++)                     //Packet payload
        boosterCommsData[DATA_STATUS][dataBufLocUC] = dataBuf[dataBufLocUC];                     //PacketDataST7540();
        boosterCommsData[DATA_LENGTH][index] = Data_Length;
    ISC_Packet_Index = index;
    PacketForPiIdentifier = CMD_SEND_DEFAULT;
}

void Update_DC_Data(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf){                                                                 //Function to update the default data set
    unsigned char index = 0;
    unsigned char dataBufLocUC;

    index = getISC_Index(ISC_SN);

    for(dataBufLocUC = 0; dataBufLocUC < Data_Length; dataBufLocUC++)                     //Packet payload
        boosterCommsData[DATA_DC_VOLT][dataBufLocUC] = dataBuf[dataBufLocUC];                     //PacketDataST7540();
        boosterCommsData[DATA_LENGTH][index] = Data_Length;
    ISC_Packet_Index = index;
    PacketForPiIdentifier = CMD_SEND_DC;
}

void Update_AC_Data(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf){                                                                 //Function to update the default data set
    unsigned char index = 0;
    unsigned char dataBufLocUC;

    index = getISC_Index(ISC_SN);

    for(dataBufLocUC = 0; dataBufLocUC < Data_Length; dataBufLocUC++)                     //Packet payload
        boosterCommsData[DATA_AC_VOLT][dataBufLocUC] = dataBuf[dataBufLocUC];                     //PacketDataST7540();
        boosterCommsData[DATA_LENGTH][index] = Data_Length;
    ISC_Packet_Index = index;
    PacketForPiIdentifier = CMD_SEND_AC;
}

void Update_Blast_Loop_Data(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf){                                                                 //Function to update the default data set
    unsigned char index = 0;
    unsigned char dataBufLocUC;

    index = getISC_Index(ISC_SN);

    for(dataBufLocUC = 0; dataBufLocUC < Data_Length; dataBufLocUC++)                     //Packet payload
        boosterCommsData[DATA_BL_VOLT][dataBufLocUC] = dataBuf[dataBufLocUC];                     //PacketDataST7540();
        boosterCommsData[DATA_LENGTH][index] = Data_Length;
    ISC_Packet_Index = index;
    PacketForPiIdentifier = CMD_SEND_BLAST_VALUE;
}

void Update_Serial_Number(unsigned short ISC_SN, unsigned short Data_Length, char *dataBuf){              //CHECK IF WE NEED???                                                   //Function to update the default data set
    unsigned char index = 0;
    unsigned char dataBufLocUC;
    //unsigned short DataTemp;
    //unsigned char debug = 0;

    index = getISC_Index(ISC_SN);

    //for(index = 0; index < Data_Length / 2; index++){                     //Packet payload
        //data = boosterCommsData[DATA_STATUS][(index * 2) + 1];

//    for(dataBufLocUC = 0; dataBufLocUC < Data_Length / 2; dataBufLocUC++){                     //Packet payload
//        DataTemp = (dataBuf[dataBufLocUC * 2]) << 8;
//        DataTemp += dataBuf[(dataBufLocUC * 2) + 1];
//        boosterCommsData[DATA_SERIAL_NUMBER][dataBufLocUC] = DataTemp;                     //PacketDataST7540();
//    }

    for(dataBufLocUC = 0; dataBufLocUC < Data_Length; dataBufLocUC++){                     //Packet payload
        boosterCommsData[DATA_SERIAL_NUMBER][dataBufLocUC] = dataBuf[dataBufLocUC];        //PacketDataST7540();
    }
        boosterCommsData[DATA_LENGTH][index] = Data_Length;
    ISC_Packet_Index = index;
    //PacketForPiIdentifier = CMD_SN_LIST_CHANGED;
    PacketForPiIdentifier = CMD_GET_SN;
}

unsigned short getISC_SN(unsigned char ISC_Index){
    return ISC_SN_ArrayUIG[ISC_Index - 1];
}

unsigned char getISC_Index(unsigned short ISC_SN){
    //unsigned char index = 0;
    int index = 0;

    for (index = 0; index < ISC_SN_Array_SizeUCG; index++){
        if (ISC_SN_ArrayUIG[index] == ISC_SN)
        //if (ISC_SN_ArrayUIG[index][0] == ISC_SN)
            return index + 1;
    }

    if (index == 0)
        Add_ISC_SN(ISC_SN);
    return 1;
}

void Add_ISC_SN(unsigned short ISC_SN){

    if (ISC_SN_Array_SizeUCG == 0){                                                         //We have no ISCs listed
        ISC_SN_Array_SizeUCG++;
        ISC_SN_ArrayUIG[0] = ISC_SN;
        New_ISC_SN = 1; //It is a new serial number
        //ISC_SN_ArrayUIG[0][0] = ISC_SN;
    }
    else{                                                                                   //We already have an ISC listed
        if (Check_Clash_ISC_SN(ISC_SN) != 1){
            ISC_SN_ArrayUIG[ISC_SN_Array_SizeUCG] = ISC_SN;
            //ISC_SN_ArrayUIG[ISC_SN_Array_SizeUCG][0] = ISC_SN;
            ISC_SN_Array_SizeUCG++;
            New_ISC_SN = 1;                 //If we dont have a Clash, then it is a new Serial Number
        }
    }
}

unsigned char Check_Clash_ISC_SN(unsigned short ISC_SN){
    for (int i = 0; i < ISC_SN_Array_SizeUCG; i++){
        if (ISC_SN_ArrayUIG[i] == ISC_SN)
        //if (ISC_SN_ArrayUIG[i][0] == ISC_SN)
            return 1;
    }
    return 0;
}

void ResetDeArmedTimer(void){
    DeArmTimerEnabled = 0;
    ClearToFire = 0;
}

void StartDeArmedTimer(void){

    if (DeArmTimerEnabled == 0){
        DeArmHighTickUIG = 0;
        DeArmTimerEnabled = 1;
        DeArmTickCG = 0;        
    }
}

void StartPreArmedTimer(void){
    PATEnabled = 1;
    HighSpeedTickPAUIG = 0;
    LowSpeedTickPATCG = 0;    
}

void StopPreArmedTimer(void){
    PATEnabled = 0;
    HighSpeedTickPAUIG = 0;
    LowSpeedTickPATCG = 0;    
}

void StartPostFireTimer(void){  //Timer to count if the system is ARMED for too long after a blast
    PBTEnabled = 1;
    HighSpeedTickUIG = 0;
    LowSpeedTickPBTCG = 0;    
}

void StopPostFireTimer(void){
    PBTEnabled = 0;
    HighSpeedTickUIG = 0;
    LowSpeedTickPBTCG = 0;    
}

void StartBlastTimer(void){ //Timer to count if the Blast was too short
    //HighSpeedTickUIG = 0;
    BTEnabled = 1;
    //HighSpeedTickBTUIG = 0;
    LowSpeedTickBTCG = 0;    
}

void StopBlastTimer(void){
    BTEnabled = 0;
    HighSpeedTickUIG = 0;
    LowSpeedTickBTCG = 0;    
}

void checkCableFaults(void){

    ReadCableFault();
    ReadEarthLeakage();
    if (cableFaultProblemUCG == 1){                      //We have a Cable/EL Fault        
        CableFaultStatusUCG = CABLEFAULT;
    }

    else if (earthLeakageProblemUCG == 1){                      //We have a Cable/EL Fault
        CableFaultStatusUCG = EARTHFAULT;
    }

    else{
        CableFaultStatusUCG = CLEAR;
    }
}

void FIRE(void){

    while(ReadMAINS_ZeroCrossing() == 1);
                             //Burn off till first pulse edge comes through
    StartBlastTimer();
    for (int i = 0; i < 240; i++){
        CLRWDT();
        while(ReadMAINS_ZeroCrossing() == 0 && BTHighSpeedTickUIG < 160);                   //4 whole cycles missed
          GenerateMissingPulse();
          if (BTHighSpeedTickUIG > 160){
             AlarmStatusUCG = FAULT;
             CFLEDFlashEnable = 1;
             break;
          }
    }

    StopBlastTimer();
    StartPostFireTimer();
}

void GenerateMissingPulse(void){ //Lets through N Pulses

     WaitTickCount(8);     //Wait some time after the edge so we only turn on after ZC
     TurnSSR1On();    
    
     for (HighPulseCountCG = 0; HighPulseCountCG < HighCycle; HighPulseCountCG++){ //

         while(ReadMAINS_ZeroCrossing() == 1){                          //Burn off till pulse goes low
             //LowSpeedTickBTCG = 0;
             BTHighSpeedTickUIG = 0;
             if (HighPulseCountCG == (HighCycle - 1)){
                    WaitTickCount(6);
                    TurnSSR1Off();
                }
        }
        if (BTHighSpeedTickUIG < 160){
            while(ReadMAINS_ZeroCrossing() == 0 && BTHighSpeedTickUIG < 160){                          //Burn off till pulse goes high
            }
        }
     } //END for loop         
}

inline void WaitNewTick(void){                                                  //Wait for the start of a new tick
    while(!(statusFlagsUSG & FLAG_TICK));
    statusFlagsUSG &= ~FLAG_TICK;
}

void WaitTickCount(unsigned short tickCountUS){                                 //Wait a certain number of ticks
    while(tickCountUS--)
        WaitNewTick();
}

void InitSystem(void){

    OSCTUNEbits.PLLEN = 1;                                                      //Enable PLL
    OSCCONbits.IRCF = 0b111;                                                    //Setup Internal FOSC = 16MHz
    OSCCONbits.OSTS = 0;                                                        //Clock runs from Internal Oscillator
    OSCCONbits.SCS = 0b11;                                                      //Internal Oscillator block

    //Configure ADC
    ADCON2bits.ADFM = 1;                                                        //Right justify result
    ADCON2bits.ACQT = 0b010;                                                    //Sample for 4 TAD
    ADCON2bits.ADCS = 0b110;                                                    //Set to Fosc/64 clock
    ADCON0bits.ADON = 1;                                                        //Turn on the ADC

    //Setup Timer2 module
    //Interrupt generated when TMR2 = PR2 = 125 = 500us (changed to 122 because of ISR overhead)
    //Inst.Cycle x Prescalar x Postscalar x PR2 = Tick Time
    //   250ns   x    16     x      1     x 125 = 500us

    PMD0bits.TMR2MD = 0;					//Enable Timer2
    T2CONbits.T2OUTPS = 0b0000;                                 //Timer2 postscalar = 1:1 (1ticks every Tcy)
    T2CONbits.TMR2ON = 0;					//Timer2 OFF, switch on just before main loop
    T2CONbits.T2CKPS = 0b11;                                    //Timer2 prescalar = 1:16
    TMR2 = 0;                                                   //Preload value of 0
    INTCONbits.GIEH = 1;					//Enable all unmasked interrupts
    INTCONbits.PEIE = 1;					//Enable all unmasked peripheral interrupts
    PIR1bits.TMR2IF = 0;					//Clear interrupt flag
    PIE1bits.TMR2IE = 1;					//Enable Timer2 interrupt
    IPR1bits.TMR2IP = 0;					//Timer2 interrupt = high priority
    PR2 = 122;                                                  //Register to match Timer2 value to
    T2CONbits.TMR2ON = 1;                                       //Turn on Timer2

    //PIR1.RC1IF = 0;                                           //EUSART1 Receive Interrupt Flag bit - (cleared when RCREG1 is read)
    PIE1bits.RC1IE = 1;                                         //Enables the EUSART1 receive interrupt
    INTCONbits.RBIE = 1;                                        //Enables the IOCx port change interrupt
    //INTCON2bits.RBIP = 0/1;                                   //RB Port Change Interrupt Priority bit
    IOCBbits.IOCB4 = 1;                                         //Enable RB4 as an IOC pin

    //Configure A2Ds
    TRIS_CFAULT_READ = 1;               //Cable Fault Read
    TRIS_MAINS_ZEROCROSSING = 1;        //Mains/Zero Crossing detect
    ANSEL_MAINS_ZEROCROSSING = 0;
    TRIS_EARTH_LEAKAGE = 1;            //EARTH LEAKAGE
    TRIS_FIRE_OUT = 1;                   //FIRE OUT

    //STANDARD INPUT Configuration
    TRIS_FIRE_SWITCH = 1;                //FIRE SWITCH IN
    ANSEL_FIRE_SWITCH = 0;
    //TRIS_RESET_SWITCH = 1;               //RESET SWITCH IN - Cannot define as always an input

    //OUTPUTS
    TRIS_BUZZER_ALARM_LED = 0;        //BUZZER/ALARM LED
    LAT_BUZZER_ALARM_LED = 0;
    TRIS_SSR1 = 0;                        //SSR RELAY
    LAT_SSR1 = 0;
    TRIS_CABLE_FAULT = 0;                //CABLE FAULT
    LAT_CABLE_FAULT = 0;
    TRIS_RELAY1 = 0;                     //RELAY 1
    LAT_RELAY1 = 0;

    //LED definitions
    TRIS_CABLE_FAULT_LED = 0;           //CABLE FAULT LED
    LAT_CABLE_FAULT_LED = 0;
    TRIS_TEST_LED = 0;                  //TEST LED
    LAT_TEST_LED = 0;
    TRIS_DIAG1_LED = 0;                  //DIAGNOSTIC LED 1
    LAT_DIAG1_LED = 0;
    TRIS_DIAG2_LED = 0;                  //DIAGNOSTIC LED 2
    LAT_DIAG2_LED = 0;

    //UART DEFINITIONS - Both TRIS bits set to '1' as per datasheet
    ANSEL_RX = 0;
    ANSEL_TX = 0;
    TRIS_RX = 1;                       //RX PIN
    TRIS_TX = 1;                       //TX PIN
    UART_Init(9600);
    //Read from UART to clear interrupt flag

    TurnRelayOn(); //Keep Mains off of the output
    TurnSSR1Off(); //Double check the relay is off

    AlarmStatusUCG = FAULT;

    checkCableFaults();

    NewCFStatusUCG = 0;
    ClearToFire = 0;
    CFLEDFlashEnable = 0;
    CFLEDFlashState = 0;

    DeArmTimerEnabled = 0;
    DeArmTickCG = 0;

    if(InitST7540()){
        LAT_DIAG2_LED = 1;
        ReceiveNewDataST7540();
    }

}

unsigned short ReadAnalogVoltage(unsigned char channelC){

    ADCON1bits.PVCFG = 0b00;                                                    //Reference set to VCC

    switch(channelC){
        
        case(ADC_VOLT_EL):
            ADCON0bits.CHS = 0b01110;                                           //Select RC2, AN14, EL Test, pin 13 as input
            break;
        case(ADC_VOLT_CF):
            ADCON0bits.CHS = 0b00000;                                           //Select RA0, AN0, CF Read, pin 2 as input
            break;
        case(ADC_VOLT_ZC):
            ADCON0bits.CHS = 0b00001;                                           //Select RA1, AN1, ZC Read, pin 3 as input
            break;
        case(ADC_VOLT_FO):
            ADCON0bits.CHS = 0b01101;                                           //Select RB5, AN13, FO Read, pin 26 as input
            break;
        
    }

    ADCON0bits.GO_nDONE = 1;                                                    //Start conversion
    while(ADCON0bits.GO_nDONE);                                                 //Wait for conversion to finish

    return ADRES;
}

void ReadEarthLeakage(void){   
    unsigned short earthLeakageValueUS;

    TurnCableFaultDetectOn();                                                              //Enable EL test
    //LAT_TEST_LED = ON;
    __delay_us(1000);                                                             //Wait at least round trip delay for 5km (with Vfac of 0.6 is about 55uS)
    //LAT_TEST_LED = OFF;
    earthLeakageValueUS = ReadAnalogVoltage(ADC_VOLT_EL);                       //Read the EL test pin
    TurnCableFaultDetectOff();                                                              //Disable EL test

    earthLeakageProblemUCG = (earthLeakageValueUS > VOLT_EL_HIGH)?1:0;
}

void ReadCableFault(void){

    unsigned short cableFaultValueUS;

    __delay_us(24);                                                             
    cableFaultValueUS = ReadAnalogVoltage(ADC_VOLT_CF);
    cableFaultProblemUCG = (cableFaultValueUS > VOLT_CF_HIGH)?1:0;
}

unsigned char ReadMAINS_ZeroCrossing(void){
    unsigned char mainsZero_CrossingValueUSG2;
    
    //mainsZero_CrossingValueUSG = ReadAnalogVoltage(ADC_VOLT_ZC);
    mainsZero_CrossingValueUSG = PORT_MAINS_ZEROCROSSING;
    WaitTickCount(1); //Debounce
    mainsZero_CrossingValueUSG2 = PORT_MAINS_ZEROCROSSING;
    if (mainsZero_CrossingValueUSG == mainsZero_CrossingValueUSG2)
        return mainsZero_CrossingValueUSG;
    else
        return 0;//NULL
}

void ReadFireOut(void){    
    
    for (int i = 0; i < 50; i++){
        //__delay_us(24);
        while(ReadMAINS_ZeroCrossing() == 0);
        FireOutFloat += (float)ReadAnalogVoltage(ADC_VOLT_FO);
        while(ReadMAINS_ZeroCrossing() == 1);
    }    
    FireOutFloat = FireOutFloat/50.0;
}

void TurnRelayOn(void){
    LAT_RELAY1 = ON;
    if (RelayStatusUCG == OFF)
        Pi_Status_Update == DEFAULT_UPDATE;
    RelayStatusUCG = ON;
}

void TurnRelayOff(void){
    LAT_RELAY1 = OFF;
    if (RelayStatusUCG == ON)
        Pi_Status_Update == DEFAULT_UPDATE;
    RelayStatusUCG = OFF;
}

void TurnSSR1On(void){
    LAT_SSR1 = ON;
}

void TurnSSR1Off(void){
    LAT_SSR1 = OFF;
}

void TurnCableFaultDetectOn(void){
    LAT_CABLE_FAULT = ON;
}

void TurnCableFaultDetectOff(void){
    LAT_CABLE_FAULT = OFF;
}

void TurnBuzzer_AlarmOn(void){
    LAT_BUZZER_ALARM_LED = ON;
}

void TurnBuzzer_AlarmOff(void){
    LAT_BUZZER_ALARM_LED = OFF;
}

unsigned char getKeySwitchState(void){                                      //Half Wave rectification,

                                               //Detect Half cycle - 10ms, therefore 0V for min 10mS before rising voltage occurs
    ReadMAINS_ZeroCrossing();                                               //500uS tick, therefore 10mS is 20 Ticks
    if (mainsZero_CrossingValueUSG == VOLT_ZC_THRESHOLD){                    //Measured voltage above the threshold, therefore switch closed
        KeySwitchStateUCG = ARMED;
        //LAT_DIAG1_LED = ON;
    }
    else {                      //Wait quater of a cycle
        WaitTickCount(15);
        ReadMAINS_ZeroCrossing();  //See if we have a signal       
        if (mainsZero_CrossingValueUSG == VOLT_ZC_THRESHOLD){                    //Measured voltage above the threshold, therefore switch closed
            KeySwitchStateUCG = ARMED;           
        }
        else{           //Wait half a cycle
            WaitTickCount(10);
            ReadMAINS_ZeroCrossing();
            if (mainsZero_CrossingValueUSG == VOLT_ZC_THRESHOLD){                    //Measured voltage above the threshold, therefore switch closed
                KeySwitchStateUCG = ARMED;              
            }
            else
                KeySwitchStateUCG = DISARMED;

        }
    }
    
    return KeySwitchStateUCG;
}

unsigned char getFireButtonState(void){
    unsigned char FireButtonStatusfirstUC;
    unsigned char FireButtonStatussecondUC;
   

    FireButtonStatusfirstUC = PORT_FIRE_SWITCH;
    WaitTickCount(20);                           //Debounce for 10mS
    FireButtonStatussecondUC = PORT_FIRE_SWITCH;
    
    if (FireButtonStatusfirstUC != FireButtonStatussecondUC)
        return DEPRESSED;
    else{
        if (FireButtonStatussecondUC == 1)
            return DEPRESSED;                   //Inverted Logic
        else
            return PRESSED;
    }
    //return (FireButtonStatusfirstUC == FireButtonStatussecondUC)?1:0;
}

unsigned char getResetButtonState(void){
    unsigned char ResetButtonStatusfirstUC;
    unsigned char ResetButtonStatussecondUC;

    ResetButtonStatusfirstUC = PORT_RESET_SWITCH;
    WaitTickCount(50);                           //Debounce for 2mS
    ResetButtonStatussecondUC = PORT_RESET_SWITCH;

     if (ResetButtonStatusfirstUC != ResetButtonStatussecondUC)
        return DEPRESSED;
     else{
        if (ResetButtonStatussecondUC == 1)
            return DEPRESSED;
        else
            return PRESSED;
    }    
}

void ModemTXTest(void){
    /*
       while(TransmitBusyST7540());
       LAT_TEST_LED = 0;
       WaitTickCount(100);
       CreateMessageST7540(0xC3C3, 0xC3C3, 0xBB, 5, "abcde");
       StartTransmitST7540();
       while(TransmitBusyST7540());
       LAT_TEST_LED = 1;
       WaitTickCount(100);
       CreateMessageST7540(0xC3C3, 0xC3C3, 0xCC, 5, "abcde");
       StartTransmitST7540();
       */
}

void EEPROMWrite(){
    unsigned int address;
    unsigned char q=0;
    address = 0x0200;
    INTCONbits.GIEH = 0;					//Disable all unmasked interrupts
    Write_b_eep (address, EEPWrite[q]); // write into to EEPROM
        address++; //increment the address of EEPROM to next location
    /* Checks & waits the status of ER bit in EECON1 register */
        Busy_eep();
        q++;
    Write_b_eep (address, EEPWrite[q]); // write into to EEPROM
        address++; //increment the address of EEPROM to next location
    /* Checks & waits the status of ER bit in EECON1 register */
    Busy_eep();
    INTCONbits.GIEH = 1;					//Enable all unmasked interrupts

}

void EEPROMRead(void){
    unsigned int address;
    address = 0x0200;
    EEPRead[0] = Read_b_eep (address); //read the EEPROM data written previously from corresponding address
    address++;
    EEPRead[1] = Read_b_eep (address); //read the EEPROM data written previously from corresponding address

}

/*
void EEPROMTest(void){

    unsigned char q=0;
    unsigned int address;
    address = 0x0200;
    // Write single byte to Internal EEP
    for(q=0;q<16;q++){
        Write_b_eep (address, EEPWrite[q]); // write into to EEPROM
        address++; //increment the address of EEPROM to next location
    // Checks & waits the status of ER bit in EECON1 register
        Busy_eep();
    }
    address = 0x0200; // initialize the starting address
    Error = 0; //clear the error flag
    //* Read single byte from Internal EEP
    for(q=0;q<16;q++){
        EEPRead[q] = Read_b_eep (address++); //read the EEPROM data written previously from corresponding address
        if ( EEPRead[q] != EEPWrite[q] ) //check if the data read abck is same as that was written

        {
            Error=1; //if the data read/ write match does not occur, then flag the error status
        }
    }
}*/
void Write_b_eep( unsigned int badd,unsigned char bdat )
{
	char GIE_BIT_VAL = 0;
	EEADR = (badd & 0x0ff);
  	EEDATA = bdat;
  	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	GIE_BIT_VAL = INTCONbits.GIE;
	INTCONbits.GIE = 0;
	EECON2 = 0x55;
	EECON2 = 0xAA;
	EECON1bits.WR = 1;
	while(EECON1bits.WR);				//Wait till the write completion
	INTCONbits.GIE = GIE_BIT_VAL;
	EECON1bits.WREN = 0;
}

void Busy_eep ( void )
{
	while(EECON1bits.WR);
}


unsigned char Read_b_eep( unsigned int badd )
{
	EEADR = (badd & 0x0ff);
  	EECON1bits.CFGS = 0;
	EECON1bits.EEPGD = 0;
	EECON1bits.RD = 1;
	Nop();							//Nop may be required for latency at high frequencies
	Nop();							//Nop may be required for latency at high frequencies
	return ( EEDATA );              // return with read byte
}