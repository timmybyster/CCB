#include "ST7540.h"
#include <xc.h>
#include "main.h"
#include "UART.h"

int ReceivedPLMBufferIndexIG = 0;
int TransmitPLMBufferIG = 0;
char ReceivedPLMBufferCG[PLM_RECEIVE_BUFFER_LENGTH];
char TransmitPLMBufferCG[PLM_TRANSMIT_BUFFER_LENGTH];
char PiDefaultData[2];

unsigned char boosterCommsData[DATA_TYPE_COUNT][60];
unsigned char boosterCommsDataReverse[60];
unsigned char test;

unsigned char routerData[60];

unsigned short CBB_SN_ArrayUIG[MAX_NO_CBB];
unsigned short CBB_SN_ArrayUIG_Missing[MAX_NO_CBB];
unsigned char CBB_SN_Array_ReverseUSG[MAX_NO_CBB * 2];
unsigned char CBB_SN_Array_Missing_ReverseUSG[MAX_NO_CBB * 2];
unsigned short CBB_Switch_TempUSG;
unsigned int CBB_PARENT_SNArrayUIG[MAX_NO_CBB];
unsigned char CBB_SN_ArrayIndexUCG = 0;
unsigned char CBB_SN_Array_SizeUCG = 0;
unsigned char CBB_SN_Array_MissingUCG = 0;
unsigned int HighSpeedTickUIG;
unsigned int HighSpeedTickPAUIG;
unsigned int BTHighSpeedTickUIG;
unsigned int CFLEDFlashTickUIG = 0;
unsigned int IdleTimerUIG = 0;
unsigned int CF_STAUTS_TimerUIG = 0;
unsigned int ELT_Counter = 0;
unsigned int CheckCBBTimerUIG = 0;
unsigned int CCB_ComsCrashTimer = 0;
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

unsigned char fireButton;
unsigned char resetButton;
unsigned char keySwitch;

char HalfCycleCounterCG;
char MissingPulseCounterCG;
char ModeCG;
char ToggleRelay = 0;
unsigned char RelayStatusUCG;
unsigned char KeySwitchStatusUCG;
unsigned char FireButtonStatusUCG;
unsigned char CableFaultStatusUCG;
unsigned char FiringStatusUCG;
unsigned char ResetStatusUCG;
unsigned char NewCFStatusUCG;
unsigned char AlarmStatusUCG;
unsigned char CFLEDFlashState;
unsigned char PacketForPiIdentifier = 0b11111111;
unsigned char CBB_Transmit_Packet_Ready = 0;
unsigned char Pi_Status_Update = 0;
unsigned char New_CBB_SN = 0;
unsigned char BroadcastCheckUC = 0;
unsigned char Discover_CBB_ModeUCG = 0;
unsigned char Show_Missing_CBB = 0;
unsigned char Discovered_New_CBB = 0;

char HighPulseCountCG;

unsigned char KeySwitchStateUCG;

unsigned char earthLeakageProblemUCG;
unsigned char cableFaultProblemUCG;
unsigned short cableFaultCounter;
unsigned short earthLeakageCounter;
unsigned short earthLeakageStart;

unsigned short mainsZero_CrossingValueUSG;
short long fireOutValueSLG;
float FireOutFloat;
unsigned int CBB_Packet_Index = 0;
unsigned int CBB_CF_Index = 0;
unsigned int CCB_Default_Data;
unsigned int CCB_Faults_Data;

unsigned short statusFlagsUSG;

unsigned short temp;

unsigned char EEPWrite[5], EEPRead[5];
unsigned char Production_CBB_SN_Counter[2];

void InitSystem(void);
void InitReset(void);
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
void getResetAndFireButtonState(void);
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
void Receive_CBB_Packet(void);
void Receive_Pi_Packet(void);
unsigned short getCBB_SN(unsigned char CBB_Index);
void Reverse_CBB_SN_List(void);
void BroadcastGetCBB_SNumbers(void);
void GetIB651_SNumbers(unsigned short CBB_SN);
void Add_CBB_SN(unsigned short CBB_SN);
unsigned char Check_Clash_CBB_SN(unsigned short CBB_SN);
void Update_Open_Relay(unsigned short CBB_SN);
void Update_Close_Relay(unsigned short CBB_SN);
void Update_Ping_Command(unsigned short CBB_SN);
void Update_ARM_CBB(unsigned short CBB_SN);
void Update_DISARM_CBB(unsigned short CBB_SN);
void Update_CBB_Cable_Fault(unsigned short CBB_SN);
void Update_Default_Data(unsigned short CBB_SN, unsigned short Data_Length, char *dataBuf);
void Update_DC_Data(unsigned short CBB_SN, unsigned short Data_Length, char *dataBuf);
void Update_AC_Data(unsigned short CBB_SN, unsigned short Data_Length, char *dataBuf);
void Update_Blast_Loop_Data(unsigned short CBB_SN, unsigned short Data_Length, char *dataBuf);
void Update_Serial_Number(unsigned short CBB_SN, unsigned short Data_Length, char *dataBuf);
unsigned char getCBB_Index(unsigned short CBB_SN);
void Inspect_Default_Data_Packet(unsigned short Data_Length);
unsigned short getCCBDefaultData(void);
void Transmit_Pi_Default_Data(void);
unsigned short getParentForCBB(unsigned short CBB_SN);
void AddParentForCBB(unsigned short CBB_SN, unsigned short CBB_PARENT_SN);
void DEBUG_Print_Packet(void);
void Check_ALL_CBB(void);
void EEPROMTest(void);
void EEPROMWrite(void);
void EEPROMClearReset(void);
void EEPROMSetReset(void);
void EEPROMRead(void);
void Assign_CBB_New_SN(void);
void CheckBroadcastPacket(unsigned char Command);
void Transmit_NULL_Packet(void);
void Transmit_BLAST_Command_Packet(void);


void Write_b_eep( unsigned int badd,unsigned char bdat );
unsigned char Read_b_eep( unsigned int badd );
void Busy_eep ( void );

void checkKeySwitch(void);

extern unsigned char DataReadyUART(void);
extern unsigned char LineIdleST7540(void);
extern char *PacketDataST7540(void);
extern char UART_Init(const long int baudrate);
extern char UART_Read(void);
extern void UART_Write(char data);
extern void UART_TX_ISRHandler(void);
extern void CreateMessageUART(unsigned short packetSourceUS, unsigned char commandUC, unsigned char dataLenUC, char *dataBuf);
extern void SendUARTPacket(void);
extern unsigned short PacketReadParamUART(unsigned char paramName);
extern char CheckCRCUART(void);
extern void resetUARTPointers(void);

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
extern void CreatePiToCCBUARTMessage(unsigned char Command, unsigned short packetSourceUS);
extern void CreateMessageUARTDEBUG(unsigned char dataLenUC,unsigned short packetSourceUS, unsigned short packetDestUS,unsigned char packetNOUC,unsigned char commandUC,char *dataBuf);

void interrupt isr(void){

    if(PIR1bits.TMR2IF != 0){
        PIR1bits.TMR2IF = 0;
        statusFlagsUSG |= FLAG_TICK;        
        UART_TX_ISRHandler();        

        checkKeySwitch();
        ELT_Counter++;                                                          //add 1 to the ELT counter, cleared by the ELT tester
        
        if (CF_STAUTS_TimerUIG < (20000)){                                      //10 seconds
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
        
        if (BTEnabled){
            BTHighSpeedTickUIG++;
        }
        else{                       
            BTHighSpeedTickUIG = 0;
            
            if (IdleTimerUIG > 2000){                                           //1 second then trigger null packet to let CBB's know you are here
                                                                                //Transmit Null Packet down the line
                Transmit_NULL_Packet();
                IdleTimerUIG = 0;
                CCB_ComsCrashTimer++;                                           //increment counter every 1 second when coms is active
            }
            else
                IdleTimerUIG++;
                                        
            CheckCBBTimerUIG++;
        }

        if (PATEnabled){
            if (HighSpeedTickPAUIG < 2000){                                     //2000 Ticks is 1 second: 500us per tick.
                HighSpeedTickPAUIG++;
            }
            else{
            HighSpeedTickPAUIG = 0;
            LowSpeedTickPATCG++;
            }
        }        
        if (HighSpeedTickUIG < 2000){                                           //2000 Ticks is 1 second: 500us per tick.
            HighSpeedTickUIG++;
        }
        else{
            HighSpeedTickUIG = 0;
            LowSpeedTickCG++;
            
            if (PBTEnabled){
                LowSpeedTickPBTCG++;                
            }        
        }
        
        if (LowSpeedTickPBTCG >= 60){                                           //Post Blast Timer
            AlarmStatusUCG = FAULT;
            CFLEDFlashEnable = 1;
        }
        if (LowSpeedTickPATCG >= 120){                                          //Pre Arm Timer adjusted to 2 minutes
            AlarmStatusUCG = FAULT;
            CFLEDFlashEnable = 1;
        }
        if (DeArmTickCG >= 7){                                                  //Amount of seconds the keyswitch needs to be in the disarmed position before firing is enabled
            ClearToFire = 1;              
        }
        else
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
    
    if(RCSTA1bits.OERR){                                                        //Clear the Receive Overflow bit
        RCSTA1bits.CREN = 0;                                                    //By clearing the Continuous Receive Enable Flag
        RCSTA1bits.CREN = 1;                                                    //Re-enable it to allow for new reception
    }
    
    if (INTCONbits.RBIF){                                                       //else if(neg pin change on SSX){
        INTCONbits.RBIF = 0;
        RXReadyISRHandlerST7540();
    }
}

void main(void){

    InitSystem();
    Pi_Status_Update = CLEAR_UPDATE;
    LAT_CABLE_FAULT_LED = OFF;
    WDTCONbits.SWDTEN = 1;                                                      //Enable WDT 

while(1){        
    CLRWDT();
    if((ELT_Counter >= 500 && earthLeakageCounter > 0) || ELT_Counter >= 1000){
        earthLeakageStart=1;                                                    //start the test
        TurnCableFaultDetectOn();   
    }
    
    getResetAndFireButtonState();
    
    checkCableFaults();                                                         //check cable faults normally all the time, should it clear then clear the cable as soon as
    
    if (DataReadyST7540()){
        if(PacketReadParamST7540(ST7540_CRC_VALID)){// && PacketReadParamST7540(ST7540_DEST)==CCB_SN){
            Receive_CBB_Packet();
            IdleTimerUIG = 0;           
        }
        ReceiveNewDataST7540();
    }
    if (DataReadyUART()){                                                       //We have received a packet, now process the packet
        ClearDataReady();
        CCB_ComsCrashTimer =0;                                                  //we have received data from the PI thus reset counter
        if(PacketReadParamUART(UART_CRC_VALID)){
            Receive_Pi_Packet();
        }
    }

    if (CBB_Transmit_Packet_Ready == 1){
        
        if (!TransmitBusyST7540() && LineIdleST7540()){                         //Only allowed to transmit if the we are not already transmitting and not receiving
            
            StartTransmitST7540();
            CBB_Transmit_Packet_Ready = 0;
            IdleTimerUIG = 0;
        }
    }                                                                            //reset is pressed or past when reset is pressed but fault clears in the field
    
    if (CableFaultStatusUCG == CABLEFAULT || CableFaultStatusUCG == EARTHFAULT){  //We have a cable fault
        if (NewCFStatusUCG == 1){                                               //We have a new cable fault
            NewCFStatusUCG = 0;
            Pi_Status_Update = DEFAULT_UPDATE;
            AlarmStatusUCG = FAULT;                                             //Turn on general alarm
        }
        LAT_CABLE_FAULT_LED = ON;                                               //Have cable fault so turn LED on
    }
    else{                                                                       //We dont have a cable fault any more
        if (CFLEDFlashEnable == 0 && AlarmStatusUCG == CLEAR){
                LAT_CABLE_FAULT_LED = OFF;
                TurnRelayOff();                                                 //Fault Clear, turn relay back on
                CF_STAUTS_TimerUIG = 0;
                if (NewCFStatusUCG == 0){                                       //We no longer have cable fault
                NewCFStatusUCG = 1;                                             //Fault is cleared, hence next time a fault occurs, it is a new fault
                Pi_Status_Update = DEFAULT_UPDATE;
            }
        }                        
    }
        
    if (getResetButtonState() == PRESSED){                                      //Reset button is pressed    
        if (getKeySwitchState() == DISARMED){
           AlarmStatusUCG = CLEAR;                                              //Clear the general alarm
           if (CableFaultStatusUCG == CLEAR){                                   //If the cable fault is clear?
                LAT_CABLE_FAULT_LED = OFF;                                      //Turn the LED off
                TurnRelayOff();                                                 //Fault Clear, turn relay back on
                CFLEDFlashEnable = 0;
                CFLEDFlashState = 0;
           }
           else
               CF_STAUTS_TimerUIG = 20000;
        }
        else{
            AlarmStatusUCG = FAULT;
             CFLEDFlashEnable = 1;
        }
    }
    if (getKeySwitchState() == DISARMED){      
        if(KeySwitchStatusUCG == ARMED)
            Pi_Status_Update = DEFAULT_UPDATE;

            KeySwitchStatusUCG = DISARMED;
            StopPostFireTimer();
            StopPreArmedTimer();
        }
    else{ //Keyswitch is in the Armed State
        if(KeySwitchStatusUCG == DISARMED)
            Pi_Status_Update = DEFAULT_UPDATE;
        
        KeySwitchStatusUCG = ARMED;
        
        if (PATEnabled == 0)                                                    //Check if the Armed state timer is running yet
            StartPreArmedTimer();                                               //If not, start the timer
        
            ClearToFire = 1;
        }
    
        //CCB has timed out with no comms from the PI = 4 minutes
    if(CCB_ComsCrashTimer > CCB_COMSCRASH_COUNT){          
        //also need to make it reset not turn on the alarm
        EEPROMSetReset();                                                       //set the reset flags to ensure that it doesnt alarm.
        resetUARTPointers();
        ResetStatusUCG = 1;
        Pi_Status_Update = DEFAULT_UPDATE;
        CCB_ComsCrashTimer= (CCB_COMSCRASH_COUNT-10);                           //subtract 10 counts from it.
    }
    
    if (Pi_Status_Update == DEFAULT_UPDATE){                                    //Default Data has updated and we require letting the Pi know            
        Pi_Status_Update = CLEAR_UPDATE;
        getCCBDefaultData();
        Transmit_Pi_Default_Data();
    }
    
    if (Pi_Status_Update == ERROR_UPDATE){                                      //An error has occured and we need to let the Pi know of the error

        Pi_Status_Update = CLEAR_UPDATE;
    }
        
    if (getFireButtonState() == PRESSED){                                       //The fire button has been pressed
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
            FiringStatusUCG=1;                                                  //firing has begun
            getCCBDefaultData();
            Transmit_Pi_Default_Data();
            SendUARTPacket(); 
            
            ClearToFire = 0;
            //Blasting and therefore stop the prearmed timer
            StopPreArmedTimer();
            //Change the relay over - ie OFF
            TurnRelayOn();                                                      //Patch Mains through
            //Wait long enough for relay to close so we can read the mains voltage before firing
            WaitTickCount(100);
            TurnSSR1On();
            WaitTickCount(100);
            FireOutFloat = 0;
            ReadFireOut();
            FireOutFloat = 1000.0;
            TurnSSR1Off();

            if (FireOutFloat > 815.0){                                          //confirm it is not currently firing.            
                //Fire
                
                Transmit_BLAST_Command_Packet();
                CLRWDT();
                WaitTickCount(2000*4);                                          //Delay so that the booster comms line goes down
                CLRWDT();
                
                BTEnabled = 1;                                                  // engage firing counter
                FIRE();
                
                WaitTickCount(1000);
                BTEnabled = 0;                                                  // engage communication mode counter
                CCB_ComsCrashTimer = CCB_COMSCRASH_COUNT-60;                    // jump the counter to only allow a 1 min leeway before resetting must happen
            }
            else{
                AlarmStatusUCG = FAULT;
            }
            //Always Turn SSR OFF as precautionary measure
            
            FiringStatusUCG=0;                                                  //firing is complete
            TurnSSR1Off();
            //We have fired and required Disarm prior to refiring

            //After Firing turn the relay back
            TurnRelayOff();                                                     //Keep Mains off of the output                
        }
    }
    else{
        if (FireButtonStatusUCG == PRESSED)
            Pi_Status_Update = DEFAULT_UPDATE;
        FireButtonStatusUCG = DEPRESSED;
    }
    
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
    }                                                                           //END WHILE LOOP
}

void Receive_CBB_Packet(void){
    unsigned short tempCbbSerial = PacketReadParamST7540(ST7540_SOURCE);
    switch(PacketReadParamST7540(ST7540_CMD)){                          //Extract the command from the Packet
        case CMD_GET_SN :
            CreateMessageUART(PacketReadParamST7540(ST7540_SOURCE), CMD_PI_SN_IB651, PacketReadParamST7540(ST7540_DATA_LEN), PacketDataST7540());
            SendUARTPacket();
            return;

        case CMD_SEND_DEFAULT :
            CreateMessageUART(PacketReadParamST7540(ST7540_SOURCE), CMD_PI_DEFAULT_DATA, PacketReadParamST7540(ST7540_DATA_LEN), PacketDataST7540());
            SendUARTPacket();
            return;

        case CMD_FORCE_DEFAULT :
            CreateMessageUART(PacketReadParamST7540(ST7540_SOURCE), CMD_PI_DEFAULT_DATA, PacketReadParamST7540(ST7540_DATA_LEN), PacketDataST7540());
            SendUARTPacket();
            return;

        case CMD_AB1_UID :
            CreateMessageUART(PacketReadParamST7540(ST7540_SOURCE), CMD_PI_AB1_UID, PacketReadParamST7540(ST7540_DATA_LEN), PacketDataST7540());
            SendUARTPacket();
            CreateMessageST7540(CCB_SN, PacketReadParamST7540(ST7540_SOURCE), CMD_AB1_UID, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            return;

        case CMD_AB1_DATA :
            CreateMessageUART(PacketReadParamST7540(ST7540_SOURCE), CMD_PI_AB1_DATA, PacketReadParamST7540(ST7540_DATA_LEN), PacketDataST7540());
            SendUARTPacket();
            CreateMessageST7540(CCB_SN, PacketReadParamST7540(ST7540_SOURCE), CMD_AB1_DATA, 0, "");
            LAT_CABLE_FAULT_LED = !LAT_CABLE_FAULT_LED;
            CBB_Transmit_Packet_Ready = 1;
            return;

        case(CMD_CBB_NEW_SN):
            tempCbbSerial = (tempCbbSerial << 8) & 0XFF00;
            tempCbbSerial |= (PacketReadParamST7540(ST7540_SOURCE) >> 8) & 0XFF;
            CreateMessageUART(CCB_SN, CMD_PI_SN_CBBS, PacketReadParamST7540(ST7540_DATA_LEN) + 2, &tempCbbSerial);
            SendUARTPacket();
            return;

        case(PING_CBB):                                                 //Receiving ACK for PING
            tempCbbSerial = (tempCbbSerial << 8) & 0XFF00;
            tempCbbSerial |= (PacketReadParamST7540(ST7540_SOURCE) >> 8) & 0XFF;
            CreateMessageUART(CCB_SN, CMD_PI_SN_CBBS, PacketReadParamST7540(ST7540_DATA_LEN) + 2, &tempCbbSerial);
            SendUARTPacket();
            return;

    }       
}

void Receive_Pi_Packet(void){                                                   //We have received a request from the Pi to aquire data from an CBB
    
    unsigned char BroadcastUC;
            
    temp = PacketReadParamUART(UART_HEADER);
    //We have received a packet from the PI, now we need to extract the info and perform the relevant task
    CheckBroadcastPacket(PacketReadParamUART(UART_CMD));
    switch(PacketReadParamUART(UART_CMD)){//Extract the command from the Packet
        case(CMD_PI_DEFAULT_DATA):
            //Get IB651 Default Values
            BroadcastUC = CMD_SEND_DEFAULT;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;         
            return;
        case(CMD_PI_FORCE_DEFAULT):
            //Get IB651 Default Values
            BroadcastUC = CMD_FORCE_DEFAULT;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_SN_IB651):
            //Get serial number list of the IB651s connected to particular CBB
            BroadcastUC = CMD_GET_SN;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            return;    
        case(CMD_PI_SN_CBBS):          
            //Ordered list of CBB's, stored on the CCB - No downstream request required
            Check_ALL_CBB();
            return;     
        case(CMD_PI_PING_CBB):                                                       
            //Ping the relevant CBB
            BroadcastUC = PING_CBB;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            Discover_CBB_ModeUCG = 0;
            CF_STAUTS_TimerUIG = 0;
            return;
        case(CMD_PI_PING_CBB_B):
            //Ping the relevant CBB
            BroadcastUC = PING_CBB;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_ARM_CBB):                                                        
            //Arm an CBB
            BroadcastUC = ARM_CBB;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_DISARM):                                                     
            //Disarm an CBB
            BroadcastUC = DISARM_CBB;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            return;
        case(CMD_PI_OPEN_RELAY):
            //set CBB Relay open
            BroadcastUC = CMD_OPEN_RELAY;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;         
            return;
        case(CMD_PI_CLOSE_RELAY):
            //Set CBB relay closed
            BroadcastUC = CMD_CLOSE_RELAY;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;         
            return;
        case(CMD_PI_CLEAR_CBB_LIST):
            //Clear the list of CBB's normally for remapping purposes
                for (int i = 0; i < CBB_SN_Array_SizeUCG; i++)
                    CBB_SN_ArrayUIG[i] = 0; //clear the logs
                
                CBB_SN_Array_SizeUCG=0; //clear the logs       
            return;
        case(CMD_PI_CLEAR_ALARM):
            //Clear Alarm remotely
            AlarmStatusUCG = CLEAR;                    
            return;
            
        case (CMD_PI_AB1_UID):
            BroadcastUC = CMD_GET_SN;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            return;
            
        case (CMD_PI_AB1_DATA):
            BroadcastUC = CMD_SEND_DEFAULT;
            if (BroadcastCheckUC)
                BroadcastUC |= 0b10000000;
            CreateMessageST7540(CCB_SN, PacketReadParamUART(UART_SN), BroadcastUC, 0, "");
            CBB_Transmit_Packet_Ready = 1;
            return;
            
        default :
            return;
                    
    }//END SWITCH
}

void Transmit_BLAST_Command_Packet(void){
    if (!TransmitBusyST7540() && LineIdleST7540()){                             //Only allowed to transmit if the we are not already transmitting and not receiving
        CreateMessageST7540(CCB_SN, CBB_SN_BROADCAST_ADD, CMD_BLAST_COMMAND, 0, "");
                StartTransmitST7540();
                CBB_Transmit_Packet_Ready = 0;
    }
}

void Transmit_NULL_Packet(void){
    CreateMessageST7540(CCB_SN, CBB_NULL_SN, PING_CBB, 0, "");
    CBB_Transmit_Packet_Ready = 1;
}

void Check_ALL_CBB(void){
    CreateMessageST7540(CCB_SN, CBB_SN_BROADCAST_ADD, PING_CBB, 0, "");
    CBB_Transmit_Packet_Ready = 1;
}

void Transmit_Pi_Default_Data(void){
    CreateMessageUART(CCB_SN, CMD_PI_CCB_DEFAULT, 2, PiDefaultData);
    SendUARTPacket();
}

void Update_Ping_Command(unsigned short CBB_SN){
    CBB_Packet_Index = getCBB_Index(CBB_SN);
}

void CheckBroadcastPacket(unsigned char Command){

 BroadcastCheckUC = (Command & 0b10000000);
}

void Reverse_CBB_SN_List(void){
    unsigned char TempChar;
    unsigned short TempShort;   

    for (int i = 0; i < CBB_SN_Array_SizeUCG; i++){
        TempShort = CBB_SN_ArrayUIG[i];       
        TempChar = (TempShort >> 8);
        CBB_SN_Array_ReverseUSG[i*2] = TempChar;
        TempChar = TempShort & 0b11111111;
        CBB_SN_Array_ReverseUSG[(i*2)+1] = TempChar;
    }

}

unsigned short getCBB_SN(unsigned char CBB_Index){
    return CBB_SN_ArrayUIG[CBB_Index - 1];
}

unsigned char getCBB_Index(unsigned short CBB_SN){
    int index = 0;

    for (index = 0; index < CBB_SN_Array_SizeUCG; index++){
        if (CBB_SN_ArrayUIG[index] == CBB_SN)
            return index + 1;
    }

    if (index == 0)
        Add_CBB_SN(CBB_SN);
    return 1;
}

void Add_CBB_SN(unsigned short CBB_SN){

    if (CBB_SN_Array_SizeUCG == 0){                                             //We have no CBBs listed
        CBB_SN_Array_SizeUCG++;
        CBB_SN_ArrayUIG[0] = CBB_SN;
        New_CBB_SN = 1; 
    }
    else{                                                                       //We already have an CBB listed
        if (Check_Clash_CBB_SN(CBB_SN) != 1){
            CBB_SN_ArrayUIG[CBB_SN_Array_SizeUCG] = CBB_SN;
            CBB_SN_Array_SizeUCG++;
            New_CBB_SN = 1;                                                     //If we dont have a Clash, then it is a new Serial Number
        }
    }
}

unsigned char Check_Clash_CBB_SN(unsigned short CBB_SN){
    for (int i = 0; i < CBB_SN_Array_SizeUCG; i++){
        if (CBB_SN_ArrayUIG[i] == CBB_SN){
            
            return 1;
        }
    }
    return 0;
}

void Assign_CBB_New_SN(void){
    unsigned char CBB_NEW_SN[2];
    //Method
    EEPROMRead();
    CBB_NEW_SN[0] = EEPRead[0];                                                 //MSB
    CBB_NEW_SN[1] = EEPRead[1];                                                 //LSB
    //Get Last CBB_SN from EEPROM

    if (EEPRead[1] == 0xFF){                                                    //We need to roll over LSB and increment MSB
        CBB_NEW_SN[1] = EEPRead[1] + 1;
        CBB_NEW_SN[0] = 0x00;
    }
    else{
        CBB_NEW_SN[0] = EEPRead[0];                                             //MSB
        CBB_NEW_SN[1] = EEPRead[1] + 1;                                         //LSB
    }

    EEPWrite[0] = CBB_NEW_SN[0];
    EEPWrite[1] = CBB_NEW_SN[1];
    EEPROMWrite();
    //Increment CBB_SN in EEPROM
    //Send CBB_SN down to CBB without a SN - PING COMMAND WITH SN in DATA Field?
    CreateMessageST7540(CCB_SN, CBB_DEFAULT_SN, CMD_CBB_NEW_SN, 2, CBB_NEW_SN);
    CBB_Transmit_Packet_Ready = 1;
}

unsigned short getCCBDefaultData(void){
    unsigned short returningData = 0b00000000;

    if (KeySwitchStatusUCG)
        returningData |= 0b10000000;                                            //Keyswitch Status
    if (RelayStatusUCG)                                                         //Isolation Relay Status
        returningData |= 0b01000000;
    if (FireButtonStatusUCG)                                                    //Fire Button Status - COMPLETE STILL
        returningData |= 0b00100000;
    if (CableFaultStatusUCG == CABLEFAULT)                                      //Cable Fault
        returningData |= 0b00010000;
    if (CableFaultStatusUCG == EARTHFAULT)                                      //Earth Leakage Fault
        returningData |= 0b00001000;
    if (FiringStatusUCG)                                                        //Is the unit firing.
        returningData |= 0b00000100;
    if (ResetStatusUCG)                                                        //Reset BIT, indicates that the unit has lost coms and will reset quitly
        returningData |= 0b00000010;
        PiDefaultData[0] = 0b00000101;                                          //0b01010101;
        PiDefaultData[1] = returningData;
    return returningData;
}

void Inspect_Default_Data_Packet(unsigned short Data_Length){
    unsigned char index = 0;   
    unsigned char data;
    unsigned char CF;
    unsigned char EL;
    unsigned char Fault;

    for(index = 0; index < Data_Length / 2; index++){                           //Packet payload
        data = boosterCommsData[DATA_STATUS][(index * 2) + 1];
        EL = data & 0b0000100;
        EL = EL >> 2;
        CF = data & 0b0001000;
        CF = data >> 3;
        if (EL == 1 || CF == 1)
            Fault = 1;
    }
}

void Update_Open_Relay(unsigned short CBB_SN){
    CBB_Packet_Index = getCBB_Index(CBB_SN);
    PacketForPiIdentifier = CMD_OPEN_RELAY;
}

void Update_Close_Relay(unsigned short CBB_SN){
    CBB_Packet_Index = getCBB_Index(CBB_SN);
    PacketForPiIdentifier = CMD_CLOSE_RELAY;
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

void StartPostFireTimer(void){                                                  //Timer to count if the system is ARMED for too long after a blast
    PBTEnabled = 1;
    HighSpeedTickUIG = 0;
    LowSpeedTickPBTCG = 0;    
}

void StopPostFireTimer(void){
    PBTEnabled = 0;
    HighSpeedTickUIG = 0;
    LowSpeedTickPBTCG = 0;    
}

void StartBlastTimer(void){                                                     //Timer to count if the Blast was too short
    BTEnabled = 1;
    LowSpeedTickBTCG = 0;    
}

void StopBlastTimer(void){
    BTEnabled = 0;
    HighSpeedTickUIG = 0;
    LowSpeedTickBTCG = 0;    
}

void checkCableFaults(void){

    
    //if in ELT test mode then finish the test before anything else
    if(earthLeakageStart==1){
        ReadEarthLeakage();                                                     //then conduct the test
        earthLeakageStart=0;                                                    //and clear the start bit
        ELT_Counter=0;                                                          //clear the counter
        TurnCableFaultDetectOff();                                              //turn off the test
        
    }
    else{
        ReadCableFault();                                                       //only do the test far away from ELT
    }
    
    if (cableFaultProblemUCG == 1){                                             //We have a Cable/EL Fault        
        CableFaultStatusUCG = CABLEFAULT;
    }
    else if (earthLeakageProblemUCG == 1){                                      //We have a Cable/EL Fault
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
    for (int i = 0; i < 240; i++){                                              //changed to 240 for 2min timeout was 120
        CLRWDT();
        while(ReadMAINS_ZeroCrossing() == 0 && BTHighSpeedTickUIG < 160);       //4 whole cycles missed
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

void GenerateMissingPulse(void){                                                //Lets through N Pulses

     WaitTickCount(8);                                                          //Wait some time after the edge so we only turn on after ZC
     TurnSSR1On();    
    
     for (HighPulseCountCG = 0; HighPulseCountCG < HighCycle; HighPulseCountCG++){//

         while(ReadMAINS_ZeroCrossing() == 1){                                  //Burn off till pulse goes low
             //LowSpeedTickBTCG = 0;
             BTHighSpeedTickUIG = 0;
             if (HighPulseCountCG == (HighCycle - 1)){
                    WaitTickCount(6);
                    TurnSSR1Off();
                }
        }
        if (BTHighSpeedTickUIG < 160){
            while(ReadMAINS_ZeroCrossing() == 0 && BTHighSpeedTickUIG < 160){   //Burn off till pulse goes high
            }
        }
     }                                                                          //END for loop         
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
    
    //Configure A2Ds
    TRIS_CFAULT_READ = 1;                                                       //Cable Fault Read
    TRIS_MAINS_ZEROCROSSING = 1;                                                //Mains/Zero Crossing detect
    ANSEL_MAINS_ZEROCROSSING = 0;
    TRIS_EARTH_LEAKAGE = 1;                                                     //EARTH LEAKAGE
    TRIS_FIRE_OUT = 1;                                                          //FIRE OUT

    //STANDARD INPUT Configuration
    TRIS_FIRE_SWITCH = 1;                                                       //FIRE SWITCH IN
    ANSEL_FIRE_SWITCH = 0;
    TRIS_RESET_SWITCH = 1;                                                      //RESET SWITCH IN - Cannot define as always an input

    //OUTPUTS
    TRIS_BUZZER_ALARM_LED = 0;                                                  //BUZZER/ALARM LED
    LAT_BUZZER_ALARM_LED = 0;
    TRIS_SSR1 = 0;                                                              //SSR RELAY
    LAT_SSR1 = 0;
    TRIS_CABLE_FAULT = 0;                                                       //CABLE FAULT
    LAT_CABLE_FAULT = 0;
    TRIS_RELAY1 = 0;                                                            //RELAY 1
    LAT_RELAY1 = 1; //0

    //LED definitions
    TRIS_CABLE_FAULT_LED = 0;                                                   //CABLE FAULT LED
    LAT_CABLE_FAULT_LED = 0;
    TRIS_DIAG1_LED = 0;                                                         //DIAGNOSTIC LED 1
    LAT_DIAG1_LED = 0;
    TRIS_DIAG2_LED = 0;                                                         //DIAGNOSTIC LED 2
    LAT_DIAG2_LED = 0;

    //UART DEFINITIONS - Both TRIS bits set to '1' as per datasheet
    ANSEL_RX = 0;
    ANSEL_TX = 0;
    TRIS_RX = 1;                                                                //RX PIN
    TRIS_TX = 1;                                                                //TX PIN
    
    
    
    //Setup Timer2 module
    //Interrupt generated when TMR2 = PR2 = 125 = 500us (changed to 122 because of ISR overhead)
    //Inst.Cycle x Prescalar x Postscalar x PR2 = Tick Time
    //   250ns   x    16     x      1     x 125 = 500us

    PMD0bits.TMR2MD = 0;                                                        //Enable Timer2
    T2CONbits.T2OUTPS = 0b0000;                                                 //Timer2 postscalar = 1:1 (1ticks every Tcy)
    T2CONbits.TMR2ON = 0;                                                       //Timer2 OFF, switch on just before main loop
    T2CONbits.T2CKPS = 0b11;                                                    //Timer2 prescalar = 1:16
    TMR2 = 0;                                                                   //Preload value of 0
    
    //timer 0 setup to restart RX communication
    //16Mhz clock/4 = 4Mhz  (/256 prescaller = 15625)
    //2^16 = 65536  (/15625 = 4.19s) per interrup
    //this if we want a 5 minute reconfigure then interrupt counter should =
//    RX_TimoutCounter = 0;
//    RX_TimoutCounter2 = 0;
//    T0CON = 0b00000111;         //16 bit mode 256 prescaller
//    INTCONbits.INT0IF = 0;      //clear th flag   
//    INTCONbits.INT0IE = 1;      //set as an interrupt    
//    INTCON2bits.TMR0IP = 1;     //high priority interrupt
                                                                                //EUSART1 Receive Interrupt Flag bit - (cleared when RCREG1 is read)
    PIE1bits.RC1IE = 1;                                                         //Enables the EUSART1 receive interrupt
    INTCONbits.RBIE = 1;                                                        //Enables the IOCx port change interrupt
    IOCBbits.IOCB4 = 1;                                                         //Enable RB4 as an IOC pin
    
    TXSTA1bits.BRGH = 0;                                                        //Setting High Baud Rate
    BAUDCONbits.BRG16 = 0;
    UART_Init(9600);
    //Read from UART to clear interrupt flag
    
    //Configure ADC
    ADCON2bits.ADFM = 1;                                                        //Right justify result
    ADCON2bits.ACQT = 0b010;                                                    //Sample for 4 TAD
    ADCON2bits.ADCS = 0b110;                                                    //Set to Fosc/64 clock
    ADCON0bits.ADON = 1;                                                        //Turn on the ADC

    INTCONbits.PEIE = 1;                                                        //Enable all unmasked peripheral interrupts
    PIR1bits.TMR2IF = 0;                                                        //Clear interrupt flag
    PIE1bits.TMR2IE = 1;                                                        //Enable Timer2 interrupt
    IPR1bits.TMR2IP = 0;                                                        //Timer2 interrupt = high priority
    PR2 = 122;                                                                  //Register to match Timer2 value to
    T2CONbits.TMR2ON = 1;                                                       //Turn on Timer2
    INTCONbits.GIEH = 1;                                                        //Enable all unmasked interrupts
    
    
    TurnRelayOff();                                                             //Keep Mains off of the output
    TurnSSR1Off();                                                              //Double check the relay is off
    
    FiringStatusUCG = 0;                                                        //indicate that there is no fault at the start
    ResetStatusUCG = 0;                                                         //indicate that coms should be fine now
    
    EEPROMRead();                                                               //pull the coms data
    if(EEPRead[2]!= 0xAA)                                                       //if it is equal to AA then just clear.
    {
        AlarmStatusUCG = FAULT;
        checkCableFaults();
    }
    else
        AlarmStatusUCG = CLEAR;
    
    EEPROMClearReset();                                                         //ensure on start that it is always cleared
            
    NewCFStatusUCG = 0;
    ClearToFire = 0;
    CFLEDFlashEnable = 0;
    CFLEDFlashState = 0;

    DeArmTimerEnabled = 0;
    DeArmTickCG = 0;
    
    if(InitST7540()){
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

    //we have now been in a test state for 50ms=+- thus start sampling
    
    int Failed=1;                                                               //tracks if we have any passes
    for(int i=0; i<40 ;i++){                                                    //conduct 40 samples
        __delay_us(10);                                                         //wait between samples
        earthLeakageValueUS = ReadAnalogVoltage(ADC_VOLT_EL);                   //Read the EL test pin    
        if(earthLeakageValueUS < VOLT_EL_HIGH){                                 //if it is not a EL fault then clear it.
            Failed=0;           //clear the failed marker
            earthLeakageCounter =0;
        }
    }
    
    if(Failed){                                                                 //if we fail then add 1 to the counter
        (earthLeakageCounter== COUNTER_EL)?0:earthLeakageCounter++;             //if counter <40 then ++ else leave counter cableFaultCounter = 
    }
    else{
        earthLeakageCounter=0;
    }
    
    earthLeakageProblemUCG = (earthLeakageCounter == COUNTER_EL)?1:0;           //if the counter has reached the trigger limit then trigger

}

void ReadCableFault(void){

    unsigned short cableFaultValueUS;

    __delay_us(24);                                                             
    cableFaultValueUS = ReadAnalogVoltage(ADC_VOLT_CF);
    if(cableFaultValueUS > VOLT_CF_HIGH){
        (cableFaultCounter== COUNTER_CF)?0:cableFaultCounter++;                 //if counter <40 then ++ else leave counter cableFaultCounter = 
    }
    else{
        cableFaultCounter =0;
    }
    cableFaultProblemUCG = (cableFaultCounter == COUNTER_CF)?1:0;
            
}

unsigned char ReadMAINS_ZeroCrossing(void){
    unsigned char mainsZero_CrossingValueUSG2;
    
    mainsZero_CrossingValueUSG = PORT_MAINS_ZEROCROSSING;
    WaitTickCount(1);                                                           //Debounce
    mainsZero_CrossingValueUSG2 = PORT_MAINS_ZEROCROSSING;
    if (mainsZero_CrossingValueUSG == mainsZero_CrossingValueUSG2)
        return mainsZero_CrossingValueUSG;
    else
        return 0;                                                               //NULL;
}

void ReadFireOut(void){    
    
    for (int i = 0; i < 50; i++){
        while(ReadMAINS_ZeroCrossing() == 0);
        FireOutFloat += (float)ReadAnalogVoltage(ADC_VOLT_FO);
        while(ReadMAINS_ZeroCrossing() == 1);
    }    
    FireOutFloat = FireOutFloat/50.0;
}

void TurnRelayOff(void){
    LAT_RELAY1 = OFF;                                                           //ON;
    if (RelayStatusUCG == OFF)
        Pi_Status_Update = DEFAULT_UPDATE;
    RelayStatusUCG = ON;
}

void TurnRelayOn(void){
    LAT_RELAY1 = ON;                                                            //OFF;
    if (RelayStatusUCG == ON)
        Pi_Status_Update = DEFAULT_UPDATE;
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

void checkKeySwitch(void){
    static unsigned char highCount;
    static unsigned char lowCount;
    unsigned char zeroCross;
    
    zeroCross = PORT_MAINS_ZEROCROSSING;
    if(zeroCross){
        lowCount = 0;
        highCount++;
        if(highCount > 10)
            keySwitch = 1;
    }
    else{
        highCount = 0;
        lowCount ++;
        if(lowCount > 40)
            keySwitch = 0;
    }
}

unsigned char getKeySwitchState(void){                                          //Half Wave rectification,
    return keySwitch;
}

void getResetAndFireButtonState(void){
    unsigned char FireButtonStatusfirstUC;
    unsigned char FireButtonStatussecondUC;
    unsigned char ResetButtonStatusfirstUC;
    unsigned char ResetButtonStatussecondUC;

    FireButtonStatusfirstUC = PORT_FIRE_SWITCH;
    ResetButtonStatusfirstUC = PORT_RESET_SWITCH;
    
    WaitTickCount(20);                                                          //Debounce for 10mS
    
    FireButtonStatussecondUC = PORT_FIRE_SWITCH;
    ResetButtonStatussecondUC = PORT_RESET_SWITCH;

    if (FireButtonStatusfirstUC != FireButtonStatussecondUC)
        fireButton = DEPRESSED;
    else{
        if (FireButtonStatussecondUC == 1)
            fireButton = DEPRESSED;                                                   //Inverted Logic
        else
            fireButton = PRESSED;
    }
    
    if (ResetButtonStatusfirstUC != ResetButtonStatussecondUC)
        resetButton = DEPRESSED;
     else{
        if (ResetButtonStatussecondUC == 1)
            resetButton = DEPRESSED;
        else
            resetButton = PRESSED;
    }    
}

unsigned char getFireButtonState(void){
    return fireButton;
}

unsigned char getResetButtonState(void){
    return resetButton;
}

void EEPROMWrite(){
    unsigned int address;
    unsigned char q=0;
    address = 0x0200;
    INTCONbits.GIEH = 0;                                                        //Disable all unmasked interrupts
    Write_b_eep (address, EEPWrite[q]);                                         // write into to EEPROM

        address++;                                                              //increment the address of EEPROM to next location
    /* Checks & waits the status of ER bit in EECON1 register */
        Busy_eep();
        q++;
    Write_b_eep (address, EEPWrite[q]);                                         // write into to EEPROM
        address++;                                                              //increment the address of EEPROM to next location
    /* Checks & waits the status of ER bit in EECON1 register */
    Busy_eep();
    INTCONbits.GIEH = 1;                                                        //Enable all unmasked interrupts

}

void EEPROMClearReset(){
    unsigned int address;
    address = 0x0202;
    INTCONbits.GIEH = 0;                                                        //Disable all unmasked interrupts
    Write_b_eep (address, 0x00);                                                //write into to EEPROM
                                                                                //increment the address of EEPROM to next location
    /* Checks & waits the status of ER bit in EECON1 register */
        Busy_eep();
    INTCONbits.GIEH = 1;                                                        //Enable all unmasked interrupts
}

void EEPROMSetReset(){
    unsigned int address;
    address = 0x0202;
    INTCONbits.GIEH = 0;                                                        //Disable all unmasked interrupts
    Write_b_eep (address, 0xAA);                                                //write into to EEPROM
                                                                                //increment the address of EEPROM to next location
    /* Checks & waits the status of ER bit in EECON1 register */
        Busy_eep();
    INTCONbits.GIEH = 1;                                                        //Enable all unmasked interrupts
}

void EEPROMRead(void){
    unsigned int address;
    address = 0x0200;
    EEPRead[0] = Read_b_eep (address);                                          //read the EEPROM data written previously from corresponding address
    address++;
    EEPRead[1] = Read_b_eep (address);                                          //read the EEPROM data written previously from corresponding address
    address++;
    EEPRead[2] = Read_b_eep (address);                                          //read the EEPROM data for the reset bits
    address++;
}

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
	while(EECON1bits.WR);                                                       //Wait till the write completion
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
	Nop();                                                                      //Nop may be required for latency at high frequencies
	Nop();                                                                      //Nop may be required for latency at high frequencies
	return ( EEDATA );                                                          // return with read byte
}