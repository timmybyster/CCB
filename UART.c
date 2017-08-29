#include "UART.h"
#include "main.h"

//ReadRS232
unsigned char flagUART = 0;
unsigned char flagUARTRead = 0;
int ReceivedPiBufferIndexIG = 0;
int TransmitPiBufferIG = 0;
unsigned char PiBufferTXLenUCA = 0;
unsigned char PiBufferTXNEXTUCA = 0;
unsigned char PiBufferRXNEXTUCA = 0;
char ReceivedPiBufferCG[PI_RECEIVE_BUFFER_LENGTH];                              //Character Buffer to receive packets from the Pi, Should be fixed length of 7
char TransmitPiBufferCG[80];

//Function prototypes

char UART_Init(const long int baudrate);
char UART_Read(void);
void UART_Write(char data);
void UART_TX_ISRHandler(void);
void CreateMessageUART(unsigned short packetSourceUS, unsigned char commandUC, unsigned char dataLenUC, char *dataBuf);
void SendUARTPacket(void);
unsigned char DataReadyUART(void);
unsigned short PacketReadParamUART(unsigned char paramName);
unsigned char CheckCRCUART(void);
void ClearDataReady(void);
void CreateMessageUARTTest(unsigned char Command);
void CreatePiToIBCUARTMessage(unsigned char Command, unsigned short packetSourceUS);
void CreateMessageUARTDEBUG(unsigned char dataLenUC,unsigned short packetSourceUS, unsigned short packetDestUS,unsigned char packetNOUC,unsigned char commandUC,char *dataBuf);

//External function prototypes
extern unsigned short CRC16(char *, unsigned short);


char UART_Init(const long int baudrate)
{
  unsigned int x;
  x = (_XTAL_FREQ*4 - baudrate*64)/(baudrate*64);                               //SPBRG for Low Baud Rate
  if(x>255)                                                                     //If High Baud Rage Required
  {
    x = (_XTAL_FREQ*4 - baudrate*16)/(baudrate*16);                             //SPBRG for High Baud Rate
    TXSTA1bits.BRGH = 1;                                                        //Setting High Baud Rate
  }
  if(x<256)
  {
    x=25;
    SPBRG1 = x;                                                                 //Writing SPBRG Register
    TXSTA1bits.SYNC = 0;                                                        //Setting Asynchronous Mode, ie UART
    RCSTA1bits.SPEN = 1;                                                        //Enables Serial Port
    RCSTA1bits.CREN = 1;                                                        //Enables Continuous Reception
    TXSTA1bits.TXEN = 1;                                                        //Enables Transmission
    return 1;                                                                   //Returns 1 to indicate Successful Completion
  }
  return 0;                                                                     //Returns 0 to indicate UART initialization failed
}

char UART_Read(void)
{
    char charRS232;   
    charRS232 = RCREG1;                                                         //get a single character off the USART line                  
        if((PI_RECEIVE_BUFFER_LENGTH - 1) == PiBufferRXNEXTUCA){                //Entire packet read?       
            flagUARTRead = 1;
            ReceivedPiBufferCG[PiBufferRXNEXTUCA] = charRS232;
            PiBufferRXNEXTUCA = 0;                                              //Indicate we have received a full packet      
            return;
        }
        else if (PiBufferRXNEXTUCA > PI_RECEIVE_BUFFER_LENGTH  || flagUARTRead == 1){
            PiBufferRXNEXTUCA = 0;                                              //ensure it never gets stuck
        }
        else{            
            flagUARTRead = 0;
            ReceivedPiBufferCG[PiBufferRXNEXTUCA] = charRS232;
            PiBufferRXNEXTUCA++;
            return;
        }    
}

void resetUARTPointers(void){
    flagUARTRead = 1;
    PiBufferRXNEXTUCA = 0;
}

unsigned short PacketReadParamUART(unsigned char paramName){
    unsigned short retValueUS = 0;

    switch(paramName){        
        case(UART_HEADER):                                                      //Retrieve the source address
            retValueUS |= ((unsigned short) ReceivedPiBufferCG[0] << 8);
            retValueUS |= ReceivedPiBufferCG[1];
            return retValueUS;
        case(UART_CMD):                                                         //Retrieve the command
            return ReceivedPiBufferCG[2];
        case(UART_SN):                                                          //Retrieve the source address
            retValueUS |= ((unsigned short) ReceivedPiBufferCG[3] << 8);
            retValueUS |= ReceivedPiBufferCG[4];
            return retValueUS;
        case(UART_CRC_VALID):                                                   //Does the CRC match
            return CheckCRCUART();
    }

    return 0;
}

unsigned char CheckCRCUART(void){
    unsigned short expectedCRCUS = 0;
    unsigned short receivedCRCUS = 0;
    unsigned char packetLenUC;
    unsigned short packetCRCUS = 0;

    packetLenUC = PI_RECEIVE_BUFFER_LENGTH-2;
    expectedCRCUS = CRC16(ReceivedPiBufferCG, packetLenUC);                     //Calculate the expected CRC
    receivedCRCUS |= (((unsigned short) ReceivedPiBufferCG[5]) << 8);
    receivedCRCUS |= ReceivedPiBufferCG[5+1];

    return (expectedCRCUS == receivedCRCUS);                                    //Compare to the received CRC
}

unsigned char DataReadyUART(void){
    return flagUARTRead;                                                        //Is the received data ready
}

void ClearDataReady(void){
    flagUARTRead = 0;
}

//Function used to create downstream messages from the Pi to the IBC to simulate commands from the Pi to the IBC
//through an interface terminal on a PC
void CreatePiToIBCUARTMessage(unsigned char Command, unsigned short packetSourceUS){
    unsigned short packetCRCUS = 0;
    unsigned short packetCRCUStemp = 0;

    TransmitPiBufferCG[0] = 0xAA;                                               //0x42;//0xAA;   //B                                             //Preamble
    TransmitPiBufferCG[1] = 0xAA;                                               //0x41;//0xAA;

    TransmitPiBufferCG[2] = Command;                                            //0x2B;//Command;                                                //Command

    TransmitPiBufferCG[3] = packetSourceUS >> 8;                                //Serial No MSB
    TransmitPiBufferCG[4] = packetSourceUS;                                     //Serial No LSB

    packetCRCUS = CRC16(TransmitPiBufferCG, ((unsigned short)PI_TRANSMIT_MIN_BUFFER_LENGTH - 2));
    packetCRCUStemp = CRC16(TransmitPiBufferCG, 5);                             //Calcualte CRC
    TransmitPiBufferCG[5] = packetCRCUStemp >> 8;
    TransmitPiBufferCG[6] = packetCRCUStemp;

            PiBufferTXLenUCA = 6;
            Nop();
}

void CreateMessageUARTTest(unsigned char Command){
    //unsigned char dataBufLocUC;
    unsigned short packetCRCUS = 0;
    unsigned short packetCRCUStemp = 0;    

    TransmitPiBufferCG[0] = 0xAA;                                               //0x42;//0xAA;   //B                                             //Preamble
    TransmitPiBufferCG[1] = 0xAA;                                               //0x41;//0xAA;

    TransmitPiBufferCG[2] = Command;                                            //0x2B;//Command;                                             //Command

    TransmitPiBufferCG[3] = 0x31;                                               //Serial No MSB
    TransmitPiBufferCG[4] = 0x30;                                               //Serial No LSB

    packetCRCUS = CRC16(TransmitPiBufferCG, ((unsigned short)PI_TRANSMIT_MIN_BUFFER_LENGTH - 2));
    packetCRCUStemp = CRC16(TransmitPiBufferCG, 5);                             //Calcualte CRC
    TransmitPiBufferCG[5] = packetCRCUStemp >> 8;
    TransmitPiBufferCG[6] = packetCRCUStemp;
    //TransmitPiBufferCG[5] = 0x43; //C
    //TransmitPiBufferCG[6] = 0x52; //R
            
            PiBufferTXLenUCA = 6;
            Nop();

    ReceivedPiBufferCG[0] = TransmitPiBufferCG[0];
    ReceivedPiBufferCG[1] = TransmitPiBufferCG[1];
    ReceivedPiBufferCG[2] = TransmitPiBufferCG[2];
    ReceivedPiBufferCG[3] = TransmitPiBufferCG[3];
    ReceivedPiBufferCG[4] = TransmitPiBufferCG[4];
    ReceivedPiBufferCG[5] = TransmitPiBufferCG[5];
    ReceivedPiBufferCG[6] = TransmitPiBufferCG[6];

    flagUARTRead = 1;

}

void CreateMessageUARTDEBUG(unsigned char dataLenUC,unsigned short packetSourceUS, unsigned short packetDestUS,unsigned char packetNOUC,unsigned char commandUC,char *dataBuf){
    unsigned char dataBufLocUC;
    unsigned short packetCRCUS;

    TransmitPiBufferCG[0] = 0xF3;                                               //Preamble
    TransmitPiBufferCG[1] = 0xCF;

    TransmitPiBufferCG[2] = dataLenUC;

    TransmitPiBufferCG[3] = packetSourceUS >> 8;                                //IBC-1 S/N
    TransmitPiBufferCG[4] = packetSourceUS;



    TransmitPiBufferCG[5] = packetDestUS >> 8;                                  //IBC-1 S/N
    TransmitPiBufferCG[6] = packetDestUS;

    TransmitPiBufferCG[7] = packetNOUC;

    TransmitPiBufferCG[8] = commandUC;

    TransmitPiBufferCG[9] = 0xFF;

    for(dataBufLocUC = 0; dataBufLocUC < dataLenUC; dataBufLocUC++)             //Packet payload
        TransmitPiBufferCG[dataBufLocUC + 10] = dataBuf[dataBufLocUC];
    dataBufLocUC += 10;

    packetCRCUS = CRC16(TransmitPiBufferCG, 10 + dataLenUC - 2);                                                    //Calcualte CRC
    TransmitPiBufferCG[dataBufLocUC++] = packetCRCUS >> 8;
    TransmitPiBufferCG[dataBufLocUC] = packetCRCUS;
    PiBufferTXLenUCA = dataBufLocUC++;

    //PiBufferTXLenUCA = dataBufLocUC;

}

void CreateMessageUART(unsigned short packetSourceUS, unsigned char commandUC, unsigned char dataLenUC, char *dataBuf){
    unsigned char dataBufLocUC;
    unsigned short packetCRCUS;

    TransmitPiBufferCG[0] = 0xAA;                                               //Preamble
    TransmitPiBufferCG[1] = 0xAA;

    TransmitPiBufferCG[2] = PI_TRANSMIT_MIN_BUFFER_LENGTH + dataLenUC;          //Packet length

    TransmitPiBufferCG[3] = commandUC;                                          //Command

    TransmitPiBufferCG[4] = packetSourceUS >> 8;                                //IBC-1 S/N
    TransmitPiBufferCG[5] = packetSourceUS;

    for(dataBufLocUC = 0; dataBufLocUC < dataLenUC; dataBufLocUC++)             //Packet payload
        TransmitPiBufferCG[dataBufLocUC + 6] = dataBuf[dataBufLocUC];
    dataBufLocUC += 6;

    packetCRCUS = CRC16(TransmitPiBufferCG, PI_TRANSMIT_MIN_BUFFER_LENGTH + dataLenUC - 2);                                                    //Calcualte CRC
    TransmitPiBufferCG[dataBufLocUC++] = packetCRCUS >> 8;
    TransmitPiBufferCG[dataBufLocUC] = packetCRCUS;
    PiBufferTXLenUCA = dataBufLocUC++;
}

void SendUARTPacket(void){

    flagUART |= FLAG_UART_TX_ACTIVE;                                            //Set TX state as active
    UART_Write(TransmitPiBufferCG[0]);                                          //Write first byte
    PiBufferTXNEXTUCA = 1;                                                      //Set index
}

void UART_Write(char data)
{
  while(!TXSTA1bits.TRMT);                                                      //Check the buffer is empty
  TXREG1 = data;                                                                //Write character into buffer
}

void UART_TX_ISRHandler(void){

    if(flagUART & FLAG_UART_TX_ACTIVE){                                         //Are we in TX mode
        if(PiBufferTXNEXTUCA > PiBufferTXLenUCA){                               //Done transmitting buffer?
            flagUART &= ~FLAG_UART_TX_ACTIVE;                                   //Disable TX mode
            return;
        }
        UART_Write(TransmitPiBufferCG[PiBufferTXNEXTUCA++]);                    //Write next byte        
    }

}
