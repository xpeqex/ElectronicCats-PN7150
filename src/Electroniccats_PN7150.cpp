/**
 * NXP PN7150 Driver 
 * Authors: 
 *        Salvador Mendoza - @Netxing - salmg.net
 *        Andres Sabas - Electronic Cats - Electroniccats.com
 *
 *  March 2020
 * 
 * This code is beerware; if you see me (or any other collaborator 
 * member) at the local, and you've found our code helpful, 
 * please buy us a round!
 * Distributed as-is; no warranty is given.
 *
 * Some methods and ideas were extract from https://github.com/Strooom/PN7150
 *
 *
 */
#include "Electroniccats_PN7150.h"
//#define RW_SUPPORT                 // Compile with reader/writer support
#define CARDEMU_SUPPORT          // Compile with emulation support


unsigned char DiscoveryTechnologies[] = {
#ifdef CARDEMU_SUPPORT
        MODE_LISTEN | MODE_POLL
#endif
#ifdef RW_SUPPORT
            MODE_POLL | TECH_PASSIVE_NFCA,
            MODE_POLL | TECH_PASSIVE_NFCF,
            MODE_POLL | TECH_PASSIVE_NFCB,
            MODE_POLL | TECH_PASSIVE_15693,
#endif
};

static uint8_t gNextTag_Protocol = PROT_UNDETERMINED;

Electroniccats_PN7150::Electroniccats_PN7150(uint8_t IRQpin, uint8_t VENpin, uint8_t I2Caddress): 
    _IRQpin(IRQpin), 
    _VENpin(VENpin), 
    _I2Caddress(I2Caddress)
{
    pinMode(_IRQpin, INPUT);                                             
    pinMode(_VENpin, OUTPUT);    
}

int Electroniccats_PN7150::begin() {
    Wire.begin();   
    digitalWrite(_VENpin, HIGH);                                         
    delay(1);                                                            
    digitalWrite(_VENpin, LOW);                                           
    delay(1);                                                           
    digitalWrite(_VENpin, HIGH);                                         
    delay(3);
    return SUCCESS;
}

bool Electroniccats_PN7150::hasMessage() const {
    return (HIGH == digitalRead(_IRQpin)); // PN7150 indicates it has data by driving IRQ signal HIGH
}

uint8_t Electroniccats_PN7150::writeData(uint8_t txBuffer[], uint32_t txBufferLevel) const {
    uint32_t nmbrBytesWritten = 0;
    Wire.beginTransmission((uint8_t)_I2Caddress);                    
    nmbrBytesWritten = Wire.write(txBuffer, txBufferLevel);          
    if (nmbrBytesWritten == txBufferLevel) {                             
        byte resultCode;
        resultCode = Wire.endTransmission();                            
        return resultCode;
    }
    else {
        return 4; // Could not properly copy data to I2C buffer, so treat as other error, see i2c_t3
    }
}

uint32_t Electroniccats_PN7150::readData(uint8_t rxBuffer[]) const {
    uint32_t bytesReceived; // keeps track of how many bytes we actually received
    if (hasMessage()){       // only try to read something if the PN7150 indicates it has something
        bytesReceived = Wire.requestFrom((int)_I2Caddress, 3);           // first reading the header, as this contains how long the payload will be

        rxBuffer[0] = Wire.read();
        rxBuffer[1] = Wire.read();
        rxBuffer[2] = Wire.read();
        uint8_t payloadLength = rxBuffer[2];
        if (payloadLength > 0) {
            bytesReceived += Wire.requestFrom((int)_I2Caddress, payloadLength);      // then reading the payload, if any
            uint32_t index = 3;
            while (index < bytesReceived) {
                rxBuffer[index] = Wire.read();
                index++;
            }
            index = 0;
        }
    }
    else {
        bytesReceived = 0;
    }
    return bytesReceived;
}

bool Electroniccats_PN7150::isTimeOut() const {                  
    return ((millis() - timeOutStartTime) >= timeOut);
}

void Electroniccats_PN7150::setTimeOut(unsigned long theTimeOut){ 
    timeOutStartTime = millis();
    timeOut = theTimeOut;
}

bool Electroniccats_PN7150::getMessage(uint16_t timeout){         // check for message using timeout, 5 milisec as default
    setTimeOut(timeout);
    rxMessageLength = 0;
    while(!isTimeOut()){
        rxMessageLength = readData(rxBuffer);
        if (rxMessageLength) break;
        else if(timeout == 1331) setTimeOut(timeout);
    }
    return rxMessageLength;
}

int Electroniccats_PN7150::wakeupNCI() {                         // the device has to wake up using a core reset
    uint8_t NCICoreReset[] = {0x20, 0x00, 0x01, 0x01};
    //uint8_t Answer[6];
    uint16_t NbBytes = 0;

    /* Reset RF settings restauration flag */
    (void) writeData(NCICoreReset, 4); 
    getMessage();
    NbBytes = rxMessageLength;
    if ((NbBytes == 0) || (rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x00)) {
        return ERROR;
    }
    getMessage();
    NbBytes = rxMessageLength;
    if (NbBytes != 0) {
        //NCI_PRINT_BUF("NCI << ", Answer, NbBytes);
        /* Is CORE_GENERIC_ERROR_NTF ? */
        if ((rxBuffer[0] == 0x60) && (rxBuffer[1] == 0x07)) {
            /* Is PN7150B0HN/C11004 Anti-tearing recovery procedure triggered ? */
            //if ((rxBuffer[3] == 0xE6)) gRfSettingsRestored_flag = true;
        }
        else {
            return ERROR;
        }
    }
    return SUCCESS;
}

int Electroniccats_PN7150::connectNCI(){
    uint8_t i = 2;
    uint8_t NCICoreInit[] = {0x20, 0x01, 0x00};
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;

    // Open connection to NXPNCI 
    begin();
    // Loop until NXPNCI answers 
    while(wakeupNCI() != SUCCESS) {
        if(i-- == 0) return ERROR;
        delay(500);
    }

    (void) writeData(NCICoreInit, sizeof(NCICoreInit));
    getMessage();
    if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x01) || (rxBuffer[3] != 0x00)) return ERROR;

    // Retrieve NXP-NCI NFC Controller generation 
    if (rxBuffer[17+rxBuffer[8]] == 0x08) gNfcController_generation = 1;
    else if (rxBuffer[17+rxBuffer[8]] == 0x10) gNfcController_generation = 2;

    // Retrieve NXP-NCI NFC Controller FW version 
    gNfcController_fw_version[0] = Answer[17+Answer[8]]; //0xROM_CODE_V
    gNfcController_fw_version[1] = Answer[18+Answer[8]]; //0xFW_MAJOR_NO
    gNfcController_fw_version[2] = Answer[19+Answer[8]]; //0xFW_MINOR_NO

    return SUCCESS;
}

int Electroniccats_PN7150::GetFwVersion(){
    return ((gNfcController_fw_version[0] & 0xFF ) << 16) | ((gNfcController_fw_version[1] & 0xFF ) << 8) | (gNfcController_fw_version[2] & 0xFF);
}

int Electroniccats_PN7150::ConfigMode(){
    unsigned mode = 0
#ifdef CARDEMU_SUPPORT
              | MODE_CARDEMU
#endif
#ifdef RW_SUPPORT
              | MODE_RW
#endif
    ;
    uint8_t Command[MAX_NCI_FRAME_SIZE];
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;
    uint8_t Item = 0;
    uint8_t NCIDiscoverMap[] = {0x21, 0x00};

#ifdef CARDEMU_SUPPORT
    //Emulation mode
    const uint8_t DM_CARDEMU[] = {0x4, 0x2, 0x2};
    const uint8_t R_CARDEMU[] = {0x1, 0x3, 0x0, 0x1, 0x4};
#endif
#ifdef RW_SUPPORT
    //RW Mode
    const uint8_t DM_RW[] = {0x1, 0x1, 0x1, 0x2, 0x1, 0x1, 0x3, 0x1, 0x1, 0x4, 0x1, 0x2, 0x80, 0x01, 0x80};
    uint8_t NCIPropAct[] = {0x2F, 0x02, 0x00};
#endif

    uint8_t NCIRouting[] = {0x21, 0x01, 0x07, 0x00, 0x01};
    uint8_t NCISetConfig_NFCA_SELRSP[] = {0x20, 0x02, 0x04, 0x01, 0x32, 0x01, 0x00};

    if(mode == 0) return SUCCESS;

    /* Enable Proprietary interface for T4T card presence check procedure */

#ifdef RW_SUPPORT
    if (mode == MODE_RW){
        (void) writeData(NCIPropAct, sizeof(NCIPropAct)); 
        getMessage();

        if ((rxBuffer[0] != 0x4F) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00)) return ERROR;
    }
#endif
    //* Building Discovery Map command 
    Item = 0;

#ifdef CARDEMU_SUPPORT
    if (mode & MODE_CARDEMU) {
        memcpy(&Command[4+(3*Item)], DM_CARDEMU, sizeof(DM_CARDEMU));
        Item++;
    }
#endif
#ifdef RW_SUPPORT
    if (mode & MODE_RW) {
        memcpy(&Command[4+(3*Item)], DM_RW, sizeof(DM_RW));
        Item+=sizeof(DM_RW)/3;
    }
#endif
    if (Item != 0) {
        memcpy(Command, NCIDiscoverMap, sizeof(NCIDiscoverMap));
        Command[2] = 1 + (Item*3);
        Command[3] = Item;
        (void) writeData(Command, 3 + Command[2]); 
        getMessage(10);
        if ((rxBuffer[0] != 0x41) || (rxBuffer[1] != 0x00) || (rxBuffer[3] != 0x00)){
            return ERROR;
        }
    }

    //* Configuring routing 
    Item = 0;
#ifdef CARDEMU_SUPPORT
    if (mode & MODE_CARDEMU) {
        memcpy(&Command[5+(5*Item)], R_CARDEMU, sizeof(R_CARDEMU));
        Item++;
    }
    if (Item != 0) {
        memcpy(Command, NCIRouting, sizeof(NCIRouting));
        Command[2] = 2 + (Item*5);
        Command[4] = Item;
        (void) writeData(Command, 3 + Command[2]); 
        getMessage();
        if ((rxBuffer[0] != 0x41) || (rxBuffer[1] != 0x01) || (rxBuffer[3] != 0x00))
            return ERROR;
    }
    //* Setting NFCA SEL_RSP 
    if (mode & MODE_CARDEMU) 
        NCISetConfig_NFCA_SELRSP[6] += 0x20;

    if (NCISetConfig_NFCA_SELRSP[6] != 0x00){
        (void) writeData(NCISetConfig_NFCA_SELRSP, sizeof(NCISetConfig_NFCA_SELRSP)); 
        getMessage();

        if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00))
            return ERROR;
        else 
            return SUCCESS;
    }
#endif
    return SUCCESS;
}


int Electroniccats_PN7150::StartDiscovery(){
    unsigned char TechTabSize = sizeof(DiscoveryTechnologies);
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;

    uint8_t NCIStartDiscovery[30];
    uint8_t NCIStartDiscovery_length = 0;

    NCIStartDiscovery[0] = 0x21;
    NCIStartDiscovery[1] = 0x03;
    NCIStartDiscovery[2] = (TechTabSize * 2) + 1;
    NCIStartDiscovery[3] = TechTabSize;
    for (uint8_t i = 0; i<TechTabSize; i++) {
        NCIStartDiscovery[(i*2)+4] = DiscoveryTechnologies[i];
        NCIStartDiscovery[(i*2)+5] = 0x01;
    }

    NCIStartDiscovery_length = (TechTabSize * 2) + 4;
    (void) writeData(NCIStartDiscovery, NCIStartDiscovery_length); 
    getMessage();

    if ((rxBuffer[0] != 0x41) || (rxBuffer[1] != 0x03) || (rxBuffer[3] != 0x00))
        return ERROR; 
    else 
        return SUCCESS;
}

bool Electroniccats_PN7150::CardModeSend (unsigned char *pData, unsigned char DataSize){
    bool status;
    uint8_t Cmd[MAX_NCI_FRAME_SIZE];
    uint8_t Ans[MAX_NCI_FRAME_SIZE];
    uint16_t AnsSize;

    /* Compute and send DATA_PACKET */
    Cmd[0] = 0x00;
    Cmd[1] = 0x00;
    Cmd[2] = DataSize;
    memcpy(&Cmd[3], pData, DataSize);
    (void) writeData(Cmd, DataSize+3); 
    return status;
}

bool Electroniccats_PN7150::CardModeReceive (unsigned char *pData, unsigned char *pDataSize) {
    bool status = NFC_ERROR;
    uint8_t Ans[MAX_NCI_FRAME_SIZE];
    uint16_t AnsSize;
    (void) writeData(Ans, sizeof(Ans)); 
    getMessage();

    /* Is data packet ? */
    if ((rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00)) {
        *pDataSize = rxBuffer[2];
        memcpy(pData, &rxBuffer[3], *pDataSize);
        status = SUCCESS;
    }
    else{
        return NFC_ERROR;
    }
    return NFC_SUCCESS;
}

static void Electroniccats_PN7150::FillInterfaceInfo(RfIntf_t* pRfIntf, uint8_t* pBuf){
    uint8_t i, temp;

    switch(pRfIntf->ModeTech){
        case (MODE_POLL | TECH_PASSIVE_NFCA):
            memcpy(pRfIntf->Info.NFC_APP.SensRes, &pBuf[0], 2);
            temp = 2;
            pRfIntf->Info.NFC_APP.NfcIdLen = pBuf[temp];
            temp++;
            memcpy(pRfIntf->Info.NFC_APP.NfcId, &pBuf[3], pRfIntf->Info.NFC_APP.NfcIdLen);
            temp+=pBuf[2];
            pRfIntf->Info.NFC_APP.SelResLen = pBuf[temp];
            temp++;

            if(pRfIntf->Info.NFC_APP.SelResLen == 1) 
                pRfIntf->Info.NFC_APP.SelRes[0] = pBuf[temp];

            temp+=4;
            if(pBuf[temp] != 0){
                temp++;
                pRfIntf->Info.NFC_APP.RatsLen = pBuf[temp];
                memcpy(pRfIntf->Info.NFC_APP.Rats, &pBuf[temp+1], pBuf[temp]);
            }
            else{
                pRfIntf->Info.NFC_APP.RatsLen = 0;
            }
            break;

        case (MODE_POLL | TECH_PASSIVE_NFCB):
            pRfIntf->Info.NFC_BPP.SensResLen = pBuf[0];
            memcpy(pRfIntf->Info.NFC_BPP.SensRes, &pBuf[1], pRfIntf->Info.NFC_BPP.SensResLen);
            temp = pBuf[0] + 4;
            if(pBuf[temp] != 0){
                temp++;
                pRfIntf->Info.NFC_BPP.AttribResLen = pBuf[temp];
                memcpy(pRfIntf->Info.NFC_BPP.AttribRes, &pBuf[temp+1], pBuf[temp]);
            }
            else{
                pRfIntf->Info.NFC_BPP.AttribResLen = 0;
            }
            break;

        case (MODE_POLL | TECH_PASSIVE_NFCF):
            pRfIntf->Info.NFC_FPP.BitRate = pBuf[0];
            pRfIntf->Info.NFC_FPP.SensResLen = pBuf[1];
            memcpy(pRfIntf->Info.NFC_FPP.SensRes, &pBuf[2], pRfIntf->Info.NFC_FPP.SensResLen);
            break;

        case (MODE_POLL | TECH_PASSIVE_15693):
            pRfIntf->Info.NFC_VPP.AFI = pBuf[0];
            pRfIntf->Info.NFC_VPP.DSFID = pBuf[1];

            for(i=0; i<8; i++) 
                pRfIntf->Info.NFC_VPP.ID[7-i] = pBuf[2+i];

            break;

        default:
            break;
    }
}

bool Electroniccats_PN7150::ReaderTagCmd (unsigned char *pCommand, unsigned char CommandSize, unsigned char *pAnswer, unsigned char *pAnswerSize){
    bool status = ERROR;
    uint8_t Cmd[MAX_NCI_FRAME_SIZE];
    uint8_t Ans[MAX_NCI_FRAME_SIZE];
    uint16_t AnsSize;

    /* Compute and send DATA_PACKET */
    Cmd[0] = 0x00;
    Cmd[1] = 0x00;
    Cmd[2] = CommandSize;
    memcpy(&Cmd[3], pCommand, CommandSize);

    (void) writeData(Cmd, CommandSize+3); 
    getMessage(1000);
    /* Wait for Answer 1S */

    if ((rxBuffer[0] == 0x0) && (rxBuffer[1] == 0x0))
        status = SUCCESS;

    *pAnswerSize = rxBuffer[2];
    memcpy(pAnswer, &rxBuffer[3], *pAnswerSize);

    return status;
}

bool Electroniccats_PN7150::WaitForDiscoveryNotification(RfIntf_t *pRfIntf){
    uint8_t NCIRfDiscoverSelect[] = {0x21, 0x04, 0x03, 0x01, PROT_ISODEP, INTF_ISODEP};
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;
    static uint8_t gNextTag_Protocol = PROT_UNDETERMINED;

    do {
        getMessage(1331); //Infinite loop, waiting for response
    }while ((rxBuffer[0] != 0x61) || ((rxBuffer[1] != 0x05) && (rxBuffer[1] != 0x03)));
    Serial.println("After loop");

    gNextTag_Protocol = PROT_UNDETERMINED;

    /* Is RF_INTF_ACTIVATED_NTF ? */
    if (rxBuffer[1] == 0x05){
        pRfIntf->Interface = rxBuffer[4];
        pRfIntf->Protocol = rxBuffer[5];
        pRfIntf->ModeTech = rxBuffer[6];
        pRfIntf->MoreTags = false;
        FillInterfaceInfo(pRfIntf, &rxBuffer[10]);
    }
    else{ /* RF_DISCOVER_NTF */
        pRfIntf->Interface = INTF_UNDETERMINED;
        pRfIntf->Protocol = rxBuffer[4];
        pRfIntf->ModeTech = rxBuffer[5];
        pRfIntf->MoreTags = true;

        /* Get next NTF for further activation */
        do {
            if(!getMessage(100))   
                return ERROR;    
        } while ((rxBuffer[0] != 0x61) || (rxBuffer[1] != 0x03));
        gNextTag_Protocol = rxBuffer[4];

        /* Remaining NTF ? */

        while(rxBuffer[rxMessageLength-1] == 0x02) getMessage(100);

        /* In case of multiple cards, select the first one */
        NCIRfDiscoverSelect[4] = pRfIntf->Protocol;
        if (pRfIntf->Protocol == PROT_ISODEP) NCIRfDiscoverSelect[5] = INTF_ISODEP;
        else if (pRfIntf->Protocol == PROT_NFCDEP) NCIRfDiscoverSelect[5] = INTF_NFCDEP;
        else if (pRfIntf->Protocol == PROT_MIFARE) NCIRfDiscoverSelect[5] = INTF_TAGCMD;
        else NCIRfDiscoverSelect[5] = INTF_FRAME;

        (void) writeData(NCIRfDiscoverSelect, sizeof(NCIRfDiscoverSelect)); 
        getMessage(100);

        if ((rxBuffer[0] == 0x41) || (rxBuffer[1] == 0x04) || (rxBuffer[3] == 0x00)){
            (void) writeData(rxBuffer, rxMessageLength);
            getMessage(100);

            if ((rxBuffer[0] == 0x61) || (rxBuffer[1] == 0x05)){
                pRfIntf->Interface = rxBuffer[4];
                pRfIntf->Protocol = rxBuffer[5];
                pRfIntf->ModeTech = rxBuffer[6];
                FillInterfaceInfo(pRfIntf, &rxBuffer[10]);
            }
        }
    }

    /* In case of unknown target align protocol information */
    if (pRfIntf->Interface == INTF_UNDETERMINED) pRfIntf->Protocol = PROT_UNDETERMINED;

    return SUCCESS;
}

void Electroniccats_PN7150::ProcessReaderMode(RfIntf_t RfIntf, RW_Operation_t Operation){
    switch (Operation) {
#ifndef NO_NDEF_SUPPORT
        case READ_NDEF:
            //Working on it
            break;

        case WRITE_NDEF:
            //Working on it
            break;
#endif
        case PRESENCE_CHECK:
            PresenceCheck(RfIntf);
            break;
        default:
            break;
    }
}


void Electroniccats_PN7150::PresenceCheck(RfIntf_t RfIntf){
    bool status;
    uint8_t i;
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;

    uint8_t NCIPresCheckT1T[] = {0x00, 0x00, 0x07, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t NCIPresCheckT2T[] = {0x00, 0x00, 0x02, 0x30, 0x00};
    uint8_t NCIPresCheckT3T[] = {0x21, 0x08, 0x04, 0xFF, 0xFF, 0x00, 0x01};
    uint8_t NCIPresCheckIsoDep[] = {0x2F, 0x11, 0x00};
    uint8_t NCIPresCheckIso15693[] = {0x00, 0x00, 0x0B, 0x26, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t NCIDeactivate[] = {0x21, 0x06, 0x01, 0x01};
    uint8_t NCISelectMIFARE[] = {0x21, 0x04, 0x03, 0x01, 0x80, 0x80};

    switch (RfIntf.Protocol) {
        case PROT_T1T:
            do{
                delay(500);
                (void) writeData(NCIPresCheckT1T, sizeof(NCIPresCheckT1T)); 
                getMessage();
                getMessage(100);
            } while ((rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00));
            break;

        case PROT_T2T:
            do{
                delay(500);
                (void) writeData(NCIPresCheckT2T, sizeof(NCIPresCheckT2T)); 
                getMessage();
                getMessage(100);
            } while ((rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00) && (rxBuffer[2] == 0x11));
            break;

        case PROT_T3T:
            do{
                delay(500);
                (void) writeData(NCIPresCheckT3T, sizeof(NCIPresCheckT3T)); 
                getMessage();
                getMessage(100);
            } while ((rxBuffer[0] == 0x61) && (rxBuffer[1] == 0x08) && ((rxBuffer[3] == 0x00) || (rxBuffer[4] > 0x00)));
            break;

        case PROT_ISODEP:
            do{
                delay(500);
                (void) writeData(NCIPresCheckIsoDep, sizeof(NCIPresCheckIsoDep)); 
                getMessage();
                getMessage(100);
            } while ((rxBuffer[0] == 0x6F) && (rxBuffer[1] == 0x11) && (rxBuffer[2] == 0x01) && (rxBuffer[3] == 0x01));
            break;

        case PROT_ISO15693:
            do{
                delay(500);
                for(i=0; i<8; i++) NCIPresCheckIso15693[i+6] = RfIntf.Info.NFC_VPP.ID[7-i];
                (void) writeData(NCIPresCheckIso15693, sizeof(NCIPresCheckIso15693)); 
                getMessage();
                getMessage(100);  
                status = ERROR;
                if (rxMessageLength) status = SUCCESS;
            } while ((status == SUCCESS) && (rxBuffer[0] == 0x00) && (rxBuffer[1] == 0x00) && (rxBuffer[rxMessageLength-1] == 0x00));
            break;

        case PROT_MIFARE:
            do{
                delay(500);
                /* Deactivate target */
                (void) writeData(NCIDeactivate, sizeof(NCIDeactivate)); 
                getMessage();
                getMessage(100);  

                /* Reactivate target */
                (void) writeData(NCISelectMIFARE, sizeof(NCISelectMIFARE)); 
                getMessage();
                getMessage(100);  
            } while ((rxBuffer[0] == 0x61) && (rxBuffer[1] == 0x05));
            break;

        default:
            /* Nothing to do */
            break;
    }
}



bool Electroniccats_PN7150::ReaderReActivate(RfIntf_t *pRfIntf){
    uint8_t NCIDeactivate[] = {0x21, 0x06, 0x01, 0x01};
    uint8_t NCIActivate[] = {0x21, 0x04, 0x03, 0x01, 0x00, 0x00};
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;

    /* First de-activate the target */
    (void) writeData(NCIDeactivate, sizeof(NCIDeactivate)); 
    getMessage();
    getMessage(100);  

    /* Then re-activate the target */
    NCIActivate[4] = pRfIntf->Protocol;
    NCIActivate[5] = pRfIntf->Interface;

    (void) writeData(NCIDeactivate, sizeof(NCIDeactivate)); 
    getMessage();
    getMessage(100);  

    if((rxBuffer[0] != 0x61) || (rxBuffer[1] != 0x05)) return ERROR;
    return SUCCESS;
}


bool Electroniccats_PN7150::ReaderActivateNext(RfIntf_t *pRfIntf){
    uint8_t NCIStopDiscovery[] = {0x21, 0x06, 0x01, 0x01};
    uint8_t NCIRfDiscoverSelect[] = {0x21, 0x04, 0x03, 0x02, PROT_ISODEP, INTF_ISODEP};
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;
    bool status = ERROR;

    pRfIntf->MoreTags = false;

    if (gNextTag_Protocol == PROT_UNDETERMINED){
        pRfIntf->Interface = INTF_UNDETERMINED;
        pRfIntf->Protocol = PROT_UNDETERMINED;
        return ERROR;
    }

    /* First disconnect current tag */
    (void) writeData(NCIStopDiscovery, sizeof(NCIStopDiscovery)); 
    getMessage();

    if((rxBuffer[0] != 0x41) && (rxBuffer[1] != 0x06) && (rxBuffer[3] != 0x00)) return ERROR;
    getMessage(100);

    if((rxBuffer[0] != 0x61) && (rxBuffer[1] != 0x06)) return ERROR;

    NCIRfDiscoverSelect[4] = gNextTag_Protocol;
    if (gNextTag_Protocol == PROT_ISODEP) NCIRfDiscoverSelect[5] = INTF_ISODEP;
    else if (gNextTag_Protocol == PROT_ISODEP) NCIRfDiscoverSelect[5] = INTF_NFCDEP;
    else if (gNextTag_Protocol == PROT_MIFARE) NCIRfDiscoverSelect[5] = INTF_TAGCMD;
    else NCIRfDiscoverSelect[5] = INTF_FRAME;

    (void) writeData(NCIRfDiscoverSelect, sizeof(NCIRfDiscoverSelect)); 
    getMessage();

    if ((rxBuffer[0] == 0x41) && (rxBuffer[1] == 0x04) && (rxBuffer[3] == 0x00)){
        getMessage(100);
        if ((rxBuffer[0] == 0x61) || (rxBuffer[1] == 0x05)){
            pRfIntf->Interface = rxBuffer[4];
            pRfIntf->Protocol = rxBuffer[5];
            pRfIntf->ModeTech = rxBuffer[6];
            FillInterfaceInfo(pRfIntf, &Answer[10]);
            status = SUCCESS;
        }
    }

    return status;
}

bool Electroniccats_PN7150::StopDiscovery(void){
    uint8_t NCIStopDiscovery[] = {0x21, 0x06, 0x01, 0x00};
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;

    (void) writeData(NCIStopDiscovery, sizeof(NCIStopDiscovery)); 
    getMessage();
    getMessage(1000);

    return SUCCESS;
}