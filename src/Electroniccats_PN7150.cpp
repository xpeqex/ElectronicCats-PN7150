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


static uint8_t gNextTag_Protocol = PROT_UNDETERMINED;

unsigned char DiscoveryTechnologiesCE[] = {
        MODE_LISTEN | MODE_POLL
};

unsigned char DiscoveryTechnologiesRW[] = {
            MODE_POLL | TECH_PASSIVE_NFCA,
            MODE_POLL | TECH_PASSIVE_NFCF,
            MODE_POLL | TECH_PASSIVE_NFCB,
            MODE_POLL | TECH_PASSIVE_15693
};

Electroniccats_PN7150::Electroniccats_PN7150(uint8_t IRQpin, uint8_t VENpin, uint8_t I2Caddress): 
    _IRQpin(IRQpin), 
    _VENpin(VENpin), 
    _I2Caddress(I2Caddress)
{
    pinMode(_IRQpin, INPUT);                                             
    pinMode(_VENpin, OUTPUT);    
}

uint8_t Electroniccats_PN7150::begin() {
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
        bytesReceived = Wire.requestFrom(_I2Caddress, (uint8_t)3);           // first reading the header, as this contains how long the payload will be

        rxBuffer[0] = Wire.read();
        rxBuffer[1] = Wire.read();
        rxBuffer[2] = Wire.read();
        uint8_t payloadLength = rxBuffer[2];
        if (payloadLength > 0) {
            bytesReceived += Wire.requestFrom(_I2Caddress, (uint8_t)payloadLength);       // then reading the payload, if any
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

uint8_t Electroniccats_PN7150::wakeupNCI() {                         // the device has to wake up using a core reset
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

uint8_t Electroniccats_PN7150::connectNCI(){
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
    gNfcController_fw_version[0] = rxBuffer[17+rxBuffer[8]]; //0xROM_CODE_V
    gNfcController_fw_version[1] = rxBuffer[18+rxBuffer[8]]; //0xFW_MAJOR_NO
    gNfcController_fw_version[2] = rxBuffer[19+rxBuffer[8]]; //0xFW_MINOR_NO
    Serial.println("0xROM_CODE_V: " + String(gNfcController_fw_version[0], HEX));
    Serial.println("FW_MAJOR_NO: " + String(gNfcController_fw_version[1], HEX));
    Serial.println("0xFW_MINOR_NO: " + String(gNfcController_fw_version[2], HEX));
    Serial.println("gNfcController_generation: " + String(gNfcController_generation, HEX));

    return SUCCESS;
}

int Electroniccats_PN7150::GetFwVersion(){
    return ((gNfcController_fw_version[0] & 0xFF ) << 16) | ((gNfcController_fw_version[1] & 0xFF ) << 8) | (gNfcController_fw_version[2] & 0xFF);
}

uint8_t Electroniccats_PN7150::ConfigMode(uint8_t modeSE){
    unsigned mode = 0 | (modeSE == 1 ? MODE_RW : MODE_CARDEMU);
    uint8_t Command[MAX_NCI_FRAME_SIZE];
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;
    uint8_t Item = 0;
    uint8_t NCIDiscoverMap[] = {0x21, 0x00};

    //Emulation mode
    const uint8_t DM_CARDEMU[] = {0x4, 0x2, 0x2};
    const uint8_t R_CARDEMU[] = {0x1, 0x3, 0x0, 0x1, 0x4};
    //RW Mode
    const uint8_t DM_RW[] = {0x1, 0x1, 0x1, 0x2, 0x1, 0x1, 0x3, 0x1, 0x1, 0x4, 0x1, 0x2, 0x80, 0x01, 0x80};
    uint8_t NCIPropAct[] = {0x2F, 0x02, 0x00};


    uint8_t NCIRouting[] = {0x21, 0x01, 0x07, 0x00, 0x01};
    uint8_t NCISetConfig_NFCA_SELRSP[] = {0x20, 0x02, 0x04, 0x01, 0x32, 0x01, 0x00};

    if(mode == 0) return SUCCESS;

    /* Enable Proprietary interface for T4T card presence check procedure */
    if (modeSE == 1){
        if (mode == MODE_RW){
            (void) writeData(NCIPropAct, sizeof(NCIPropAct)); 
            getMessage();

            if ((rxBuffer[0] != 0x4F) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00)) return ERROR;
        }
    }
    
    //* Building Discovery Map command 
    Item = 0;
    if (mode & MODE_CARDEMU and modeSE == 2) {
        memcpy(&Command[4+(3*Item)], DM_CARDEMU, sizeof(DM_CARDEMU));
        Item++;
    }
    if (mode & MODE_RW and modeSE == 1) {
        memcpy(&Command[4+(3*Item)], DM_RW, sizeof(DM_RW));
        Item+=sizeof(DM_RW)/3;
    }
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
    if(modeSE == 2){
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
    }
    return SUCCESS;
}


uint8_t Electroniccats_PN7150::StartDiscovery(uint8_t modeSE){
    unsigned char TechTabSize = sizeof(modeSE == 1 ? DiscoveryTechnologiesRW : DiscoveryTechnologiesCE);
    uint8_t Answer[MAX_NCI_FRAME_SIZE];
    uint16_t AnswerSize;

    uint8_t NCIStartDiscovery[30];
    uint8_t NCIStartDiscovery_length = 0;

    NCIStartDiscovery[0] = 0x21;
    NCIStartDiscovery[1] = 0x03;
    NCIStartDiscovery[2] = (TechTabSize * 2) + 1;
    NCIStartDiscovery[3] = TechTabSize;
    for (uint8_t i = 0; i<TechTabSize; i++) {
        NCIStartDiscovery[(i*2)+4] = (modeSE == 1 ? DiscoveryTechnologiesRW[i] : DiscoveryTechnologiesCE[i]);
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

void Electroniccats_PN7150::FillInterfaceInfo(RfIntf_t* pRfIntf, uint8_t* pBuf){
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
    getMessage();
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

bool Electroniccats_PN7150::ConfigureSettings(void)
{

#if NXP_CORE_CONF
/* NCI standard dedicated settings
 * Refer to NFC Forum NCI standard for more details
 */
uint8_t NxpNci_CORE_CONF[]={0x20, 0x02, 0x05, 0x01,         /* CORE_SET_CONFIG_CMD */
    0x00, 0x02, 0x00, 0x01                                  /* TOTAL_DURATION */
};
#endif

#if NXP_CORE_CONF_EXTN
/* NXP-NCI extension dedicated setting
 * Refer to NFC controller User Manual for more details
 */
uint8_t NxpNci_CORE_CONF_EXTN[]={0x20, 0x02, 0x0D, 0x03,    /* CORE_SET_CONFIG_CMD */
    0xA0, 0x40, 0x01, 0x00,                                 /* TAG_DETECTOR_CFG */
    0xA0, 0x41, 0x01, 0x04,                                 /* TAG_DETECTOR_THRESHOLD_CFG */
    0xA0, 0x43, 0x01, 0x00                                  /* TAG_DETECTOR_FALLBACK_CNT_CFG */
};
#endif

#if NXP_CORE_STANDBY
/* NXP-NCI standby enable setting
 * Refer to NFC controller User Manual for more details
 */
uint8_t NxpNci_CORE_STANDBY[]={0x2F, 0x00, 0x01, 0x01};    /* last byte indicates enable/disable */
#endif
    
#if NXP_TVDD_CONF
/* NXP-NCI TVDD configuration
 * Refer to NFC controller Hardware Design Guide document for more details
 */
/* RF configuration related to 1st generation of NXP-NCI controller (e.g PN7120) */
uint8_t NxpNci_TVDD_CONF_1stGen[]={0x20, 0x02, 0x05, 0x01, 0xA0, 0x13, 0x01, 0x00};

/* RF configuration related to 2nd generation of NXP-NCI controller (e.g PN7150)*/
 #if (NXP_TVDD_CONF == 1)
 /* CFG1: Vbat is used to generate the VDD(TX) through TXLDO */
 uint8_t NxpNci_TVDD_CONF_2ndGen[]={0x20, 0x02, 0x07, 0x01, 0xA0, 0x0E, 0x03, 0x02, 0x09, 0x00};
 #else
 /* CFG2: external 5V is used to generate the VDD(TX) through TXLDO */
 uint8_t NxpNci_TVDD_CONF_2ndGen[]={0x20, 0x02, 0x07, 0x01, 0xA0, 0x0E, 0x03, 0x06, 0x64, 0x00};
 #endif
#endif

#if NXP_RF_CONF
/* NXP-NCI RF configuration
 * Refer to NFC controller Antenna Design and Tuning Guidelines document for more details
 */
/* RF configuration related to 1st generation of NXP-NCI controller (e.g PN7120) */
/* Following configuration is the default settings of PN7120 NFC Controller */
uint8_t NxpNci_RF_CONF_1stGen[]={0x20, 0x02, 0x38, 0x07,
    0xA0, 0x0D, 0x06, 0x06, 0x42, 0x01, 0x00, 0xF1, 0xFF,   /* RF_CLIF_CFG_TARGET          CLIF_ANA_TX_AMPLITUDE_REG */
    0xA0, 0x0D, 0x06, 0x06, 0x44, 0xA3, 0x90, 0x03, 0x00,   /* RF_CLIF_CFG_TARGET          CLIF_ANA_RX_REG */
    0xA0, 0x0D, 0x06, 0x34, 0x2D, 0xDC, 0x50, 0x0C, 0x00,   /* RF_CLIF_CFG_BR_106_I_RXA_P  CLIF_SIGPRO_RM_CONFIG1_REG */
    0xA0, 0x0D, 0x04, 0x06, 0x03, 0x00, 0x70,               /* RF_CLIF_CFG_TARGET          CLIF_TRANSCEIVE_CONTROL_REG */
    0xA0, 0x0D, 0x03, 0x06, 0x16, 0x00,                     /* RF_CLIF_CFG_TARGET          CLIF_TX_UNDERSHOOT_CONFIG_REG */
    0xA0, 0x0D, 0x03, 0x06, 0x15, 0x00,                     /* RF_CLIF_CFG_TARGET          CLIF_TX_OVERSHOOT_CONFIG_REG */
    0xA0, 0x0D, 0x06, 0x32, 0x4A, 0x53, 0x07, 0x01, 0x1B    /* RF_CLIF_CFG_BR_106_I_TXA    CLIF_ANA_TX_SHAPE_CONTROL_REG */
};

/* RF configuration related to 2nd generation of NXP-NCI controller (e.g PN7150)*/
/* Following configuration relates to performance optimization of OM5578/PN7150 NFC Controller demo kit */
uint8_t NxpNci_RF_CONF_2ndGen[]={0x20, 0x02, 0x94, 0x11,
    0xA0, 0x0D, 0x06, 0x04, 0x35, 0x90, 0x01, 0xF4, 0x01,    /* RF_CLIF_CFG_INITIATOR        CLIF_AGC_INPUT_REG */
    0xA0, 0x0D, 0x06, 0x06, 0x30, 0x01, 0x90, 0x03, 0x00,    /* RF_CLIF_CFG_TARGET           CLIF_SIGPRO_ADCBCM_THRESHOLD_REG */
    0xA0, 0x0D, 0x06, 0x06, 0x42, 0x02, 0x00, 0xFF, 0xFF,    /* RF_CLIF_CFG_TARGET           CLIF_ANA_TX_AMPLITUDE_REG */
    0xA0, 0x0D, 0x06, 0x20, 0x42, 0x88, 0x00, 0xFF, 0xFF,    /* RF_CLIF_CFG_TECHNO_I_TX15693 CLIF_ANA_TX_AMPLITUDE_REG */
    0xA0, 0x0D, 0x04, 0x22, 0x44, 0x23, 0x00,                /* RF_CLIF_CFG_TECHNO_I_RX15693 CLIF_ANA_RX_REG */
    0xA0, 0x0D, 0x06, 0x22, 0x2D, 0x50, 0x34, 0x0C, 0x00,    /* RF_CLIF_CFG_TECHNO_I_RX15693 CLIF_SIGPRO_RM_CONFIG1_REG */
    0xA0, 0x0D, 0x06, 0x32, 0x42, 0xF8, 0x00, 0xFF, 0xFF,    /* RF_CLIF_CFG_BR_106_I_TXA     CLIF_ANA_TX_AMPLITUDE_REG */
    0xA0, 0x0D, 0x06, 0x34, 0x2D, 0x24, 0x37, 0x0C, 0x00,    /* RF_CLIF_CFG_BR_106_I_RXA_P   CLIF_SIGPRO_RM_CONFIG1_REG */
    0xA0, 0x0D, 0x06, 0x34, 0x33, 0x86, 0x80, 0x00, 0x70,    /* RF_CLIF_CFG_BR_106_I_RXA_P   CLIF_AGC_CONFIG0_REG */
    0xA0, 0x0D, 0x04, 0x34, 0x44, 0x22, 0x00,                /* RF_CLIF_CFG_BR_106_I_RXA_P   CLIF_ANA_RX_REG */
    0xA0, 0x0D, 0x06, 0x42, 0x2D, 0x15, 0x45, 0x0D, 0x00,    /* RF_CLIF_CFG_BR_848_I_RXA     CLIF_SIGPRO_RM_CONFIG1_REG */
    0xA0, 0x0D, 0x04, 0x46, 0x44, 0x22, 0x00,                /* RF_CLIF_CFG_BR_106_I_RXB     CLIF_ANA_RX_REG */
    0xA0, 0x0D, 0x06, 0x46, 0x2D, 0x05, 0x59, 0x0E, 0x00,    /* RF_CLIF_CFG_BR_106_I_RXB     CLIF_SIGPRO_RM_CONFIG1_REG */
    0xA0, 0x0D, 0x06, 0x44, 0x42, 0x88, 0x00, 0xFF, 0xFF,    /* RF_CLIF_CFG_BR_106_I_TXB     CLIF_ANA_TX_AMPLITUDE_REG */
    0xA0, 0x0D, 0x06, 0x56, 0x2D, 0x05, 0x9F, 0x0C, 0x00,    /* RF_CLIF_CFG_BR_212_I_RXF_P   CLIF_SIGPRO_RM_CONFIG1_REG */
    0xA0, 0x0D, 0x06, 0x54, 0x42, 0x88, 0x00, 0xFF, 0xFF,    /* RF_CLIF_CFG_BR_212_I_TXF     CLIF_ANA_TX_AMPLITUDE_REG */
    0xA0, 0x0D, 0x06, 0x0A, 0x33, 0x80, 0x86, 0x00, 0x70     /* RF_CLIF_CFG_I_ACTIVE         CLIF_AGC_CONFIG0_REG */
};
#endif

#if NXP_CLK_CONF
/* NXP-NCI CLOCK configuration
 * Refer to NFC controller Hardware Design Guide document for more details
 */
 #if (NXP_CLK_CONF == 1)
  /* Xtal configuration */
  uint8_t NxpNci_CLK_CONF[]={0x20, 0x02, 0x05, 0x01,        /* CORE_SET_CONFIG_CMD */
    0xA0, 0x03, 0x01, 0x08                                  /* CLOCK_SEL_CFG */
  };
  #else
  /* PLL configuration */
  uint8_t NxpNci_CLK_CONF[]={0x20, 0x02, 0x09, 0x02,        /* CORE_SET_CONFIG_CMD */
    0xA0, 0x03, 0x01, 0x11,                                 /* CLOCK_SEL_CFG */
    0xA0, 0x04, 0x01, 0x01                                  /* CLOCK_TO_CFG */
  };
  #endif
#endif

    //uint8_t Answer[MAX_NCI_FRAME_SIZE];
    //uint16_t AnswerSize;
    uint8_t NCICoreReset[] = {0x20, 0x00, 0x01, 0x00};
    uint8_t NCICoreInit[] = {0x20, 0x01, 0x00};
    bool gRfSettingsRestored_flag = false;

#if (NXP_TVDD_CONF | NXP_RF_CONF)
    uint8_t *NxpNci_CONF;
    uint16_t NxpNci_CONF_size = 0;
#endif
#if (NXP_CORE_CONF_EXTN | NXP_CLK_CONF | NXP_TVDD_CONF | NXP_RF_CONF)
    uint8_t currentTS[32] = __TIMESTAMP__;
    uint8_t NCIReadTS[] = {0x20, 0x03, 0x03, 0x01, 0xA0, 0x14};
    uint8_t NCIWriteTS[7+32] = {0x20, 0x02, 0x24, 0x01, 0xA0, 0x14, 0x20};
#endif
    bool isResetRequired = false;

    /* Apply settings */
#if NXP_CORE_CONF
    if (sizeof(NxpNci_CORE_CONF) != 0)
    {
        isResetRequired = true;
        //NxpNci_HostTransceive(NxpNci_CORE_CONF, sizeof(NxpNci_CORE_CONF), Answer, sizeof(Answer), &AnswerSize);
        //if ((Answer[0] != 0x40) || (Answer[1] != 0x02) || (Answer[3] != 0x00) || (Answer[4] != 0x00)) return NXPNCI_ERROR;
        (void) writeData(NxpNci_CORE_CONF, sizeof(NxpNci_CORE_CONF));
        getMessage();
		if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00)) 
          {
              Serial.println("NxpNci_CORE_CONF");
              return ERROR;
          }
    }
#endif

#if NXP_CORE_STANDBY
if (sizeof(NxpNci_CORE_STANDBY) != 0)
	{

        (void)(writeData(NxpNci_CORE_STANDBY, sizeof(NxpNci_CORE_STANDBY)));
        getMessage();
		if ((rxBuffer[0] != 0x4F) || (rxBuffer[1] != 0x00) || (rxBuffer[3] != 0x00)) 
        {
          Serial.println("NxpNci_CORE_STANDBY");
          return ERROR;
        }
	}
#endif

    /* All further settings are not versatile, so configuration only applied if there are changes (application build timestamp) 
       or in case of PN7150B0HN/C11004 Anti-tearing recovery procedure inducing RF setings were restored to their default value */
#if (NXP_CORE_CONF_EXTN | NXP_CLK_CONF | NXP_TVDD_CONF | NXP_RF_CONF)
    /* First read timestamp stored in NFC Controller */
    if(gNfcController_generation == 1) NCIReadTS[5] = 0x0F;
    //NxpNci_HostTransceive(NCIReadTS, sizeof(NCIReadTS), Answer, sizeof(Answer), &AnswerSize);
    (void) writeData(NCIReadTS, sizeof(NCIReadTS)); 
    getMessage();
    if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x03) || (rxBuffer[3] != 0x00)) 
    {
        Serial.println("read timestamp ");
        return ERROR;
    }
    /* Then compare with current build timestamp, and check RF setting restauration flag */
    /*if(!memcmp(&rxBuffer[8], currentTS, sizeof(currentTS)) && (gRfSettingsRestored_flag == false))
    {
        // No change, nothing to do
    }
    else
    {
        */
    /* Apply settings */
#if NXP_CORE_CONF_EXTN
    if (sizeof(NxpNci_CORE_CONF_EXTN) != 0)
	{
        (void) writeData(NxpNci_CORE_CONF_EXTN, sizeof(NxpNci_CORE_CONF_EXTN));
        getMessage();
		if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00)) 
          {
              Serial.println("NxpNci_CORE_CONF_EXTN");
              return ERROR;
          }
	}
#endif

#if NXP_CLK_CONF
        if (sizeof(NxpNci_CLK_CONF) != 0)
        {
            isResetRequired = true;
            
            (void) writeData(NxpNci_CLK_CONF, sizeof(NxpNci_CLK_CONF)); 
            getMessage();
            //NxpNci_HostTransceive(NxpNci_CLK_CONF, sizeof(NxpNci_CLK_CONF), Answer, sizeof(Answer), &AnswerSize);
            if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00)) 
            {
              Serial.println("NxpNci_CLK_CONF");
              return ERROR;
            } 
        }
#endif

#if NXP_TVDD_CONF
    if (NxpNci_CONF_size != 0)
	{
		
        (void) writeData(NxpNci_TVDD_CONF_2ndGen, sizeof(NxpNci_TVDD_CONF_2ndGen));
        getMessage();
		if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00)) 
          {Serial.println("NxpNci_CONF_size");
          return ERROR;}
	}
#endif

#if NXP_RF_CONF
	if (NxpNci_CONF_size != 0)
	{
		
        (void) writeData(NxpNci_RF_CONF_2ndGen, sizeof(NxpNci_RF_CONF_2ndGen));
        getMessage();
		if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00)) 
          {
              Serial.println("NxpNci_CONF_size");
              return ERROR;
          }
	}
#endif
        /* Store curent timestamp to NFC Controller memory for further checks */
        if(gNfcController_generation == 1) NCIWriteTS[5] = 0x0F;
        memcpy(&NCIWriteTS[7], currentTS, sizeof(currentTS));
        //NxpNci_HostTransceive(NCIWriteTS, sizeof(NCIWriteTS), Answer, sizeof(Answer), &AnswerSize);
        (void) writeData(NCIWriteTS, sizeof(NCIWriteTS)); 
        getMessage();
        if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x02) || (rxBuffer[3] != 0x00) || (rxBuffer[4] != 0x00)) 
        {
            Serial.println("NFC Controller memory");
            return ERROR;
        }
    //}
#endif

    if(isResetRequired)
    {
        /* Reset the NFC Controller to insure new settings apply */
        //NxpNci_HostTransceive(NCICoreReset, sizeof(NCICoreReset), Answer, sizeof(Answer), &AnswerSize);
        (void) writeData(NCICoreReset, sizeof(NCICoreReset)); 
        getMessage();
        if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x00) || (rxBuffer[3] != 0x00)) 
        {
            Serial.println("insure new settings apply");
            return ERROR;
        }

        (void) writeData(NCICoreInit, sizeof(NCICoreInit)); 
        getMessage();
        if ((rxBuffer[0] != 0x40) || (rxBuffer[1] != 0x01) || (rxBuffer[3] != 0x00)) 
        {
        Serial.println("insure new settings apply 2");
        return ERROR;
        }
    }
    return SUCCESS;
}

bool Electroniccats_PN7150::testAntenna(void)
{

    // Check NFC antenna
	 uint8_t testAntenna[] = {0x2F, 0x3D, 0x02, 0xC8, 0x60, 0x03};
     uint8_t response[MAX_NCI_FRAME_SIZE] = {0};
     uint16_t respLen;

    //NxpNci_HostTransceive(NCIRfOn, sizeof(NCIRfOn), Answer, sizeof(Answer), &AnswerSize);

    (void) writeData(testAntenna, sizeof(testAntenna)); 
    getMessage();
    if ((rxBuffer[0] != 0x4F) || (rxBuffer[1] != 0x3D) || (rxBuffer[3] != 0x00)) 
    {
      Serial.println("NxpNci_FactoryTest_RfOn");
      return ERROR;
    }

 Serial.println("Antenna read: " + String(rxBuffer[0], HEX));
 Serial.println("Antenna read: " + String(rxBuffer[1], HEX));
 Serial.println("Antenna read: " + String(rxBuffer[2], HEX));
 Serial.println("Antenna read: " + String(rxBuffer[3], HEX));
 Serial.println("Antenna read: " + String(rxBuffer[4], HEX));
 Serial.println("Antenna read: " + String(rxBuffer[5], HEX));

    return SUCCESS;
}

void Electroniccats_PN7150::test005()
    {
    // This will write CORE_REST_CMD to the PN7150, and then Check if we receive CORE_RESET_RSP back.. See NCI specification V1.0 section 4.1
    // I am using the reset behaviour of the NCI to test send and response here, as it is otherwise difficult to trigger a read
    Serial.println("Test 005 Cycle ---- Start");

	uint8_t tmpBuffer[] = { 0x20, 0x00, 0x01, 0x01 };
    //write(tmpBuffer, 4);
    (void) writeData(tmpBuffer, sizeof(tmpBuffer)); 

    delay(5); // How much delay do you need to check if there is an answer from the Device ?

    uint8_t tmpRxBuffer[260];
    uint32_t nmbrBytesReceived;

    nmbrBytesReceived = readData(tmpRxBuffer);

    if (6 == nmbrBytesReceived)
        {
        Serial.print(nmbrBytesReceived);
        Serial.println(" bytes received, 6 bytes expected - ok");
        if (0x40 == tmpRxBuffer[0])
            {
            Serial.println("byte[0] = 0x40 : MT = Control Packet Response, PBF = 0, GID = Core = 0 - ok");
            }
        else
            {
            Serial.print("byte[0] = ");
            Serial.print(tmpRxBuffer[0]);
            Serial.println(" - error");
            }

        if (0x00 == tmpRxBuffer[1])
            {
            Serial.println("byte[1] = 0x00 : OID = CORE_RESET_RSP - ok");
            }
        else
            {
            Serial.print("byte[1] = ");
            Serial.print(tmpRxBuffer[1]);
            Serial.println(" - error");
            }

        if (0x03 == tmpRxBuffer[2])
            {
            Serial.println("byte[2] = 0x03 : payload length = 3 bytes - ok");
            }
        else
            {
            Serial.print("byte[2] = ");
            Serial.print(tmpRxBuffer[2]);
            Serial.println(" - error");
            }

        Serial.print("byte[3] = Status = ");							// See NCI V1.0 Specification Table 94. 0x00 = Status_OK
        Serial.print(tmpRxBuffer[3]);
        Serial.println("");

        Serial.print("byte[4] = NCI Version = ");						// See NCI V1.0 Specification Table 6. 0x17 = V1.7 ?? Not sure about this as I don't have official specs from NCI as they are quite expensive
        Serial.print(tmpRxBuffer[4]);
        Serial.println("");

        Serial.print("byte[5] = Configuration Status = ");				// See NCI V1.0 Specification Table 7. 0x01 = NCI RF Configuration has been reset
        Serial.print(tmpRxBuffer[5]);
        Serial.println("");
        }
    else
        {
        Serial.print(nmbrBytesReceived);
        Serial.println(" bytes received, 6 bytes expected - error");
        }

    Serial.println("Test 005 Cycle ---- End");
    delay(1000);
    }
