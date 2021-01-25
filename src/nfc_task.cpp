/*
*         Copyright (c), NXP Semiconductors Caen / France
*
*                     (C)NXP Semiconductors
*       All rights are reserved. Reproduction in whole or in part is
*      prohibited without the written consent of the copyright owner.
*  NXP reserves the right to make changes without notice at any time.
* NXP makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
*particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. NXP must not be liable for any loss or damage
*                          arising from its use.
*/
#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <tool.h>
#include <ndef_helper.h>
#include <Electroniccats_PN7150.h>


//#define RW_NDEF_WRITING
//#define RW_RAW_EXCHANGE
//#define CARDEMU_RAW_EXCHANGE

/* Discovery loop configuration according to the targeted modes of operation */
unsigned char DiscoveryTechnologies[] = {
#if defined P2P_SUPPORT || defined RW_SUPPORT
    MODE_POLL | TECH_PASSIVE_NFCA,
    MODE_POLL | TECH_PASSIVE_NFCF,
#endif // if defined P2P_SUPPORT || defined RW_SUPPORT
#ifdef RW_SUPPORT
    MODE_POLL | TECH_PASSIVE_NFCB,
    MODE_POLL | TECH_PASSIVE_15693,
#endif // ifdef RW_SUPPORT
#ifdef P2P_SUPPORT
    /* Only one POLL ACTIVE mode can be enabled, if both are defined only NFCF applies */
    MODE_POLL | TECH_ACTIVE_NFCA,
    //MODE_POLL | TECH_ACTIVE_NFCF,
#endif // ifdef P2P_SUPPORT
#if defined P2P_SUPPORT || defined CARDEMU_SUPPORT
    MODE_LISTEN | TECH_PASSIVE_NFCA,
#endif // if defined P2P_SUPPORT || defined CARDEMU_SUPPORT
#if defined CARDEMU_SUPPORT
    MODE_LISTEN | TECH_PASSIVE_NFCB,
#endif // if defined CARDEMU_SUPPORT
#ifdef P2P_SUPPORT
    MODE_LISTEN | TECH_PASSIVE_NFCF,
    MODE_LISTEN | TECH_ACTIVE_NFCA,
    MODE_LISTEN | TECH_ACTIVE_NFCF,
#endif // ifdef P2P_SUPPORT
};

/* Mode configuration according to the targeted modes of operation */
#ifdef CARDEMU_SUPPORT
              | NXPNCI_MODE_CARDEMU
#endif // ifdef P2P_SUPPORT
#ifdef P2P_SUPPORT
              | NXPNCI_MODE_P2P
#endif // ifdef CARDEMU_SUPPORT
#ifdef RW_SUPPORT
              | NXPNCI_MODE_RW
#endif // ifdef RW_SUPPORT
;

//#if defined P2P_SUPPORT || defined RW_SUPPORT
void NdefPull_Cb(unsigned char *pNdefMessage, unsigned short NdefMessageSize)
{
    unsigned char *pNdefRecord = pNdefMessage;
    NdefRecord_t NdefRecord;
    unsigned char save;

    if (pNdefMessage == NULL)
    {
        Serial.println("--- Provisioned buffer size too small or NDEF message empty");
        return;
    }

    while (pNdefRecord != NULL)
    {
        Serial.println("--- NDEF record received:");

        NdefRecord = DetectNdefRecordType(pNdefRecord);

        switch(NdefRecord.recordType)
        {
        case MEDIA_VCARD:
            {
                save = NdefRecord.recordPayload[NdefRecord.recordPayloadSize];
                NdefRecord.recordPayload[NdefRecord.recordPayloadSize] = '\0';
                Serial.print("vCard:");
                //Serial.println(NdefRecord.recordPayload);
                NdefRecord.recordPayload[NdefRecord.recordPayloadSize] = save;
            }
            break;

        case WELL_KNOWN_SIMPLE_TEXT:
            {
                save = NdefRecord.recordPayload[NdefRecord.recordPayloadSize];
                NdefRecord.recordPayload[NdefRecord.recordPayloadSize] = '\0';
                Serial.print("Text record:"); 
                //Serial.println(&NdefRecord.recordPayload[NdefRecord.recordPayload[0]+1]);
                NdefRecord.recordPayload[NdefRecord.recordPayloadSize] = save;
            }
            break;

        case WELL_KNOWN_SIMPLE_URI:
            {
                save = NdefRecord.recordPayload[NdefRecord.recordPayloadSize];
                NdefRecord.recordPayload[NdefRecord.recordPayloadSize] = '\0';
                Serial.print("URI record: "); 
                //Serial.println(ndef_helper_UriHead(NdefRecord.recordPayload[0]), &NdefRecord.recordPayload[1]);
                NdefRecord.recordPayload[NdefRecord.recordPayloadSize] = save;
            }
            break;

        case MEDIA_HANDOVER_WIFI:
            {
                unsigned char index = 0, i;

                Serial.println ("--- Received WIFI credentials:");
                if ((NdefRecord.recordPayload[index] == 0x10) && (NdefRecord.recordPayload[index+1] == 0x0e)) index+= 4;
                while(index < NdefRecord.recordPayloadSize)
                {
                    if (NdefRecord.recordPayload[index] == 0x10)
                    {
                        if (NdefRecord.recordPayload[index+1] == 0x45) {
                            Serial.print("- SSID = "); 
                            for(i=0;i<NdefRecord.recordPayload[index+3];i++) 
                        Serial.print(NdefRecord.recordPayload[index+4+i]); 
                        Serial.println ("");
                        }
                        else if (NdefRecord.recordPayload[index+1] == 0x03) {
                          Serial.print("- Authenticate Type = ");
                          Serial.println(ndef_helper_WifiAuth(NdefRecord.recordPayload[index+5]));
                        }
                        else if (NdefRecord.recordPayload[index+1] == 0x0f) 
                        {
                            Serial.print("- Encryption Type = "); 
                            Serial.println(ndef_helper_WifiEnc(NdefRecord.recordPayload[index+5]));
                        }
                        else if (NdefRecord.recordPayload[index+1] == 0x27) {
                            Serial.print("- Network key = "); 
                            for(i=0;i<NdefRecord.recordPayload[index+3];i++) 
                            Serial.print("#"); 
                            Serial.println("");
                            }
                        index += 4 + NdefRecord.recordPayload[index+3];
                    }
                    else continue;
                }
            }
            break;

        case WELL_KNOWN_HANDOVER_SELECT:
            Serial.print("Handover select version "); 
            Serial.print(NdefRecord.recordPayload[0] >> 4); 
            Serial.println(NdefRecord.recordPayload[0] & 0xF);
            break;

        case WELL_KNOWN_HANDOVER_REQUEST:
            Serial.print("Handover request version "); 
            Serial.print(NdefRecord.recordPayload[0] >> 4); 
            Serial.println(NdefRecord.recordPayload[0] & 0xF);
            break;

        case MEDIA_HANDOVER_BT:
            Serial.print("BT Handover payload = ");
            //Serial.print(NdefRecord.recordPayload);
            //Serial.println(NdefRecord.recordPayloadSize);
            break;

        case MEDIA_HANDOVER_BLE:
            Serial.print("BLE Handover payload = "); 
            //Serial.print(NdefRecord.recordPayload); 
            //Serial.println(NdefRecord.recordPayloadSize);
            break;

        case MEDIA_HANDOVER_BLE_SECURE:
            Serial.print("   BLE secure Handover payload = "); 
            //Serial.println(NdefRecord.recordPayload, NdefRecord.recordPayloadSize);
            break;

        default:
            Serial.println("Unsupported NDEF record, cannot parse");
            break;
        }
        pNdefRecord = GetNextRecord(pNdefRecord);
    }

    Serial.println("");
}
//#endif // if defined P2P_SUPPORT || defined RW_SUPPORT

//#if defined P2P_SUPPORT || defined CARDEMU_SUPPORT
const char NDEF_MESSAGE[] = { 0xD1,   // MB/ME/CF/1/IL/TNF
        0x01,   // TYPE LENGTH
        0x07,   // PAYLOAD LENTGH
        'T',    // TYPE
        0x02,   // Status
        'e', 'n', // Language
        'T', 'e', 's', 't' };

void NdefPush_Cb(unsigned char *pNdefRecord, unsigned short NdefRecordSize) {
    Serial.println("--- NDEF Record sent");
}
//#endif // if defined P2P_SUPPORT || defined CARDEMU_SUPPORT
