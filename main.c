/******************************************************************************
            USB Custom Demo, Host

This file provides the main entry point to the Microchip USB Custom
Host demo.  This demo shows how a PIC24F system could be used to
act as the host, controlling a USB device running the Microchip Custom
Device demo.

******************************************************************************/

/******************************************************************************
* Filename:        main.c
* Dependancies:    USB Host Driver with Generic Client Driver
* Processor:       PIC24F256GB1xx
* Hardware:        Explorer 16 with USB PICtail Plus
* Compiler:        C30 v2.01/C32 v0.00.18
* Company:         Microchip Technology, Inc.

Software License Agreement

The software supplied herewith by Microchip Technology Incorporated
(the 鼎ompany・ for its PICmicroｮ Microcontroller is intended and
supplied to you, the Company痴 customer, for use solely and
exclusively on Microchip PICmicro Microcontroller products. The
software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to
civil liability for the breach of the terms and conditions of this
license.

THIS SOFTWARE IS PROVIDED IN AN 鄭S IS・CONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.


*******************************************************************************/

#include <stdlib.h>
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "usb_config.h"
#include "USB/usb.h"
#include "USB/usb_host_generic.h"
#include "user.h"
#include "LCDBlocking.h"
#include "timer.h"

#define DEBUG_MODE

#define MAX_BDDEV_NUM 4

/* 4021 */
#define J1_START  LATBbits.LATB9
#define J1_SELECT LATBbits.LATB8
#define J1_B      LATBbits.LATB7
#define J1_A      LATBbits.LATB5
#define J1_DOWN   LATAbits.LATA0
#define J1_UP     LATAbits.LATA1
#define J1_LEFT   LATBbits.LATB3
#define J1_RIGHT  LATBbits.LATB2
#define PRESS   0
#define RELEASE 1

// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************

        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_NODIV & IESO_ON)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
 
// *****************************************************************************


// Application States
typedef enum
{
    BT_INITIALIZE = 0,                // Initialize the app when a device is attached
    BT_STATE_IDLE,                    // Inactive State
    BT_STATE_PROCESS,
	BT_STATE_WRITE_CLASS,
	BT_STATE_READ_EP1,
	BT_STATE_READ_CLASS_WAITING,
	BT_STATE_WRITE_ACL,
	BT_STATE_READ_ACL_HCI,
	BT_STATE_READ_ACL_WAITING,
	BT_STATE_READ_HCI_WAITING,
	BT_STATE_READ_HCI,

    BT_STATE_ERROR                    // An error has occured
} BT_STATE;

// Hci States
typedef enum
{
	HCI_CMD_RESET = 0,                // Initialize the hci when a device is attached
		HCI_CMD_RESET_END,
	HCI_CMD_READ_BD_ADDR,
		HCI_CMD_READ_BD_ADDR_END,
	HCI_CMD_LOCAL_NAME,
		HCI_CMD_LOCAL_NAME_END,
	HCI_CMD_CLASS_DEVICE,
		HCI_CMD_CLASS_DEVICE_WRITE_END,
	HCI_CMD_SCAN_ENABLE,
		HCI_CMD_SCAN_ENABLE_WRITE_END,
	HCI_CMD_INQUIRY,
		HCI_CMD_INQUIRY_RESULT,
		HCI_CMD_INQUIRY_STATUS,
	HCI_CMD_CREAT_CONNECTION,
		HCI_CMD_CONNECTION_ACCEPTED,
		HCI_CMD_SAVE_HANDLE,

	L2CAP_CON_REQ11,
	L2CAP_CON_RESP11,
	L2CAP_CONFIG_REQ11,
	L2CAP_CONFIG_RESP11,
	L2CAP_CONFIG_REQ_HOST11,

	L2CAP_CON_REQ13,
	L2CAP_CON_RESP13,
	L2CAP_CONFIG_REQ13,
	L2CAP_CONFIG_RESP13,
	L2CAP_CONFIG_REQ_HOST13,

	HID_READ_FIRST_DATA,
	HID_WRITE_DATA,
	HID_READ_DATA,

	PROG_END
} HCI_STATE;


// *****************************************************************************
// *****************************************************************************
// Global Variables
// *****************************************************************************
// *****************************************************************************

BYTE	deviceAddress;  // Address of the device on the USB
BT_STATE  DemoState;      // Current state of the demo application
HCI_STATE  HciState;      // Current state of the demo application
WORD data_size;
int data_num;

unsigned char buf[64];
unsigned char buf1[64];

typedef struct BDdevice {
//unsigned char local_bd_addr[6];//
	unsigned char remote_bd_addr[6];//
	unsigned char clock_offset[2];
	unsigned char handle[2];//a handle for ACL 
	unsigned char src_cid[2];//HID interrupt (data) after hid_flag=2
	unsigned char dst_cid[2];
	unsigned char src_cid1[2];//HID control after hid_flag=3
	unsigned char dst_cid1[2];
	//unsigned char hid_flag=0;
	//unsigned char EP2busy_flag=0;
	unsigned char page_scan_rep_mode;

	unsigned short wii_ctrl_state_back;
} BDDevice_t;

BDDevice_t bddev[MAX_BDDEV_NUM];

int bddev_num = 0;
int find_dev_num = 0;

int step;
char message[30];
int end_num;

unsigned short wii_ctrl_state_now=0;

//******************************************************************************
//******************************************************************************
// Local Routines
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        InitializeSystem
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         TRUE if successful, FALSE if not.
 *
 * Side Effects:    See below
 *
 * Overview:        This routine initializes the processor and peripheral,
 *                  setting clock speeds and enabling any required
 *                  features.
 *************************************************************************/

BOOL InitializeSystem ( void )
{
	unsigned int pll_startup_counter = 600;
	CLKDIVbits.PLLEN = 1;
	while(pll_startup_counter--);

	/// ポートの入出力モード設定
	TRISB = 0x8000;				// ポートB RB15(U2RX) input
	TRISA = 0x0000;

	CNPU1 =0x00CF;				//CN(0,1),2,3,6,7 //0,1は有効にならない？
	CNPU2 =0x68E0;				//CN21,22,23, 27, 29,30

	// Configure U2RX - put on pin 17 (RP8)
	RPINR19bits.U2RXR = 14;
	// Configure U2TX - put on pin 16 (RP7)
	RPOR7bits.RP15R = 5;
    // Init UART
    UART2Init();

    // Set Default demo state
    DemoState = BT_INITIALIZE;
	HciState=HCI_CMD_RESET;

	/* 4021 */
	J1_START    = RELEASE;
	J1_SELECT   = RELEASE;
	J1_B        = RELEASE;
	J1_A        = RELEASE;
	J1_DOWN  = RELEASE;
	J1_UP    = RELEASE;
	J1_LEFT  = RELEASE;
	J1_RIGHT = RELEASE;

    return TRUE;
} // InitializeSystem


/*************************************************************************
 * Function:        CheckForNewAttach
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          deviceAddress (global)
 *                  Updates the device address when an attach is found.
 *
 * Returns:         TRUE if a new device has been attached.  FALSE,
 *                  otherwise.
 *
 * Side Effects:    Prints attach message
 *
 * Overview:        This routine checks to see if a new device has been
 *                  attached.  If it has, it records the address.
 *************************************************************************/

BOOL CheckForNewAttach ( void )
{
    // Try to get the device address, if we don't have one.
    if (deviceAddress == 0)
    {
        GENERIC_DEVICE_ID DevID;

        DevID.vid   = 0x04D8;
        DevID.pid   = 0x000C;
        #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
            DevID.serialNumberLength = 0;
            DevID.serialNumber = NULL;
        #endif

        if (USBHostGenericGetDeviceAddress(&DevID))
        {
            deviceAddress = DevID.deviceAddress;
            UART2PrintString( "Generic demo device attached - polled, deviceAddress=" );
            UART2PutDec( deviceAddress );
            UART2PrintString( "\r\n" );
            return TRUE;
        }
    }

    return FALSE;

} // CheckForNewAttach



/*************************************************************************
 * Function:        ManageDemoState
 *
 * Preconditions:   The DemoState global variable must be initialized to
 *                  DEMO_STATE_IDLE (0).  (This occurs on reset.)
 *
 * Input:           DemoState (global)
 *                  Actions selected based value of DemoState on function
 *                  entry.
 *
 *                  deviceAddress (global)
 *                  May use device address to access device, depending on
 *                  state.
 *
 *                  DataPacket (global)
 *                  May read data from packet buffer, depending on state.
 *
 * Output:          DemoState (global)
 *                  Updates demo state as appropriate.
 *
 *                  DataPacket (global)
 *                  May cause data in the packet buffer to be updated,
 *                  depending on state.
 *
 * Returns:         None
 *
 * Side Effects:    Depend on state transition
 *
 * Overview:        This routine maintains the state of the application,
 *                  updateing global data and taking actions as necessary
 *                  to maintain the custom demo operations.
 *************************************************************************/
void ManageDemoState ( void )
{
    BYTE RetVal;

    // Watch for device detaching
    if (USBHostGenericDeviceDetached(deviceAddress) && deviceAddress != 0)
    {
		#ifdef DEBUG_MODE
        UART2PrintString( "Generic demo device detached - polled\r\n" );
		#endif
        DemoState = BT_INITIALIZE;
		HciState=HCI_CMD_RESET;
        deviceAddress   = 0;
    }

    switch (DemoState)
    {
    case BT_INITIALIZE:
        DemoState = BT_STATE_IDLE;
        break;

    /** Idle State:  Loops here until attach **/
    case BT_STATE_IDLE:
        if (CheckForNewAttach())
        {
			DemoState = BT_STATE_PROCESS;
			HciState = HCI_CMD_RESET;
        }
        break;

    case BT_STATE_PROCESS:
		switch (HciState)
		{
//HCI layer***********************************************************************
		case HCI_CMD_RESET:
			buf1[0]=0x03;
			buf1[1]=0x0c;
			buf1[2]=0;
			data_size=3;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_RESET_END;
		    break;

		case HCI_CMD_RESET_END:
			end_num=0x0e;// When buf1[0]==end_num, reading process ends. See BT_STATE_READ_EP1
			strcpy(message,"HCI_CMD_RESET: ");//message for BT_STATE_READ_EP1
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_READ_BD_ADDR;
		    break;

//********************************************************************************
		case HCI_CMD_READ_BD_ADDR:
			buf1[0]=0x09;
			buf1[1]=0x10;
			buf1[2]=0;
			data_size=3;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_READ_BD_ADDR_END;
			break;

		case HCI_CMD_READ_BD_ADDR_END:
			end_num=0x0e;strcpy(message,"HCI_CMD_READ_BD_ADDR: ");
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_LOCAL_NAME;
		    break;

//********************************************************************************
		case HCI_CMD_LOCAL_NAME:
			buf1[0]=0x13;
			buf1[1]=0x0c;
			buf1[2]=0x04;
			buf1[3]='a';
			buf1[4]='b';
			buf1[5]='c';
			buf1[6]=0x00;
			data_size=7;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_LOCAL_NAME_END;
			break;

		case HCI_CMD_LOCAL_NAME_END:
			end_num=0x0e;strcpy(message,"HCI_CMD_LOCAL_NAME: ");
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_CLASS_DEVICE;
		    break;

//********************************************************************************
		case HCI_CMD_CLASS_DEVICE:
			buf1[0]=0x24;
			buf1[1]=0x0c;
			buf1[2]=0x03;
			buf1[3]=0x04;//joystick
			buf1[4]=0x05;//joystick
			buf1[5]=0x00;

			data_size=6;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_CLASS_DEVICE_WRITE_END;
			break;

		case HCI_CMD_CLASS_DEVICE_WRITE_END:
			end_num=0x0e;strcpy(message,"HCI_CMD_CLASS_DEVICE: ");
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_SCAN_ENABLE;
		    break;

//********************************************************************************
		case HCI_CMD_SCAN_ENABLE:
			buf1[0]=0x1a;
			buf1[1]=0x0c;
			buf1[2]=0x01;
			buf1[3]=0x03;
			data_size=4;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_SCAN_ENABLE_WRITE_END;
			break;

		case HCI_CMD_SCAN_ENABLE_WRITE_END:
			end_num=0x0e;strcpy(message,"HCI_CMD_SCAN_ENABLE: ");
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_INQUIRY;
		    break;

//********************************************************************************
		case HCI_CMD_INQUIRY:
			//Inquiry (search bluetooth dongles around you)
			buf1[0]=0x01;
			buf1[1]=0x04;
			buf1[2]=0x05;
			buf1[3]=0x33;
			buf1[4]=0x8b;
			buf1[5]=0x9e;
			buf1[6]=0x05;//waiting time (5 * 1.28 sec)
			buf1[7]=0x03;//if there are bluetooth devices around you, you should give a maximum number here. 
			data_size=8;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_INQUIRY_RESULT;
			break;

		case HCI_CMD_INQUIRY_RESULT:
			//Here it is assumed that only one bluetooth device of scan enable exists around you.
			//if there are some devices of scan enable, you mustchange following codes to select one of them. 
			end_num=0x02;strcpy(message,"HCI_CMD_INQUIRY_RESULT: ");
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_INQUIRY_STATUS;
			break;

		case HCI_CMD_INQUIRY_STATUS:
			//find two or more controllers.
			if(buf1[0]!=0x01) {
				//copy slave BD address
				bddev[find_dev_num].remote_bd_addr[0]=buf1[3];
				bddev[find_dev_num].remote_bd_addr[1]=buf1[4];
				bddev[find_dev_num].remote_bd_addr[2]=buf1[5];
				bddev[find_dev_num].remote_bd_addr[3]=buf1[6];
				bddev[find_dev_num].remote_bd_addr[4]=buf1[7];
				bddev[find_dev_num].remote_bd_addr[5]=buf1[8];
				//copy page_scan_repetitation_mode
				bddev[find_dev_num].page_scan_rep_mode=buf1[9];
				//copy clock offset
				bddev[find_dev_num].clock_offset[0]=buf1[15];
				bddev[find_dev_num].clock_offset[1]=buf1[16];

				find_dev_num++;
				buf1[0]=0xff;
				HciState = HCI_CMD_INQUIRY_RESULT;
				return;

			} else {
				end_num=0x01;strcpy(message,"HCI_CMD_INQUIRY_STATUS: ");//buf1[2] must be zero
				DemoState = BT_STATE_READ_EP1;
				HciState = HCI_CMD_CREAT_CONNECTION;
			}
			break;

//********************************************************************************
		case HCI_CMD_CREAT_CONNECTION:
			buf1[0]=0x05;
			buf1[1]=0x04;
			buf1[2]=0x0D;
			buf1[3]=bddev[bddev_num].remote_bd_addr[0];//******************************************
			buf1[4]=bddev[bddev_num].remote_bd_addr[1];//BD address (6 octets) of the slave bluetooth
			buf1[5]=bddev[bddev_num].remote_bd_addr[2];//
			buf1[6]=bddev[bddev_num].remote_bd_addr[3];//
			buf1[7]=bddev[bddev_num].remote_bd_addr[4];//
			buf1[8]=bddev[bddev_num].remote_bd_addr[5];//******************************************
			buf1[9]=0x10;//0x0010 DH1
			buf1[10]=0x00;
			buf1[11]=bddev[bddev_num].page_scan_rep_mode;//01
			buf1[12]=0x00;//00
			buf1[13]=bddev[bddev_num].clock_offset[0];//copy clock offset
			buf1[14]=bddev[bddev_num].clock_offset[1];//
			buf1[15]=0x00;
			data_size=16;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_CONNECTION_ACCEPTED;
			break;

		case HCI_CMD_CONNECTION_ACCEPTED:
			end_num=0x03;strcpy(message,"HCI_CMD_CONNECTION_ACCEPTED: ");
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_SAVE_HANDLE;
		    break;

		case HCI_CMD_SAVE_HANDLE:
			if(buf1[2]!=0x00) {HciState = HCI_CMD_CREAT_CONNECTION; break;}//when connection is failed, retry.
			bddev[bddev_num].handle[0]=buf1[3];bddev[bddev_num].handle[1]=buf1[4]+0x20;//save connection handle and add (PB flag + BC flag)
			HciState = 	L2CAP_CON_REQ11;
		    break;

//HCI_ACL ************************************************************************
//OPEN a channel for HID_Control 0x0011
		case  L2CAP_CON_REQ11:

			buf[0]=bddev[bddev_num].handle[0];
			buf[1]=bddev[bddev_num].handle[1];
			buf[2]=0x0c;//length of sending data+4 
			buf[3]=0x00;//
			buf[4]=0x08;//length of sending data
			buf[5]=0x00;//
			buf[6]=0x01;//
			buf[7]=0x00;//
			buf[8]=0x02;//
			buf[9]=0x01;//
			buf[10]=0x04;//
			buf[11]=0x00;//
			buf[12]=0x11;//HID_Control
			buf[13]=0x00;//HID_Control
			buf[14]=0x44;//
			buf[15]=0x00;//

			data_size=16;

			DemoState = BT_STATE_WRITE_ACL;
			HciState = L2CAP_CON_RESP11;

			break;

//********************************************************************************
		case  L2CAP_CON_RESP11:

			if(step==2) {HciState = L2CAP_CONFIG_REQ11;step=0;break;}	
			strcpy(message,"L2CAP_CON_RESP11 \r\n");
			step++;
			DemoState = BT_STATE_READ_ACL_HCI;

			break;

//********************************************************************************
		case  L2CAP_CONFIG_REQ11:
			bddev[bddev_num].dst_cid[0]=buf[12];
			bddev[bddev_num].dst_cid[1]=buf[13];
			bddev[bddev_num].src_cid[0]=buf[14];
			bddev[bddev_num].src_cid[1]=buf[15];		

			buf[0]=bddev[bddev_num].handle[0];
			buf[1]=bddev[bddev_num].handle[1];
			buf[2]=0x10;//length of sending data+4 
			buf[3]=0x00;//
			buf[4]=0x0c;//length of sending data
			buf[5]=0x00;//
			buf[6]=0x01;//
			buf[7]=0x00;//
			buf[8]=0x04;//
			buf[9]=0x02;//
			buf[10]=0x08;//
			buf[11]=0x00;//
			buf[12]=bddev[bddev_num].dst_cid[0];//
			buf[13]=bddev[bddev_num].dst_cid[1];//
			buf[14]=0x00;//
			buf[15]=0x00;//
			buf[16]=0x01;//
			buf[17]=0x02;//
			buf[18]=0x40;//64 Bytes
			buf[19]=0x00;//

			data_size=20;

			DemoState = BT_STATE_WRITE_ACL;
			HciState= L2CAP_CONFIG_RESP11;

			break;

//********************************************************************************
		case  L2CAP_CONFIG_RESP11:

			if(step==2) {HciState = L2CAP_CONFIG_REQ_HOST11;step=0;break;}	

			strcpy(message,"L2CAP_CONFIG_RESP11 \r\n");
			step++;
			DemoState = BT_STATE_READ_ACL_HCI;

			break;

//********************************************************************************
		case  	L2CAP_CONFIG_REQ_HOST11:

			buf[21]=buf[19];//copy Configuration Options
			buf[20]=buf[18];//
			buf[19]=buf[17];//
			buf[18]=buf[16];//

			buf[0]=bddev[bddev_num].handle[0];
			buf[1]=bddev[bddev_num].handle[1];
			buf[2]=0x12;//length of sending data+4 
			buf[3]=0x00;//
			buf[4]=0x0e;//length of sending data
			buf[5]=0x00;//
			buf[6]=0x01;//
			buf[7]=0x00;//
			buf[8]=0x05;//
//			buf[9]+=0x01;//
			buf[10]=0x0a;//
			buf[11]=0x00;//
			buf[12]=bddev[bddev_num].dst_cid[0];//
			buf[13]=bddev[bddev_num].dst_cid[1];//
			buf[14]=0x00;//
			buf[15]=0x00;//
			buf[16]=0x00;//
			buf[17]=0x00;//

			data_size=22;

			DemoState = BT_STATE_WRITE_ACL;
			HciState =L2CAP_CON_REQ13;

			break;

//********************************************************************************
//OPEN a channel for HID_Interrupt 0x0013

		case  L2CAP_CON_REQ13:
			buf[0]=bddev[bddev_num].handle[0];
			buf[1]=bddev[bddev_num].handle[1];
			buf[2]=0x0c;//length of sending data+4 
			buf[3]=0x00;//
			buf[4]=0x08;//length of sending data
			buf[5]=0x00;//
			buf[6]=0x01;//
			buf[7]=0x00;//
			buf[8]=0x02;//
			buf[9]+=0x01;//
			buf[10]=0x04;//
			buf[11]=0x00;//
			buf[12]=0x13;//
			buf[13]=0x00;//
			buf[14]=0x45;//
			buf[15]=0x00;//

			data_size=16;

			DemoState = BT_STATE_WRITE_ACL;
			HciState= L2CAP_CON_RESP13;

			break;

//********************************************************************************
		case  L2CAP_CON_RESP13:

			strcpy(message,"L2CAP_CON_RESP13 \r\n");
			DemoState = BT_STATE_READ_ACL_HCI;
			HciState= L2CAP_CONFIG_REQ13;
			break;

//********************************************************************************
		case  L2CAP_CONFIG_REQ13:
			bddev[bddev_num].dst_cid1[0]=buf[12];
			bddev[bddev_num].dst_cid1[1]=buf[13];
			bddev[bddev_num].src_cid1[0]=buf[14];
			bddev[bddev_num].src_cid1[1]=buf[15];		

			buf[0]=bddev[bddev_num].handle[0];
			buf[1]=bddev[bddev_num].handle[1];
			buf[2]=0x10;//length of sending data+4 
			buf[3]=0x00;//
			buf[4]=0x0c;//length of sending data
			buf[5]=0x00;//
			buf[6]=0x01;//
			buf[7]=0x00;//
			buf[8]=0x04;//
			buf[9]+=0x01;//
			buf[10]=0x08;//
			buf[11]=0x00;//
			buf[12]=bddev[bddev_num].dst_cid1[0];//
			buf[13]=bddev[bddev_num].dst_cid1[1];//
			buf[14]=0x00;//
			buf[15]=0x00;//
			buf[16]=0x01;//
			buf[17]=0x02;//
			buf[18]=0x40;//
			buf[19]=0x00;//

			data_size=20;

			DemoState = BT_STATE_WRITE_ACL;
			HciState= L2CAP_CONFIG_RESP13;

			break;

//********************************************************************************
		case  L2CAP_CONFIG_RESP13:

			if(step==2) {HciState = L2CAP_CONFIG_REQ_HOST13;step=0;break;}	

			strcpy(message,"L2CAP_CONFIG_RESP13 \r\n");
			step++;
			DemoState = BT_STATE_READ_ACL_HCI;

			break;

//********************************************************************************
		case  L2CAP_CONFIG_REQ_HOST13:
			buf[21]=buf[19];//copy Configuration Options
			buf[20]=buf[18];//
			buf[19]=buf[17];//
			buf[18]=buf[16];//

			buf[0]=bddev[bddev_num].handle[0];
			buf[1]=bddev[bddev_num].handle[1];
			buf[2]=0x12;//length of sending data+4 
			buf[3]=0x00;//
			buf[4]=0x0e;//length of sending data
			buf[5]=0x00;//
			buf[6]=0x01;//
			buf[7]=0x00;//
			buf[8]=0x05;//
//			buf[9]+=0x01;//
			buf[10]=0x0a;//
			buf[11]=0x00;//
			buf[12]=bddev[bddev_num].dst_cid1[0];//
			buf[13]=bddev[bddev_num].dst_cid1[1];//
			buf[14]=0x00;//
			buf[15]=0x00;//
			buf[16]=0x00;//
			buf[17]=0x00;//

			data_size=22;

			DemoState = BT_STATE_WRITE_ACL;
			HciState = HID_READ_FIRST_DATA;
			break;


//*****HID START******************************************************************
		case   HID_READ_FIRST_DATA:
			strcpy(message,"HID_READ_FIRST_DATA \r\n");

			DemoState = BT_STATE_READ_ACL_HCI;
			HciState= HID_WRITE_DATA;

			break;

//********************************************************************************
		case  HID_WRITE_DATA:
			buf[0]=bddev[bddev_num].handle[0];
			buf[1]=bddev[bddev_num].handle[1];
			buf[2]=0x07;//length of sending
			buf[3]=0x00;//
			buf[4]=0x03;
			buf[5]=0x00;
			buf[6]=bddev[bddev_num].dst_cid1[0];
			buf[7]=bddev[bddev_num].dst_cid1[1];
			buf[8]=0xa2;
			buf[9]=0x11;
			buf[10]=1 << (bddev_num+4); // Turn on Wiimote LED which indicates player number.

			data_size=11;

			DemoState = BT_STATE_WRITE_ACL;

			bddev_num++;
			if(bddev_num < find_dev_num) {			
				HciState =	HCI_CMD_CREAT_CONNECTION;
			} else { 
				HciState =	HID_READ_DATA;
			}
			break;

//********************************************************************************
		case  HID_READ_DATA:

			strcpy(message,"HID_READ_DATA \r\n");

			DemoState = BT_STATE_READ_ACL_HCI;
			HciState= HID_READ_DATA;
			break;

		case  PROG_END:
			break;

		}
		break;

//********************************************************************************
//　READ-WRITE FUNCTIONS
//********************************************************************************
//WRITE ENDPOINT 0
    case BT_STATE_WRITE_CLASS:
        if (!USBHostGenericTxIsBusy(deviceAddress))
		{
            if ( (RetVal=USBHostGenericClassRequest( deviceAddress, buf1, data_size )) == USB_SUCCESS )
            {
			//UART2PrintString( "HCI COMMAND SENT\r\n" );	
            DemoState = BT_STATE_PROCESS;
            }
            else
            {
			UART2PrintString( "Write Class Error !\r\n" );	
            }
        }
        break;


//READ ENDPOINT 1
    case BT_STATE_READ_EP1:
        if (!USBHostGenericRx1IsBusy(deviceAddress))
        {
            if ( (RetVal=USBHostGenericRead(deviceAddress, buf1, DATA_PACKET_LENGTH)) == USB_SUCCESS )
            {
                DemoState = BT_STATE_READ_CLASS_WAITING;
                //UART2PrintString( "READ EP1\r\n" );
            }
            else
            {
                UART2PrintString( "Device Read Error !\r\n" );
            }
        }
        break;

    case BT_STATE_READ_CLASS_WAITING:
        if (!USBHostGenericRx1IsBusy(deviceAddress)){
			if(buf1[0]==0x01){ DemoState =BT_STATE_PROCESS; break;}
			if(buf1[0]!=end_num){ DemoState =BT_STATE_READ_EP1; break;}

			#ifdef DEBUG_MODE
			UART2PrintString(message);
			for(data_num=0;data_num<buf1[1]+2;data_num++)
      	          {UART2PutHex(buf1[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

           DemoState = BT_STATE_PROCESS;
		}
        break;


//WRITE ENDPOINT 2
    case BT_STATE_WRITE_ACL:

			#ifdef DEBUG_MODE
			for(data_num=0;data_num<buf[2]+4;data_num++)
	        {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

        if (!USBHostGenericTxIsBusy(deviceAddress))
		{
            if ( (RetVal=USBHostGenericAclWrite( deviceAddress, buf, data_size )) == USB_SUCCESS )
            {
			//UART2PrintString( "HCI COMMAND SENT\r\n" );	
            DemoState = BT_STATE_PROCESS;
            }
            else
            {
			UART2PrintString( "Write Acl Error !\r\n" );	
            }
        }
        break;

//READ ENDPOINT 2 & 1 
    case BT_STATE_READ_ACL_HCI:
        if (!USBHostGenericRx2IsBusy(deviceAddress))
        {
            if ( (RetVal=USBHostGenericAclRead(deviceAddress, buf, DATA_PACKET_LENGTH)) == USB_SUCCESS )
            {
                DemoState = BT_STATE_READ_ACL_WAITING;
            }
            else
            {
                UART2PrintString( "Read Acl Error !\r\n" );
            }
        }
        break;

    case BT_STATE_READ_ACL_WAITING:
        if (!USBHostGenericRx2IsBusy(deviceAddress)){

			#ifdef DEBUG_MODE
			UART2PrintString(message);
			for(data_num=0;data_num<buf[2]+4;data_num++)
   	  	        {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "　\r\n" );
			#endif

			{
#define WII_2		((short)0x0001)
#define WII_1		((short)0x0002)
#define WII_B		((short)0x0004)
#define WII_A		((short)0x0008)
#define WII_MINUS	((short)0x0010)
#define WII_HOME	((short)0x0080)
#define WII_D		((short)0x0100)
#define WII_U		((short)0x0200)
#define WII_R		((short)0x0400)
#define WII_L		((short)0x0800)
#define WII_PLUS	((short)0x1000)
#define WII_CNTRLBIT_MERGE (8)
//buf[10,11]がWiiコントローラ状態

				int i = 0;
				int curr_controller = 0;
				int find_num = bddev_num+1;
				// find controller
				for(i=0; i< find_num; i++) {
					if( bddev[i].handle[0] == buf[0]) {
						break;
					}
				}
				if ( i == find_num ) {
					UART2PrintString( "Unknown handle:" );
					UART2PutHex(buf[0]);
					UART2PrintString( "\r\n" );

					break; // error
				} else {
					curr_controller = i;
				}

				if(curr_controller != 0) {
					unsigned short diff;
					UART2PrintString( "Wii : " );
	   	  	        UART2PutHex(buf[10]);UART2PutHex(buf[11]);
					UART2PrintString( "　\r\n" );

					wii_ctrl_state_now = (buf[10] << WII_CNTRLBIT_MERGE)|buf[11];

					/* 前回との差分bitを取り出す */
					diff = bddev[curr_controller].wii_ctrl_state_back ^ wii_ctrl_state_now;
					UART2PrintString( "back: " );
					UART2PutHex(bddev[curr_controller].wii_ctrl_state_back>>WII_CNTRLBIT_MERGE);UART2PutHex(bddev[curr_controller].wii_ctrl_state_back &0x0F);
					UART2PrintString( "　\r\n" );
					UART2PrintString( "now : " );
					UART2PutHex(wii_ctrl_state_now>>WII_CNTRLBIT_MERGE);UART2PutHex(wii_ctrl_state_now &0x0F);
					UART2PrintString( "　\r\n" );

					UART2PrintString( "diff: " );
					UART2PutHex(diff>>WII_CNTRLBIT_MERGE);UART2PutHex(diff &0x0F);
					UART2PrintString( "　\r\n" );

					if( diff & WII_2 ) {
						UART2PrintString( "WII_2\r\n" );
						if( wii_ctrl_state_now & WII_2 ) {
							J1_A = PRESS;
						} else {
							J1_A = RELEASE;
						}
					}

					if( diff & WII_1 ) {
						UART2PrintString( "WII_1\r\n" );
						if( wii_ctrl_state_now & WII_1 ) {
							J1_B = PRESS;
						} else {
							J1_B = RELEASE;
						}
					}

					if( diff & WII_PLUS ) { //SELECT
						UART2PrintString( "WII_PLUS\r\n" );
						if( wii_ctrl_state_now & WII_PLUS ) {
							J1_SELECT = PRESS;
						} else {
							J1_SELECT = RELEASE;
						}
					}

					if( diff & WII_MINUS) { //START
						UART2PrintString( "WII_MINUS\r\n" );
						if( wii_ctrl_state_now & WII_MINUS ) {
							J1_START = PRESS;
						} else {
							J1_START = RELEASE;
						}
					}

					if( diff & WII_U) { //上
						UART2PrintString( "WII_U\r\n" );
						if( wii_ctrl_state_now & WII_U ) {
							J1_UP = PRESS;
						} else {
							J1_UP = RELEASE;
						}
					}

					if( diff & WII_D) { //下
						UART2PrintString( "WII_D\r\n" );
						if( wii_ctrl_state_now & WII_D ) {
							J1_DOWN = PRESS;
						} else {
							J1_DOWN = RELEASE;
						}
					}

					if( diff & WII_L) { //左
						UART2PrintString( "WII_L\r\n" );
						if( wii_ctrl_state_now & WII_L ) {
							J1_LEFT = PRESS;
						} else {
							J1_LEFT = RELEASE;
						}
					}

					if( diff & WII_R) { //右
						UART2PrintString( "WII_R\r\n" );
						if( wii_ctrl_state_now & WII_R ) {
							J1_RIGHT = PRESS;
						} else {
							J1_RIGHT = RELEASE;
						}
					}

					bddev[curr_controller].wii_ctrl_state_back = wii_ctrl_state_now;
				}
			}
			DemoState = BT_STATE_READ_HCI;
		}
        break;

	case BT_STATE_READ_HCI:
		buf1[0]=0xff;
        if (!USBHostGenericRx1IsBusy(deviceAddress))
        {
            if ( (RetVal=USBHostGenericRead(deviceAddress, buf1, DATA_PACKET_LENGTH)) == USB_SUCCESS )
            {
                DemoState = BT_STATE_READ_HCI_WAITING;
                //UART2PrintString( "READ EP1\r\n" );
            }
            else
            {
                UART2PrintString( "Device Read Error !\r\n" );
            }
        }
        break;

    case BT_STATE_READ_HCI_WAITING:
        if (!USBHostGenericRx1IsBusy(deviceAddress)){

			#ifdef DEBUG_MODE
			UART2PrintString( "HCI: " );
			for(data_num=0;data_num<buf1[1]+2;data_num++)
      	          {UART2PutHex(buf1[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			if(buf1[0]==0xff) {DemoState = BT_STATE_PROCESS;}
			else {DemoState = BT_STATE_READ_HCI;}
		}
        break;

//********************************************************************************

    /** Error state:  Hold here until detached **/
    case BT_STATE_ERROR:                          // To Do: Flash LEDs
        break;

    default:
        DemoState = BT_INITIALIZE;
        break;
    }

    DelayMs(1); // 1ms delay

} // ManageDemoState


//******************************************************************************
//******************************************************************************
// USB Support Functions
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        USB_ApplicationEventHandler
 *
 * Preconditions:   The USB must be initialized.
 *
 * Input:           event       Identifies the bus event that occured
 *
 *                  data        Pointer to event-specific data
 *
 *                  size        Size of the event-specific data
 *
 * Output:          deviceAddress (global)
 *                  Updates device address when an attach or detach occurs.
 *
 *                  DemoState (global)
 *                  Updates the demo state as appropriate when events occur.
 *
 * Returns:         TRUE if the event was handled, FALSE if not
 *
 * Side Effects:    Event-specific actions have been taken.
 *
 * Overview:        This routine is called by the Host layer or client
 *                  driver to notify the application of events that occur.
 *                  If the event is recognized, it is handled and the
 *                  routine returns TRUE.  Otherwise, it is ignored (or
 *                  just "sniffed" and the routine returns FALSE.
 *************************************************************************/

BOOL USB_ApplicationEventHandler ( BYTE address, USB_EVENT event, void *data, DWORD size )
{
    #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
        BYTE i;
    #endif

    // Handle specific events.
    switch (event)
    {
        case EVENT_GENERIC_ATTACH:
            if (size == sizeof(GENERIC_DEVICE_ID))
            {
                deviceAddress   = ((GENERIC_DEVICE_ID *)data)->deviceAddress;
                DemoState = BT_STATE_PROCESS; HciState=HCI_CMD_RESET;//YTS !!!!!!!!!!!!!!!!!
                UART2PrintString( "Generic demo device attached - event, deviceAddress=" );
                UART2PutDec( deviceAddress );
                UART2PrintString( "\r\n" );
                #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
                    for (i=1; i<((GENERIC_DEVICE_ID *)data)->serialNumberLength; i++)
                    {
                        UART2PutChar( ((GENERIC_DEVICE_ID *)data)->serialNumber[i] );
                    }
                #endif
                UART2PrintString( "\r\n" );
                return TRUE;
            }
            break;

        case EVENT_GENERIC_DETACH:
            deviceAddress   = 0;
            DemoState = BT_INITIALIZE;//YTS
            UART2PrintString( "Generic demo device detached - event\r\n" );
            return TRUE;

        case EVENT_GENERIC_TX_DONE:           // The main state machine will poll the driver.
        case EVENT_GENERIC_RX1_DONE://YTS
        case EVENT_GENERIC_RX2_DONE://YTS
            return TRUE;

        case EVENT_VBUS_REQUEST_POWER:
            // We'll let anything attach.
            return TRUE;

        case EVENT_VBUS_RELEASE_POWER:
            // We aren't keeping track of power.
            return TRUE;

        case EVENT_HUB_ATTACH:
            UART2PrintString( "\r\n***** USB Error - hubs are not supported *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSUPPORTED_DEVICE:
            UART2PrintString( "\r\n***** USB Error - device is not supported *****\r\n" );
            return TRUE;
            break;

        case EVENT_CANNOT_ENUMERATE:
            UART2PrintString( "\r\n***** USB Error - cannot enumerate device *****\r\n" );
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            UART2PrintString( "\r\n***** USB Error - client driver initialization error *****\r\n" );
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            UART2PrintString( "\r\n***** USB Error - out of heap memory *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
            UART2PrintString( "\r\n***** USB Error - unspecified *****\r\n" );
            return TRUE;
            break;

        case EVENT_SUSPEND:
        case EVENT_DETACH:
        case EVENT_RESUME:
        case EVENT_BUS_ERROR:
            return TRUE;
            break;

        default:
            break;
    }

    return FALSE;

} // USB_ApplicationEventHandler


//******************************************************************************
//******************************************************************************
// Main
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        main
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         Never exits
 *
 * Side Effects:    Runs the application
 *
 * Overview:        This is the USB Custom Demo Application's main entry
 *                  point.
 *************************************************************************/

int main ( void )
{
    // Initialize the processor and peripherals.
    if ( InitializeSystem() != TRUE )
    {
        UART2PrintString( "\r\n\r\nCould not initialize USB Custom Demo App - system.  Halting.\r\n\r\n" );
        while (1);
    }
    if ( USBHostInit(0) == TRUE )
    {
        UART2PrintString( "\r\n\r\n***** USB Custom Demo App Initialized *****\r\n\r\n" );
    }
    else
    {
        UART2PrintString( "\r\n\r\nCould not initialize USB Custom Demo App - USB.  Halting.\r\n\r\n" );
        while (1);
    }

    // Main Processing Loop
    while (1)
    {
        // This demo does not check for overcurrent conditions.  See the
        // USB Host - Data Logger for an example of overcurrent detection
        // with the PIC24F and the USB PICtail Plus.

        // Maintain USB Host State
        USBHostTasks();
        // Maintain Demo Application State
        ManageDemoState();
    }

    return 0;

} // main


/*************************************************************************
 * EOF main.c
 */

