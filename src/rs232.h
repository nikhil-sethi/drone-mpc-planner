/*
 * This file contains the usb serial interface to retrieve data from the Delfly camera
 */

#ifndef rs232_INCLUDED
#define rs232_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>


#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>


	int RS232_OpenComport( int);
	int RS232_PollComport( unsigned char *, int);
	int RS232_SendByte( unsigned char);
	int RS232_SendBuf( unsigned char *, int);
	void RS232_CloseComport();
	void RS232_cputs( const char *);
	int RS232_IsDCDEnabled();
	int RS232_IsCTSEnabled();
	int RS232_IsDSREnabled();
	void RS232_enableDTR();
	void RS232_disableDTR();
	void RS232_enableRTS();
	void RS232_disableRTS();


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif


