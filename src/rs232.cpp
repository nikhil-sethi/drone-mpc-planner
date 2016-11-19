#include "rs232.h"
#include <iostream>


int Cport,
    error;

struct termios new_port_settings,
       old_port_settings[30];

int RS232_OpenComport( int baudrate)
{
  int baudr, status;

  switch(baudrate)
  {
    case      50 : baudr = B50;
                   break;
    case      75 : baudr = B75;
                   break;
    case     110 : baudr = B110;
                   break;
    case     134 : baudr = B134;
                   break;
    case     150 : baudr = B150;
                   break;
    case     200 : baudr = B200;
                   break;
    case     300 : baudr = B300;
                   break;
    case     600 : baudr = B600;
                   break;
    case    1200 : baudr = B1200;
                   break;
    case    1800 : baudr = B1800;
                   break;
    case    2400 : baudr = B2400;
                   break;
    case    4800 : baudr = B4800;
                   break;
    case    9600 : baudr = B9600;
                   break;
    case   19200 : baudr = B19200;
                   break;
    case   38400 : baudr = B38400;
                   break;
    case   57600 : baudr = B57600;
                   break;
    case  115200 : baudr = B115200;
                   break;
    case  230400 : baudr = B230400;
                   break;
    case  460800 : baudr = B460800;
                   break;
    case  500000 : baudr = B500000;
                   break;
    case  576000 : baudr = B576000;
                   break;
    case  921600 : baudr = B921600;
                   break;
    case 1000000 : baudr = B1000000;
                   break;
    case 3000000 : baudr = B3000000;
                   break;
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }

  Cport= open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
  if(Cport==-1)
  {
    perror("unable to open comport ");
    return(1);
  }

  error = tcgetattr(Cport, old_port_settings );
  if(error==-1)
  {
    close(Cport);
    perror("unable to read portsettings ");
    return(1);
  }
  memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

  new_port_settings.c_cflag = baudr | CS8 | CLOCAL | CREAD;
  new_port_settings.c_iflag = IGNPAR;
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
  new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */
  error = tcsetattr(Cport, TCSANOW, &new_port_settings);
  if(error==-1)
  {
    close(Cport);
    perror("unable to adjust portsettings ");
    return(1);
  }

  if(ioctl(Cport, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
    return(1);
  }

  status |= TIOCM_DTR;    /* turn on DTR */
  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
    return(1);
  }

  return(0);
}


int RS232_PollComport( unsigned char *buf, int size)
{
  int n;

  n = read(Cport, buf, size);

  return(n);
}


int RS232_SendByte( unsigned char byte)
{
  int n;

  n = write(Cport, &byte, 1);
  if(n<0)  return(1);

  return(0);
}


int RS232_SendBuf( unsigned char *buf, int size)
{
  return(write(Cport, buf, size));
}


void RS232_CloseComport()
{
  int status;

  if(ioctl(Cport, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus\n");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */
  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus\n");
  }

  tcsetattr(Cport, TCSANOW, old_port_settings );
  close(Cport);
  //std::cout << "Rs232 close result: " << res << std::endl;
}

/*
Constant  Description
TIOCM_LE  DSR (data set ready/line enable)
TIOCM_DTR DTR (data terminal ready)
TIOCM_RTS RTS (request to send)
TIOCM_ST  Secondary TXD (transmit)
TIOCM_SR  Secondary RXD (receive)
TIOCM_CTS CTS (clear to send)
TIOCM_CAR DCD (data carrier detect)
TIOCM_CD  Synonym for TIOCM_CAR
TIOCM_RNG RNG (ring)
TIOCM_RI  Synonym for TIOCM_RNG
TIOCM_DSR DSR (data set ready)

http://linux.die.net/man/4/tty_ioctl
*/

int RS232_IsDCDEnabled( )
{
  int status;

  ioctl(Cport, TIOCMGET, &status);

  if(status&TIOCM_CAR) return(1);
  else return(0);
}

int RS232_IsCTSEnabled( )
{
  int status;

  ioctl(Cport, TIOCMGET, &status);

  if(status&TIOCM_CTS) return(1);
  else return(0);
}

int RS232_IsDSREnabled( )
{
  int status;

  ioctl(Cport, TIOCMGET, &status);

  if(status&TIOCM_DSR) return(1);
  else return(0);
}

void RS232_enableDTR( )
{
  int status;

  if(ioctl(Cport, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status |= TIOCM_DTR;    /* turn on DTR */

  if(ioctl(Cport, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_disableDTR( )
{
  int status;

  if(ioctl(Cport, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */

  if(ioctl(Cport, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_enableRTS( )
{
  int status;

  if(ioctl(Cport, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_disableRTS( )
{
  int status;

  if(ioctl(Cport, TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport, TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_cputs( const char *text)  /* sends a string to serial port */
{
  while(*text != 0)   RS232_SendByte( *(text++));
}


