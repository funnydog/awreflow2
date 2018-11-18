/*
 * Andy's Workshop Reflow Controller ATMega328p firmware
 * Copyright (c) 2015 Andy Brown. http://www.andybrown.me.uk
 * Please see website for licensing terms.
 */

#pragma once


namespace awreflow {


  /*
   * Class to handle the bluetooth communications. Commands are read asynchronously. A command consists of
   * the following 5 bytes:
   *   [0,1] = 0xAA, 0x55
   *   [2]   = <command ID>
   *   [3,4] = 16-bit sequence number
   * 
   * The sender expects the same sequence number in the response. The 0xAA, 0x55 preamble is used to
   * resync this receiver in the event of communication loss. A response looks like this:
   *   [0,1] = 0xAA, 0x55
   *   [2]   = <command ID>
   *   [3,4] = 16-bit sequence number
   *   [5]   = ACK/NACK
   *   [6]   = <command specific response>
   */

  class Bluetooth {

    protected:
      uint8_t _commandData[10];   // must be large enough for the biggest response
      uint8_t _commandPos;        // remaining data count to transmit back

    protected:
      bool isValidCommand(Command c) const;
      void sendATCommand(const char *command) const;

    public:
      Bluetooth();

      bool readByte(uint8_t& b) const;

      void writeReply(Acknowledge ackStatus,const void *data,uint8_t dataSize);
      Command getCommandReceived();
      uint8_t getSingleByteParameter() const;
  };


  /*
   * Constructor
   */


  inline Bluetooth::Bluetooth() {

    _commandPos=0;

    // 9600 baud: (http://wormfood.net/avrbaudcalc.php?postbitrate=9600&postclock=8&hidetables=1)
 
    UBRR0H=0;
    UBRR0L=51;

    UCSR0A=0;
    UCSR0B=(1 << RXEN0) | (1 << TXEN0);                   // enable RX, TX
    UCSR0C=(1 << UCSZ01) | (1 << UCSZ00);                 // 8-N-1
  }


  /*
   * Send an AT command to the adaptor
   */

  inline void Bluetooth::sendATCommand(const char *command) const {

    const char *ptr;

    // send the string

    for(ptr=command;*ptr;ptr++) {
      while(!(UCSR0A & (1 << UDRE0)));
      UDR0=*ptr;
    }

    // wait for completion then delay 1/10s

    while(!(UCSR0A & (1 << UDRE0)));
    MillisecondTimer::delay(100);
  }


  /*
   * Get the command received, if any
   */

  inline Command Bluetooth::getCommandReceived() {

    // protection

    if(_commandPos>=sizeof(_commandData))
      return Command::NONE;

    // is something ready?

    if((UCSR0A & (1 << RXC0))==0)
      return Command::NONE;

    // move data into storage

    _commandData[_commandPos]=UDR0;

    // any invalid data resets the reading (this command is lost)

    if((_commandPos==0 && _commandData[0]!=0xAA) ||
       (_commandPos==1 && _commandData[1]!=0x55) ||
       (_commandPos==2 && !isValidCommand(static_cast<Command>(_commandData[2])))) {
      _commandPos=0;
    }
    else
      _commandPos++;

    // done?

    Command cmd=static_cast<Command>(_commandData[2]);

    if((_commandPos==5 && (cmd==Command::READ_TEMPERATURE ||
                           cmd==Command::READ_SETTINGS)) || 

       (_commandPos==6 && (cmd==Command::SET_DUTY_CYCLE ||
                           cmd==Command::SET_SENSOR_OFFSET ||
                           cmd==Command::SET_LCD_BACKLIGHT ||
                           cmd==Command::SET_LCD_CONTRAST))) {

      _commandPos=0;
      return cmd;
    }

    return Command::NONE;
  }


  /*
   * Get the single byte command parameter
   */

  inline uint8_t Bluetooth::getSingleByteParameter() const {
    return _commandData[5];
  }


  /*
   * Write a reply back with optional payload
   */

  inline void Bluetooth::writeReply(Acknowledge ackStatus,const void *data,uint8_t dataSize) {

    uint8_t i;

    // the original 5 byte header must still be in _commandBuffer
    // append the ACK/NACK

    _commandData[5]=static_cast<uint8_t>(ackStatus);

    // write the optional payload

    if(dataSize)
      memcpy(&_commandData[6],data,dataSize);

    // send out

    for(i=0;i<5+1+dataSize;i++) {
      while(!(UCSR0A & (1 << UDRE0)));
      UDR0=_commandData[i];
    }
  }


  /*
   * Check if this is a valid command
   */

  inline bool Bluetooth::isValidCommand(Command c) const {
    return c>=Command::READ_TEMPERATURE && c<=Command::READ_SETTINGS;
  }
}
