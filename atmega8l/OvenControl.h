/*
 * Andy's Workshop Reflow Controller ATMega328p firmware
 * Copyright (c) 2015 Andy Brown. http://www.andybrown.me.uk
 * Please see website for licensing terms.
 */

#pragma once


namespace awreflow {


  /*
   * Class to control the oven duty cycle on/off. This is achieved through AC phase control. A zero-crossing
   * interrupt starts the control of each phase. We use this interrupt to start a timer to count up to a
   * calculated value. When the counter is reached the triac gate is enabled for a defined number of ticks
   * which we use the timer's overflow interrupt to tell us about. The triac will stay on until the next
   * zero crossing.
   */

  class OvenControl {

    protected:
      volatile uint8_t _percentage;
      volatile uint8_t _counter;

      static const PROGMEM uint8_t LookupTable[99];   // percentages 1..99

    protected:
      void stopTimer() const;
    
    public:
      OvenControl();

      void on() const;
      void off() const;

      void setDutyCycle(uint8_t percentage);

      void zeroCrossingHandler();
      void timerComparatorHandler() const;
      void timerOverflowHandler() const;
  };


  /*
   * Constructor
   */

  inline OvenControl::OvenControl() {

    // reset variables

    _counter=0;
    _percentage=0;

    // initialise the callback

    extern OvenControl *OvenControlInstance;
    OvenControlInstance=this;

    // timer2 is used for async control of the pulse timing

    stopTimer();

    // enable comparator A and overflow interrupts

    TIMSK2 |= (1 << OCIE2A) | (1 << TOIE2);
  }


  /*
   * Set a new duty cycle percentage.
   */

  inline void OvenControl::setDutyCycle(uint8_t percentage) {

    uint16_t newCounter;

    // percentages between 1 and 99 inclusive use the lookup table to translate a linear
    // demand for power to a position on the phase angle axis

    if(percentage>0 && percentage<100)
      percentage=pgm_read_byte(&LookupTable[percentage-1]);

    // calculate the new counter value

    newCounter=((TICKS_PER_HALF_CYCLE-MARGIN_TICKS-TRIAC_PULSE_TICKS)*(100-percentage))/100;

    // set the new state with interrupts off because 16-bit writes are not atomic

    cli();
    _counter=newCounter;
    _percentage=percentage;
    sei();
  }


  /*
   * Stop the timer
   */

   inline void OvenControl::stopTimer() const {
    TCCR2B=0;
   }


  /*
   * Switch off
   */

  inline void OvenControl::off() const {
    stopTimer();
    GpioActivateOven::reset();
  }


  /*
   * Switch on
   */

  inline void OvenControl::on() const {
    stopTimer();
    GpioActivateOven::set();
  }


  /*
   * The zero crossing ISR handler
   */

  inline void OvenControl::zeroCrossingHandler() {

    // 0 is an off switch
    // round up or down a percentage that strays into the end-zone where we have a margin
    // wide enough to cater for the minimum pulse width and the delay in the
    // zero crossing firing

    if(_percentage==0) {
      GpioActivateOven::reset();
      return;
    }
    else if(_percentage==100 || _counter==0)       // either user asked for 100 or calc rounds up to 100
      GpioActivateOven::set();
    else if(_counter>TICKS_PER_HALF_CYCLE-TRIAC_PULSE_TICKS-MARGIN_TICKS) {       // comparison to a constant (fast)
      
      if(_counter>(TICKS_PER_HALF_CYCLE-(TRIAC_PULSE_TICKS-MARGIN_TICKS/2))) {    // also a constant comparison
        
        // round half up to completely off

        GpioActivateOven::reset();
        return;
      }
      else
        _counter=TICKS_PER_HALF_CYCLE-TRIAC_PULSE_TICKS-MARGIN_TICKS;
    }

    // counter is acceptable, or has been rounded down to be acceptable

    OCR2A=_counter;
    TCNT2=0;
    TCCR2B=(1 << CS20) | (1 << CS21) | (1 << CS22);      // start timer at 8MHz / 1024 = 128uS per tick
  }


  /*
   * The comparator ISR handler
   */

  inline void OvenControl::timerComparatorHandler() const {

    // activate the triac gate. we must hold it active for a minimum amount
    // of time before switching it off

    GpioActivateOven::set();

    // the overflow interrupt will fire when the minimum pulse width is reached

    TCNT2=256-TRIAC_PULSE_TICKS;
  } 


  /*
   * The overflow ISR handler
   */

  inline void OvenControl::timerOverflowHandler() const {

    // turn off the oven

    GpioActivateOven::reset();
   
    // turn off the timer. the zero-crossing handler will restart it

    TCCR2B=0;
  }
}
