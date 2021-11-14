
#define Encoder_r_A_PIN  7
#define Encoder_r_B_PIN  23
#define Encoder_l_A_PIN  26
float speed_r,speed_l; //right an dleft speed compuated form encoder
long old_count_r,old_count_l,encount_r,encount_l;
volatile long count_Encoder_r; // used by encoder to count the rotation
volatile bool oldEncoder_r_A;  // used by encoder to remember prior state of A
volatile bool oldEncoder_r_B;  // used by encoder to remember prior state of B

volatile long count_Encoder_l; // used by encoder to count the rotation
volatile bool oldEncoder_l_A;  // used by encoder to remember prior state of A
volatile bool oldEncoder_l_B;  // used by encoder to remember prior state of B

/**********ISR funtion, called when interpted**************/
//right wheel encoder
ISR( INT6_vect ) {

  // First, Read in the new state of the encoder pins.
  // Standard pins, so standard read functions.
  boolean newEncoder_r_B = digitalRead( Encoder_r_B_PIN );
  boolean newEncoder_r_A = digitalRead( Encoder_r_A_PIN );

  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  newEncoder_r_A ^= newEncoder_r_B;

  byte state = 0;
  state = state | ( newEncoder_r_A  << 3 );
  state = state | ( newEncoder_r_B  << 2 );
  state = state | ( oldEncoder_r_A  << 1 );
  state = state | ( oldEncoder_r_B  << 0 );

  if( state == 1 ) {
    count_Encoder_r = count_Encoder_r - 1;
  } else if( state == 2 ) {
    count_Encoder_r = count_Encoder_r + 1;
  } else if( state == 4 ) {
    count_Encoder_r = count_Encoder_r + 1;    
  } else if( state == 7 ) {
    count_Encoder_r = count_Encoder_r - 1;
  } else if( state == 8 ) {
    count_Encoder_r = count_Encoder_r - 1;
  } else if( state == 11 ) { 
    count_Encoder_r = count_Encoder_r + 1;
  } else if( state == 13 ) {
    count_Encoder_r = count_Encoder_r + 1;
  } else if( state == 14 ) {
    count_Encoder_r = count_Encoder_r - 1; 
  }
  
  // Save current state as old state for next call.
  oldEncoder_r_A = newEncoder_r_A;
  oldEncoder_r_B = newEncoder_r_B;
};

//left wheel encoder
ISR( PCINT0_vect ) {

  boolean newEncoder_l_B = PINE & (1<<PINE2);
  //boolean newEncoder_l_B = PINE & B00000100;  // Does same as above.

  // Standard read fro the other pin.
  boolean newEncoder_l_A = digitalRead( Encoder_l_A_PIN ); // 26 the same as A8

  // Some clever electronics combines the
  // signals and this XOR restores the 
  // true value.
  newEncoder_l_A ^= newEncoder_l_B;


  byte state = 0;                   
  state = state | ( newEncoder_l_A  << 3 );
  state = state | ( newEncoder_l_B  << 2 );
  state = state | ( oldEncoder_l_A  << 1 );
  state = state | ( oldEncoder_l_B  << 0 );

  if( state == 1 ) {
    count_Encoder_l = count_Encoder_l - 1;
  } else if( state == 2 ) {
    count_Encoder_l = count_Encoder_l + 1;
  } else if( state == 4 ) {
    count_Encoder_l = count_Encoder_l + 1;    
  } else if( state == 7 ) {
    count_Encoder_l = count_Encoder_l - 1;
  } else if( state == 8 ) {
    count_Encoder_l = count_Encoder_l - 1;
  } else if( state == 11 ) { 
    count_Encoder_l = count_Encoder_l + 1;  
  } else if( state == 13 ) {
    count_Encoder_l = count_Encoder_l + 1;
  } else if( state == 14 ) {
    count_Encoder_l = count_Encoder_l - 1;
  }

  // Save current state as old state for next call.
  oldEncoder_l_A = newEncoder_l_A;
  oldEncoder_l_B = newEncoder_l_B; 
};


void setupEncoder1() {

  // Initialise our count value to 0.
  count_Encoder_r = 0;
  
  // Initialise the prior A & B signals
  // to zero, we don't know what they were.
  oldEncoder_r_A = 0;
  oldEncoder_r_B = 0;

  // Setup pins for encoder 1
  pinMode( Encoder_r_A_PIN, INPUT );
  pinMode( Encoder_r_B_PIN, INPUT );

  EIMSK = EIMSK & ~(1<<INT6);
  //EIMSK = EIMSK & B1011111; // Same as above.
  
  // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
  // Used to set up INT6 interrupt
  EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60

  EIFR |= ( 1 << INTF6 );
  //EIFR |= B01000000;  // same as above

  EIMSK |= ( 1 << INT6 );
  //EIMSK |= B01000000; // Same as above

};

void setupEncoder0() {

    // Initialise our count value to 0.
    count_Encoder_l = 0;
    
    // Initialise the prior A & B signals
    // to zero, we don't know what they were.
    oldEncoder_l_A = 0;
    oldEncoder_l_B = 0;

    DDRE = DDRE & ~(1<<DDE6);

    PORTE = PORTE | (1<< PORTE2 );
    //PORTE = PORTE | 0B00000100;

    // Encoder0 uses conventional pin 26
    pinMode( Encoder_l_A_PIN, INPUT );
    digitalWrite( Encoder_l_A_PIN, HIGH ); // Encoder 0 xor
    
    // Disable interrupt first
    PCICR = PCICR & ~( 1 << PCIE0 );
    // PCICR &= B11111110;  // Same as above
    
    // 11.1.7 Pin Change Mask Register 0 – PCMSK0
    PCMSK0 |= (1 << PCINT4);
    
    // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
    PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

    // Enable
    PCICR |= (1 << PCIE0);
};
