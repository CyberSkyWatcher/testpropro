//
// Created by juanf on 4/29/2020.
//

#define EEPROM_WR 0xA0
#define EEPROM_RD 0xA1

// Variables LCD
sbit LCD_RS at RB4_bit;
sbit LCD_EN at RB5_bit;
sbit LCD_D4 at RB0_bit;
sbit LCD_D5 at RB1_bit;
sbit LCD_D6 at RB2_bit;
sbit LCD_D7 at RB3_bit;

sbit LCD_RS_Direction at TRISB4_bit;
sbit LCD_EN_Direction at TRISB5_bit;
sbit LCD_D4_Direction at TRISB0_bit;
sbit LCD_D5_Direction at TRISB1_bit;
sbit LCD_D6_Direction at TRISB2_bit;
sbit LCD_D7_Direction at TRISB3_bit;
//

/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;


// Variables Texte
char txt1[] = "Latino_Gang";
char txt2[] = " PRESS D1 ";
//

// VARIABLES BOOL --> boutons + flags
bit buttonstate;
bit buttonstate1;
bit buttonstate2;
bit reglage_flag;
bit run_flag;
bit timer_flag;
//

// variables programme
unsigned short i;
int bufferLCD[8];                //buffer d'affichage lcd / uart
int consigne;
//

void Interrupt(){
    // Timer interuption routine
    if (TMR0IF_bit){
        TMR0IF_bit = 0;
        TMR0H      = 0x0B;
        TMR0L      = 0xDC;
        //timer_flag = 1;
    }
/*if(RC1IF_bit == 1){             // Checks for Receive Interrupt Flag bit
    incomingByte  = UART_Read();   // Storing read data
    //UART1_Write(incomingByte);
    switch (incomingByte){
      case '<': //start acquisition of a new command
        index = 0;
        //memset(command_line, 0, MAX_CMD_LEN - 1);//Set command to 0
        break;
      case '>': // finish acquisition of current command
        command_line[index+1] = 0;
        isCommandReady = 1;
        break;
      default : // command acquisition
        command_line[index++] = incomingByte;
    }
  }*/
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Fonction d'?criture d'un octet dans l'Eeprom
void WriteToEEprom(unsigned short Address, unsigned short Data)
{
    I2C1_Start();                                   // issue I2C start signal
    I2C1_Wr(EEPROM_WR);                             // send byte via I2C  (device address + W)
    I2C1_Wr(Address);                               // send byte (address of EEPROM location)
    I2C1_Wr(Data);                                  // send data (data to be written)
    I2C1_Stop();                                    // issue I2C stop signal
}

// Fonction de lecture d'un octet de l'Eeprom
unsigned short ReadFromEEprom(unsigned short Address)
{
    unsigned short dataa;
    I2C1_Start();                                       // issue I2C start signal
    I2C1_Wr(EEPROM_WR);                                 // send byte via I2C  (device address + W)
    I2C1_Wr(Address);                                   // send byte (data address)
    I2C1_Repeated_Start();                              // issue I2C signal repeated start
    I2C1_Wr(EEPROM_RD);                                 // send byte (device address + R)
    dataa = I2C1_Rd(0u);                                // Read the data (NO acknowledge)
    I2C1_Stop();                                        // issue I2C stop signal
    return dataa;
}

unsigned short Reglage_dist(void){
/* enregistrement de la distance minamale désirée
   Minimum 1m !! sécurité max 11m */
    int valeur;
    int distance;
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);
    while(reglage_flag){
        memset(bufferLCD,0,sizeof(bufferLCD));
        valeur = ADC_Read(23);
        distance = map(valeur,0,1023,1,11);
        sprinti(bufferLCD,"Dist MIN:%02d", distance);
        Lcd_Out(2,4,"REGLAGE");
        Lcd_Out(3,5,bufferLCD);
        Lcd_Out_CP("m");
        if (Button(&PORTD, 0, 1, 1)) {
            buttonstate = 1;
        }
        if (buttonstate && Button(&PORTD, 0, 1, 0)) {
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Cmd(_LCD_CURSOR_OFF);
            memset(bufferLCD,0,sizeof(bufferLCD));
            sprinti(bufferLCD,"CONSIGNE:%02d", distance);
            Lcd_Out(4,1,bufferLCD);
            Lcd_Out_CP("m");
            run_flag = 1;
            reglage_flag=0;
            buttonstate = 0;
        }
    }
    return distance;
}

void button_check(void){
    // START
    if (Button(&PORTD, 0, 1, 1)) {
        buttonstate = 1;
    }
    if (buttonstate && Button(&PORTD, 0, 1, 0)) {
        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Cmd(_LCD_CURSOR_OFF);
        //Lcd_Out(2,10,statut_Sart);
        run_flag = 1;
        buttonstate = 0;
    }

    // STOP
    if (Button(&PORTD, 1, 1, 1)) {
        buttonstate1 = 1;
    }
    if (buttonstate1 && Button(&PORTD, 1, 1, 0)) {
        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Cmd(_LCD_CURSOR_OFF);
        run_flag = 0;
        buttonstate1 = 0;
    }
}
float mesure_distance(void)
{
    // EZIO --> ultrasons  ==> j'ai déjà commencé mais ça donne pas grand chose
    /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
    /*digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);*/
    LATD.f4 = 1;
    delay_us(10);
    LATD.f4 = 0;
    /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
    //long measure = Time_dateDiff();                    //pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);

    /* 3. Calcul la distance à partir du temps mesuré */
    //float distance_mm = measure / 2.0 * SOUND_SPEED;

    //return distance_mm;
}

void main()
{
    reglage_flag = 1;
    run_flag = 0;
    buttonstate = 0;
    buttonstate1 = 0;
    buttonstate2 = 0;

    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 16;
    TRISD0_bit = 1;
    TRISD1_bit = 1;
    TRISD2_bit = 1;
    TRISD5_bit = 1;
    ADC_Init();
    UART1_Init(9600);
    I2C1_Init(100000);
    Delay_100ms();
    Lcd_Init();

    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);
    Lcd_Out(1,4,txt1);
    delay_ms(1000);
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Out(1,4,txt2);

    //initialisation innterruption timer
    T0CON         = 0x85;
    TMR0H         = 0x0B;
    TMR0L         = 0xDC;
    GIE_bit = 1;
    TMR0IE_bit = 1;
    //
    //interuptions UART
    RC1IE_bit = 1;                    // turn ON interrupt on UART1 receive
    RC1IF_bit = 0;                    // Clear interrupt flag
    PEIE_bit  = 1;                    // Enable peripheral interrupts
    GIE_bit   = 1;                    // Enable GLOBAL interrupts
    //

    consigne = Reglage_dist();
    for(;;)
    {
        button_check();
        if(run_flag){

        }
        /*memset(bufferLCD,0,sizeof(bufferLCD));
         sprinti(bufferLCD,"%02d", consigne);
         Lcd_Out(4,1,bufferLCD);*/
    }
}