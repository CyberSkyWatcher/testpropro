#include <stdio.h>
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
//

// variables programme
unsigned short i;
int bufferLCD[8];                //buffer d'affichage lcd / uart
unsigned short adc_val;          // variable adc
int consigne;
//

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
unsigned short Reglage_dist(void){
/* enregistrement de la distance minamale désirée
   Minimum 1m !! sécurité max 11m */
    int valeur;
    unsigned short distance;
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);
    while(reglage_flag){
        if (Button(&PORTD, 0, 1, 1)) {
            buttonstate = 1;
        }
        if (buttonstate && Button(&PORTD, 0, 1, 0)) {
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Cmd(_LCD_CURSOR_OFF);
            //Lcd_Out(2,10,statut_Sart);
            reglage_flag=0;
            buttonstate = 0;
        }
        memset(bufferLCD,0,sizeof(bufferLCD));
        valeur = ADC_Read(23);
        distance = map(valeur,0,1023,1,11);
        sprinti(bufferLCD,"Dist MIN:%02d", distance);
        Lcd_Out(2,4,"REGLAGE");
        Lcd_Out(3,5,bufferLCD);
        Lcd_Out_CP("m");
    }
    return distance;
}


float mesure_distance(void)
{

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

    consigne = Reglage_dist();

    for(;;)
    {
        /*memset(bufferLCD,0,sizeof(bufferLCD));
         sprinti(bufferLCD,"%02d", consigne);
         Lcd_Out(4,1,bufferLCD);*/
    }
}
