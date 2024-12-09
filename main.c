/***********************************************************
************************************************************
**										Imports															**
************************************************************
************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

/***********************************************************
***********************************************************
**									Defines	& constants										**
************************************************************
************************************************************/
#define PART_TM4C123GH6PM  
#define servotiltLimit_h 2400 		//bottom ,South,
#define servotiltLimit_l 800			//top ,North,
#define servoazimuthLimit_h 1800	//right ,East,
#define servoazimuthLimit_l 200		//left ,West,

/***********************************************************
************************************************************
**							variables initialization									**
************************************************************
************************************************************/
uint32_t servotilt_val = 1600;	//initial position and comparing start value of the North-South 
uint32_t servoazimuth_val = 200;	//initial position and comparing start value of the East-West

uint32_t pui32ADC0_s1_Value[4];	// hold ldr reading from PE1
uint32_t pui32ADC0_s2_Value[4];	// hold ldr reading from PE2
uint32_t pui32ADC0_s3_Value[4];	// hold ldr reading from PE0
uint32_t pui32ADC0_s4_Value[4];	// hold ldr reading from PE3

int topl;	// top left
int topr;	// top right
int botl;	// bottom left
int botr;	// bottom right

/***********************************************************
************************************************************
**					binary semaphore initialization								**
************************************************************
************************************************************/
SemaphoreHandle_t wBinarySemaphore ;	// semaphore from Reading_N_S to Actuating_E_W
SemaphoreHandle_t xBinarySemaphore ;	// semaphore from	Actuating_E_W to Actuating_N_S
SemaphoreHandle_t yBinarySemaphore ;	// semaphore from Actuating_N_S to Reading_E_W
SemaphoreHandle_t zBinarySemaphore ;	// semaphore from Reading_E_W to Reading_N_S

/***********************************************************
************************************************************
**								servo functions defines									**
************************************************************
************************************************************/
void servotilt(uint32_t); // Tilt Angle Control //N-S
void servoazimuth(uint32_t); // Azimuth Angle Control //E-W 

/***********************************************************
************************************************************
**							functions & tasks defines									**
************************************************************
************************************************************/
void ADC_init (void) ; // configure and initialice the ADC and its used pins to read from the LDRs and convert the reading from analog to digital values.
void PWM_init (void) ;	// configure and initialice the PWM and its used pins to actuate the servos.
void Reading_E_W (void *pvParameters) ; // right and left LDRs readings 
void Reading_N_S (void *pvParameters) ;	// top and bottom LDRs readings
void Actuating_E_W (void *pvParameters) ;	//	actuate the servo in the east west direction
void Actuating_N_S (void *pvParameters) ;	//	actuate the servo in the north south direction

/***********************************************************
************************************************************
**									PWM initialization										**
************************************************************
************************************************************/
void PWM_init (void){
	
    volatile uint32_t ui32Adjust;
	
    SysCtlClockSet(SYSCTL_SYSDIV_16 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // System Clock Set 12.5MHz
    /* PWM Interfacing
     * PWM PB5 M0 GEN 1 OUT 3
     * PWM PB6 M0 GEN 0 OUT 0 */
    SysCtlPWMClockSet( SYSCTL_PWMDIV_16 ); // PWM CLock Set 781.25KHz
		
    SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM0 ); // PWM0 Enable
		
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB ); // GPIO B Enable for PWM
		
    GPIOPinTypePWM( GPIO_PORTB_BASE, GPIO_PIN_5); // Pin Number for PWM Gen 1 for PB5
    GPIOPinTypePWM( GPIO_PORTB_BASE, GPIO_PIN_6); // Pin Number for PWM Gen 0 for PB6
		
    GPIOPinConfigure(  GPIO_PB5_M0PWM3 ); // For PB5
    GPIOPinConfigure(  GPIO_PB6_M0PWM0 ); // For PB6
		
    PWMGenConfigure( PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN ); // For PB5
    PWMGenConfigure( PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN ); // For PB6
		
    PWMGenPeriodSet( PWM0_BASE, PWM_GEN_1, 15625 ); //Total Pulse Period
    PWMGenPeriodSet( PWM0_BASE, PWM_GEN_0, 15625 ); //Total Pulse Period
		
    PWMOutputState( PWM0_BASE, PWM_OUT_3_BIT, true ); // Output for PB5
    PWMOutputState( PWM0_BASE, PWM_OUT_0_BIT, true ); // Output for PB6
		
    PWMGenEnable( PWM0_BASE, PWM_GEN_1 ); // Enable the Gen 1
    PWMGenEnable( PWM0_BASE, PWM_GEN_0 ); // Enable the Gen 0
		
}

/***********************************************************
************************************************************
**									ADC initialization										**
************************************************************
************************************************************/
void ADC_init (void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // ADC0 Enable	
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1); // ADC1 Enable
		
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // GPIO E Enable for ADC
		
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // Initialize PE3 as an ADC input pin	
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); // Initialize PE2 as an ADC input pin
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); // Initialize PE1 as an ADC input pin
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // Initialize PE0 as an ADC input pin
		
		//Configures the initiation criteria for a sample sequence
		//"Base","Sequence Number","Trigger","Priority"
	  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0); //PE1
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);	//PE2
    ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);	//PE0
    ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);	//PE3
		
		//Configures the ADC for one step of a sample sequence.
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
		ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
		
		//Allows the specified sample sequence to be captured when its trigger is detected.
    ADCSequenceEnable(ADC0_BASE, 1);	//PE1
    ADCSequenceEnable(ADC0_BASE, 2);	//PE2
    ADCSequenceEnable(ADC1_BASE, 1);	//PE0
    ADCSequenceEnable(ADC1_BASE, 2);	//PE3
		
		// Clear the ADC interrupt flag.
    ADCIntClear(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 2);
    ADCIntClear(ADC1_BASE, 1);
    ADCIntClear(ADC1_BASE, 2);
}

/***********************************************************
************************************************************
**						Reading east west from ldrs									**
************************************************************
************************************************************/
void Reading_E_W (void *pvParameters){		
	
	while(1){
		
				xSemaphoreTake(yBinarySemaphore ,portMAX_DELAY); // Take the semaphore from Actuating_N_S task
		
				// Trigger the ADC conversion.
        ADCProcessorTrigger(ADC0_BASE, 1);	//PE1
			  ADCProcessorTrigger(ADC1_BASE, 1);	//PE0
		
				// Wait for conversion to be completed.
	      while(!ADCIntStatus(ADC0_BASE, 1, false))
        {
        }
				
				while(!ADCIntStatus(ADC1_BASE, 1, false))
        {
        }
				
				// Clear the ADC interrupt flag.
        ADCIntClear(ADC0_BASE, 1);	//PE1
        ADCIntClear(ADC1_BASE, 1);	//PE0
				
				// Read ADC Value and save them in the array
				ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0_s1_Value);	//PE1
				ADCSequenceDataGet(ADC1_BASE, 1, pui32ADC0_s3_Value);	//PE0
		
				// load values from the array and save them in the corresponding variable
				topl = pui32ADC0_s3_Value[0];//PE0
        topr = pui32ADC0_s1_Value[0];//PE1
				
				xSemaphoreGive(zBinarySemaphore);	//Give the semaphore to Reading_N_S task
				
				for(int i=0;i<100;i++){} // Delay
						
	}// end of the while loop	
}// end of reading the E W

/***********************************************************
************************************************************
**						Reading north south from ldrs								**
************************************************************
************************************************************/
void Reading_N_S (void *pvParameters){
	
	while(1){

				xSemaphoreTake(zBinarySemaphore ,portMAX_DELAY); //Take the semaphore from the Reading_E_W task
		
	      // Trigger the ADC conversion.
        ADCProcessorTrigger(ADC0_BASE, 2);	//PE2
	      ADCProcessorTrigger(ADC1_BASE, 2);	//PE3

				// Wait for conversion to be completed.
	      while(!ADCIntStatus(ADC0_BASE, 2, false))
        {
        }
				
				while(!ADCIntStatus(ADC1_BASE, 2, false))
        {
        }
								
				// Clear the ADC interrupt flag.
				ADCIntClear(ADC0_BASE, 2); //PE2
				ADCIntClear(ADC1_BASE, 2);	//PE3
				
				// Read ADC Valueand save them in the array
        ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0_s2_Value); //PE2
				ADCSequenceDataGet(ADC1_BASE, 2, pui32ADC0_s4_Value);	//PE3
				
				// load values from the array and save them in the corresponding variable
        botl = pui32ADC0_s2_Value[0];//PE2
				botr = pui32ADC0_s4_Value[0];//PE3
					
				xSemaphoreGive(wBinarySemaphore);	// Give the semaphore to the Actuating_E_W task
				
				for(int i=0;i<100;i++){} //Delay
					
	}// end of the while loop
}//end of reading the N S 

/***********************************************************
************************************************************
**						actuating the azimuth angle servo 					**
**							to east west direction										**
************************************************************
************************************************************/
void Actuating_E_W (void *pvParameters){
	
		while(1){

					xSemaphoreTake(wBinarySemaphore ,portMAX_DELAY); // Take the semaphore from the Reading_N_S task
			
						int avgleft = (topl + botl) ; //average of left
						int avgright = (topr + botr) ; //average of right
	
						if (avgleft < avgright)
						{
						  if(servoazimuth_val < servoazimuthLimit_h)
								{
                  servoazimuth_val = servoazimuth_val + 5; //	move to the right, east, direction
                  servoazimuth(servoazimuth_val);	// The value the servo shall move with
								}
							else servoazimuth_val = servoazimuthLimit_h;
						}
						else if (avgright < avgleft)
						{
              if(servoazimuth_val > servoazimuthLimit_l)
								{
                  servoazimuth_val = servoazimuth_val - 5; //	move to the left, west, direction
                  servoazimuth(servoazimuth_val);	// The value the servo shall move with
								}
              else servoazimuth_val = servoazimuthLimit_l;
						}
						
						for(int i=0;i<10000;i++){} //Delay
							
					xSemaphoreGive(xBinarySemaphore); //Give the semaphore to the Actuating_N_S task
							
			}// end of the while loop
}// end of E-W actuation task

/***********************************************************
************************************************************
**						actuating the tilt angle servo 							**
**								to north south direction								**
************************************************************
************************************************************/
void Actuating_N_S (void *pvParameters){
				
	while(1){

				xSemaphoreTake(xBinarySemaphore ,portMAX_DELAY); // Take the semaphore from the Actuating_E_W task
		
	      int avgtop = (topl + topr) ; //average of top, north,
        int avgbot = (botl + botr) ; //average of bottom, south,
	
	          if (avgtop > avgbot)
          {
              if(servotilt_val < servotiltLimit_h){
                  servotilt_val = servotilt_val + 5; //	move to the bottom, south, direction
                  servotilt(servotilt_val);	// The value the servo shall move with
              }
              else servotilt_val = servotiltLimit_h;
          }
          else if (avgbot > avgtop)
          {
              if(servotilt_val > servotiltLimit_l){
                  servotilt_val = servotilt_val - 5; // move to the top, north, direction
                  servotilt(servotilt_val);	// The value the servo shall move with
              }
              else servotilt_val = servotiltLimit_l;
          }
					
				for(int i=0;i<10000;i++){}	// Delay						
											
				xSemaphoreGive( yBinarySemaphore); // Give the semaphore to Reading_E_W task
						
				}// end of the while loop
}// end of N-S actuation task

/***********************************************************
************************************************************
**											Main function											**
************************************************************
************************************************************/
int main(void)
{
	//***PWM and ADC initializations calling***
	PWM_init();
	ADC_init();
	
	//***binary semaphore creating***
	vSemaphoreCreateBinary(wBinarySemaphore);	
	vSemaphoreCreateBinary(xBinarySemaphore);
	vSemaphoreCreateBinary(yBinarySemaphore);
	vSemaphoreCreateBinary(zBinarySemaphore);

	//checking for the creation of the semaphores and start the creation and scheduling of the tasks
	if(xBinarySemaphore != NULL & yBinarySemaphore != NULL &zBinarySemaphore != NULL & wBinarySemaphore != NULL)
			{	
			xTaskCreate(Reading_E_W,   (const portCHAR*)"East-West reading", configMINIMAL_STACK_SIZE, NULL ,3,NULL); 
			xTaskCreate(Reading_N_S,   (const portCHAR*)"North-South reading", configMINIMAL_STACK_SIZE, NULL ,2,NULL); 
			xTaskCreate(Actuating_E_W, (const portCHAR*)"East West actuating", configMINIMAL_STACK_SIZE, NULL ,4,NULL);
			xTaskCreate(Actuating_N_S, (const portCHAR*)"North-South actuating", configMINIMAL_STACK_SIZE, NULL ,3,NULL);	

			vTaskStartScheduler();
			}
			
	while(1)
	{
	}
}

/***********************************************************
************************************************************
**							servo tilt angle function									**
************************************************************
************************************************************/
void servotilt(uint32_t ui32Adjust){
    PWMPulseWidthSet( PWM0_BASE, PWM_OUT_3,  ui32Adjust ); // PB5
}

/***********************************************************
************************************************************
**						servo azimuth angle function								**
************************************************************
************************************************************/
void servoazimuth(uint32_t ui32Adjust){
    PWMPulseWidthSet( PWM0_BASE, PWM_OUT_0,  ui32Adjust ); //PB6
}