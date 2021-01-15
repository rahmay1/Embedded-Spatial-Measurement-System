// 2DX4 Project - Yazdan Rahman
// 400199680

#include <stdint.h>
#include <math.h>  
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "Systick.h"
#include "PLL.h"
#include "uart.h"
#include "onboardLEDs.h"

// Initializing Port E0 to Ground
void PortE0_Init(void){	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;		 
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};	 
	GPIO_PORTE_DEN_R= 0b00000001;
	GPIO_PORTE_DIR_R |= 0b00000001;                       
	GPIO_PORTE_DATA_R=0b00000000;                             
	return;
	}

// Initializing Port L0-L3 to output to stepper motor and L4 for external LED output
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};
	GPIO_PORTL_DIR_R |= 0b00011111;      
  GPIO_PORTL_DEN_R |= 0b00011111;
	return;
}

uint16_t	dev=0x52; // Default address for ToF sensor

int status=0; // Var which is the current ToF status bit
int arr[8] = {0,0,0,0,0,0,0,0}; // Array which consists of the ToF measurement values

// The value which indicates if the program is in interrupt mode or polling mode (1 = interrupt mode)
#define isInterrupt 1

// Initializing Function Headers
void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

// ToF sensor date vars which is used to check if the ToF is operating properly
uint8_t byteData, sensorState=0; //myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

// Function which sets up the ToF sensor
void SetupToF(void){
	
	// Checking ToF data
  status = VL53L1_RdByte(dev, 0x010F, &byteData);
  //myByteArray[i++] = byteData;

  status = VL53L1_RdByte(dev, 0x0110, &byteData);
  //myByteArray[i++] = byteData;
	
	status = VL53L1_RdWord(dev, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(dev, &wordData);

	// Starting up the ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	// Flashing all LEDS to indicate this startup
	FlashAllLEDs();
	
	// Clearing interrupt used for the next interrupt
	status = VL53L1X_ClearInterrupt(dev);

  // Initializing ToF sensor with default values
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	// Enabling the ranging of the ToF
  status = VL53L1X_StartRanging(dev);
	Status_Check("StartRanging", status);
	
}

// Function which gets a distance measurement from the ToF chip and returns it
int SendTOFDistance(void) {
	
	 // Waiting for ToF to be ready for sensing
	 while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }
	
      dataReady = 0; // Resetting dataReady variable
	    status = VL53L1X_GetRangeStatus(dev, &RangeStatus); // Getting the RangeStatus
	    status = VL53L1X_GetDistance(dev, &Distance); // Getting the distance var
      FlashLED4(1); // Flashing LED 4 on the onboard MSPY board to indicate successful reading

			// Reset the interrupt for the ToF sensor for next interrupt
	    status = VL53L1X_ClearInterrupt(dev); 
			return Distance; // Returning the distance
		
}

// Defined I2C bits for certain tasks
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // Var for max number of retrys before giving up

// Function that Initializes I2C communication to ToF sensor chip
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
	
  while((SYSCTL_PRGPIO_R&0x0002) == 0){}; // Waits for sys to be ready

    GPIO_PORTB_AFSEL_R |= 0x0C;           // Enable Alt Funct on PB2, and PB3
    GPIO_PORTB_ODR_R |= 0x08;             // Enable open drain on PB3

    GPIO_PORTB_DEN_R |= 0x0C;             // Digital Enable PB2 and PB3
		
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE; // Enable master function
    I2C0_MTPR_R = 0b0000000000000101000000000111011;
}

// Function that Enables Port G0 so that ToF can be reset using XSHUT
void PortG_Init(void){
	
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};
    GPIO_PORTG_DIR_R &= 0x00; // Set PG0 to input (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01; // Disable Alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01; // Enable Digital I/O on PG0
  GPIO_PORTG_AMSEL_R &= ~0x01; // Disable Analog Funct on PG0
    return;
}

// Func that applies XSHUT to provide an active-low shutdown
// Puts ToF into hardware standby (input not level shifted)
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01; // Set PG0 to output
    GPIO_PORTG_DATA_R &= 0b11111110; // Set PGO to a low reading
    FlashAllLEDs(); // Flash All LEDS
    SysTick_Wait10ms(10); // Wait
    GPIO_PORTG_DIR_R &= ~0x01; // Set PG0 to Input (HIZ)
    
}

// Moves stepper motor forwards and waits a certain speed specified by the input parameter
void Forward(int speed){
	SysTick_Wait10ms(speed);
	GPIO_PORTL_DATA_R = 0b00001100;
	SysTick_Wait10ms(speed);
	GPIO_PORTL_DATA_R = 0b00000110;
	SysTick_Wait10ms(speed);
	GPIO_PORTL_DATA_R = 0b00000011;
	SysTick_Wait10ms(speed);
	GPIO_PORTL_DATA_R = 0b00001001;
}

// Func that enables interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Func that disables interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Func that waits for interrupt
void WaitForInt(void)
{    __asm("    wfi\n");
}

// global variable visible in Watch window of debugger
// increments at least once per button press
// GPIO Port J = Vector 67
// Bit in interrupt register = 51

// Function that sets onboard button PJ1 as an interrupt
void ExternalButton_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;				// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    // (c) Set PJ1 as input

  GPIO_PORTJ_DEN_R |= 0x02;     // Digital Enable PJ1
	GPIO_PORTJ_PCTL_R &= ~0x000000F0; // Set PJ1 as GPIO
	GPIO_PORTJ_AMSEL_R &= ~0x02;	// Disable analog funct for PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;			// Enable weak pull up resistor
  GPIO_PORTJ_IS_R &= ~0x02;     // Set PJ1 as edge-sensitive 
  GPIO_PORTJ_IBE_R &= ~0x02;    // Set PJ1 as not both edges sensitive
  GPIO_PORTJ_IEV_R &= ~0x02;    // Set PJ1 as falling edge sensitive event 
  GPIO_PORTJ_ICR_R = 0x02;      // Clear flag1
  GPIO_PORTJ_IM_R |= 0x02;      // Set Arm Interrupt for PJ1
  NVIC_PRI13_R = (NVIC_PRI13_R&0xFF00FFFF)|0x000A0000; // Set priority to 5
  NVIC_EN1_R |= 0x00080000;              // Enable Interrupt 67 in NVIC
  EnableInt();           				// Enable Interrupts
}

// Var which stores the current state of the program
int state = 0;

// The Interrupt Handler when PJ1 button is clicked
void GPIOJ_IRQHandler(void){
  GPIO_PORTJ_ICR_R = 0x02; // Acknowledge flag4
	
	// Toggles the state into a new state
	if(state == 1){
		state = 0;
	}else{
		state = 1;
	}
		
}

// Defining PI for cos and sine functions
#define PI 3.14159265

// Main Function to handle ToF, stepper motor, and LED main logic and arithmetics
int main(void){
	
	// Calling all Initializer Funcs
  PLL_Init();
	SysTick_Init();
	PortE0_Init();
	PortL_Init();
	ExternalButton_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	SetupToF();
	
	// Setting the initial x displacement as 0mm
	int x = 0;
	
	// Loop which repeats forever
	while(1){
		
		// A checkpoint to exit to main loop if needed
		loop:
		
		// Checks if state is on
		if(state == 1){
			
			// Var for while loop
			int j = 0;
			// While loop which continues for 8 iterations (0 based)
			while(j<=7){
				
				// If state is still on
				if(state == 1){
				
				// Creating a int var to count up from 0 from 64
				int i = 1;
				GPIO_PORTL_DATA_R = 0b00000000; // Turn off the external LED
				
					// Entering a while loop which counts to 64
					while(i<=64){
						
						// Calls forward to rotate the stepper motor
						Forward(1);
					
						// Increment i
						i++;
					
						// If statement that blinks the internal LED for 100ms if every 64 turns (45 deg)
						if(i == 64){
							GPIO_PORTN_DATA_R = 0b000000001;
							SysTick_Wait10ms(100);
							GPIO_PORTN_DATA_R = 0b000000000;
						}
					}
					
					// Calling func to return current ToF measurement and store it in an array 
					arr[j] = SendTOFDistance();
			
				}else{
					// If the state was toggled off in the stepper motor loop (when stepper motor is running), go back to outer loop
					goto loop;
				}
				j++; // Increments j by 1
			}
			
			state = 0; // Toggling state to 0 after stepper motor ran a complete cycle
			
		}else{
			
			// If the array is not blank
			if(arr[0] != 0){
				
				// Var which is the deg turned from initial position
				int deg = 0;
				
				// For loop which iterates from 0 to 7
				for(int i = 0;i <= 7;i++){
					// If there is any information inside the array at current index
					if(arr[i] != 0){
					// Send the x y and z coord of the ToF distance measurement to using Serial Communication
					sprintf(printf_buffer,"%d %d %d\r\n", x, (int)round(arr[i]*cos(deg*PI/180)) ,(int)round(arr[i]*sin(deg*PI/180)));
					UART_printf(printf_buffer);
					}
					
					// Incrementing deg by 45
					deg +=45;
					arr[i] = 0; // Setting array to 0
				}
				
				x+=200; // Manually Incrementing x position by 20cm each time (in mm)
				sprintf(printf_buffer,"End\r\n"); 
				UART_printf(printf_buffer);
			}
			
			GPIO_PORTL_DATA_R = 0b00010000; // Turn on the external LED
		}
	}
}


