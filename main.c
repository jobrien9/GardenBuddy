#include "msp.h"
#include "main.h"
#include "uart.h"
#include "driverlib.h"
#include "stdio.h"
#include "constants.h"
#include "stdbool.h"
#include "incomingGPS.h"
#include "math.h"
#include "robotDriver.h"

//6 MHz clock rate - if possible, remove floating point calculations to be able to g
//back to a slower clock speed
const long CLOCK_FREQUENCY = 6000000;

 //uart configuration based on TI's tool and the Lab4 instructions
const eUSCI_UART_Config clockConfig = {
      EUSCI_A_UART_CLOCKSOURCE_SMCLK, //clock source
      /*these were determined through TI's tool - EUSCI, 6MHZ, 9600bps*/
     /* 19, //clock prescalar
      8, //first mod reg
      85, //second mod reg
      */
      39,
      1,
      0,
      EUSCI_A_UART_NO_PARITY, //no parity
      EUSCI_A_UART_LSB_FIRST, //least significant bit first,
      EUSCI_A_UART_ONE_STOP_BIT, //one stop bit
      EUSCI_A_UART_MODE, //uart mode a
      EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION //oversampling
 };

//config from Lab 5
const Timer_A_UpModeConfig timerAConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_64,
        46875, //2Hz
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
};


const Timer_A_UpModeConfig PWM_TIMER_CONFIG =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_10,
        6,//MOTOR_TIMER_PERIOD,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
};

const Timer_A_UpModeConfig encoderTimerConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_64,
        938,//ENCODER_TIMER_PERIOD,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
};

const Timer_A_UpModeConfig compassTimerConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_64,
        187500,//0.5 Hertz
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
};

//these are referenced externally
volatile struct Coordinates robotLocation;
volatile struct Coordinates phoneCoordinates;

volatile struct Motor rightMotor;
volatile struct Motor leftMotor;

volatile struct Phone phoneStruct;
bool robotHasBeenSet = false;
bool phoneHasBeenSet = false;
bool isRotating = false;

//current direction the robot is facing
volatile float robotDegrees;
//use this to know if the robot has been turned in travel
float originalRobotDegrees;

//track which state we are on
int currentState = 1;

/**
 * main.c
 */
void main(void)
{
    WDT_A_holdTimer();// stop watchdog timer
    //disable all interrupts for safety
    Interrupt_disableMaster();

    //configure clock to 3MHZ because we used that in the labs
    setClock(CLOCK_FREQUENCY);

    GPSCommSetup();
    arduinoCommSetup();
    //register timer interrupt to get these values
    timerSetup();

    //setup GPIO Pins
    driverBoardInit();

    adcInit();

    Interrupt_enableMaster();

    //on startup, set the robot's current location to 0
    robotLocation.longitude = 0;
    robotLocation.latitude = 0;

    //ditto for phone
    phoneCoordinates.longitude = 0;
    phoneCoordinates.latitude = 0;

    //setup the motors
    rightMotor.ID = RIGHT_MOTOR_ID;
    rightMotor.direction = STOP;
    rightMotor.metersTraveled = 0.0;
    rightMotor.isCurrentlyMoving = false;
    rightMotor.directionChange = true;

    leftMotor.ID = LEFT_MOTOR_ID;
    leftMotor.direction = STOP;
    leftMotor.metersTraveled = 0.0;
    leftMotor.isCurrentlyMoving = false;
    leftMotor.directionChange = true;

    //state machine to control robot
    while (1){
        robotHasBeenSet = (robotLocation.latitude != 0 && robotLocation.longitude !=0) ? true : false;
        phoneHasBeenSet = (phoneCoordinates.latitude != 0 && phoneCoordinates.longitude != 0) ? true :false;
        //if the robot isn't current in travel mode, look for the GPS Coordinates and the phone's coordinates and start moving
        if (currentState == 1 && !robotHasBeenSet){
            //start timer A0 in up mode
            Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
        }else if(currentState == 1 && robotHasBeenSet){ //separated compass out to keep it from competing with the GPS module
            //disable GPS module
            Interrupt_disableInterrupt(INT_EUSCIA2);
            //testing values
            /*phoneCoordinates.latitude = 33.774233;
            phoneCoordinates.longitude = -84.398907;
            robotDegrees = 5; //set to North*/
            //this interrupt is for the compass - don't use a timer so that the interrupt is tied to whenever the arduino sends data
            //Interrupt_enableInterrupt(INT_EUSCIA3);
            currentState = 2;
        //find distance and degrees the robot is from the target
        }else if (currentState == 2 && robotHasBeenSet && phoneHasBeenSet){
            phoneStruct.location = phoneCoordinates;
            phoneStruct.distanceFromRobot = findDistance();
            //figure out how many degrees to rotate
            phoneStruct.degreesFromRobot = directionCalculator();
            currentState = 3;
        //state 3: turning the robot
        }else if (currentState == 3){
            //clockwise vs counterclockwise
            if (phoneStruct.degreesFromRobot > 0){
                rightMotor.direction = BACKWARD;
                leftMotor.direction = FORWARD;
            }else{
                rightMotor.direction = FORWARD;
                leftMotor.direction = BACKWARD;
            }
            //to prevent slipping, go slow
            rightMotor.dutyCycle = DUTY_CYCLE_SLOW;
            rightMotor.setPointSpeed = 10;//degrees per second
            leftMotor.dutyCycle = DUTY_CYCLE_SLOW;

            //set the current state to 4 to skip the turning for now
            currentState = 4; //pause the state for a bit until we are ready to finisn turning
            //motor driver timer
            //Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            //start compass timer
            //Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);
        // if the degrees traveled are within +/- 5 degrees of target and we are rotating, stop moving
        }else if (currentState == 4){
            disableMotors();
            currentState = 5;
        }else if (currentState == 5){
            //drive car with PWM
            rightMotor.direction = FORWARD;
            rightMotor.dutyCycle = DUTY_CYCLE_NORMAL;
            rightMotor.directionChange = true;
            //use the right motor for the set point
            rightMotor.setPointSpeed = SET_POINT_SPEED; //m/s
            leftMotor.direction = FORWARD;
            leftMotor.dutyCycle = DUTY_CYCLE_NORMAL;
            leftMotor.setPointSpeed = SET_POINT_SPEED;
            leftMotor.directionChange = true;
            //motor driver timer
            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            //start encoder timer
            Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
            currentState = 6;
        //figure out when the robot has traveled the distance and stop it
        }else if(currentState == 6 && rightMotor.metersTraveled >= phoneStruct.distanceFromRobot){
            disableMotors();
            currentState = 1;
        //if the robot gets moved while in transit, then reset
        //NOTE: this is commented out for now, but if isSufficientDegrees can be made less resource consuming, it can be turned back on
            //it's job is to detect if the robot gets knocked off track and is out of alignment
        //}else if (!isSufficientDegrees() && currentState == 6){
          //  currentState = 1;
            robotLocation.latitude = 0;
            robotLocation.longitude = 0;
            phoneCoordinates.latitude = 0;
            phoneCoordinates.longitude = 0;
        }
    }

}

//function to figure out if the robot's current degrees are within the acceptable distance
bool isSufficientDegrees(){
    double degreeDiff = robotDegrees - phoneStruct.degreesFromRobot;
    return abs(degreeDiff) <= DEGREES_THRESHOLD;
}

void disableMotors(){
    Timer_A_stopTimer(TIMER_A1_BASE);
    Timer_A_stopTimer(TIMER_A2_BASE);
    Timer_A_stopTimer(TIMER_A3_BASE);

    //stop and reset everything
    rightMotorLow();
    leftMotorLow();
    rightMotor.direction = STOP;
    leftMotor.direction = STOP;

    //reset motors
    rightMotor.metersTraveled = 0;
    leftMotor.metersTraveled = 0;
}

//takes the robot and phone coordinates as arguments and returns the distance in meters
float findDistance(){
    float latitudeDifference = (float)fabs(phoneCoordinates.latitude - robotLocation.latitude);
    float longitudeDifference = (float)fabs(phoneCoordinates.longitude - robotLocation.longitude);

    //use the pythagorean theorem to find the distance
    float distanceInDegrees = (float)sqrt(pow(latitudeDifference, 2) + pow(longitudeDifference, 2));
    //to convert from degrees to seconds, multiply by 360 - then multiply that by Meters per second
    return distanceInDegrees * METERS_PER_DEGREE;

}


void GPSCommSetup(){
    //Setup Pin 3.2 for RX
   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

   //setup UART for GPS comm
   if(!UART_initModule(EUSCI_A2_BASE, &clockConfig)){
           printf("GPS Module init failed!");
   }

   UART_enableModule(EUSCI_A2_BASE);

   //interrupt setup
   GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION);
   UART_clearInterruptFlag(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
   UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
   Interrupt_setPriority(INT_EUSCIA2, UART_PRIORITY);
   Interrupt_enableInterrupt(INT_EUSCIA2);
}

//setup serial communication pin to receive serial from Arduino
void arduinoCommSetup(){
    //these pins are for the Serial comm
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P9,GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    //setup UART for GPS comm
    if(!UART_initModule(EUSCI_A3_BASE, &clockConfig)){
        printf("Module init failed!");
    }

    UART_enableModule(EUSCI_A3_BASE);

    GPIO_interruptEdgeSelect(GPIO_PORT_P9, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
    UART_clearInterruptFlag(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_setPriority(INT_EUSCIA3, COMPASS_PRIORITY);
}

//set the direction which the robot is facing for future reference
//and figure out what direction the robot needs to move to face the phone
double directionCalculator(){
    originalRobotDegrees = robotDegrees;
    double yDiff = phoneStruct.location.longitude-robotLocation.longitude;
    double xDiff = phoneStruct.location.latitude - robotLocation.latitude;
    //angle between robot and phone in degrees (radians * 180/PI)
    double phoneRobotAngle = atan2(yDiff, xDiff) * (180/M_PI);
    double degreesClockwise = originalRobotDegrees - phoneRobotAngle;
    //never travel more than 180
    if (degreesClockwise > 180){
        degreesClockwise = 360 - degreesClockwise;
    }else if(degreesClockwise < -180){
        //negative means move counterclockwise
        degreesClockwise += 360;
    }
    return degreesClockwise;
}

void setClock(double frequency){
    unsigned int dcoFrequency = frequency;
   CS_setDCOFrequency(dcoFrequency);
   CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

void timerSetup(){
    // Configuring timer A0
    Timer_A_configureUpMode(TIMER_A0_BASE, &timerAConfig);
    //set priority of timer a
    Interrupt_setPriority(INT_TA0_0, TIMER_A_PRIORITY);
    Interrupt_enableInterrupt(INT_TA0_0);

    //timer for PWM
    Timer_A_configureUpMode(TIMER_A1_BASE, &PWM_TIMER_CONFIG);
    Interrupt_setPriority(INT_TA1_0, PWM_PRIORITY);
    Interrupt_enableInterrupt(INT_TA1_0);

    //timer for encoder
    Timer_A_configureUpMode(TIMER_A2_BASE, &encoderTimerConfig);
    Interrupt_setPriority(INT_TA2_0, ENCODER_PRIORITY);
    Interrupt_enableInterrupt(INT_TA2_0);

    //timer for compass
    Timer_A_configureUpMode(TIMER_A3_BASE, &compassTimerConfig);
    Interrupt_setPriority(INT_TA3_0, COMPASS_PID_PRIORITY);
    Interrupt_enableInterrupt(INT_TA3_0);
}

void driverBoardInit(){
    //AIN1
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
    //AIN2
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN6);
    //PWMA
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN7);
    //BIN1
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4);
    //BIN2
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);
    //PWMB
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);

    rightMotorLow();
    leftMotorLow();
}

//adc for hall effect sensors
void adcInit(){
    // Set reference voltage to 2.5V
    REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    REF_A_enableReferenceVoltage();

    //GPIO pin conversion - using 5.5 (right) and 5.2 (left)
   // GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN5,
        GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN2,
         GPIO_TERTIARY_MODULE_FUNCTION);

    //enable analog to digital converter module
    ADC14_enableModule();
    //set resolution to 10 bits
    ADC14_setResolution(ADC_10BIT);
    //init the ADC - SMCLK clock, default dividers, don't route to internal channel
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE);

    /*From documentation: Configures the ADC module to use a a single ADC memory location for sampling/conversion.*/
   //using the first memory location and setting repeatMode to false
   ADC14_configureSingleSampleMode(ADC_MEM0, false);
   ADC14_configureSingleSampleMode(ADC_MEM3, false);

   //setting up the ADC memory
   //first ADC memory location
   //trying to get 2.5 ref V
   //nondifferential
   ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                   ADC_INPUT_A0, ADC_NONDIFFERENTIAL_INPUTS);
   ADC14_configureConversionMemory(ADC_MEM3, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
                                    ADC_INPUT_A3, ADC_NONDIFFERENTIAL_INPUTS);


   /*The  user  will  have  to  manually  set  the  SHI  signal  (usually by
    * ADC14_toggleConversionTrigger ) at the end of each sample/conversioncycle.*/
   ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

   //enable conversion
   if (!ADC14_enableConversion()){
       printf("Something broke. Conversion enable failed.");
   }
}
