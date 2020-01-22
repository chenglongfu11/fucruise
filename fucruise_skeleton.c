#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include "os_cfg.h"

#define DEBUG 1


#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];

OS_STK Watchdog_Stack[TASK_STACKSIZE];
OS_STK Overdetection_Stack[TASK_STACKSIZE];
OS_STK Switch_Stack[TASK_STACKSIZE];
OS_STK Button_Stack[TASK_STACKSIZE];
OS_STK Extraload_Stack[TASK_STACKSIZE];

// Task Priorities
 
#define STARTTASK_PRIO     5
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12

#define WATCHDOG_PRIO    7
#define OVER_DETECTION_PRIO  6

#define SWITCH_PRIO 8
#define BUTTON_PRIO 9
#define EXTRALOAD_PRIO 11

// Task Periods
 
#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define SWITCH_PERIOD  300
#define BUTTON_PERIOD  300
#define WATCHDOG_PERIOD  300
#define OVERDETECTION_PERIOD  300
#define EXTRALOAD_PERIOD  300



/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;

OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Gas_pedal;
OS_EVENT *Mbox_Cruise;
OS_EVENT *Mbox_Cruiseactive;
OS_EVENT *Mbox_Engine;
OS_EVENT *Mbox_Topgear;

OS_EVENT *Mbox_Ledgreen;
OS_EVENT *Mbox_Ledred;
OS_EVENT *Mbox_Load;
OS_EVENT *Mbox_Overdetection;

// Semaphores

OS_EVENT *Semcontrol;
OS_EVENT *Semvehicle;
OS_EVENT *Semswitch;
OS_EVENT *Sembutton;
OS_EVENT *Semwatchdog;
OS_EVENT *Semextraload;
OS_EVENT *Semoverdetection;

// SW-Timer

OS_TMR *Tmrvehicle;
OS_TMR *Tmrcontrol;
OS_TMR *Tmrswitch;
OS_TMR *Tmrbutton;
OS_TMR *Tmrwatchdog;
OS_TMR *Tmroverdetection;
OS_TMR *Tmrextraload;

/*
 * Types
 */
enum active {on = 2, off = 1};

enum active gas_pedal = off;
enum active brake_pedal = off;
enum active top_gear = off;
enum active engine = off;
enum active cruise_control = off;
enum active cruise_active =off;

/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs

void controlCallback(void *ptmr, void *callback_arg){
  OSSemPost(Semcontrol);

}

void vehicleCallback(void *ptmr, void *callback_arg){
   OSSemPost(Semvehicle);
}

void switchCallback(void *ptmr, void *callback_arg){
  OSSemPost(Semswitch);
}

void buttonCallback(void *ptmr, void *callback_arg){
  OSSemPost(Sembutton);
}

void watchdogCallback(void *ptmr, void *callback_arg){
  OSSemPost(Semwatchdog);
}

void extraloadCallback(void *ptmr, void *callback_arg){
  OSSemPost(Semextraload);
}

void overdetectionCallback(void *ptmr, void *callback_arg){
  OSSemPost(Semoverdetection);
}





int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);
  /* 0111=0x0007, */
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);
  /*0000=0x0000 0001=0x0001 0010=0x0002 0011=0x0004 */
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */
  
  return delay;
}

static int b2sLUT[] = {0x40, //0
		       0x79, //1
		       0x24, //2
		       0x30, //3
		       0x19, //4
		       0x12, //5
		       0x02, //6
		       0x78, //7
		       0x00, //8
		       0x18, //9
		       0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  
  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
   int tmp = target_vel;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  // if(target_vel < 0){
  // out_sign = int2seven(10);
  // tmp *= -1;
  // }else{
  // out_sign = int2seven(0);
  // }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  
  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
  int pp=position;
  int output;

  if(pp<400){
       output=0x2000;
  }
  else if (pp <800){
    output = 0x1000;
  }
   else if (pp <1200){
    output = 0x0800;
  }
    else if (pp <1600){
    output = 0x0400;
  }
    else if (pp <2000){
    output = 0x0200;
  }
    else{
      output = 0x0100;
    }
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,output);
  
}

/*
 * The function 'adjust_position()' adjusts the position depending on the
 * acceleration and velocity.
 */
INT16U adjust_position(INT16U position, INT16S velocity,
		       INT8S acceleration, INT16U time_interval)
{
  INT16S new_position = position + velocity * time_interval / 1000;

  if (new_position > 2400) {
    new_position -= 2400;
  } else if (new_position < 0){
    new_position += 2400;
  }
  
  show_position(new_position);
  return new_position;
}
 
/*
 * The function 'adjust_velocity()' adjusts the velocity depending on the
 * acceleration.
 */
INT16S adjust_velocity(INT16S velocity, INT8S acceleration,  
		       enum active brake_pedal, INT16U time_interval)
{
  INT16S new_velocity;
  INT8U brake_retardation = 200;

  if (brake_pedal == off)
    new_velocity = velocity  + (float) (acceleration * time_interval) / 1000.0;
  else {
    if (brake_retardation * time_interval / 1000 > velocity)
      new_velocity = 0;
    else
      new_velocity = velocity - brake_retardation * time_interval / 1000;
  }
  
  return new_velocity;
}


/* task= button IO */
void buttonIO(void* pdata){
  INT8U err;
  int x;
  void* msg;

  INT16S* velocity=0;
  INT32S ledg=0;
    
  //enum active gas_pedal = off;
  //enum active brake_pedal = off;
  //enum active top_gear = off;
  //enum active engine = off;
  //enum active cruise_control = off;
  // enum active cruise_active = off;
    

while(1){
     ledg=0;
      OSSemPend(Sembutton,0,&err);
    msg  = OSMboxPend(Mbox_Velocity, 0, &err);
    velocity = (INT16S *)msg;
    //   msg  = OSMboxPend(Mbox_Ledgreen, 0, &err);
    // ledg= (INT8U *) msg;
      
       x=buttons_pressed();
    
    /*gas pedal */ 
  if(x & GAS_PEDAL_FLAG){
     ledg= ledg | LED_GREEN_6;
      
    gas_pedal = on;
    cruise_active = off;
  }
  else{
   
    gas_pedal =off;

  }

  /*brake control*/
  
  if(x & BRAKE_PEDAL_FLAG){
    ledg= ledg | LED_GREEN_4;
    brake_pedal=on;

     cruise_active = off; 
  }
  else{
        brake_pedal=off;
  }

  
  /*cruise control */

    if((cruise_active==on) && (x | CRUISE_CONTROL_FLAG) &&(*velocity>20) ){
     ledg= ledg | LED_GREEN_2;
      brake_pedal = off;
      gas_pedal = off;
      cruise_control = on;
  }
  else{
    cruise_control = off;
  }

    /* cruise active */
    if(cruise_active==on){
      ledg = ledg | LED_GREEN_0;
    }
    //   err = OSMboxPost(Mbox_Brake, (void *) &brake_pedal);
    // err = OSMboxPost(Mbox_Cruise, (void *) &cruise_control);
    // err = OSMboxPost(Mbox_Gas_pedal, (void *) &cruise_active);
    // err = OSMboxPost(Mbox_Cruiseactive, (void *) &gas_pedal);
    // err = OSMboxPost(Mbox_Ledgreen, (void *) &ledg);
   
  
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE,ledg);

    }
}


/*task=swithc IO*/

void switchIO(void* pdata){
  INT8U err;
  INT16S* velocity = 0;
  INT16S x;
  INT32S  ledr=0;
  void* msg;


   
  //enum active gas_pedal = off;
  //enum active brake_pedal = off;
  //enum active top_gear = off;
  //enum active engine = off;
  //enum active cruise_control = off;

while(1){
    ledr=0;
    OSSemPend(Semswitch,0,&err);
    msg  = OSMboxPend(Mbox_Velocity, 0, &err);
    velocity = (INT16S*) msg;
    //    msg = OSMboxPend(Mbox_Ledgreen, 0, &err);
    // ledg = (INT8U *) msg;
    //   msg  = OSMboxPend(Mbox_Ledred, 0, &err);
    // ledr= (INT8U *)msg;

    x=switches_pressed();
   /*Engine */
  
  if(x & ENGINE_FLAG || velocity!=0){
    ledr=  ledr | LED_RED_0;
    engine=on;
    printf("engine is on");
  }
  else{
      
      engine=off;
      printf("engine is off");
    }

    /*TOP gear */

  if(x & TOP_GEAR_FLAG){
    ledr=  ledr | LED_RED_1;
    top_gear=on;
  }
  else{
    top_gear=off;
    cruise_active=off;
  }
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,ledr);
    // IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE,ledg);
  
    //   err = OSMboxPost(Mbox_Topgear, (void *) &top_gear);
    // err = OSMboxPost(Mbox_Engine, (void *) &engine);
    // err = OSMboxPost(Mbox_Ledgreen, (void *) &ledg);
    // err = OSMboxPost(Mbox_Ledred, (void *) &ledr);

 }
}

/*
 * The task 'VehicleTask' updates the current velocity of the vehicle
 */
void VehicleTask(void* pdata)
{ 
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT8S acceleration;  /* Value between 40 and -20 (4.0 m/s^2 and -2.0 m/s^2) */
  INT8S retardation;   /* Value between 20 and -10 (2.0 m/s^2 and -1.0 m/s^2) */
  INT16U position = 0; /* Value between 0 and 20000 (0.0 m and 2000.0 m)  */
  INT16S velocity = 0; /* Value between -200 and 700 (-20.0 m/s amd 70.0 m/s) */
  INT16S wind_factor;   /* Value between -10 and 20 (2.0 m/s^2 and -1.0 m/s^2) */
  enum active brake_pedal = off;

  printf("Vehicle task created!\n");

  while(1)
    {
      OSSemPend(Semvehicle,0,&err);
      err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

      //  OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD);

      //  OSSemPost(bSemaphore,0,&err);

      /* Non-blocking read of mailbox: 
	 - message in mailbox: update throttle
	 - no message:         use old throttle
      */
      msg = OSMboxPend(Mbox_Throttle, 1, &err); 
      if (err == OS_NO_ERR) 
	throttle = (INT8U*) msg;
      msg = OSMboxPend(Mbox_Brake, 1, &err); 
      if (err == OS_NO_ERR) 
	brake_pedal = (enum active) msg;

      // vehichle cannot effort more than 80 units of throttle
      if (*throttle > 80) *throttle = 80;

      // brakes + wind
      if (brake_pedal == off)
	acceleration = (*throttle) - 1*velocity;
      else
	acceleration = -4*velocity;

	 
      if (400 <= position && position < 800)
	acceleration -= 20; // traveling uphill
      else if (800 <= position && position < 1200)
	acceleration -= 40; // traveling steep uphill
      else if (1600 <= position && position < 2000)
	acceleration += 40; //traveling downhill
      else if (2000 <= position)
	acceleration += 20; // traveling steep downhill

      printf("Position: %d m\n", position);
      printf("Velocity: %d m/s\n", velocity);
      printf("Accell: %d m/s2\n", acceleration);
      printf("Throttle: %d V\n", *throttle);

      position = position + velocity * VEHICLE_PERIOD / 1000;
      velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
       if(position > 2400)
	position = 0;

      show_velocity_on_sevenseg((INT8S) velocity);

    }
} 
 
/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 0; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* current_velocity;


     
  //enum active gas_pedal = off;
  //enum active brake_pedal = off;
  //enum active top_gear = off;
  //enum active engine = off;
  //enum active cruise_control = off;
  


  printf("Control Task created!\n");

  while(1)
    {
      OSSemPend(Semcontrol,0,&err);
      msg = OSMboxPend(Mbox_Velocity, 0, &err);      
      current_velocity = (INT16S*) msg;  /*get volocity */



      //  msg = OSMboxPend(Mbox_Engine, 1, &err); /*get engine state*/
      // if (err == OS_NO_ERR) 
      // engine = (enum active) msg;

      // msg = OSMboxPend(Mbox_Topgear, 1, &err); /*get topgear state*/
      // if (err == OS_NO_ERR) 
      // top_gear = (enum active) msg;

      // msg = OSMboxPend(Mbox_Cruise, 1, &err); /*get curise state*/
      // if (err == OS_NO_ERR) 
      // cruise_control = (enum active) msg;

      // msg = OSMboxPend(Mbox_Gas_pedal, 1, &err); /*get gas_pedal state*/
      // if (err == OS_NO_ERR) 
      // gas_pedal = (enum active) msg;

    

      if(engine == on){
 	  if(cruise_control ==on){
	    if(*current_velocity<21 || *current_velocity>29 ){
	                 throttle=40;
	      }
	  }
	  else if(gas_pedal==on){
	         throttle=40;
	  }
      }

      err=OSMboxPost(Mbox_Throttle,(void *)&throttle); 



      

      // OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);

    }
}

void extraload(void* pdata){
  INT8U err;
  int  utilization=0;
  INT32U uti_ms;
  int switches;


  while(1){

          OSSemPend(Semextraload,0,&err);
	  switches=switches_pressed();

	  utilization =( switches>>4) & 0x3f;
	  uti_ms=(INT32U)utilization*2*0.01*100;


	  if(uti_ms>100)uti_ms=100;
	  err=OSMboxPost(Mbox_Load,(void* )&uti_ms);

  }
	  
	  
       
}

void overdetection(void* pdata){
  INT8U err;
  INT32U* value=0;
  INT32U  report=0;
  void* msg;

    while(1){

      OSSemPend(Semoverdetection,0,&err);
      msg = OSMboxPend(Mbox_Load, 0, &err);
      value = (INT32U *)msg;
      if(*value>=100)report=1;
      else report=0;
      err=OSMboxPost(Mbox_Overdetection, (void*)&report);
    }

    
}

void watchdog(void* pdata){
  INT8U err;
  void* msg;
  INT32U* notok=0;
  int watch=0;

  while(1){
          OSSemPend(Semwatchdog,0,&err);
	  msg = OSMboxPend(Mbox_Overdetection,50, &err);
	 
	  if(err==OS_TIMEOUT){
	   watch=1;
	    printf("timeout, over load \n");
	  }
	  else if(err== OS_NO_ERR){
	              notok = (INT32U *)msg;
		      if(*notok==1) watch=1;
		      else watch=0;
	  }

	  if(watch==1)printf("overload\n");


  }
	  

}







/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */
  
  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
		       delay,
		       alarm_handler,
		       context) < 0)
    {
      printf("No system clock available!n");
    }

  /* 
   * Create and start Software Timer 
   */

  /*control timer */
   Tmrcontrol= OSTmrCreate(0, //delay
                            CONTROL_PERIOD/HW_TIMER_PERIOD, //period
                            OS_TMR_OPT_PERIODIC,
                           (OS_TMR_CALLBACK) controlCallback, //OS_TMR_CALLBACK
			   (void *)0,
                            "Tmrcontrol1",
                            &err);
                            
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if creation successful
      printf("Tmrcontrol created\n");
    }
   }
   
   //vehicle timer */
   Tmrvehicle = OSTmrCreate(0, //delay
                            VEHICLE_PERIOD/HW_TIMER_PERIOD, //period
                            OS_TMR_OPT_PERIODIC,
                            (OS_TMR_CALLBACK)  vehicleCallback, //OS_TMR_CALLBACK
                            (void *)0,
                            "Tmrvehicle1",
                            &err);

   /*switchIO timer */
   Tmrswitch = OSTmrCreate(0, //delay
                            SWITCH_PERIOD/HW_TIMER_PERIOD, //period
                            OS_TMR_OPT_PERIODIC,
                             (OS_TMR_CALLBACK) switchCallback, //OS_TMR_CALLBACK
			   (void *)0,
                            "Tmrswitch1",
                            &err);

   /*buttonIO timer */
   Tmrbutton = OSTmrCreate(0, //delay
                            BUTTON_PERIOD/HW_TIMER_PERIOD, //period
                            OS_TMR_OPT_PERIODIC,
                             (OS_TMR_CALLBACK) buttonCallback, //OS_TMR_CALLBACK
			   (void *)0,
                            "Tmrbutton1",
                            &err);

   /*watchdog timer*/
   Tmrwatchdog = OSTmrCreate(0, //delay
                            WATCHDOG_PERIOD/HW_TIMER_PERIOD, //period
                            OS_TMR_OPT_PERIODIC,
                             (OS_TMR_CALLBACK) watchdogCallback, //OS_TMR_CALLBACK
			     (void *)0,
                            "Tmrwatchdog1",
                            &err);

   /*overdetection timer*/
   Tmroverdetection = OSTmrCreate(0, //delay
                            OVERDETECTION_PERIOD/HW_TIMER_PERIOD, //period
                            OS_TMR_OPT_PERIODIC,
                             (OS_TMR_CALLBACK) overdetectionCallback, //OS_TMR_CALLBACK
				  (void *)0,
                            "Tmroverdetection1",
                            &err);   
   /* extra load timer*/
   Tmrextraload = OSTmrCreate(0, //delay
                            EXTRALOAD_PERIOD/HW_TIMER_PERIOD, //period
                            OS_TMR_OPT_PERIODIC,
                             (OS_TMR_CALLBACK) extraloadCallback, //OS_TMR_CALLBACK
			      (void *)0,
			      "Tmrextrload1",
                            &err);
   
                            
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if creation successful
      printf("Timer2 created\n");
    }
   }

   /*
    * Start timers
    */
   
   //start  Timer
   OSTmrStart(Tmrvehicle, &err);
    OSTmrStart(Tmrcontrol, &err);
   OSTmrStart(Tmrswitch, &err);
   OSTmrStart(Tmrbutton, &err);
   OSTmrStart(Tmrwatchdog, &err);
   OSTmrStart(Tmroverdetection, &err);
    OSTmrStart(Tmrextraload, &err);
   
   
		  //   if (DEBUG) {
		  // if (err == OS_ERR_NONE) { //if start successful
		  // printf("timer1 started\n");
		  // }
		  // }

   //start Task2 Timer
		  //  OSTmrStart(Timer2, &err);
   
		  //  if (DEBUG) {
		  // if (err == OS_ERR_NONE) { //if start successful
		  // printf("timer2 started\n");
		  // }
		  // }


   

	       

  /*
   * Creation of Kernel Objects
   */
   Semcontrol = OSSemCreate(0);
   Semvehicle = OSSemCreate(0);
   Semswitch = OSSemCreate(0);
   Sembutton = OSSemCreate(0);
   Semwatchdog = OSSemCreate(0);
   Semextraload = OSSemCreate(0);
   Semoverdetection = OSSemCreate(0);
   
  
  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Engine = OSMboxCreate((void*) 0);
  Mbox_Topgear = OSMboxCreate((void*) 0);
  Mbox_Ledgreen = OSMboxCreate((void*) 0);
  Mbox_Ledred = OSMboxCreate((void*) 0);
  Mbox_Load = OSMboxCreate((void*) 0);
  Mbox_Overdetection = OSMboxCreate((void*) 0);
   
  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
			ControlTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			CONTROLTASK_PRIO,
			CONTROLTASK_PRIO,
			(void *)&ControlTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
			VehicleTask, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			VEHICLETASK_PRIO,
			VEHICLETASK_PRIO,
			(void *)&VehicleTask_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);
  
   err = OSTaskCreateExt(
			buttonIO, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Button_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			BUTTON_PRIO,
			BUTTON_PRIO,
			(void *)&Button_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

   err = OSTaskCreateExt(
			switchIO, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Switch_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			SWITCH_PRIO,
			SWITCH_PRIO,
			(void *)&Switch_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);

   err = OSTaskCreateExt(
			watchdog, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Watchdog_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			WATCHDOG_PRIO,
			WATCHDOG_PRIO,
			(void *)&Watchdog_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);
   err = OSTaskCreateExt(
			extraload, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Extraload_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			EXTRALOAD_PRIO,
			EXTRALOAD_PRIO,
			(void *)&Extraload_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);
  err = OSTaskCreateExt(
			overdetection, // Pointer to task code
			NULL,        // Pointer to argument that is
			// passed to task
			&Overdetection_Stack[TASK_STACKSIZE-1], // Pointer to top
			// of task stack
			OVER_DETECTION_PRIO,
		        OVER_DETECTION_PRIO,
			(void *)&Overdetection_Stack[0],
			TASK_STACKSIZE,
			(void *) 0,
			OS_TASK_OPT_STK_CHK);
  
  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");
 
  OSTaskCreateExt(
		  StartTask, // Pointer to task code
		  NULL,      // Pointer to argument that is
		  // passed to task
		  (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
		  // of task stack 
		  STARTTASK_PRIO,
		  STARTTASK_PRIO,
		  (void *)&StartTask_Stack[0],
		  TASK_STACKSIZE,
		  (void *) 0,  
		  OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
	 
  OSStart();
  
  return 0;
}
