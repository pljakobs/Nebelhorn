#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS_SAMD21.h>
#include <croutine.h>
#include <deprecated_definitions.h>
#include <error_hooks.h>
#include <event_groups.h>
#include <list.h>
#include <message_buffer.h>
#include <mpu_prototypes.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <runTimeStats_hooks.h>
#include <semphr.h>
#include <stack_macros.h>
#include <stream_buffer.h>
#include <task.h>
#include <timers.h>

#include <FastLED.h>

//#include <LowPower.h>
//#include <SerialMenuCmd.h>

#define DEBUG 1

#define FEEDBACK 0
#define CHAN_A 1
#define CHAN_B 2

#define SEL_1 5
#define SEL_2 4
#define SEL_3 3
#define SEL_4 9

#define MILLISECOND 1 / portTICK_PERIOD_MS 
#define SECOND      1000 / portTICK_PERIOD_MS
#define SIG_SHORT   1*SECOND  // short signal is a second
#define SIG_LONG    4*SECOND  // long signal is four seconds
#define SIG_PAUSE   2*SECOND  // Pause
#define MINUTE      60*SECOND // a minute has 60 seconds

#define LED_PIN     10 
#define NUM_LEDS    5
#define BRIGHTNESS  63
#define LED_TYPE    WS2811
#define COLOR_ORDER RGB

#define FAIL_CHAN_A 1
#define FAIL_CHAN_B 2

CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100

TaskHandle_t signalTaskHandle[8];
TaskHandle_t checkStateTaskHandle;

//prototypes
uint8_t getInputState(void);
void setLEDs(uint8_t);
void selfTest(void);
void failChannelShort(void);
void initHardware(void);
void initLEDs(void);
void setLED(uint8_t index, CRGB color);
void showLED(void);
void hornOn(void);
void hornOff(void);
void hornPause(void);
void hornShort(void);
void hornLong(void);
void checkSignal(uint8_t state);
bool validState(uint8_t state);

void startSignal(uint8_t state);
void stopSignal(uint8_t state);

void printDebugTag(void);


static void checkState(void *pvParameters );
static void state1Signal(void *pvParameters );
static void state2Signal(void *pvParameters );
static void state4Signal(void *pvParameters );
static void state8Signal(void *pvParameters );

static SemaphoreHandle_t ledMutex=NULL;

static uint32_t startTime;

void setup() {
  startTime=millis();
  Serial.begin(115200);

  initHardware();
  initLEDs();
  delay(2000);
  Serial.println("starting self test");

  selfTest();

  xTaskCreate(checkState, "checkStateLoop", 256, NULL, tskIDLE_PRIORITY+1, &checkStateTaskHandle);
  Serial.println("finished self test, starting Scheduler");
  vTaskStartScheduler();
  Serial.println("started Scheduler");
  //signalState=getInputState();
 
}

void loop() {
  // put your main code here, to run repeatedly:
}

static void checkState(void *pvParameters ){
  static uint8_t oldState, signalState;
  Serial.println("checkstate task started");
  ledMutex = xSemaphoreCreateMutex();
  if(ledMutex == NULL){
    Serial.println("could not create semaphore");
  }
  Serial.printf("created mutex: %i\n",ledMutex);
  while(true){
    signalState=getInputState();
    if(validState(signalState)){
      if(signalState!=oldState){
        //Serial.println("got new state");
        Serial.printf("\noldState: %i, signalState: %i\n",oldState,signalState);
        //Serial.printf("oldState: %i, signalState: %i\n",oldState,signalState);
        stopSignal(oldState);
        startSignal(signalState);
        setLEDs(signalState);
        oldState=signalState;
        vTaskDelay(100*MILLISECOND);
      }
    }
  }
}

uint8_t getInputState(){
  return(!(digitalRead(SEL_4))<<3|!(digitalRead(SEL_3))<<2|!(digitalRead(SEL_2))<<1|!(digitalRead(SEL_1)));
}

void setLEDs(uint8_t state){
  if(validState(state)){
  for(uint8_t i=0;i<4;i++){
    state&(1<<i)?setLED(i+1,CRGB::Blue):setLED(i+1,CRGB::Black);
  }
  showLED();
  }
}

void selfTest(){
  uint8_t error=0;
  for(uint8_t i=1;i<=4;i++){
    leds[i]=CRGB::Yellow;
  }
  FastLED.show();

  delay(500);
  
  // check for short
  if(digitalRead(FEEDBACK)){
    failChannelShort();
  }
  
  digitalWrite(CHAN_A, true);
  delayMicroseconds(100);
  if(digitalRead(FEEDBACK)){
    leds[1]=CRGB::Green;
    leds[2]=CRGB::Green;
  }else{
    leds[1]=CRGB::Red;
    leds[2]=CRGB::Red;
    error=FAIL_CHAN_A;
  }
  FastLED.show();
  digitalWrite(CHAN_A, false);
  delay(1000);
  digitalWrite(CHAN_B, true);
  delayMicroseconds(100);
  if(digitalRead(FEEDBACK)){
    leds[3]=CRGB::Green;
    leds[4]=CRGB::Green;
  }else{
    leds[3]=CRGB::Red;
    leds[4]=CRGB::Red;
    error=FAIL_CHAN_B;
  }
  FastLED.show();
  digitalWrite(CHAN_B, false);
  if(error!=0){
    delay(2000);
  }
  delay(500);
  for(uint8_t i=1;i<=5;i++){
    leds[i]=CRGB::Black;
  }
  FastLED.show();
}

void failChannelShort(){
  while(digitalRead(FEEDBACK)){
    for(uint8_t i=1;i<=4;i++){
      leds[i]=CRGB::Red;
    }
    FastLED.show();
    delay(250);
    for(uint8_t i=1;i<=4;i++){
      leds[i]=CRGB::Black;
    }
    FastLED.show();
    delay(250);
  }
}

void initHardware(){
  pinMode(FEEDBACK, INPUT);
  pinMode(CHAN_A, OUTPUT);
  pinMode(CHAN_B, OUTPUT);

  digitalWrite(CHAN_A,LOW);
  digitalWrite(CHAN_B,LOW);
  digitalWrite(FEEDBACK,LOW);
  
  pinMode(SEL_1, INPUT);
  pinMode(SEL_2, INPUT);
  pinMode(SEL_3, INPUT);
  pinMode(SEL_4, INPUT);

  pinMode(LED_PIN, OUTPUT);
}

void initLEDs(){
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  
  for(uint8_t i=0;i<5;i++){
    leds[i]=CRGB::Black;
  }
  FastLED.show();
}

void hornOn(){
  digitalWrite(CHAN_A, HIGH);
  digitalWrite(CHAN_B, HIGH);
  setLED(0, CRGB::Yellow);
  //leds[0]=CRGB::Yellow;
  showLED();
  //FastLED.show();
}

void hornOff(){
  digitalWrite(CHAN_A, LOW);
  digitalWrite(CHAN_B, LOW);
  setLED(0, CRGB::Black);
  //leds[0]=CRGB::Black;
  showLED();
  //FastLED.show();
}

void hornShort(){
  printDebugTag();
  Serial.println("horn on");
  hornOn();
  vTaskDelay(SIG_SHORT);
  printDebugTag();
  Serial.println("horn off");
  hornOff();
}

void hornLong(){
  printDebugTag();
  Serial.println("horn on");
  hornOn();
  vTaskDelay(SIG_LONG);
  printDebugTag();
  Serial.println("horn off");
  hornOff();
}

void hornPause(){
  vTaskDelay(SIG_PAUSE);
}

void startSignal(uint8_t state){
  printDebugTag();
  Serial.printf("startSignal for state: %i \n",state);
  switch(state){
    case 1:
      xTaskCreate(state1Signal, "signalTask", 256, NULL, tskIDLE_PRIORITY+1, &signalTaskHandle[state]);
      break;
    case 2:
      xTaskCreate(state2Signal, "signalTask", 256, NULL, tskIDLE_PRIORITY+1, &signalTaskHandle[state]);
      break;
    case 4:
      xTaskCreate(state4Signal, "signalTask", 256, NULL, tskIDLE_PRIORITY+1, &signalTaskHandle[state]);
      break;
    case 8:
      xTaskCreate(state8Signal, "signalTask", 256, NULL, tskIDLE_PRIORITY+1, &signalTaskHandle[state]);
      break;
    default:
      break;
  }
}

void stopSignal(uint8_t state){
  printDebugTag();
  Serial.printf("Stopping Signal for state %i", state);
  switch(state){
    case 1:
      vTaskDelete(signalTaskHandle[state]);
      hornOff();
      break;
    case 2:
      vTaskDelete(signalTaskHandle[state]);
      hornOff();
      break;
    case 4:
      vTaskDelete(signalTaskHandle[state]);
      hornOff();
      break;
    case 8:
      vTaskDelete(signalTaskHandle[state]);
      hornOff();
      break;
    default:
      break;
  }
}

static void state1Signal(void *pvParameters ){
  /*
   * Fog, under engine
   * long long
   * repeat every two minutes
   */
  printDebugTag();
  Serial.println("fog under engine signal started");
  vTaskDelay(2*SECOND);

  while(true){
    hornLong();
    hornPause();
    hornLong();
    hornPause();
    Serial.println("two minute pause");
    vTaskDelay(2*MINUTE-SIG_LONG-SIG_SHORT);
  }
}
static void state2Signal(void *pvParameters ){
  printDebugTag();
  Serial.println("fog, sailing signal started");
  vTaskDelay(2*SECOND);

  while(true){
    hornLong();
    hornPause();
    hornShort();    
    hornPause();
    
    vTaskDelay(2*MINUTE-SIG_LONG-SIG_SHORT);
  }
}
static void state4Signal(void *pvParameters ){
  printDebugTag();
  Serial.println("fog drifting signal started");
  vTaskDelay(2*SECOND);

  while(true){
    hornLong();
    hornPause();
    hornLong();
    hornPause();
    Serial.println("two minute pause");
    vTaskDelay(2*MINUTE-2*SIG_LONG-2*SIG_SHORT);
  }
}
static void state8Signal(void *pvParameters ){
  printDebugTag();
  Serial.println("fog at anchor signal started");
  vTaskDelay(2*SECOND);
  while(true){
    hornLong();
    hornPause();

    hornLong();
    hornPause();

    hornShort();
    hornPause();
    
    Serial.println("two minute pause");
    vTaskDelay(2*MINUTE-SIG_LONG-5*SIG_SHORT);
  }
}

void setLED(uint8_t index, CRGB color){
  if(ledMutex!=NULL){
    if(xSemaphoreTake(ledMutex, (TickType_t) 100)){
      leds[index]=color;
      xSemaphoreGive(ledMutex);
    }else{
      Serial.println("failed to get mutex");
    }
  }else{
    Serial.println("mutex not available");
  }
}

void showLED(){
  if(xSemaphoreTake(ledMutex, (TickType_t) 100)){
    printDebugTag();
    for(uint8_t i=0;i<=5;i++){
      (leds[i].getLuma()>0)?Serial.printf("LED[%i] ON, ",i):Serial.printf("LED[%i] OFF, ",i);
    }
    Serial.println();
    FastLED.show();
    vTaskDelay(10);
    xSemaphoreGive(ledMutex);
  }else{
    Serial.println("failed to get mutex");
  }
}

void printDebugTag(){
  #ifdef DEBUG
    static uint32_t relativeTime;
    Serial.printf("[%s, %ims (+%ims)]:",pcTaskGetName(xTaskGetCurrentTaskHandle()),millis()-startTime, millis()-relativeTime);
    relativeTime=millis();
  #endif
}

bool validState(uint8_t state){
  switch(state){
    case 1:
    case 2:
    case 4:
    case 8:
      return true;
      break;
    default:
      return false;
      break;
  }

}