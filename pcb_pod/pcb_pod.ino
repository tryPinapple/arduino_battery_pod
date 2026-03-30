/**
  * @autor: Laurie B.
  * @date: 21-03-2021
  * 
  * @brief: PCB POD sensor/actuator management
  **/
#include <FastLED.h> 
#include <ArduinoJson.h>
#include <Servo.h>
#include <ArduinoJson.h>

/* Defines */
#define DROPPER1_PIN            9
#define DROPPER2_PIN            6
#define TORPEDO_PIN             8
#define DIP_SWITCH_CELL         11
#define DIP_SWITCH_FREE         12 // NOT USED
#define LED_PIN                 7
#define LEAK_PIN                10

#define ANALOG_CELL_PIN0        A0
#define ANALOG_CELL_PIN1        A1
#define ANALOG_CELL_PIN2        A2
#define ANALOG_CELL_PIN3        A3
#define ANALOG_CELL_PIN4        A4
#define ANALOG_TEMPERATURE_PIN  A6
#define ANALOG_LEAK_PIN         A7  //NOT USED

#define NUM_LEDS                6
#define YELLOW                  1
#define GREEN                   2
#define RED                     3
#define CONV_FACTOR1            0.00488
#define CONV_FACTOR2            0.054
#define REPEL_POS               180
#define ATTRACT_POS             0
#define DOC_CAPACITY            200

/**
  * @info: Limit voltage values to determine if cells are charged or discharged
  * https://confluence.asuqtr.com/display/SUBUQTR/PCB+Pod+Specifications (Battery cell monitoring section)
  **/
#define VOLT_4S_C1              61
#define VOLT_4S_C2              121
#define VOLT_4S_C3              182
#define VOLT_4S_C4              242

#define VOLT_5S_C1              553
#define VOLT_5S_C2              100
#define VOLT_5S_C3              151
#define VOLT_5S_C4              201
#define VOLT_5S_C5              252

/* Global Variables */
Servo dropper1; //servo obj
Servo dropper2;
Servo torpedo;


/** @Brief: Arduino pins nitialization  **/
void initPin(){
  /* DIGITAL PIN */
  pinMode(LEAK_PIN,INPUT); 
  pinMode(DIP_SWITCH_CELL,INPUT); 
  pinMode(DIP_SWITCH_FREE,INPUT); 
  pinMode(LED_PIN,OUTPUT); 

  servoInit();
}

/** @Brief: Servo motors initialization
  *  
  * @detail: Initial position for dropper = 0 degree 
  * @detail: Initial position for torpedo = 90 degree  
  **/
void servoInit() {
  int initial_dropper_pos = 0;
  int initial_torpedo_pos = 90;
  dropper1.attach(DROPPER1_PIN); //attaches the servo on pin 9 to the servo obj
  dropper2.attach(DROPPER2_PIN);
  torpedo.attach(TORPEDO_PIN);

  /* Init dropper position to 0 degree */
  dropper1.write(initial_dropper_pos);
  dropper2.write(initial_dropper_pos);
  torpedo.write(initial_torpedo_pos);
}

/** @Brief: get digital & analog input values
  * @param[in, out]: batteries cells values [0,4]
  * @param: temperature values
  * @param: leak values
  * @param: dipSwitchCell
  * @param: dipSwitchFree
  * @param: leakState
  **/
void readPin(int *val0, int *val1, int *val2, int *val3, int *val4, int *tempVal, int *leakVal,
             int *dipSwitchFree, int *dipSwitchCell, bool *leakState){
  /* DIGITAL PIN */
  *leakState = digitalRead(LEAK_PIN);
  *dipSwitchCell = digitalRead(DIP_SWITCH_CELL);
  *dipSwitchFree = digitalRead(DIP_SWITCH_FREE);
  /* ANALOG PIN */
  *val0 = analogRead(ANALOG_CELL_PIN0);  // read the input pin  
  *val1 = analogRead(ANALOG_CELL_PIN1);  
  *val2 = analogRead(ANALOG_CELL_PIN2);  
  *val3 = analogRead(ANALOG_CELL_PIN3);   
  *val4 = analogRead(ANALOG_CELL_PIN4);   
  *tempVal = analogRead(ANALOG_TEMPERATURE_PIN);  
  *leakVal = analogRead(ANALOG_LEAK_PIN);  //not used               
}

/** @Brief: Convert analogic values to tension values
  * @param: batteries cells values [0,4]
  * @param: temperature values
  **/
void conversionToVoltage(int *val0, int *val1, int *val2, int *val3, int *val4, int *tempVal, float *cell0,  
                         float *cell1, float *cell2, float *cell3, float *cell4, float *tempVolt){
  // bin to voltage battery_cell 
  *cell0 = *val0*CONV_FACTOR1;
  *cell1 = *val1*CONV_FACTOR2;
  *cell2 = *val2*CONV_FACTOR2;
  *cell3 = *val3*CONV_FACTOR2;
  *cell4 = *val4*CONV_FACTOR2;
  /* bin to voltage thermistor */
  *tempVolt = *tempVal*CONV_FACTOR1;
}

/** @Brief: Serialized data to send to ROS network
  * @param: batteries cells values [0,4]
  * @param: temperature values
  * @param: leak values
  * @param: dipSwitchCell
  * 
  * @detail: frame data: {"cell_voltage": "[float array]", "leak_sensor": "bool", "temp_sensor": "float"\n}
  **/
void jsonSerial(float *cell0, float *cell1, float *cell2, float *cell3, float *cell4, float *tempVolt,
                bool *leakState, int *dipSwitchCell){
  int array_size = 0; 
  if(dipSwitchCell){
    array_size = 5;
  }
  else{
    array_size = 4;
  }
  const size_t capacity = 2*JSON_ARRAY_SIZE(1) + JSON_ARRAY_SIZE(array_size) + JSON_OBJECT_SIZE(3);
  DynamicJsonDocument data(capacity);

  JsonArray cell_voltage = data.createNestedArray("cell_voltage");
  if(*dipSwitchCell){
    cell_voltage.add(*cell0);
  }
  cell_voltage.add(*cell1);
  cell_voltage.add(*cell2);
  cell_voltage.add(*cell3);
  cell_voltage.add(*cell4);
    
  JsonArray leak_sensor = data.createNestedArray("leak_sensor");
  leak_sensor.add(*leakState);

  JsonArray temp_sensor = data.createNestedArray("temp_sensor");
  temp_sensor.add(*tempVolt);

  serializeJson(data, Serial);
  Serial.println("\n"); 
}

/** @Brief: deserialized commands from ROS network
  *
  * @detail: frame data: {"pod actuator": "...", "actuator cmd": "..."}
  **/
void deserialization(){
  /* allocate json doc */
  DynamicJsonDocument doc(DOC_CAPACITY);

  /* Deserialize the JSON doc */
  DeserializationError error = deserializeJson(doc, Serial);

  if(error){
    return;
  }
  String pod_actuator = doc["pod actuator"];
  int actuator_pos = doc["actuator cmd"];
  if (pod_actuator == "dropper 1") {
    int dropper1_pos = doc["actuator cmd"];
    dropperPosition(&pod_actuator, &dropper1_pos);
  }
  if (pod_actuator == "dropper 2") {
    int dropper2_pos = doc["actuator cmd"];
    dropperPosition(&pod_actuator, &dropper2_pos);
  }
  if (pod_actuator == "torpedo") {
    int torpedo_pos = doc["actuator cmd"];
    torpedoPosition(&torpedo_pos);
  }
}

/** @Brief: get LED color based on sensors status
  * @param: batteries cells values [0,4]
  * @param: temperature values
  * @param: leak values
  * @param: dipSwitchCell
  * @param: LED color
  **/
void getLedColor(int *val0, int *val1, int *val2, int *val3, int *val4, int *tempVal, 
                 int *dipSwitchCell, bool *leakState, int *color){                                 
  /*warning message: T>75°, T<-40, no signal or water*/
  if(*leakState || *tempVal>=912 || *tempVal<=33){
    *color = YELLOW;
  }else{           //BATTERY CHARGE
    /* Sw1 ON ---- 5S */
    if(*dipSwitchCell){
      /* DISCHARGE */
      if(*val0>VOLT_5S_C1 && *val1>VOLT_5S_C2 && *val2>VOLT_5S_C3 && *val3>VOLT_5S_C4 && *val4>VOLT_5S_C5){
        *color = GREEN; 
      }
      /* CHARGE */
      else{
        *color = RED;   
      }
    }else{       //Sw1 OFF ---- 4S
      /* DISCHARGE */ 
      if(*val1 > VOLT_4S_C1 && *val2>VOLT_4S_C2 && *val3>VOLT_4S_C3 && *val4>VOLT_4S_C4){
        *color = GREEN;
      }
       /* CHARGE */
      else{
        *color = RED;
      }
    }      
  } 
}

/** @Brief: turn strip LED ON with a specific color
  * @param: led color
  * 
  * @detail: yellow = Temp/leak warning
  * @detail: red = batteries discharged
  * @detail: green = batteries charged
  * 
  * @info: CRGB leds[NUM_LEDS] and FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS) 
  *        lines are repeated because when they are declared in the beggining of the void 
  *        function OR the for loop, some anomalies are observed for the strip LED: some 
  *        leds are blinking instead of showing a constant light.
  **/
void light(int *color){
  for (int i = 0; i <= 5; i++){
    if(*color == YELLOW){
      CRGB leds[NUM_LEDS];
      FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
      leds[i] = CRGB (85, 45, 0);
    }
    else if(*color == GREEN){
      CRGB leds[NUM_LEDS];
      FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
      leds[i] = CRGB ( 0, 255, 0);
    }
    else{
      CRGB leds[NUM_LEDS];
      FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
      leds[i] = CRGB ( 255, 0, 0);
    }
    FastLED.show();
  }    
}

/** @Brief: allows dropper release based
  *         on a serial command from ROS network
  * @param: dropper servo number
  * @param: dropper servo position command
  * 
  * @detail: no release position = 0 degrees
  * @detail: release position = 180 degrees
  **/
void dropperPosition(String* dropper, int* dropper_pos) {
  if (*dropper == "dropper 1") {
    dropper1.write(*dropper_pos);
  }
  if (*dropper == "dropper 2") {
    dropper2.write(*dropper_pos); 
  }
}

/** @Brief: allows torpedo release based
  *         on a serial command from ROS network
  * @param: torpedo servo position command
  * 
  * @detail: neutral position = 90 degrees
  * @detail: torpedo 1 released = 80 degrees
  * @detail: torpedo 2 released = 100 degrees
  **/
void torpedoPosition(int* torpedo_pos) {
  torpedo.write(*torpedo_pos);
}

void setup() { 
  Serial.begin(9600);
  initPin();
}

/* @info: sw0 & leakV are declared but not used */
void loop() {
  int v0, v1, v2, v3, v4, temp0, leakV, sw0, sw1, colors = 0;
  float c0, c1, c2, c3, c4, temp1 = 0;
  bool leakS = NULL;
    
  readPin(&v0, &v1, &v2, &v3, &v4, &temp0, &leakV, &sw0, &sw1, &leakS);
  conversionToVoltage(&v0, &v1, &v2, &v3, &v4, &temp0, &c0, &c1, &c2, &c3, &c4, &temp1);
  jsonSerial(&c0, &c1, &c2, &c3, &c4, &temp1, &leakS, &sw1);
  deserialization();
  getLedColor(&v0, &v1, &v2, &v3, &v4, &temp0, &sw1, &leakS, &colors);
  light(&colors);
}
