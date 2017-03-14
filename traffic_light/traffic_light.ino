//various user-defined settings here

//pins for each of the traffic lights
#define T_RED_PIN 2
#define T_AMBER_PIN 3
#define T_GREEN_PIN 4

//traffic light settings
#define T_TRANSITION_TIME 2000
#define T_GREEN_TIME 10000
#define T_RED_DELAY 2000

//DO NOT EDIT BELOW THIS LINE

//The different states (can also be used as Light 'type')
#define NONE 0
#define RED 1
#define AMBER 2
#define GREEN 4

#define BIT_MASKS {RED, AMBER, GREEN}

#define ON true
#define OFF false

unsigned long now = millis(); //a variable holding the current time

#define NOW now //defining the current time as something ('now' will be updated in the mainloop. This is to try to reduce the number of 'millis' calls in the program)

//Macro functions
#define DELTA_TIME(from) (NOW - from) //a macro to calculate the difference between the current time and another time
#define ON_STR(a) (a ? "ON" : "OFF") //creates a string expression of a boolean value a in terms of "ON" (true) or "OFF" (false)
#define RED_LIGHT(light, p) light.pin = p; light.on = false; light.type = RED; //a macro to define a light as RED
#define AMBER_LIGHT(light, p) light.pin = p; light.on = false; light.type = AMBER; //a macro to define a light as AMBER
#define GREEN_LIGHT(light, p) light.pin = p; light.on = false; light.type = GREEN; //a macro to define a light as GREEN

//a physical light
typedef struct Light {
  byte pin; //the pin of the light
  byte type; //the type of the light (RED, AMBER, GREEN)
  boolean on; //is it on?
} Light;
 
//create a struct that will hold details about a traffic light
typedef struct TrafficLight {
  Light lights[3]; //an array of three physical lights

  byte state; //the current state of the traffic light

  //these are both used as timers
  unsigned long last_update; //a timestamp of the last transition (or "update")
  
  long transition_time; //the time between transitions (only for amber - green and green - amber)
  long green_time; //how long to stay green for
  long red_delay; //a delay before the red light switches to amber
} TrafficLight;

//a function that initialises a traffic light
TrafficLight initTrafficLight(int red_pin, int amber_pin, int green_pin, int transition_time, int green_time, int red_delay) {
  //creates a traffic light
  TrafficLight traffic_light;

  //uses some macros to define the type of each light
  RED_LIGHT(traffic_light.lights[0], red_pin);
  AMBER_LIGHT(traffic_light.lights[1], amber_pin);
  GREEN_LIGHT(traffic_light.lights[2], green_pin);

  traffic_light.state = RED; //the initial state of the traffic light
  traffic_light.last_update = 0; //no update has been done yet so 0
  traffic_light.transition_time = transition_time; //the time between transitions (only for amber - green and green - amber)
  traffic_light.green_time = green_time; //how long to stay green for
  traffic_light.red_delay = red_delay; //a delay before the red light switches to amber

  //turn the pins on
  pinMode(red_pin, OUTPUT);
  pinMode(amber_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);

  return traffic_light; //return the traffic light
}

//a function to set turn a Light on or off
void setLight(Light *l, boolean on) {
  digitalWrite(l->pin, on ? HIGH : LOW); //get the pin number from the light, set it to ON or OFF
  l->on = on; //store a record of whether the light is ON or OFF
}

//a function to turn an array of lights ON or OFF, given some flags
void setLights(Light *lArray, size_t len, byte flags) {
  if (flags == NONE) { //if no flags are set, we can just turn all the lights off
    for (int i = 0; i < len; i++) {
      setLight((lArray + i), OFF);
    }
  } else {
    int masks[] = BIT_MASKS; //load the bitmasks already, rather than generating them
    for (int i = 0; i < len; i++) { //loop through all the lights
      Light *light = (lArray + i); //get a reference to a Light
      
      for (int j = 0; j < 3; j++) { //loop through the bit masks
        byte mask = masks[j]; //get a bitmask at index 'j'
        if (light->type == mask) { //if the mask matches the type of light, we can begin :)
          setLight(light, flags & mask); //turn the light ON or OFF depending on whether the flag is set
        }
      }
    }
  }
}

//a function to print the status of each light in an array. It outputs either ON or OFF.
void printLightsStatus(Light *tLArray, size_t len) {
  for (int i = 0; i < len; i++) {
    Serial.print(ON_STR((tLArray + i)->on));
    Serial.print(" ");
  }
}

//a function to print the status of a traffic light
void printTrafficLight(TrafficLight *tL, int index) {
  Serial.print("L");
  Serial.print(index);
  Serial.print(": ");
  printLightsStatus(tL->lights, 3);
}

//a function to turn a light OFF if it is ON, and vice versa
void flipLight(Light *light) {
  if (light->on)
    setLight(light, OFF);
  else
    setLight(light, ON);
}

//a function that takes an array of lights, the num of lights there are, and a state, and turns the lights ON or OFF corresponding to the state
void updateLights(Light *lights, size_t num_of_lights, byte state) {
  switch (state) {
    case RED: setLights(lights, num_of_lights, RED); break;
    case AMBER | RED: setLights(lights, num_of_lights, RED | AMBER); break; //only for traffic lights
    case AMBER: setLights(lights, num_of_lights, AMBER); break; //only for traffic lights
    case GREEN: setLights(lights, num_of_lights, GREEN); break;
  }
}

//a function to set the next state of a traffic light, using it's current state
void nextTrafficLightState(TrafficLight *tL) {
  byte *state_p = &tL->state;
  switch (*state_p) {
    case RED: *state_p = AMBER | RED; break;
    case AMBER | RED: *state_p = GREEN; break;
    case GREEN: *state_p = AMBER; break;
    case AMBER: *state_p = RED; break;
  }
}

//a function that checks if the traffic light should transition to the next state, 
boolean shouldTrafficLightTransition(TrafficLight *tL) {
  long delta_time = DELTA_TIME(tL->last_update); //get the time difference from the last transition
  byte state = tL->state; //the current state of the traffic light

  if (state == RED && delta_time >= tL->red_delay) { //checking if we can transition from RED
    return true;
  }

  if ((state == (AMBER | RED) || state == AMBER) && delta_time >= tL->transition_time) { //checking if we can transition from AMBER or AMBER and RED
    return true;
  }
  
  if (state == GREEN && delta_time >= tL->green_time) { //checking if we can transition from GREEN
    return true;
  }

  return false; //we can't transition from anything yet
}

//a function to handle the transition of the traffic light
void handleTransition(TrafficLight *tL) {
  nextTrafficLightState(tL); //go to the next traffic light state
  updateLights(tL->lights, 3, tL->state); //turn on/off lights corresponding to traffic light state
  tL->last_update = NOW; //set the last transition time to now

  printTrafficLight(tL, 0); //print information about the traffic light
  Serial.println();
}

TrafficLight traffic_light;

void setup() {
  Serial.begin(9600);

  traffic_light = initTrafficLight(T_RED_PIN, T_AMBER_PIN, T_GREEN_PIN, T_TRANSITION_TIME, T_GREEN_TIME, T_RED_DELAY);
  updateLights(traffic_light.lights, 3, RED); //turn on the LEDS corresponding to state (RED)

  //print the initial state of the traffic light
  printTrafficLight(&traffic_light, 0);
}

void loop() {
  //set 'now' to the current time
  now = millis();

  if (shouldTrafficLightTransition(&traffic_light)) 
    handleTransition(&traffic_light);
}

