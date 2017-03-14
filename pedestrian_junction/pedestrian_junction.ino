//various user-defined settings here

//pins for each of the traffic lights
#define T_RED_PINS {2, 5}
#define T_AMBER_PINS {3, 6}
#define T_GREEN_PINS {4, 7}

//pins for each of the pedestrian lights
#define P_RED_PINS {8}
#define P_GREEN_PINS {9}

//traffic light settings
#define T_TRANSITION_TIME 2000
#define T_GREEN_TIME 10000
#define T_RED_DELAY 2000
#define T_AMBER_FLASH_TIME 3000
#define T_AMBER_FLASH_INTERVAL 500

//pedestrian light settings
#define P_GREEN_TIME 10000
#define P_PED_DELAY 3000

//DO NOT EDIT BELOW THIS LINE

//The different states (can also be used as Light 'type')
#define NONE 0
#define RED 1
#define AMBER 2
#define GREEN 4
#define AMBER_FLASHING 8
#define GREEN_FLASHING 9

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
  unsigned long last_flash_update; //a timestamp of the last flash
  
  long transition_time; //the time between transitions (only for amber - green and green - amber)
  long green_time; //how long to stay green for
  long red_delay; //a delay before the red light switches to amber
  long amber_flash_interval; //how long between each flash (time between on - off and off - on)
  long amber_flash_time; //how long to flash for (to safely let the pedestrian cross :) )
} TrafficLight;

typedef struct PedestrianLight {
  Light lights[2]; //an array of two physical lights

  byte state; //initial pedestrian state (usually RED)

  unsigned long last_update; //a timestamp of the last transition (or "update")
  long ped_delay; //a delay between when the traffic lights go red and when the pedestrian light turns green
  long green_time; //how long to stay green for

  boolean button_pressed; //is the pedestrian button pressed?
} PedestrianLight;

//a TJunction definition
typedef struct TJunction {
  TrafficLight *traffic_lights; //two traffic lights (array)
  PedestrianLight *ped_lights; //one pedestrian light (array) (might add more in the future)

  byte active_traffic_light_id; //index of the active traffic light
  TrafficLight *active_traffic_light; //active traffic light

  byte active_ped_light_id; //index of the active pedestrian light
  PedestrianLight *active_ped_light; //active pedestrian light

  boolean ped_mode; //is pedestrian mode enabled (i.e. has the pedestrian light sequence started)
} TJunction;

//a function that initialises a traffic light
TrafficLight initTrafficLight(int red_pin, int amber_pin, int green_pin, int transition_time, int green_time, int red_delay, int amber_flash_time, int amber_flash_interval) {
  //creates a traffic light
  TrafficLight traffic_light;

  //uses some macros to define the type of each light
  RED_LIGHT(traffic_light.lights[0], red_pin);
  AMBER_LIGHT(traffic_light.lights[1], amber_pin);
  GREEN_LIGHT(traffic_light.lights[2], green_pin);

  traffic_light.state = RED; //the initial state of the traffic light
  traffic_light.last_update = 0; //no update has been done yet, so 0
  traffic_light.last_flash_update = 0; //no flash has been done yet, so 0
  traffic_light.transition_time = transition_time; //the time between transitions (only for amber - green and green - amber)
  traffic_light.green_time = green_time; //how long to stay green for
  traffic_light.red_delay = red_delay; //a delay before the red light switches to amber
  traffic_light.amber_flash_interval = amber_flash_interval; //how long between each flash (time between on - off and off - on)
  traffic_light.amber_flash_time = amber_flash_time; //how long to flash for (to safely let the pedestrian cross :) )

  //turn the pins on
  pinMode(red_pin, OUTPUT);
  pinMode(amber_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);

  return traffic_light; //return the traffic light
}

//a function that initialises a pedestrian light
PedestrianLight initPedestrianLight(int red_pin, int green_pin, int green_time, int ped_delay) {
  PedestrianLight ped_light;

  //uses some macros to define the type of each light
  RED_LIGHT(ped_light.lights[0], red_pin);
  GREEN_LIGHT(ped_light.lights[1], green_pin);

  ped_light.state = RED; //initial pedestrian state (usually RED)
  ped_light.last_update = 0; //no update has been done yet so 0
  ped_light.green_time = green_time; //how long to stay green for
  ped_light.ped_delay = ped_delay; //a delay between when the traffic lights go red and when the pedestrian light turns green
  ped_light.button_pressed = false; //is the button on the pedestrian traffic light pressed

  //turn on the pins
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);

  return ped_light; //return the pedestrian light
}

TJunction initTJunction(TrafficLight traffic_lights[2], PedestrianLight ped_lights[1]) {
  TJunction t_junction;

  t_junction.ped_mode = false; //set junction pedestrian mode to false

  t_junction.traffic_lights = traffic_lights;
  t_junction.ped_lights = ped_lights;

  //set the current active traffic light
  t_junction.active_traffic_light_id = 0; //the active traffic light index in the 'traffic_lights' array
  t_junction.active_traffic_light = &t_junction.traffic_lights[0]; //a reference to first element in the 'traffic_lights' array

  //set the current active pedestrian light
  t_junction.active_ped_light_id = 0; //the active pedestrian light index in the 'ped_lights' array
  t_junction.active_ped_light = &t_junction.ped_lights[0]; //a reference to first element in the 'ped_lights' array

  return t_junction; //return the junction
}

//a function that checks if a traffic light should flash at all
boolean shouldTrafficLightFlash(TrafficLight *tL) {
  return tL->state == AMBER_FLASHING && //are we in the AMBER_FLASHING state? 
         NOW - tL->last_flash_update >= tL->amber_flash_interval && //has the amber_flash_interval time elapsed since we last flashed?
         NOW - tL->last_update <= tL->amber_flash_time; //are we also within the allowed time to keep flashing?
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

//a function to print the statuc of a pedestrian light
void printPedLight(PedestrianLight *pL, int index) {
  Serial.print("P");
  Serial.print(index);
  Serial.print(": ");
  printLightsStatus(pL->lights, 2);
}

//a function to print the status of the junction
void printJunction(TJunction *tJ) {
  TrafficLight *traffic_lights = tJ->traffic_lights; //get a reference to the traffic lights
  PedestrianLight *ped_lights = tJ->ped_lights; //get a reference to the pedestrian lights

  //loop through the traffic lights
  for (int i = 0; i < 2; i++) {
    printTrafficLight(traffic_lights + i, i); //print the status of the traffic light at i
    Serial.print(" ");
  }

  //loop through the pedestrian lights
  for (int i = 0; i < 1; i++) {
    printPedLight(ped_lights + i, i); //print the status of the pedestrian light at i
    Serial.print(" ");
  }

  Serial.print("PED MODE: ");
  Serial.print(ON_STR(tJ->ped_mode));

  Serial.println();
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
    case AMBER_FLASHING: //only for traffic lights
      flipLight(&lights[1]); 
      break;
    case GREEN_FLASHING:
      flipLight(&lights[1]);
      break;
  }
}

//a function to set the next state of a traffic light, using it's current state
void nextTrafficLightState(TrafficLight *tL) {
  byte *state_p = &tL->state;
  switch (*state_p) {
    case RED: *state_p = AMBER | RED; break;
    case AMBER | RED: *state_p = GREEN; break;
    case GREEN: *state_p = AMBER; break;
    case AMBER_FLASHING: *state_p = GREEN; break;
    case AMBER: *state_p = RED; break;
  }
}

//a function to set the next state of a pedestrian light, using it's current state
void nextPedLightState(PedestrianLight *pL) {
  byte *state_p = &pL->state;
  switch (*state_p) {
    case RED: *state_p = GREEN; break;
    case GREEN: *state_p = GREEN_FLASHING; break;
    case GREEN_FLASHING: *state_p = RED; break;
  }
}

//a function to handle the flashing of traffic and pedestrian lights
void handleFlashing(TJunction *tJ) {
  //get references to current traffic and pedestrian lights
  TrafficLight *tL = tJ->active_traffic_light; 
  PedestrianLight *pL = tJ->active_ped_light;

  tL->last_flash_update = NOW; //update the flash timer

  //this is where we actually update the LED states
  updateLights(pL->lights, 2, pL->state);
  updateLights(tL->lights, 3, tL->state);

  //print information about the pedestrian light
  printPedLight(pL, 0);
  Serial.println();
}

boolean handlePedestrianLight(TJunction *tJ) {
  TrafficLight *traffic_light = tJ->active_traffic_light;
  PedestrianLight *ped_light = tJ->active_ped_light;
  
  if ((ped_light->button_pressed || tJ->ped_mode) && traffic_light->state == RED) { //checks if the pedestrian light is pressed, and both traffic lights are RED (can just check the active one, since if the active one is RED, both are RED)
    if (ped_light->button_pressed) { //if button pressed
      ped_light->button_pressed = false; //make the button unpressed
      ped_light->last_update = NOW; //reset the pedestrian update timer
      tJ->ped_mode = true; //set the junction in pedestrian mode
    }
    
    long delta_time = DELTA_TIME(ped_light->last_update); //get the delta time since the last update
    if (ped_light->state == RED && delta_time >= ped_light->ped_delay) { //if the current state is RED and waited past the pedestrian delay (probably a few seconds)
      nextPedLightState(ped_light); //go to the next ped light state (GREEN)
      updateLights(ped_light->lights, 2, ped_light->state); //turn on the LEDs according to the state (GREEN)

      printPedLight(ped_light, 0);//output information about the pedestrian light
      Serial.println();
    } else if (ped_light->state == GREEN && delta_time >= ped_light->green_time) { //otherwise, if the current state is GREEN and we've waited past the GREEN wait time, we're gonna start flashing GREEN
      traffic_light->state = AMBER_FLASHING; //set the current active traffic light state to AMBER_FLASHING
      traffic_light->last_update = NOW; //reset the traffic light timer as this is what we'll use to check when the lights should transition
      
      nextPedLightState(ped_light); //go to the next pedestrian light state (GREEN_FLASHING)
      
      tJ->ped_mode = false; //we're no longer in pedestrian mode
    }

    return true; //return true if we're in pedestrian mode
  }
  
  return false; //otherwise return false
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

  if (state == AMBER_FLASHING && delta_time >= tL->amber_flash_time) { //checking if we can transition from AMBER FLASHING
    return true;
  }

  return false; //we can't transition from anything yet
}

//a function to set the current active traffic light to the next one in the array
void nextLight(TJunction *tJ) {
  ++tJ->active_traffic_light_id %= 2; //we increase the current traffic light index, and use modulo to make sure we do not go above 2
  
  tJ->active_traffic_light = &tJ->traffic_lights[tJ->active_traffic_light_id]; //we set the active traffic light to the next one using the index that we update above
  tJ->active_traffic_light->last_update = NOW; //we update the transition timer for the traffic light
}

//a function to handle the transition of the junction
void handleTransition(TJunction *tJ) {
  TrafficLight *active_traffic_light = tJ->active_traffic_light;
  PedestrianLight *active_ped_light = tJ->active_ped_light;

  //make the pedestrian light next if it isn't already
  if (active_ped_light->state != RED) {
    active_ped_light->state = RED;
    updateLights(active_ped_light->lights, 2, active_ped_light->state); //turn on/off lights corresponding to state (in this case RED)
  }

  nextTrafficLightState(active_traffic_light); //go to the next traffic light state
  updateLights(active_traffic_light->lights, 3, active_traffic_light->state); //turn on/off lights corresponding to traffic light state
  active_traffic_light->last_update = NOW; //set the last transition time to now

  printJunction(tJ); //print information about the junction

  if (active_traffic_light->state == RED) { //if the current state is RED (in other words, if we have just transitioned to RED)
    nextLight(tJ); //go to the next traffic light
  }
}

//a function to move the state of a junction forward, if it needs to
void tickJunction(TJunction *tJ) {
  if (handlePedestrianLight(tJ)) //if we did some stuff in pedestrian mode
    return; //don't continue
    
  if (shouldTrafficLightTransition(tJ->active_traffic_light)) { //if we should transition to the next traffic light state
    handleTransition(tJ); //handle the transition to the next state
  } else if (shouldTrafficLightFlash(tJ->active_traffic_light)) { //otherwise, we may need to flash AMBER for the traffic lights, and GREEN for the pedestrian lights
    handleFlashing(tJ); //handle flashing
  }
}

TJunction t_junction;

void setup() {
  Serial.begin(9600);
  
  int t_reds[2] = T_RED_PINS;
  int t_ambers[2] = T_AMBER_PINS;
  int t_greens[2] = T_GREEN_PINS;

  int p_reds[] = P_RED_PINS;
  int p_greens[] = P_GREEN_PINS;

  //allocate two traffic lights
  TrafficLight t_lights[2];

  //allocate one pedestrian light
  PedestrianLight p_lights[1];

  //here we initialise two traffic lights for the junction
  for (int i = 0; i < 2; i++) {
    t_lights[i] = initTrafficLight(t_reds[i], t_ambers[i], t_greens[i], T_TRANSITION_TIME, T_GREEN_TIME, T_RED_DELAY, T_AMBER_FLASH_TIME, T_AMBER_FLASH_INTERVAL);
    updateLights(t_lights[i].lights, 3, RED); //turn on the LEDS corresponding to state (RED)
  }

  //here we initialise one pedestrian light for the junction (it's in a loop as this code may support more in the future)
  for (int i = 0; i < 1; i++) {
    p_lights[i] = initPedestrianLight(p_reds[i], p_greens[i], P_GREEN_TIME, P_PED_DELAY);
    updateLights(p_lights[i].lights, 2, RED); //turn on the LEDS corresponding to state (RED)
  }

  //the junction data
  t_junction = initTJunction(t_lights, p_lights);

  //print the initial state of the junction
  printJunction(&t_junction);
}

void loop() {
  //set 'now' to the current time
  now = millis();

  if (Serial.read() > 0) { //if something has been input into the console
    t_junction.ped_lights[0].button_pressed = true; //this equates to a button being pressed on the first pedestrian light (may add more in the future)
  }

  tickJunction(&t_junction); //update the junction if needed
}

