int FIRE_FLAG = 0;
int OBSTACLE_FLAG = 0;


enum State {
  STATE_A,
  STATE_B,
  STATE_C,
  STATE_D,
  STATE_E
};


State currentState = STATE_A;
State previousState = STATE_A;

void setup(){
  
}

void loop(){
  previousState = currentState;// THIS LOGIC MAY NOT BE CORRECT, DOUBLE CHECK LATER
  
  // The switch-case statement represents the FSM
  switch(currentState) {
    case STATE_A:
      

      //FUNCTIONS AND SHIT GO HERE
      
      if (OBSTACLE_FLAG) { 
        currentState = STATE_E;
      }
      if (!OBSTACLE_FLAG && FIRE_FLAG) { 
        currentState = STATE_C;
      }else{
        currentState = STATE_B;
      }
      
      break;

    case STATE_B:
      
      if (OBSTACLE_FLAG) { 
        currentState = STATE_E;
      }
      if (!OBSTACLE_FLAG && FIRE_FLAG) { 
        currentState = STATE_C;
      }else{
        currentState = STATE_A;
      }
      break;

    case STATE_C:
      
      // Transition to the next state
      if (!FIRE_FLAG && OBSTACLE_FLAG) {
        currentState = STATE_E;
      }
      if(FIRE_FLAG && !OBSTACLE_FLAG){
        currentState = STATE_C;
      }
      if(FIRE_FLAG && OBSTACLE_FLAG){
        currentState = STATE_D;
      }else{
        currentState = STATE_B;
      }
      break;


      case STATE_D:
      
      // Transition to the next state
      if (FIRE_FLAG) {
        currentState = STATE_D;
      }else{
        currentState = STATE_A;
      }
      break;


      case STATE_E:
     
      // Transition to the next state
      currentState = previousState;    
      break;
  }
}
