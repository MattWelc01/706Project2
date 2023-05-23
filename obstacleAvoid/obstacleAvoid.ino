
///////////////////////////////CODE FOR OBSTACLE AVOID BELOW THIS LINE////////////////////////////////////////


void setup(){
  
}



void obstacleAvoid(void){
  if(SONAR_FLAG){
    if(!IRLS && (IRLF + IRRF == 1)){
      //strafe left
      strafe(-1);
    }else if(!IRRS && (IRLF + IRRF == 1)){
      //strafe right
      strafe(1);
    }else if(((IRLS && IRRS) && (IRLF || IRRF)){
      //escape
    }else if(IRLF && IRRF){
      //find max dist
    }else{
      //escape THIS IS NOT MOST EFFICIENT PATH
    }

    
  }
  
  if(!SONAR_FLAG){
    if(!IRLS && (IRRF || IRLF)){
      //STRAFE LEFT
      strafe(-1);
    }

    if(!IRRS && (IRRF || IRLF)){
      //STRAFE RIGHT
      strafe(1);
    }
    
    if(IRLS && IRRS && (IRLF || IRRF)){
      //ESCAPE
    }

    if(IRLS && IRRS && !(IRLF || IRRF)){
      //SRAFE MIN DIST
    }
    if(!IRLF && !IRRF){
      //DRIVE FORWARD OPEN LOOP A BIT
    }
  }
}


void loop(){
  
}
