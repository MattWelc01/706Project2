void setup(){

  
}


void loop(){
  
}

void obstacleAvoid(void){
  if(SONAR_FLAG){
    if(!IRLS && (IRLF + IRRF == 1)){
      //strafe left
    }else if(!IRRS && (IRLF + IRRF == 1)){
      //strafe right
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
    }

    if(!IRRS && (IRRF || IRLF)){
      //STRAFE RIGHT
    }
    
    if(IRLS && IRRS && (IRLF || IRRF)){
      //ESCAPE
    }

    if(IRLS && IRRS && !(IRLF || IRRF)){
      //SRAFE MIN DIST
    }
    if(!IRLF && !IRRF)){
      //DRIVE FORWARD OPEN LOOP A BIT
    }
  }
}
