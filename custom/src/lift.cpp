#include "vex.h"
#include "../custom/include/lift.h"
#include "../custom/include/robot-config.h"

#include <cmath>

using namespace vex;


void lift(){
  while(1){
    if(controller_1.ButtonR1.pressing()){
        cascade.spin(fwd,12,volt);
    }else if(controller_1.ButtonR2.pressing()){
      cascade.spin(reverse,12,volt);
    }else{
      cascade.stop(hold);
    }
    
  }
}


void run_intake(){
  while(1){
    if(controller_1.ButtonL1.pressing()){
        intake.spin(fwd,12,volt);
    }else if(controller_1.ButtonR2.pressing()){
      intake.spin(reverse,12,volt);
    }else{
      intake.stop(coast);
    }
    
  }
}