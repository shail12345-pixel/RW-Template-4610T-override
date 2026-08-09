// Format: returnType functionName();


double getLiftHeight();
bool liftDown();

void cascadeToHeight(double height){
  int direction;
    if(getLiftHeight()<height){
      direction =1;
    }else{
      direction =-1;
    }
    while(fabs(getLiftHeight()-height)){
        cascade.spin(fwd,12*direction,volt);
    }
}

double alliance = 3;
double neutral = 6;
double midfield = 12;

double cup = 7;
double pin = 5;

void cascadeToHeight(double height);
void cascadeToHeight(double pinCount, double cupCount, std::string goal, double buffer = 2);
void cascadePlusHeight(double height);

void exampleAuton();
void exampleAuton2();
void tunePid();
void autonOne();
