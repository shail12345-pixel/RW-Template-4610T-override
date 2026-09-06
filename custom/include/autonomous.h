// Format: returnType functionName();


double getLiftHeight();
bool liftDown();

// void cascadeToHeight(double height){
//   int direction;
//     if(getLiftHeight()<height){
//       direction =1;
//     }else{
//       direction =-1;
//     }
//     while(fabs(getLiftHeight()-height)){
//         cascade.spin(fwd,12*direction,volt);
//     }
// }

extern double alliance;
extern double neutral;
extern double midfield;

extern double cup;
extern double pin;

// extern void cascadeToHeight(double height);
// extern void cascadeToHeight(double pinCount, double cupCount, std::string goal, double buffer = 2);
// extern void cascadePlusHeight(double height);

void exampleAuton();
void exampleAuton2();
void tunePid();
void autonOne();
void liftPID_tuner();
void fullLiftTest();
void brainD();
void simple();
void qual1();

void qual1_lift();

void qual1_wrist();

void printText(const char* text);
