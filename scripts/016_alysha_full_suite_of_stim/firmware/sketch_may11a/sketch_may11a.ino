// Pin 13 has the LED on Teensy 3.0

// cable color codes for point grey cam tigger cable
// White = Power ground
// Brown = Power +12V
// Gray = Trig ground
// Pink = Trig +

const int CAM_L_TRIG_PIN = 5;
const int CAM_R_TRIG_PIN = 7;

const int BOARD_LED_PIN = 13;
//BNC breakouts
const int WB_PIN = 2; //sync from wb analyzer
const int THOR_LED_PIN = 3;
const int TEST_BNC_PIN = 4;

const int WB_PER_FRAME = 3;
const int WINGBEAT_TIMEOUT_MS = 10; //50Hz 
const float TIME_ALPHA = 0.9;
const float TRIG_PHASE = 0.5;
const int EPI_PULSE_US = 100;

volatile int t = micros();
volatile bool wb_detected = false;
volatile int dt = 0;
volatile float est_period = 0;

bool epi_armed = false;
int trigger_armed = true;
int wb_count = 0;
int trigger_time = 0;
int looptime = 0;

//IntervalTimer camTimer;

void wb_isr() {
  //calculate delta_t frequency for phase.
  wb_detected = true;
  int tmp = micros();
  dt = tmp-t;
  t = tmp;
  est_period = est_period*TIME_ALPHA + (1-TIME_ALPHA)*float(dt);
}


void timeout_wb() {
  //calculate delta_t frequency for phase.
  wb_detected = true;
  int tmp = micros();
  dt = tmp-t;
  t = tmp;
  est_period = est_period*TIME_ALPHA + (1-TIME_ALPHA)*float(dt);
}

// the setup routine runs once when you press reset:
void setup() {
  //Serial.begin(9600);                
  // initialize the digital pin as an output.
  pinMode(CAM_L_TRIG_PIN, OUTPUT);
  pinMode(CAM_R_TRIG_PIN, OUTPUT);
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(THOR_LED_PIN, OUTPUT);
  pinMode(TEST_BNC_PIN, OUTPUT);
  pinMode(WB_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WB_PIN),wb_isr,FALLING);
}

// the loop routine runs over and over again forever:
void loop() {
  int t_copy;
  float est_period_copy;
  int wb_detected_copy;
  noInterrupts();
  t_copy = t;
  est_period_copy = est_period;
  wb_detected_copy = wb_detected;
  interrupts();
  looptime = micros()-t_copy;
  
  if(trigger_armed){
    // Wait for a wingstroke and then trigger the camera.
    // Log the time of the wingstroke so that a strobe can
    // be fired at the correct phase. 
    if(wb_detected_copy){
      digitalWrite(CAM_L_TRIG_PIN,HIGH);
      digitalWrite(CAM_R_TRIG_PIN,HIGH);
      trigger_armed = false;
      noInterrupts(); wb_detected = false; interrupts();
    }
    if (looptime > (1000*WINGBEAT_TIMEOUT_MS)){
          noInterrupts();timeout_wb();interrupts();
    }
  }else{
    // Fire blue pulses at the prescribed phase after each wingstroke,
    // Count the number of pulses and end the exposure and 
    // end the camera trigger after the count is reached.
    // Then wait a short time to allow the camera to reset and re-arm the trigger.
    if (wb_count < WB_PER_FRAME){
      if(epi_armed){
        //if looptime exceeds the duty cyle delay fire the blue pulse, increment the pulse count
        //and re-arm the epi trigger
        if(looptime > int(est_period_copy*TRIG_PHASE)){
          digitalWrite(BOARD_LED_PIN,HIGH);
          digitalWrite(THOR_LED_PIN,HIGH);
          digitalWrite(TEST_BNC_PIN,HIGH);
          
          delayMicroseconds(EPI_PULSE_US);
          
          digitalWrite(BOARD_LED_PIN,LOW);
          digitalWrite(THOR_LED_PIN,LOW);
          digitalWrite(TEST_BNC_PIN,LOW);
          epi_armed = false;
          wb_count += 1;
        }
      } else{
        // wait for the next wingstroke and then re-arm the epi trigger
        if (wb_detected_copy){
          epi_armed = true;
          noInterrupts();wb_detected = false;interrupts();
        }else if (looptime > (1000*WINGBEAT_TIMEOUT_MS)){
          noInterrupts();timeout_wb();interrupts();
          //Serial.println(int(est_period_copy*TRIG_PHASE));
        }
      }
    } else{
      // wb count has been exceeded end the exposure
      digitalWrite(CAM_L_TRIG_PIN,LOW);
      digitalWrite(CAM_R_TRIG_PIN,LOW);
      trigger_armed = true;
      wb_count = 0;
    }
  }
  //Serial.println(int(est_period_copy*TRIG_PHASE));
  //}
}
