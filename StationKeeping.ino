
#define enA 9
#define in1 6
#define in2 7

// PWM signal
float motorPWM = 0;

// Current reading variables
float I;
int count;
int read_time = 2000; // time interval to read over
unsigned long current_timer;
float readings[100];
float max_current = 0.0;
float current_sum = 0.0;
int current_counter = 0;

// States
bool read_current = false;
bool move = false;
bool all_stop = false;
bool balanced = true;

// Position
float position;
float desired_position = 45.0;
float error;
unsigned long time = 0.0;
float dt = 0.0;
float prev_position = 0.0;

// Angular Motion
float omega;
float desired_omega = 3.0;

// Butterworth 
float xa[] = {0,0,0};
float ya[] = {0,0,0};
int ka = 0;

float xp[] = {0,0,0};
float yp[] = {0,0,0};
int k = 0;

float xc[] = {0,0,0};
float yc[] = {0,0,0};
int kc = 0;

// PID Gains ---- Motor driver got really warm from over working
float kp = 8.6;
float ki = 0.84;
float kd = 0;

float prev_i = 0;
float prev_d = 0;
float i = 0.0;
bool reset = false;

void setup() {
  
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  Serial.begin(115200);

  delay(200);
  while(millis()<1000){
    get_current();
    get_position();
  }
  Serial.println("Welcome");
}

float butterworth(float signal){
  xa[0] = signal;

  // Compute the filtered signal
  // (second order Butterworth example)
  float ba[] = {0.00024132, 0.00048264, 0.00024132};
  float aa[] = {1.95558189, -0.95654717};
  ya[0] = aa[0]*ya[1] + aa[1]*ya[2] +
               ba[0]*xa[0] + ba[1]*xa[1] + ba[2]*xa[2];

  if(ka % 3 ==0)
  {
    // This extra conditional statement is here to reduce
    // the number of times the data is sent through the serial port
    // because sending data through the serial port
    // messes with the sampling frequency
    
    // For the serial monitor
    //Serial.print(x[0] * 270.0/1023.0 - 33.0);
    //Serial.print(" ");
    //Serial.println(y[0] * 270.0/1023.0 - 33.0);
  }

  delay(1); // Wait 1ms
  for(int i = 1; i >= 0; i--){
    xa[i+1] = xa[i]; // store xi
    ya[i+1] = ya[i]; // store yi
  }
  
  ka = ka+1;
  return ya[0];
}

float get_position(){  // 2nd Order Butterworth Low Pass Filter
  // Test signal
  
  xp[0] = analogRead(A1);

  // Compute the filtered signal
  // (second order Butterworth example)
  float bp[] = {0.00024132, 0.00048264, 0.00024132};
  float ap[] = {1.95558189, -0.95654717};
  yp[0] = ap[0]*yp[1] + ap[1]*yp[2] +
               bp[0]*xp[0] + bp[1]*xp[1] + bp[2]*xp[2];

  if(k % 3 ==0)
  {
    // This extra conditional statement is here to reduce
    // the number of times the data is sent through the serial port
    // because sending data through the serial port
    // messes with the sampling frequency
    
    // For the serial monitor
    //Serial.print(x[0] * 270.0/1023.0 - 33.0);
    //Serial.print(" ");
    //Serial.println(y[0] * 270.0/1023.0 - 33.0);
  }

  delay(1); // Wait 1ms
  for(int i = 1; i >= 0; i--){
    xp[i+1] = xp[i]; // store xi
    yp[i+1] = yp[i]; // store yi
  }
  
  k = k+1;
  return yp[0] * 282.216 / 1023 - 177.332;
}

float get_current(){
  xc[0] = analogRead(A0);

  // Compute the filtered signal
  // (second order Butterworth example)
  float bc[] = {0.00024132, 0.00048264, 0.00024132};
  float ac[] = {1.95558189, -0.95654717};
  yc[0] = ac[0]*yc[1] + ac[1]*yc[2] +
               bc[0]*xc[0] + bc[1]*xc[1] + bc[2]*xc[2];

  if(kc % 3 ==0)
  {
    // This extra conditional statement is here to reduce
    // the number of times the data is sent through the serial port
    // because sending data through the serial port
    // messes with the sampling frequency
    
    // For the serial monitor
    //Serial.print((2.5 - (xc[0] * (5.0 / 1023.0))) /0.066);
    //Serial.print(" ");
    //Serial.println((2.5 - (yc[0] * (5.0 / 1023.0))) /0.066);
  }

  delay(1); // Wait 1ms
  for(int i = 1; i >= 0; i--){
    xc[i+1] = xc[i]; // store xi
    yc[i+1] = yc[i]; // store yi
  }
  
  kc = kc+1;
  return (2.5 - (yc[0] * (5.0 / 1023.0))) /0.066 - 0.74;
}

float derivative(float error, bool reset){
  if (reset){
    prev_d = error;
  }
  dt = 0.05; // convert to seconds

  float d = (error - prev_d) / dt;
  prev_d = error;

  return d;
}

float integral(float error,bool reset){
  if (reset){
    i = 0;
    prev_i = 0;
  }
  dt = 0.05; // convert to seconds

  i += (error + prev_i)/2 * dt;
  
  prev_i = error;

  return i;
}

bool take_current_reading(bool read_current, unsigned long current_timer, int read_time){
  if (read_current && millis() - current_timer < read_time){ // Reads current for specified lenght of time
    I += get_current();
    count++;
    
    return true;
  }else if (millis() - current_timer >= read_time && read_current){ // End of current reading 
    Serial.print("Current:");
    Serial.println(I / float(count));
    
    // reset variables
    count = 0;
    I = 0;  
    return false;
  }else{
    // something is wrong reset variables. print error. return to main loop.
    Serial.println("Current Reading Error");
    count = 0;
    I = 0; 
    return false;
  }
  
  
}

void printStatus(float kp, float ki,float error,float max_current,float avg_current){ 
// Prints instantaneous current, position, desired position, PID constants, and balancing status
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Angular Position [deg]: ");
  Serial.println(get_position());
  Serial.print("Desired Position [deg]: ");
  Serial.println(desired_position);
  Serial.print("PID Gains (kp, ki): ");
  Serial.print(kp); Serial.print(", ");
  Serial.println(ki);
  Serial.print("Max Current from previous move [A]: ");
  Serial.println(max_current);
  Serial.print("Average Current [A]: ");
  Serial.println(avg_current);
  Serial.println("~");
  
}

void validInputs(){ // Prints valid inputs
  String inputs_available[7] = {"balance - toggles PID control","read current - Takes current reading","%(angular_position) - changes desired angular position","#p(gain)","#i(gain)","#d(gain)","$(motor_speed) - Manual PWM signal"};
  Serial.println("\nValid Inputs:");
  for (int i = 0;i<7;i++){
    Serial.println(inputs_available[i]);
  }
}

void loop() {
  // Completes the following tasks each loop
  // -------------------------------------------------------
  // 1. Serial Input 
  // 2. Read Current 
  // 3. Calculate Error 
  // 4. Calculate PID (if balance is active)
  // 5. Send PWM signal
  // 7. Check if balanced 
  
  // 1. Serial Input
  if(Serial.available()){ // Take Serial Input
  
    String input = Serial.readString();
    input.trim();
    
    if(input == "!" || all_stop){ // Shuts off motor

      move = false;
      balanced = true;
      motorPWM = 0;
      all_stop = false;

    }else if(input == "read current" && read_current == false){ // Enter to begin current reading
    
      Serial.println("Current Reading Started");
      read_current = true;
      current_timer = millis();
      
    }else if(input == "read current" && read_current == true){ // Current reading in progress
      
      Serial.println("active");
      
    }else if(input.charAt(0) == '@'){ // Changes current read time
      read_time = 0;
      for(int i = 1;i<input.length();i++){
         read_time = (input.charAt(i)-48) * pow(10, (input.length() - i - 1));
      }
    }else if(input == "move"){ // Enter to begin Station Keeping. no integral term
      
      move = !move;
      if(move){
        Serial.println("Position Control Activated");
        reset = true;

      }else{
        Serial.println("Position Control Deactivated");
        motorPWM = 0;
      }
      
    }else if(input.charAt(0) == '$' && !move && balanced){ // Set manual PWM signal for Nik

      motorPWM = 0;
      int neg = 1;
      
      if(input.charAt(1) == '-'){
        neg = 2;
      }

      for(int i = neg; i < input.length(); i++){
        motorPWM += (input.charAt(i)-48) * pow(10, (input.length() - i - 1));
      }

      if(neg == 2){
        motorPWM = -1 * motorPWM;
      }
      Serial.println(motorPWM);
      if (motorPWM < 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
      }else if (motorPWM > 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
      }  
      motorPWM = abs(motorPWM);


    }else if(input.charAt(0) == '%'){ // Enter Desired Position to station keep
      
      desired_position = 0;
      for(int i = 1; i < input.length(); i++){
        desired_position += (input.charAt(i)-48) * pow(10, (input.length() - i - 1));
      }
      Serial.print("New Position: ");
      Serial.println(desired_position);
      reset = true;
      
    }else if(input.charAt(0) == '#'){ 
    // Set new PID constants Example: change proportional constant to 10 (#p10)
      
      if(input.charAt(1) == 'p'){
        kp = 0;
        for(int i = 2; i < input.length(); i++){
          kp += (input.charAt(i)-48) * pow(10, (input.length() - i - 1));
        }
        Serial.print("kp: ");
        Serial.println(kp);
      }else if(input.charAt(1) == 'i'){
        ki = 0;
        for(int i = 2; i < input.length(); i++){
          ki += (input.charAt(i)-48) * pow(10, (input.length() - i - 1));
        }
        Serial.print("ki: ");
        Serial.println(ki);
      }else if(input.charAt(1) == 'd'){
        kd = 0;
        for(int i = 2; i < input.length(); i++){
          kd += (input.charAt(i)-48) * pow(10, (input.length() - i - 1));
        }
        Serial.print("kd: ");
        Serial.println(kd);
      }
    }else if(input == "?"){ // Print System Status
      Serial.println("System Status\n----------");
      printStatus(kp, ki,error,max_current,current_sum / current_counter);
    }else{ // Invalid input
      Serial.println("invalid input");
      validInputs();
    }
  }
  
  // 2. Current Reading
  if(read_current){
    read_current = take_current_reading(read_current, current_timer, read_time); 
  }
  
  // 3. Calculate Error
  

  // Serial.println(omega);

  // 4. Calculate Output to Move
  dt = millis() - time;
  if(dt > 5){
    position = get_position();
  
    error = desired_position - position;
  
    omega = (position - prev_position) / 0.005;
    prev_position = position;

    if(abs(error) > 3){
      desired_omega = 3.0;
    }else if(abs(error) <= 3){
      desired_omega = 1.0;
    }
    
    time = millis();
  }

  if(move && dt > 5 && abs(error) > 0.5){
    if(reset){
      max_current = 0.0;
      current_counter = 0;
      current_sum = 0.0;
    }
    float current = get_current();
    if(abs(current) > abs(max_current)) max_current = current;
    current_sum += current;
    current_counter++;
    
    float i = integral(error/abs(error)*desired_omega - omega, reset);
    //float d = derivative(desired_omega - omega, reset);
    reset = false;

    motorPWM = kp * (error/abs(error)*desired_omega - omega) + ki * i;
    
    if (motorPWM < 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }else if (motorPWM > 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }  
    
    //motorPWM = butterworth(motorPWM);
    
    motorPWM = constrain(motorPWM, -150, 150);
    
    /*Serial.print("Omega:");
    Serial.print(omega);
    Serial.print(", ");
    Serial.print("Position:");
    Serial.print(position);
    Serial.print(", ");
    Serial.print("PWM:");
    Serial.println(motorPWM);*/
    
    motorPWM = abs(motorPWM);
    
  }else if(move && abs(error) < 0.5 && !reset && abs(omega) < 0.03){
    motorPWM = 0;
    reset = true;
  }
  
  // 5. Send Signal to Motor

  // Send PWM signal
 
  analogWrite(enA, motorPWM);
  
  // 6. Check if balanced only when PID is active
  /*if(balance){
    if (derivative(time, error) == 0 && error == 0){ // Balanced Achieved. Beam is not moving and error = 0
      Serial.println("Balanced");
      balance = false;
    }else if (derivative(time,error) == 0 && error != 0) { // Balanced Not Achieved. Beam is not moving but there is still error
      Serial.println("Balance not achieved. Holding");
    }
  }*/
  
}
