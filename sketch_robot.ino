class WheelController {
private:
  int forward_input_;
  int backward_input_;
  int speed_input_;
  int speed_;
  int speed_offset_;

public:
  WheelController(int forward_input, int backward_input, int speed_input) {
    this->forward_input_ = forward_input;
    this->backward_input_ = backward_input;
    this->speed_input_ = speed_input;
    this->speed_ = 0;
    this->speed_offset_ = 0;
  }

  void SetOffset(int offset) {
    this->speed_offset_ = offset;
    SetSpeed(this->speed_);
  }

  void GetOffest() {
    return this->speed_offset_;
  }

  void SetSpeed(int speed) {
    speed += this->speed_offset_;
    if (speed < 0)
      speed = 0;
    else if (speed > 255)
      speed = 255;

    this->speed_ = speed;
    analogWrite(this->speed_input_, this->speed_);
  }

  int GetSpeed() {
    return this->speed_;
  }

  void MoveForward() {
    digitalWrite(this->forward_input_, HIGH);
    digitalWrite(this->backward_input_, LOW);
  }

  void MoveBackwards() {
    digitalWrite(this->forward_input_, LOW);
    digitalWrite(this->backward_input_, HIGH);
  }

  void Stop() {
    digitalWrite(this->forward_input_, LOW);
    digitalWrite(this->backward_input_, LOW);
  }
};

class RobotController {
private:
  WheelController* left_wheel_;
  WheelController* right_wheel_;
  int speed_;

public:
  RobotController(WheelController* left_wheel, WheelController* right_wheel) {
    this->left_wheel_ = left_wheel;
    this->right_wheel_ = right_wheel;
    this->speed_ = 0;
  }

  void SetSpeed(int speed) {
    if (speed < 0)
      speed = 0;
    else if (speed > 255)
      speed = 255;

    this->speed_ = speed;
    left_wheel_->SetSpeed(speed_);
    right_wheel_->SetSpeed(speed_);
  }

  int GetSpeed() {
    return this->speed_;
  }

  void MoveForward() {
    left_wheel_->MoveForward();
    right_wheel_->MoveForward();
  }

  void MoveBackwards() {
    left_wheel_->MoveBackwards();
    right_wheel_->MoveBackwards();
  }

  void Stop() {
    left_wheel_->Stop();
    right_wheel_->Stop();
  }

  void TurnDegree(int degree) {
    if (degree == 0) return;
    Stop();
    delay(10);

    if (degree > 0) {
      left_wheel_->MoveForward();
      right_wheel_->MoveBackwards();
    } else {
      left_wheel_->MoveBackwards();
      right_wheel_->MoveForward();
    }

    delay(4.11 * abs(degree));

    Stop();
  }

  void TurnLeft() {
    TurnDegree(-90);
  }

  void TurnRight() {
    TurnDegree(90);
  }

  void TurnAround() {
    TurnDegree(180);
  }
};

RobotController* robot_controller;

const int ENA = 5;
const int ENB = 6;
const int IN1 = 8;
const int IN2 = 7;
const int IN3 = 9;
const int IN4 = 10;
const int TRIG = 4;
const int ECHO = 3;
void setup() {
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  WheelController* left_wheel = new WheelController(IN1, IN2, ENA);
  WheelController* right_wheel = new WheelController(IN3, IN4, ENB);
  right_wheel->SetOffset(25);
  robot_controller = new RobotController(left_wheel, right_wheel);
  robot_controller->SetSpeed(100);

  robot_controller->MoveForward();

  /*
  robot_controller->TurnLeft();
  delay(2000);
  robot_controller->TurnDegree(45);
  */
}

int distance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH);
  int s = t / 58;
  return s;
}

int time = 0;
void loop() {

  if (time >= 10000) {
    robot_controller->Stop();
    return;
  }
  time += 1;

  int d = distance();
  Serial.println(d);
  
  if (d <= 20)
    robot_controller->SetSpeed(50);
  else {
    robot_controller->SetSpeed(100);
  }

  if (d <= 10) {
    robot_controller->Stop();
    delay(1000);
    robot_controller->SetSpeed(100);
    robot_controller->TurnLeft();
    delay(1000);
    robot_controller->MoveForward();
    time += 2000;
  }
  
}