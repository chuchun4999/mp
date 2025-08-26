/*
 * mode 1: Obstacle Detection Mode
 * echopin 8
 * trigpin 7
 * leftIR A0
 * rightIR A1
 * neck servo 11
 * left servo 9                    
 * right servo 10
 */
#include <Servo.h>


// 초음파 모듈 핀 설정
int echoPin = 8;
int trigPin = 7;


// 서보모터 핀 설정
int leftServo = 9;
int rightServo = 10;
int neckServo = 11;


// 서보모터 라이브러리 객체 생성
Servo neck;
Servo leftWheel;
Servo rightWheel;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);


  // 초음파 모듈 핀 연결
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);


  // 서보모터 핀 연결
  neck.attach(neckServo);
  leftWheel.attach(leftServo);
  rightWheel.attach(rightServo);


  // 서보모터 초기 각도 설정
  neck.write(90);
  leftWheel.write(90);
  rightWheel.write(90);
}


void loop() {
  if (Serial.available() > 0) {
    // 들어온 데이터를 char 형식으로 읽음
    char command = Serial.read();

    // 입력된 명령어에 따라 동작 수행
    switch (command) {
      case 'W':
      case 'w':
        Serial.println("앞으로 이동");
        goForward();
        break;
      case 'S':
      case 's':
        Serial.println("뒤로 이동");
        goBackward();
        break;
      case 'A':
      case 'a':
        Serial.println("좌회전");
        leftTurn();
        break;
      case 'D':
      case 'd':
        Serial.println("우회전");
        rightTurn();
        break;
      case 'Q':
      case 'q':
        Serial.println("정지");
        stopMove();
        break;
    }
  }
}




// 전진 함수 : 왼쪽바퀴는 반시계방향, 오른쪽바퀴는 시계방향 회전
void goForward(){
  neck.write(90);
  leftWheel.write(140);
  rightWheel.write(70);
  Serial.println(F("go Forward"));
}
void goBackward(){
  neck.write(90);
  leftWheel.write(70);
  rightWheel.write(120);
  Serial.println(F("go Forward"));
  
}

// 좌회전 함수 : 왼쪽바퀴는 시계방향, 오른쪽바퀴는 시계방향 회전
void leftTurn(){
  leftWheel.write(80);
  rightWheel.write(65);
  Serial.println(F("Turn Left"));  
}


// 우회전 함수 : 왼쪽바퀴는 반시계방향, 오른쪽바퀴는 반시계방향 회전
void rightTurn(){
  leftWheel.write(140);
  rightWheel.write(93);
  Serial.println(F("Turn Right"));
}


// 정지 함수 : 왼쪽 바퀴, 오른쪽 바퀴 정지
void stopMove(){
  // stop moving
  leftWheel.write(90);
  rightWheel.write(90);
  Serial.println(F("stop"));
}
