#define BLYNK_TEMPLATE_ID "TMPL6Wnxd_vR2"
#define BLYNK_TEMPLATE_NAME "Smart Shop"
#define BLYNK_AUTH_TOKEN "5k1Ubk1Xs_sS2BOvL8MkXAgqePrTXnE6"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// ==== WIFI ====
char ssid[] = "Quang Hai 2";
char pass[] = "thanhlathanh2k5";

// ==== KHAI BÁO CHÂN ====
#define TRIG_IN 14
#define ECHO_IN 27
#define TRIG_OUT 26
#define ECHO_OUT 25
#define BUZZER_PIN 15
#define LED_PIN 32
#define SERVO_PIN 4
#define LDR_PIN 34

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo doorServo;

// ==== FSM ====
enum State {
  IDLE,
  DETECT_IN,
  DETECT_OUT,
  SHOW_BYE,
  SHOW_HELLO
};
State state = IDLE;
unsigned long stateStartTime = 0;
unsigned long byeStartTime = 0;
unsigned long helloStartTime = 0;
unsigned long lastProcessedTime = 0;

const unsigned long DETECT_TIMEOUT = 3000;
const unsigned long BYE_DURATION = 4000;
const unsigned long COOLDOWN_TIME = 2000;

int peopleCount = 0;
bool isDoorLocked = false;
bool isShopClosed = false;
bool manualLight = false;
bool lightIsOn = false;

// ==== MỞ CỬA HẸN GIỜ ====
bool doorIsOpening = false;
unsigned long doorOpenStartTime = 0;
const unsigned long DOOR_OPEN_DURATION = 4000;

#define LDR_THRESHOLD_ON 1600
#define LDR_THRESHOLD_OFF 1800
unsigned long lastLightCheck = 0;
const unsigned long LIGHT_CHECK_INTERVAL = 1000;

// ==== BLYNK ====
BlynkTimer timer;

BLYNK_WRITE(V0) {  // LightManual
  manualLight = param.asInt();
}

BLYNK_WRITE(V3) {  // DoorLock
  isDoorLocked = param.asInt();
  if (isDoorLocked && isShopClosed) {
    isShopClosed = false;
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Han hanh phuc vu!");
    Blynk.virtualWrite(V4, 0);
  }
}

BLYNK_WRITE(V4) {  // CloseShop
  isShopClosed = param.asInt();
  if (isShopClosed) {
    if (isDoorLocked) {
      isDoorLocked = false;
      Blynk.virtualWrite(V3, 0);
    }
    digitalWrite(LED_PIN, LOW);
    lightIsOn = false;
    lcd.clear();
    lcd.noBacklight();
    Blynk.virtualWrite(V5, 0);
  } else {
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Han hanh phuc vu!");
  }
}

void updateBlynk() {
  Blynk.virtualWrite(V2, peopleCount);
  Blynk.virtualWrite(V5, lightIsOn);
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  pinMode(TRIG_IN, OUTPUT);
  pinMode(ECHO_IN, INPUT);
  pinMode(TRIG_OUT, OUTPUT);
  pinMode(ECHO_OUT, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  doorServo.attach(SERVO_PIN);
  doorServo.write(5);  // Cửa đóng

  lcd.setCursor(0, 0);
  lcd.print("Han hanh phuc vu!");

  timer.setInterval(1000L, updateBlynk);
}

long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 15000);
  long distance = duration * 0.034 / 2;
  if (duration == 0 || distance < 2) return -1;
  return distance;
}

void buzz(int times, int duration = 200) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
}

void updateLight() {
  int ldr = analogRead(LDR_PIN);
  Serial.print("LDR: ");
  Serial.println(ldr);

  if (isShopClosed) {
    digitalWrite(LED_PIN, LOW);
    lightIsOn = false;
    return;
  }

  if (manualLight) {
    digitalWrite(LED_PIN, HIGH);
    lightIsOn = true;
  } else if (peopleCount > 0) {
    if (!lightIsOn && ldr < LDR_THRESHOLD_ON) {
      digitalWrite(LED_PIN, HIGH);
      lightIsOn = true;
    } else if (lightIsOn && ldr > LDR_THRESHOLD_OFF) {
      digitalWrite(LED_PIN, LOW);
      lightIsOn = false;
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    lightIsOn = false;
  }
}

void smoothMoveTo(int targetAngle) {
  int currentAngle = doorServo.read();
  if (targetAngle > currentAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      doorServo.write(angle);
      delay(10);  // Điều chỉnh mượt hơn hoặc nhanh hơn
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      doorServo.write(angle);
      delay(10);
    }
  }
}

void openDoor() {
  if (!doorIsOpening) {
    smoothMoveTo(90);  // Quay mượt tới 90 độ
    doorIsOpening = true;
    doorOpenStartTime = millis();
    Serial.println("==> Mo cua");
  }
}

void checkDoorAutoClose() {
  if (doorIsOpening && millis() - doorOpenStartTime >= DOOR_OPEN_DURATION) {
    smoothMoveTo(5);  // Quay mượt về 5 độ
    doorIsOpening = false;
    Serial.println("==> Dong cua");
  }
}

void loop() {
  Blynk.run();
  timer.run();
  checkDoorAutoClose();

  long distIn = readDistanceCM(TRIG_IN, ECHO_IN);
  long distOut = readDistanceCM(TRIG_OUT, ECHO_OUT);
  bool inDetected = (distIn > 0 && distIn < 10);
  bool outDetected = (distOut > 0 && distOut < 15);
  unsigned long now = millis();

  if (isDoorLocked || isShopClosed) {
    inDetected = false;
    outDetected = false;
  }

  if (state == IDLE && (now - lastProcessedTime < COOLDOWN_TIME)) {
    delay(50);
    return;
  }

  switch (state) {
    case IDLE:
      if (inDetected) {
        Serial.println("Time IN");
        state = DETECT_IN;
        stateStartTime = now;
      } else if (outDetected) {
        Serial.println("Time OUT");
        state = DETECT_OUT;
        stateStartTime = now;
      }
      break;

    case DETECT_IN:
      openDoor();

      if (outDetected) {
        peopleCount++;
        Serial.println("==> Co nguoi vao");
        Serial.print("So nguoi hien tai: ");
        Serial.println(peopleCount);

        buzz(1);
        helloStartTime = now;
        state = SHOW_HELLO;
        lastProcessedTime = now;
      } else if (now - stateStartTime > DETECT_TIMEOUT) {
        Serial.println("Timeout IN -> Huy");
        state = IDLE;
      }
      break;

    case DETECT_OUT:
      openDoor();

      // Hiển thị tạm biệt NGAY LẬP TỨC nếu chưa hiển thị
      static bool isFarewellShown = false;
      if (!isFarewellShown) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Tam biet va");
        lcd.setCursor(0, 1);
        lcd.print("hen gap lai!");
        isFarewellShown = true;
        Serial.println("LCD: Tam biet (mac dinh)");
      }

      // Chờ xác nhận có người ra thật
      if (inDetected) {
        if (peopleCount > 0) peopleCount--;
        Serial.println("==> Co nguoi ra");
        Serial.print("So nguoi hien tai: ");
        Serial.println(peopleCount);

        buzz(2);
        byeStartTime = now;
        state = SHOW_BYE;
        lastProcessedTime = now;
        isFarewellShown = false;  // reset cho lần sau
      } else if (now - stateStartTime > DETECT_TIMEOUT) {
        Serial.println("Timeout OUT -> Huy");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Han hanh phuc vu!");
        state = IDLE;
        isFarewellShown = false;  // reset luôn
      }
      break;

    case SHOW_HELLO:
      if (now - helloStartTime >= BYE_DURATION) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Han hanh phuc vu!");
        state = IDLE;
      }
      break;

    case SHOW_BYE:
      if (now - byeStartTime >= BYE_DURATION) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Han hanh phuc vu!");
        state = IDLE;
      }
      break;
  }

  if (now - lastLightCheck >= LIGHT_CHECK_INTERVAL) {
    lastLightCheck = now;
    updateLight();
  }

  delay(50);
}
