#include <WioLTEforArduino.h>
#include <stdio.h>
#include <Omron2SMPB02E_WIO.h>

#define INTERVAL        (9000)
#define RECEIVE_TIMEOUT (1000)
#define RECEIVE_MODE  0

// uncomment following line to use Temperature & Humidity sensor
#define SENSOR_PIN    (WIOLTE_D38)

WioLTE Wio;
Omron2SMPB02E_WIO sensor;

int flag;

void setup() {
  delay(200);

  SerialUSB.println("");
  SerialUSB.println("--- START ---------------------------------------------------");

  SerialUSB.println("### I/O Initialize.");
  Wio.Init();

  Wio.LedSetRGB(0, 180, 0);

  SerialUSB.println("### Power supply ON.");
  Wio.PowerSupplyLTE(true);
  Wio.PowerSupplyGrove(true);
  delay(500);

  SerialUSB.println("### Turn on or reset.");
  if (!Wio.TurnOnOrReset()) {
    SerialUSB.println("### ERROR! ###");
    return;
  }

  SerialUSB.println("### Connecting to \"soracom.io\".");
  if (!Wio.Activate("soracom.io", "sora", "sora")) {
    SerialUSB.println("### ERROR! ###");
    return;
  }

  //時刻同期(ntp.nict.jp)
  SerialUSB.print("### Sync time.(ntp.nict.jp)");
  if (!Wio.SyncTime("ntp.nict.jp")) {
    SerialUSB.println(" -> NG");
    return;
  }else{
    SerialUSB.println(" -> OK");
  }

  TemperatureAndHumidityBegin(SENSOR_PIN);

  // 絶対圧センサーを使用するための初期化処理
  sensor.begin();
  sensor.set_mode(MODE_NORMAL);
  delay(300);

  SerialUSB.println("### Setup completed.");

  Wio.LedSetRGB(0, 0, 0);
}

void loop() {
  Wio.LedSetRGB(0, 0, 0);

  char data[1024];

  float temp;
  float humi;

  int connectId;
  connectId = Wio.SocketOpen("uni.soracom.io", 23080, WIOLTE_UDP);

   // 温度情報を取得
  float tmp = sensor.read_temp();

  // 気圧情報を取得
  float pressure = sensor.read_pressure();

  if (!TemperatureAndHumidityRead(&temp, &humi)) {
    SerialUSB.println("ERROR!");
    goto err;
  }

  struct tm now;
  if (!Wio.GetTime(&now)) {
    SerialUSB.println("### ERROR! ###");
    goto err;
  }

  SerialUSB.print("UTC:");
  SerialUSB.print(asctime(&now));
  
  SerialUSB.print("Current humidity = ");
  SerialUSB.print(humi);
  SerialUSB.print("%  ");

  SerialUSB.print("temperature = ");
  SerialUSB.print(temp);
  SerialUSB.println("C");

  SerialUSB.print("temperature = ");
  SerialUSB.print(tmp);
  SerialUSB.print(" [degC]  ");

  SerialUSB.print("pressure = ");
  SerialUSB.print(pressure);
  SerialUSB.println(" [Pa]");

  if ((now.tm_min % 10 == 0) && (flag == 0)) {
    flag  = 1;
    // 送信データ生成
    sprintf(data, "{\"temperature_HDT11\":%f,\"humidity\":%f,\"absolute_pressure\":%f,\"temperature\":%f}", temp, humi, pressure, tmp);

    Wio.LedSetRGB(0, 0, 180);
    SerialUSB.println("### Open.");
    if (connectId < 0) {
     SerialUSB.println("### ERROR! ###");
     goto err;
    }

    SerialUSB.println("### Send.");
    SerialUSB.print("Send:");
    SerialUSB.print(data);
    SerialUSB.println("");

    if (!Wio.SocketSend(connectId, data)) {
      SerialUSB.println("### ERROR! ###");
      goto err_close;
     }  
  }

  if (now.tm_min % 10 != 0) {
    flag  = 0;
  }

  #if RECEIVE_MODE
    SerialUSB.println("### Receive.");
    int length;
    length = Wio.SocketReceive(connectId, data, sizeof (data), RECEIVE_TIMEOUT);
    if (length < 0) {
      SerialUSB.println("### ERROR! ###");
      goto err_close;
    }
    if (length == 0) {
      SerialUSB.println("### RECEIVE TIMEOUT! ###");
      goto err_close;
    }
    SerialUSB.print("Receive:");
    SerialUSB.print(data);
  #endif
  
  Wio.LedSetRGB(0, 0, 0);

err_close:
  SerialUSB.println("### Close.");
  if (!Wio.SocketClose(connectId)) {
    SerialUSB.println("### ERROR! ###");
    goto err;
  }

err:
  delay(INTERVAL);
}

////////////////////////////////////////////////////////////////////////////////////////
//

#ifdef SENSOR_PIN

int TemperatureAndHumidityPin;

void TemperatureAndHumidityBegin(int pin)
{
  TemperatureAndHumidityPin = pin;
  DHT11Init(TemperatureAndHumidityPin);
}

bool TemperatureAndHumidityRead(float* temperature, float* humidity)
{
  byte data[5];

  DHT11Start(TemperatureAndHumidityPin);
  for (int i = 0; i < 5; i++) data[i] = DHT11ReadByte(TemperatureAndHumidityPin);
  DHT11Finish(TemperatureAndHumidityPin);

  if(!DHT11Check(data, sizeof (data))) return false;
  if (data[1] >= 10) return false;
  if (data[3] >= 10) return false;

  *humidity = (float)data[0] + (float)data[1] / 10.0f;
  *temperature = (float)data[2] + (float)data[3] / 10.0f;

  return true;
}

#endif // SENSOR_PIN

////////////////////////////////////////////////////////////////////////////////////////
//

#ifdef SENSOR_PIN

void DHT11Init(int pin)
{
  digitalWrite(pin, HIGH);
  pinMode(pin, OUTPUT);
}

void DHT11Start(int pin)
{
  // Host the start of signal
  digitalWrite(pin, LOW);
  delay(18);

  // Pulled up to wait for
  pinMode(pin, INPUT);
  while (!digitalRead(pin)) ;

  // Response signal
  while (digitalRead(pin)) ;

  // Pulled ready to output
  while (!digitalRead(pin)) ;
}

byte DHT11ReadByte(int pin)
{
  byte data = 0;

  for (int i = 0; i < 8; i++) {
    while (digitalRead(pin)) ;

    while (!digitalRead(pin)) ;
    unsigned long start = micros();

    while (digitalRead(pin)) ;
    unsigned long finish = micros();

    if ((unsigned long)(finish - start) > 50) data |= 1 << (7 - i);
  }

  return data;
}

void DHT11Finish(int pin)
{
  // Releases the bus
  while (!digitalRead(pin)) ;
  digitalWrite(pin, HIGH);
  pinMode(pin, OUTPUT);
}

bool DHT11Check(const byte* data, int dataSize)
{
  if (dataSize != 5) return false;

  byte sum = 0;
  for (int i = 0; i < dataSize - 1; i++) {
    sum += data[i];
  }

  return data[dataSize - 1] == sum;
}

#endif // SENSOR_PIN

////////////////////////////////////////////////////////////////////////////////////////
