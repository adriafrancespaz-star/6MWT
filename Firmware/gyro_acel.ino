#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup(void) {
  Serial.begin(115200);
  delay(1000);

  Serial.println("--- INICIANDO SISTEMA ---");
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("Buscando BNO055...");
  if (!bno.begin()) {
    Serial.println("ERROR: No se detecta el BNO055.");
    Serial.println("1. Revisa que SDA esté en pin 21 y SCL en pin 22.");
    Serial.println("2. Revisa que el sensor tenga 3.3V y GND.");
    while (1) delay(10);
  }

  Serial.println("¡Sensor detectado con éxito!");
  bno.setExtCrystalUse(true);

  // Cabecera CSV
  Serial.println("tiempo_ms, acel_x, acel_y, acel_z, giro_x, giro_y, giro_z");
}

void loop(void) {
  static unsigned long ultimaMedicion = 0;
  const unsigned long INTERVALO = 200;

  unsigned long ahora = millis();

  if (ahora - ultimaMedicion >= INTERVALO) {
    ultimaMedicion = ahora;

    // Aceleración lineal (sin gravedad) en m/s²
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Velocidad angular del giroscopio en grados/segundo
    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // Imprimimos todo en una sola línea CSV
    Serial.print(ahora);
    Serial.print(", ");
    Serial.print(accel.x(), 4);
    Serial.print(", ");
    Serial.print(accel.y(), 4);
    Serial.print(", ");
    Serial.print(accel.z(), 4);
    Serial.print(", ");
    Serial.print(gyro.x(), 4);
    Serial.print(", ");
    Serial.print(gyro.y(), 4);
    Serial.print(", ");
    Serial.println(gyro.z(), 4);
  }
}