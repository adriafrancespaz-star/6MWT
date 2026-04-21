#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Definimos los pines I2C estándar del ESP32
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // 0x28 es la dirección por defecto

void setup(void) {
  // Iniciamos Serial a 115200. ¡Ajusta tu monitor a esta velocidad!
  Serial.begin(115200);
  delay(1000); 
  
  Serial.println("--- INICIANDO SISTEMA ---");

  // Forzamos al ESP32 a usar los pines correctos
  Serial.println("Configurando I2C...");
  Wire.begin(I2C_SDA, I2C_SCL);

  // Intentamos iniciar el sensor
  Serial.println("Buscando BNO055...");
  if (!bno.begin()) {
    Serial.println("ERROR: No se detecta el BNO055.");
    Serial.println("1. Revisa que SDA esté en pin 21 y SCL en pin 22.");
    Serial.println("2. Revisa que el sensor tenga 3.3V y GND.");
    while (1) delay(10); // Se detiene aquí si hay error
  }

  Serial.println("¡Sensor detectado con éxito!");
  
  // Usar cristal externo para mayor estabilidad
  bno.setExtCrystalUse(true);
}

void loop(void) {
  sensors_event_t event;
  bno.getEvent(&event);

  /* Mostramos la orientación (Ángulos de Euler) */
  Serial.print("Angulo Z (Heading): ");
  Serial.print(event.orientation.x, 2);
  Serial.print(" | Y (Pitch): ");
  Serial.print(event.orientation.y, 2);
  Serial.print(" | X (Roll): ");
  Serial.println(event.orientation.z, 2);

  delay(2000); 
}