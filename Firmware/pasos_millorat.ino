#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================
// CONFIGURACIÓN GENERAL
// ============================================================

#define I2C_SDA 21
#define I2C_SCL 22

#define SERVICE_UUID    "12345678-1234-1234-1234-123456789012"
#define CMD_CHAR_UUID   "12345678-1234-1234-1234-123456789013"
#define STEPS_CHAR_UUID "12345678-1234-1234-1234-123456789014"

// Parámetros de detección — ajusta UMBRAL_SUBIDA si sigue contando mal
const float        UMBRAL_SUBIDA  = 0.8;  // m/s² — umbral mínimo para considerar un pico
const unsigned long DEBOUNCE_MS   = 250;  // ms mínimos entre pasos
const unsigned long INTERVALO_IMU = 20;   // ms entre lecturas (50 Hz)
const unsigned long INTERVALO_NOTIF = 1000; // ms entre notificaciones BLE

// Filtro de media móvil
const int VENTANA_FILTRO = 5;

// ============================================================
// VARIABLES GLOBALES
// ============================================================

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

BLEServer*         pServer    = nullptr;
BLECharacteristic* pStepsChar = nullptr;

bool deviceConnected = false;
bool midiendo        = false;

int           pasosTotales    = 0;
unsigned long ultimoPaso      = 0;
unsigned long tiempoInicio    = 0;
unsigned long ultimaLectura   = 0;
unsigned long ultimaNotif     = 0;

// Detección por máximo local
float magAnterior      = 0;
float magAnteAnterior  = 0;

// Filtro de media móvil
float bufferMag[VENTANA_FILTRO] = {0};
int   idxFiltro = 0;

// ============================================================
// FILTRO DE MEDIA MÓVIL
// ============================================================

float aplicarFiltro(float nuevaMag) {
  bufferMag[idxFiltro] = nuevaMag;
  idxFiltro = (idxFiltro + 1) % VENTANA_FILTRO;
  float suma = 0;
  for (int i = 0; i < VENTANA_FILTRO; i++) suma += bufferMag[i];
  return suma / VENTANA_FILTRO;
}

// ============================================================
// ENVÍO DE RESULTADOS POR BLE
// ============================================================

void enviarResultado(bool esFinal) {
  if (!deviceConnected) return;

  unsigned long seg = (millis() - tiempoInicio) / 1000;
  char buffer[80];

  if (esFinal) {
    snprintf(buffer, sizeof(buffer),
             "FIN | Pasos: %d | Tiempo: %lus | Cadencia: %.1f p/min",
             pasosTotales,
             seg,
             seg > 0 ? (pasosTotales / (float)seg) * 60.0f : 0.0f);
  } else {
    snprintf(buffer, sizeof(buffer),
             "Pasos: %d | t: %lus",
             pasosTotales, seg);
  }

  pStepsChar->setValue(buffer);
  pStepsChar->notify();
  Serial.print("[BLE] Enviado: ");
  Serial.println(buffer);
}

// ============================================================
// CALLBACKS BLE
// ============================================================

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[BLE] Cliente conectado.");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    midiendo        = false;
    Serial.println("[BLE] Cliente desconectado. Reiniciando advertising...");
    pServer->startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = pChar->getValue().c_str();
    cmd.trim();
    Serial.print("[CMD] Recibido: ");
    Serial.println(cmd);

    if (cmd == "Go" || cmd == "Iniciar prueba") {
      // Reset completo de todos los contadores y estado
      pasosTotales   = 0;
      ultimoPaso     = 0;
      magAnterior    = 0;
      magAnteAnterior = 0;
      idxFiltro      = 0;
      for (int i = 0; i < VENTANA_FILTRO; i++) bufferMag[i] = 0;
      tiempoInicio   = millis();
      midiendo       = true;
      Serial.println("[CMD] >> Medicion iniciada.");

    } else if (cmd == "Stop" || cmd == "Parar") {
      midiendo = false;
      Serial.println("[CMD] >> Medicion detenida.");
      enviarResultado(true);
    }
  }
};

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== PODOMETRO BLE + BNO055 ===");

  // --- Inicializar IMU ---
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 no detectado. Revisa el cableado.");
    while (1) delay(10);
  }
  bno.setExtCrystalUse(true);
  Serial.println("[IMU] BNO055 listo.");

  // --- Inicializar BLE ---
  BLEDevice::init("ESP32-Podometro");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Característica CMD: el móvil escribe aquí
  BLECharacteristic* pCmdChar = pService->createCharacteristic(
    CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdChar->setCallbacks(new CmdCallbacks());

  // Característica STEPS: el ESP32 notifica aquí
  pStepsChar = pService->createCharacteristic(
    STEPS_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pStepsChar->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("[BLE] Advertising iniciado. Esperando conexion y comando 'Go'...");
}

// ============================================================
// LOOP — LECTURA IMU + DETECCIÓN DE PASOS
// ============================================================

void loop() {
  if (!midiendo) return;

  unsigned long ahora = millis();

  // --- Lectura IMU a 50 Hz ---
  if (ahora - ultimaLectura >= INTERVALO_IMU) {
    ultimaLectura = ahora;

    // Aceleración lineal sin gravedad
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Magnitud del vector 3D → funciona en cualquier orientación del dispositivo
    float magRaw = sqrt(accel.x() * accel.x() +
                        accel.y() * accel.y() +
                        accel.z() * accel.z());

    // Aplicamos el filtro de media móvil para eliminar ruido
    float mag = aplicarFiltro(magRaw);

    // --- Detección por máximo local ---
    // Un paso se cuenta cuando la muestra anterior es mayor que sus
    // dos vecinas (pico) Y supera el umbral mínimo
    bool esPicoLocal = (magAnterior > mag) &&
                       (magAnterior > magAnteAnterior) &&
                       (magAnterior > UMBRAL_SUBIDA);

    if (esPicoLocal && (ahora - ultimoPaso > DEBOUNCE_MS)) {
      pasosTotales++;
      ultimoPaso = ahora;
      Serial.print("[PASO] Total: ");
      Serial.println(pasosTotales);
    }

    // Desplazamos la ventana de muestras
    magAnteAnterior = magAnterior;
    magAnterior     = mag;
  }

  // --- Notificación BLE cada segundo ---
  if (ahora - ultimaNotif >= INTERVALO_NOTIF) {
    ultimaNotif = ahora;
    enviarResultado(false);
  }
}