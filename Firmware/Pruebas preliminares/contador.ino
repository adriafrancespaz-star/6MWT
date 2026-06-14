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

// UUIDs del servicio y características BLE
#define SERVICE_UUID    "12345678-1234-1234-1234-123456789012"
#define CMD_CHAR_UUID   "12345678-1234-1234-1234-123456789013" // Escritura (móvil → ESP32)
#define STEPS_CHAR_UUID "12345678-1234-1234-1234-123456789014" // Notificación (ESP32 → móvil)

// Parámetros de detección de pasos
const float    UMBRAL_SUBIDA  = 1.5;  // m/s² — umbral para detectar inicio de pico
const float    UMBRAL_BAJADA  = 0.8;  // m/s² — umbral para confirmar el pico (histéresis)
const unsigned long DEBOUNCE_MS     = 300;  // ms mínimos entre pasos (~200 pasos/min máx)
const unsigned long INTERVALO_IMU   = 20;   // ms entre lecturas del IMU (50 Hz)
const unsigned long INTERVALO_NOTIF = 1000; // ms entre notificaciones BLE

// ============================================================
// VARIABLES GLOBALES
// ============================================================

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

BLEServer*         pServer    = nullptr;
BLECharacteristic* pStepsChar = nullptr;

bool deviceConnected = false;
bool midiendo        = false;

int           pasosTotales  = 0;
bool          enPico        = false;
unsigned long ultimoPaso    = 0;
unsigned long tiempoInicio  = 0;
unsigned long ultimaLectura = 0;
unsigned long ultimaNotif   = 0;

// ============================================================
// BLOQUE 1 — CALLBACKS BLE
// ============================================================

// Gestión de conexión/desconexión
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

// Recepción de comandos desde el móvil
class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = pChar->getValue().c_str();
    cmd.trim();
    Serial.print("[CMD] Recibido: ");
    Serial.println(cmd);

    if (cmd == "Go" || cmd == "Iniciar prueba") {
      pasosTotales = 0;
      enPico       = false;
      ultimoPaso   = 0;
      tiempoInicio = millis();
      midiendo     = true;
      Serial.println("[CMD] >> Medicion iniciada.");

    } else if (cmd == "Stop" || cmd == "Parar") {
      midiendo = false;
      Serial.println("[CMD] >> Medicion detenida.");
      enviarResultado(true); // envío final con etiqueta "FIN"
    }
  }

  // Declaramos enviarResultado para usarlo dentro del callback
  // (la definición real está más abajo como función global)
  void enviarResultado(bool esFinal);
};

// ============================================================
// BLOQUE 2 — ENVÍO DE RESULTADOS POR BLE
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

// Resolvemos la llamada dentro del callback
void CmdCallbacks::enviarResultado(bool esFinal) {
  ::enviarResultado(esFinal);
}

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
  BLEDevice::init("ESP32-Grup4");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Característica de comandos: el móvil escribe aquí
  BLECharacteristic* pCmdChar = pService->createCharacteristic(
    CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCmdChar->setCallbacks(new CmdCallbacks());

  // Característica de pasos: el ESP32 notifica aquí
  pStepsChar = pService->createCharacteristic(
    STEPS_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pStepsChar->addDescriptor(new BLE2902()); // necesario para suscribirse en nRF Connect

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("[BLE] Advertising iniciado. Esperando conexion y comando 'Go'...");
}

// ============================================================
// LOOP — BLOQUE 1: LECTURA IMU + BLOQUE 2: DETECCIÓN DE PASOS
// ============================================================

void loop() {
  if (!midiendo) return;

  unsigned long ahora = millis();

  // --- Lectura IMU a 50 Hz ---
  if (ahora - ultimaLectura >= INTERVALO_IMU) {
    ultimaLectura = ahora;

    // Aceleración lineal (gravedad compensada por el BNO055)
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Magnitud del vector 3D
    float mag = sqrt(accel.x() * accel.x() +
                     accel.y() * accel.y() +
                     accel.z() * accel.z());

    // --- Detección de pico con histéresis ---
    // Subida: la magnitud supera UMBRAL_SUBIDA → estamos en un pico
    if (!enPico && mag > UMBRAL_SUBIDA) {
      enPico = true;
    }
    // Bajada: la magnitud vuelve a bajar de UMBRAL_BAJADA → pico completado = 1 paso
    if (enPico && mag < UMBRAL_BAJADA) {
      enPico = false;
      if (ahora - ultimoPaso > DEBOUNCE_MS) {
        pasosTotales++;
        ultimoPaso = ahora;
        Serial.print("[PASO] Total: ");
        Serial.println(pasosTotales);
      }
    }
  }

  // --- Notificación periódica por BLE ---
  if (ahora - ultimaNotif >= INTERVALO_NOTIF) {
    ultimaNotif = ahora;
    enviarResultado(false);
  }
}