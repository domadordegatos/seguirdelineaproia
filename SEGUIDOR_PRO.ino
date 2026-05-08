#include <QTRSensors.h>

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

QTRSensors qtr;

// --- MAPEO DE PINES ESP32 ---
// Sensor QTR 1..6
#define SENSOR_QTR_1 36
#define SENSOR_QTR_2 39
#define SENSOR_QTR_3 34
#define SENSOR_QTR_4 35
#define SENSOR_QTR_5 32
#define SENSOR_QTR_6 33

// Puente H - Motor derecho
#define PWMA 25
#define AIN1 26
#define AIN2 27

// Puente H - Motor izquierdo
#define PWMB 13
#define BIN1 12
#define BIN2 14

// Botones y LED
#define PINBOTON1 16
#define PINBOTON2 17
#define PINLED 5

// Ajuste de cableado de botones:
// - Si usas resistencias pull-down externas y el boton conecta a 3V3 al presionar:
//   BOTON_MODO_PIN = INPUT y BOTON_NIVEL_PRESIONADO = HIGH
// - Si usas solo pull-up interno y el boton conecta a GND al presionar:
//   BOTON_MODO_PIN = INPUT_PULLUP y BOTON_NIVEL_PRESIONADO = LOW
const uint8_t BOTON_MODO_PIN = INPUT;
const uint8_t BOTON_NIVEL_PRESIONADO = HIGH;

// --- CONSTANTES PID ---
float Kp = 0.5;
float Kd = 2.0;
float Ki = 0.0;

// --- VELOCIDADES ---
int velBase = 150;
int velMax = 255;

// Ajuste de sentido de motores para compensar montaje/cableado.
// Si un motor gira al reves, cambia su valor a -1.
const int8_t INVERTIR_MOTOR_IZQ = 1;
const int8_t INVERTIR_MOTOR_DER = -1;

// Variables internas del control
int setpoint = 2500;
int lastError = 0;
int integral = 0;

enum ModoRobot {
  MODO_LECTURA,
  MODO_CENTRADO,
  MODO_PID
};

ModoRobot modoActual = MODO_LECTURA;
bool pidInicializado = false;

void moverMotores(int velIzquierda, int velDerecha);
void calibrarSensores();
void detenerMotores();
void frenarMotores();
void imprimirLecturasLinea();
void mantenerCentroEnSitio();
void girarEnSitio(int velocidad, bool haciaDerecha);
bool sensoresConBuenRango(uint16_t umbralRango);
bool botonPresionado(uint8_t pin);
int estadoBotonLogico(uint8_t pin);

void setup() {
  Serial.begin(115200);

  pinMode(PINLED, OUTPUT);
  pinMode(PINBOTON1, BOTON_MODO_PIN);
  pinMode(PINBOTON2, BOTON_MODO_PIN);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  detenerMotores();

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){
    SENSOR_QTR_1,
    SENSOR_QTR_2,
    SENSOR_QTR_3,
    SENSOR_QTR_4,
    SENSOR_QTR_5,
    SENSOR_QTR_6
  }, SensorCount);

  digitalWrite(PINLED, HIGH);
  Serial.println("Modo inicial: LECTURA. Presiona P1 para calibrar y autocentrar.");
}

void loop() {
  if (modoActual == MODO_LECTURA) {
    qtr.read(sensorValues);
    imprimirLecturasLinea();

    if (botonPresionado(PINBOTON1)) {
      calibrarSensores();
      pidInicializado = true;

      // Reiniciar terminos PID al arrancar control.
      lastError = 0;
      integral = 0;
      modoActual = MODO_CENTRADO;
      Serial.println("CENTRADO ACTIVO. Coloca el robot sobre la linea. P1: iniciar PID | P2: volver a LECTURA.");
    }

    delay(50);
    return;
  }

  if (botonPresionado(PINBOTON2)) {
    modoActual = MODO_LECTURA;
    detenerMotores();
    Serial.println("LECTURA ACTIVA. Presiona P1 para calibrar y autocentrar.");
    delay(200);
    return;
  }

  if (modoActual == MODO_CENTRADO) {
    mantenerCentroEnSitio();

    if (botonPresionado(PINBOTON1)) {
      lastError = 0;
      integral = 0;
      modoActual = MODO_PID;
      Serial.println("PID ACTIVO. Presiona P2 para volver a LECTURA.");
      delay(200);
    }
    return;
  }

  // MODO_PID

  uint16_t position = qtr.readLineBlack(sensorValues);

  // Estrategia de memoria fuera de linea
  if (position == 0) {
    moverMotores(-velMax, velMax);
    return;
  } else if (position == 5000) {
    moverMotores(velMax, -velMax);
    return;
  }

  int error = (int)position - setpoint;
  int derivada = error - lastError;
  integral += error;

  int ajuste = (Kp * error) + (Kd * derivada) + (Ki * integral);
  lastError = error;

  int velIzq = velBase + ajuste;
  int velDer = velBase - ajuste;

  if (velIzq > velMax) velIzq = velMax;
  if (velDer > velMax) velDer = velMax;
  if (velIzq < -velMax) velIzq = -velMax;
  if (velDer < -velMax) velDer = -velMax;

  moverMotores(velIzq, velDer);
}

void moverMotores(int velIzquierda, int velDerecha) {
  velIzquierda *= INVERTIR_MOTOR_IZQ;
  velDerecha *= INVERTIR_MOTOR_DER;

  velIzquierda = constrain(velIzquierda, -255, 255);
  velDerecha = constrain(velDerecha, -255, 255);

  // Motor izquierdo (BIN/PWMB)
  if (velIzquierda >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    velIzquierda = -velIzquierda;
  }
  analogWrite(PWMB, velIzquierda);

  // Motor derecho (AIN/PWMA)
  if (velDerecha >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    velDerecha = -velDerecha;
  }
  analogWrite(PWMA, velDerecha);
}

void girarEnSitio(int velocidad, bool haciaDerecha) {
  if (haciaDerecha) {
    // Izquierda adelante, derecha atras.
    moverMotores(velocidad, -velocidad);
  } else {
    // Izquierda atras, derecha adelante.
    moverMotores(-velocidad, velocidad);
  }
}

void calibrarSensores() {
  Serial.println("Calibrando sensores con barrido lineal (adelante/atras)...");

  const int velCalib = 28;
  const int velCalibImpulso = 48;
  const uint32_t ventanaImpulsoMs = 120;
  const uint32_t tramoMs = 1500;
  const uint32_t duracionMinimaMs = 7000;
  const uint32_t duracionMaximaMs = 18000;
  const uint32_t parpadeoRapidoMs = 60;
  const uint16_t umbralRangoSensor = 450;
  const uint16_t umbralExtremoRaw = 260;

  uint32_t inicio = millis();
  uint32_t ultimoCambioSentido = inicio;
  uint32_t ultimoParpadeo = inicio;
  bool moviendoAdelante = true;
  bool ledEncendido = false;
  bool calibracionOk = false;
  bool extremoIzquierdoVisto = false;
  bool extremoDerechoVisto = false;

  while (millis() - inicio < duracionMaximaMs) {
    if (millis() - ultimoCambioSentido >= tramoMs) {
      moviendoAdelante = !moviendoAdelante;
      ultimoCambioSentido = millis();
    }

    int velActual = velCalib;
    if (millis() - ultimoCambioSentido < ventanaImpulsoMs) {
      velActual = velCalibImpulso;
    }

    if (moviendoAdelante) {
      moverMotores(velActual, velActual);
    } else {
      moverMotores(-velActual, -velActual);
    }

    qtr.calibrate();
    qtr.read(sensorValues);

    // Imprimir raw cada ~500ms
    static uint32_t ultimoPrintCalib = 0;
    if (millis() - ultimoPrintCalib >= 500) {
      ultimoPrintCalib = millis();
      Serial.print(moviendoAdelante ? ">>" : "<<");
      Serial.print(" RAW:");
      for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(" S"); Serial.print(i + 1); Serial.print(":"); Serial.print(sensorValues[i]);
      }
      Serial.println();
    }

    if (sensorValues[0] >= umbralExtremoRaw || sensorValues[1] >= umbralExtremoRaw) {
      extremoIzquierdoVisto = true;
    }
    if (sensorValues[SensorCount - 1] >= umbralExtremoRaw || sensorValues[SensorCount - 2] >= umbralExtremoRaw) {
      extremoDerechoVisto = true;
    }

    if (millis() - inicio >= duracionMinimaMs) {
      calibracionOk = sensoresConBuenRango(umbralRangoSensor);
      if (calibracionOk && extremoIzquierdoVisto && extremoDerechoVisto) {
        break;
      }
    }

    if (millis() - ultimoParpadeo >= parpadeoRapidoMs) {
      ledEncendido = !ledEncendido;
      digitalWrite(PINLED, ledEncendido ? HIGH : LOW);
      ultimoParpadeo = millis();
    }

    delay(30);
  }

  detenerMotores();
  digitalWrite(PINLED, HIGH);

  if (calibracionOk && extremoIzquierdoVisto && extremoDerechoVisto) {
    Serial.println("Calibracion OK.");
  } else {
    Serial.println("Calibracion incompleta: revisa recorrido adelante/atras.");
  }

  // Imprimir tabla de calibracion
  Serial.println("--- Tabla calibracion (min / max / rango) ---");
  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t minV = qtr.calibrationOn.minimum ? qtr.calibrationOn.minimum[i] : 0;
    uint16_t maxV = qtr.calibrationOn.maximum ? qtr.calibrationOn.maximum[i] : 0;
    Serial.print("  S"); Serial.print(i + 1);
    Serial.print(": min="); Serial.print(minV);
    Serial.print(" max="); Serial.print(maxV);
    Serial.print(" rango="); Serial.println(maxV > minV ? maxV - minV : 0);
  }
  Serial.println("---------------------------------------------");

  Serial.println("Ahora levanta el robot, colocalo centrado sobre la linea y sueltalo.");
}

bool sensoresConBuenRango(uint16_t umbralRango) {
  if (!qtr.calibrationOn.initialized ||
      !qtr.calibrationOn.minimum ||
      !qtr.calibrationOn.maximum) {
    return false;
  }

  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t minV = qtr.calibrationOn.minimum[i];
    uint16_t maxV = qtr.calibrationOn.maximum[i];

    if (maxV <= minV) {
      return false;
    }

    if ((uint16_t)(maxV - minV) < umbralRango) {
      return false;
    }
  }

  return true;
}

// Imprime los 6 valores calibrados en una sola linea con etiqueta.
static void imprimirVals(const char* etiqueta) {
  Serial.print(etiqueta);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(" S"); Serial.print(i + 1); Serial.print(":"); Serial.print(sensorValues[i]);
  }
  Serial.println();
}

void mantenerCentroEnSitio() {
  const int umbralCentro        = 180;
  const int velCorreccionMin    = 50;
  const int velCorreccionMax    = 76;
  const int velRecuperacionExtremo = 60;
  const bool invertirSentidoCentrado = true;
  const uint16_t umbralLineaMedia = 350;
  const uint16_t umbralLineaAlta = 600;
  const uint16_t umbralRobotLevantado = 20;
  const uint16_t tiempoConfirmacionLevantadoMs = 300;
  const uint16_t ciclosCambioBarrido = 25;

  // Memoria explicita: se actualiza solo cuando la linea es visible de verdad.
  static int ultimaPosicionConocida = setpoint;
  static uint32_t inicioPosibleLevantadoMs = 0;
  static uint16_t ciclosSinLinea = 0;
  static int8_t direccionBusqueda = 1; // 1: derecha, -1: izquierda

  // readCalibrated no modifica la memoria interna de readLineBlack
  qtr.readCalibrated(sensorValues);

  uint16_t maximo = 0;
  uint8_t sensoresMedios = 0;
  uint8_t sensoresAltos = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t v = sensorValues[i];
    if (v > maximo) maximo = v;
    if (v >= umbralLineaMedia) sensoresMedios++;
    if (v >= umbralLineaAlta) sensoresAltos++;
  }

  // Siempre imprime los valores raw calibrados para diagnostico
  Serial.print("VALS:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(" S"); Serial.print(i + 1); Serial.print(":"); Serial.print(sensorValues[i]);
  }
  Serial.print(" max="); Serial.print(maximo);
  Serial.print(" med="); Serial.print(sensoresMedios);
  Serial.print(" alt="); Serial.print(sensoresAltos);

  // Solo considerar LEVANTADO si se sostiene por un tiempo continuo.
  if (maximo < umbralRobotLevantado) {
    if (inicioPosibleLevantadoMs == 0) {
      inicioPosibleLevantadoMs = millis();
    }
  } else {
    inicioPosibleLevantadoMs = 0;
  }

  if (inicioPosibleLevantadoMs != 0 && (millis() - inicioPosibleLevantadoMs) >= tiempoConfirmacionLevantadoMs) {
    detenerMotores();
    Serial.println(" -> LEVANTADO");
    delay(20);
    return;
  }

  // Linea visible robusta: 1 sensor muy alto o al menos 2 sensores medios.
  bool lineaVisible = (sensoresAltos >= 1) || (sensoresMedios >= 2);

  int error;
  uint16_t posImpresion;

  if (lineaVisible) {
    ciclosSinLinea = 0;
    posImpresion = qtr.readLineBlack(sensorValues);
    error = (int)posImpresion - setpoint;
    ultimaPosicionConocida = (int)posImpresion;  // actualizar solo con linea real
    direccionBusqueda = (error >= 0) ? 1 : -1;
  } else {
    // Linea perdida: primero buscar hacia el ultimo lado conocido.
    // Si no reaparece, alternar direccion para barrer y no quedarse clavado.
    ciclosSinLinea++;
    if (ciclosSinLinea == 1) {
      direccionBusqueda = (ultimaPosicionConocida >= setpoint) ? 1 : -1;
    }
    if ((ciclosSinLinea % ciclosCambioBarrido) == 0) {
      direccionBusqueda = -direccionBusqueda;
    }
    posImpresion = (uint16_t)ultimaPosicionConocida;
    error = (direccionBusqueda > 0) ? 2500 : -2500;
  }

  int magnitudError = abs(error);

  Serial.print(lineaVisible ? " [VIS]" : " [PERD]");
  Serial.print(" pos="); Serial.print(posImpresion);
  Serial.print(" err="); Serial.print(error);

  // Solo CENTRADO si la linea ES visible Y error es pequenio
  if (lineaVisible && magnitudError <= umbralCentro) {
    detenerMotores();
    Serial.println(" -> CENTRADO OK");
    delay(20);
    return;
  }

  int velCorreccion;
  if (!lineaVisible) {
    velCorreccion = velCorreccionMin;
    Serial.print(error > 0 ? " -> BUSCANDO_DER vel=" : " -> BUSCANDO_IZQ vel=");
    Serial.println(velCorreccion);
  } else {
    bool enExtremo = (posImpresion < 400 || posImpresion > 4600);
    if (enExtremo) {
      // En el borde del arreglo, corregir mas lento evita que se escape mas.
      velCorreccion = velRecuperacionExtremo;
    } else {
      velCorreccion = map(magnitudError, umbralCentro, 2500, velCorreccionMin, velCorreccionMax);
      velCorreccion = constrain(velCorreccion, velCorreccionMin, velCorreccionMax);
    }
    Serial.print(error > 0 ? " -> CORR_DER vel=" : " -> CORR_IZQ vel=");
    Serial.println(velCorreccion);
  }

  // En algunos montajes el signo de correccion queda invertido aunque el PID avance bien.
  // Este selector permite corregir sin tocar el resto del firmware.
  bool girarDerecha = (error > 0);
  if (invertirSentidoCentrado) {
    girarDerecha = !girarDerecha;
  }
  girarEnSitio(velCorreccion, girarDerecha);

  delay(20);
}

void detenerMotores() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// Frenado activo: ambas entradas HIGH en TB6612 = cortocircuito controlado (brake mode).
// Detiene el motor mucho mas rapido que coast (evita inercia al centrar).
void frenarMotores() {
  analogWrite(PWMA, 255);
  analogWrite(PWMB, 255);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

void imprimirLecturasLinea() {
  int p1 = estadoBotonLogico(PINBOTON1);
  int p2 = estadoBotonLogico(PINBOTON2);

  Serial.print("P1:");
  Serial.print(p1);
  Serial.print(" P2:");
  Serial.print(p2);

  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(" S");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(sensorValues[i]);
  }
  Serial.println();
}

bool botonPresionado(uint8_t pin) {
  if (estadoBotonLogico(pin) == 1) {
    delay(30);
    while (estadoBotonLogico(pin) == 1) {
      delay(1);
    }
    return true;
  }
  return false;
}

int estadoBotonLogico(uint8_t pin) {
  return (digitalRead(pin) == BOTON_NIVEL_PRESIONADO) ? 1 : 0;
}
