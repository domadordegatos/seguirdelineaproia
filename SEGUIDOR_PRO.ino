#include <QTRSensors.h>
#include <WiFi.h>
#include <WebServer.h>

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

// --- WIFI / WEB ---
const char* WIFI_AP_SSID = "SEGUIDOR_PRO";
const char* WIFI_AP_PASS = "12345678";
WebServer server(80);

const uint8_t WEB_LOG_CAP = 80;
String webLogs[WEB_LOG_CAP];
uint8_t webLogHead = 0;
uint8_t webLogCount = 0;

String ultimaAccion = "INIT";
uint16_t ultimaPosicion = 2500;
int ultimoError = 0;
int ultimaVelIzq = 0;
int ultimaVelDer = 0;
uint16_t ultimoMaximo = 0;
uint8_t ultimoMedios = 0;
uint8_t ultimoAltos = 0;
bool ultimaLineaVisible = false;
uint32_t ultimoRegistroWebMs = 0;
bool ledWebBlinkActivo = false;
uint8_t ledWebBlinkPendientes = 0;
uint32_t ledWebBlinkUltimoMs = 0;
const uint16_t LED_WEB_BLINK_INTERVAL_MS = 90;

const char WEB_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no" />
  <title>Seguidor PRO</title>
  <style>
    :root { --bg:#0f1720; --card:#1b2430; --fg:#e7edf4; --muted:#9ab; --ok:#2ecc71; --warn:#f1c40f; }
    body { margin:0; font-family:Verdana, sans-serif; background:linear-gradient(180deg,#111923,#0b1118); color:var(--fg); touch-action:manipulation; }
    .wrap { max-width:1000px; margin:0 auto; padding:16px; }
    .grid { display:grid; gap:12px; grid-template-columns:repeat(auto-fit,minmax(280px,1fr)); }
    .card { background:var(--card); border-radius:10px; padding:12px; box-shadow:0 3px 12px rgba(0,0,0,.25); }
    h1,h2 { margin:6px 0 10px; }
    label { display:block; margin:8px 0 4px; color:var(--muted); font-size:13px; }
    input { width:100%; padding:8px; border-radius:8px; border:1px solid #334; background:#0f1620; color:#fff; text-align:center; }
    button { padding:10px 12px; border:0; border-radius:8px; background:#2d89ef; color:#fff; cursor:pointer; font-size:18px; }
    .kv { display:grid; grid-template-columns:120px 1fr; gap:4px 8px; font-size:14px; }
    .mono { font-family:Consolas, monospace; font-size:13px; white-space:pre-wrap; }
    textarea { width:100%; min-height:260px; resize:vertical; border-radius:8px; border:1px solid #334; background:#0f1620; color:#e8f0f8; padding:8px; }
    .ctl { display:grid; grid-template-columns:56px 1fr 56px; gap:8px; align-items:center; margin-bottom:8px; }
    .ctl button { height:42px; }
    .step { color:var(--muted); font-size:12px; margin-top:-4px; margin-bottom:8px; }
    .ok { color:var(--ok); }
    .warn { color:var(--warn); }
  </style>
</head>
<body>
  <div class="wrap">
    <h1>Seguidor PRO - Panel WiFi</h1>
    <div class="grid">
      <div class="card">
        <h2>PID / Control</h2>
        <label>Kp</label>
        <div class="ctl"><button onclick="ajustar('kp',-0.01)">-</button><input id="kp" type="number" step="0.01" /><button onclick="ajustar('kp',0.01)">+</button></div>
        <div class="step">Paso: 0.01</div>

        <label>Ki</label>
        <div class="ctl"><button onclick="ajustar('ki',-0.001)">-</button><input id="ki" type="number" step="0.001" /><button onclick="ajustar('ki',0.001)">+</button></div>
        <div class="step">Paso: 0.001</div>

        <label>Kd</label>
        <div class="ctl"><button onclick="ajustar('kd',-0.01)">-</button><input id="kd" type="number" step="0.01" /><button onclick="ajustar('kd',0.01)">+</button></div>
        <div class="step">Paso: 0.01</div>

        <label>Vel Base</label>
        <div class="ctl"><button onclick="ajustar('velBase',-2)">-</button><input id="velBase" type="number" step="1" /><button onclick="ajustar('velBase',2)">+</button></div>
        <div class="step">Paso: 2</div>

        <label>Vel Max</label>
        <div class="ctl"><button onclick="ajustar('velMax',-2)">-</button><input id="velMax" type="number" step="1" /><button onclick="ajustar('velMax',2)">+</button></div>
        <div class="step">Paso: 2</div>

        <label>Setpoint</label>
        <div class="ctl"><button onclick="ajustar('setpoint',-5)">-</button><input id="setpoint" type="number" step="1" /><button onclick="ajustar('setpoint',5)">+</button></div>
        <div class="step">Paso: 5</div>

        <div id="saveMsg" class="mono"></div>
      </div>
      <div class="card">
        <h2>Estado</h2>
        <div class="kv">
          <div>Modo</div><div id="modo" class="mono"></div>
          <div>Accion</div><div id="accion" class="mono"></div>
          <div>Posicion</div><div id="pos" class="mono"></div>
          <div>Error</div><div id="err" class="mono"></div>
          <div>Motores</div><div id="motores" class="mono"></div>
          <div>Linea</div><div id="linea" class="mono"></div>
          <div>Sensores</div><div id="sens" class="mono"></div>
        </div>
      </div>
    </div>
    <div class="card" style="margin-top:12px;">
      <h2>Log (copiable)</h2>
      <textarea id="logs" readonly></textarea>
    </div>
  </div>

<script>
async function getJSON(url){ const r = await fetch(url); return await r.json(); }
async function getText(url){ const r = await fetch(url); return await r.text(); }

function decimales(campo){
  if (campo === 'ki') return 3;
  if (campo === 'kp' || campo === 'kd') return 2;
  return 0;
}

async function enviarCampo(campo, valor){
  const form = new URLSearchParams();
  form.set(campo, String(valor));
  const r = await fetch('/set', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:form.toString()
  });
  document.getElementById('saveMsg').textContent = await r.text();
  setTimeout(()=>document.getElementById('saveMsg').textContent='', 1200);
}

async function ajustar(campo, delta){
  const el = document.getElementById(campo);
  const cur = Number(el.value || 0);
  const nd = decimales(campo);
  const next = Number((cur + delta).toFixed(nd));
  el.value = next;
  await enviarCampo(campo, next);
}

async function refrescarEstado(){
  try {
    const s = await getJSON('/status');
    document.getElementById('modo').textContent = s.modo;
    document.getElementById('accion').textContent = s.accion;
    document.getElementById('pos').textContent = s.posicion;
    document.getElementById('err').textContent = s.error;
    document.getElementById('motores').textContent = s.velIzq + ' / ' + s.velDer;
    document.getElementById('linea').textContent = (s.lineaVisible ? 'VISIBLE' : 'PERDIDA') + ' | max=' + s.maximo + ' med=' + s.medios + ' alt=' + s.altos;
    document.getElementById('sens').textContent = s.sensores.join(' ');

    if (document.activeElement.id !== 'kp') document.getElementById('kp').value = s.Kp;
    if (document.activeElement.id !== 'ki') document.getElementById('ki').value = s.Ki;
    if (document.activeElement.id !== 'kd') document.getElementById('kd').value = s.Kd;
    if (document.activeElement.id !== 'velBase') document.getElementById('velBase').value = s.velBase;
    if (document.activeElement.id !== 'velMax') document.getElementById('velMax').value = s.velMax;
    if (document.activeElement.id !== 'setpoint') document.getElementById('setpoint').value = s.setpoint;
  } catch(e) {}
}

async function refrescarLogs(){
  try {
    const txt = await getText('/logs');
    const ta = document.getElementById('logs');
    ta.value = txt;
    ta.scrollTop = ta.scrollHeight;
  } catch(e) {}
}

setInterval(refrescarEstado, 400);
setInterval(refrescarLogs, 900);
refrescarEstado();
refrescarLogs();
</script>
</body>
</html>
)HTML";

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
const char* nombreModo(ModoRobot modo);
void agregarLogWeb(const String& linea);
void registrarEstado(const char* accion, int pos, int err, int velI, int velD, uint16_t maximo, uint8_t medios, uint8_t altos, bool lineaVisible);
void iniciarWeb();
void atenderWeb();
void handleRoot();
void handleStatus();
void handleSet();
void handleLogs();
void iniciarParpadeoLedWeb();
void actualizarParpadeoLedWeb();

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

  iniciarWeb();

  digitalWrite(PINLED, HIGH);
  Serial.println("Modo inicial: LECTURA. Presiona P1 para calibrar y autocentrar.");
  agregarLogWeb("Sistema listo. Modo LECTURA.");
}

void loop() {
  atenderWeb();
  actualizarParpadeoLedWeb();

  if (modoActual == MODO_LECTURA) {
    qtr.read(sensorValues);
    imprimirLecturasLinea();

    uint16_t maximo = 0;
    for (uint8_t i = 0; i < SensorCount; i++) if (sensorValues[i] > maximo) maximo = sensorValues[i];
    registrarEstado("LECTURA", setpoint, 0, 0, 0, maximo, 0, 0, false);

    if (botonPresionado(PINBOTON1)) {
      calibrarSensores();
      pidInicializado = true;

      // Reiniciar terminos PID al arrancar control.
      lastError = 0;
      integral = 0;
      modoActual = MODO_CENTRADO;
      Serial.println("CENTRADO ACTIVO. Coloca el robot sobre la linea. P2: iniciar PID.");
      agregarLogWeb("Entrando a MODO_CENTRADO.");
    }

    delay(50);
    return;
  }

  if (modoActual == MODO_CENTRADO) {
    mantenerCentroEnSitio();

    if (botonPresionado(PINBOTON2)) {
      lastError = 0;
      integral = 0;
      modoActual = MODO_PID;
      Serial.println("PID ACTIVO. Presiona P2 para detener y volver a ESPERA.");
      agregarLogWeb("Entrando a MODO_PID.");
      delay(200);
    }
    return;
  }

  // MODO_PID

  if (botonPresionado(PINBOTON2)) {
    modoActual = MODO_LECTURA;
    detenerMotores();
    Serial.println("LECTURA ACTIVA (ESPERA). Presiona P1 para volver a calibrar/sensar.");
    agregarLogWeb("PID detenido. Volviendo a MODO_LECTURA.");
    delay(200);
    return;
  }

  uint16_t position = qtr.readLineBlack(sensorValues);

  // Estrategia de memoria fuera de linea
  if (position == 0) {
    moverMotores(-velMax, velMax);
    registrarEstado("PID_FUERA_IZQ", 0, -2500, -velMax, velMax, 0, 0, 0, false);
    return;
  } else if (position == 5000) {
    moverMotores(velMax, -velMax);
    registrarEstado("PID_FUERA_DER", 5000, 2500, velMax, -velMax, 0, 0, 0, false);
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
  uint16_t maximo = 0;
  for (uint8_t i = 0; i < SensorCount; i++) if (sensorValues[i] > maximo) maximo = sensorValues[i];
  registrarEstado("PID", position, error, velIzq, velDer, maximo, 0, 0, true);
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
    atenderWeb();
    actualizarParpadeoLedWeb();

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
  agregarLogWeb("Calibracion finalizada.");
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
    registrarEstado("CENTRADO_LEVANTADO", setpoint, 0, 0, 0, maximo, sensoresMedios, sensoresAltos, false);
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
    registrarEstado("CENTRADO_OK", posImpresion, error, 0, 0, maximo, sensoresMedios, sensoresAltos, true);
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

  int velICom = girarDerecha ? velCorreccion : -velCorreccion;
  int velDCom = girarDerecha ? -velCorreccion : velCorreccion;
  registrarEstado(lineaVisible ? "CENTRADO_CORR" : "CENTRADO_BUSQ", posImpresion, error, velICom, velDCom, maximo, sensoresMedios, sensoresAltos, lineaVisible);

  delay(20);
}

const char* nombreModo(ModoRobot modo) {
  switch (modo) {
    case MODO_LECTURA: return "LECTURA";
    case MODO_CENTRADO: return "CENTRADO";
    case MODO_PID: return "PID";
    default: return "DESCONOCIDO";
  }
}

void agregarLogWeb(const String& linea) {
  webLogs[webLogHead] = linea;
  webLogHead = (webLogHead + 1) % WEB_LOG_CAP;
  if (webLogCount < WEB_LOG_CAP) webLogCount++;
}

void registrarEstado(const char* accion, int pos, int err, int velI, int velD, uint16_t maximo, uint8_t medios, uint8_t altos, bool lineaVisible) {
  ultimaAccion = accion;
  ultimaPosicion = (uint16_t)constrain(pos, 0, 5000);
  ultimoError = err;
  ultimaVelIzq = velI;
  ultimaVelDer = velD;
  ultimoMaximo = maximo;
  ultimoMedios = medios;
  ultimoAltos = altos;
  ultimaLineaVisible = lineaVisible;

  // Limitar frecuencia para no saturar memoria del log web.
  if (millis() - ultimoRegistroWebMs >= 140) {
    ultimoRegistroWebMs = millis();
    String linea = String(nombreModo(modoActual)) + " | " + accion +
                   " pos=" + String(ultimaPosicion) +
                   " err=" + String(ultimoError) +
                   " velI=" + String(ultimaVelIzq) +
                   " velD=" + String(ultimaVelDer) +
                   " max=" + String(ultimoMaximo);
    agregarLogWeb(linea);
  }
}

void iniciarWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/logs", HTTP_GET, handleLogs);
  server.begin();

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP activo: ");
  Serial.print(WIFI_AP_SSID);
  Serial.print(" IP: ");
  Serial.println(ip);
}

void atenderWeb() {
  server.handleClient();
}

void handleRoot() {
  server.send_P(200, "text/html", WEB_PAGE);
}

void handleStatus() {
  String sensores = "[";
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensores += String(sensorValues[i]);
    if (i + 1 < SensorCount) sensores += ",";
  }
  sensores += "]";

  String json = "{";
  json += "\"modo\":\"" + String(nombreModo(modoActual)) + "\",";
  json += "\"accion\":\"" + ultimaAccion + "\",";
  json += "\"posicion\":" + String(ultimaPosicion) + ",";
  json += "\"error\":" + String(ultimoError) + ",";
  json += "\"velIzq\":" + String(ultimaVelIzq) + ",";
  json += "\"velDer\":" + String(ultimaVelDer) + ",";
  json += "\"lineaVisible\":" + String(ultimaLineaVisible ? "true" : "false") + ",";
  json += "\"maximo\":" + String(ultimoMaximo) + ",";
  json += "\"medios\":" + String(ultimoMedios) + ",";
  json += "\"altos\":" + String(ultimoAltos) + ",";
  json += "\"Kp\":" + String(Kp, 4) + ",";
  json += "\"Ki\":" + String(Ki, 4) + ",";
  json += "\"Kd\":" + String(Kd, 4) + ",";
  json += "\"velBase\":" + String(velBase) + ",";
  json += "\"velMax\":" + String(velMax) + ",";
  json += "\"setpoint\":" + String(setpoint) + ",";
  json += "\"sensores\":" + sensores;
  json += "}";

  server.send(200, "application/json", json);
}

void handleSet() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  if (server.hasArg("velBase")) velBase = constrain(server.arg("velBase").toInt(), 0, 255);
  if (server.hasArg("velMax")) velMax = constrain(server.arg("velMax").toInt(), 0, 255);
  if (server.hasArg("setpoint")) setpoint = constrain(server.arg("setpoint").toInt(), 0, 5000);

  iniciarParpadeoLedWeb();

  String msg = "OK Kp=" + String(Kp, 3) +
               " Ki=" + String(Ki, 3) +
               " Kd=" + String(Kd, 3) +
               " velBase=" + String(velBase) +
               " velMax=" + String(velMax) +
               " setpoint=" + String(setpoint);
  agregarLogWeb("WEB_SET: " + msg);
  server.send(200, "text/plain", msg);
}

void iniciarParpadeoLedWeb() {
  ledWebBlinkActivo = true;
  ledWebBlinkPendientes = 4; // 2 parpadeos: OFF/ON/OFF/ON
  ledWebBlinkUltimoMs = millis();
}

void actualizarParpadeoLedWeb() {
  if (!ledWebBlinkActivo) return;
  if (millis() - ledWebBlinkUltimoMs < LED_WEB_BLINK_INTERVAL_MS) return;

  ledWebBlinkUltimoMs = millis();
  digitalWrite(PINLED, !digitalRead(PINLED));

  if (ledWebBlinkPendientes > 0) {
    ledWebBlinkPendientes--;
  }

  if (ledWebBlinkPendientes == 0) {
    ledWebBlinkActivo = false;
    digitalWrite(PINLED, HIGH);
  }
}

void handleLogs() {
  String out;
  uint8_t inicio = (webLogHead + WEB_LOG_CAP - webLogCount) % WEB_LOG_CAP;
  for (uint8_t i = 0; i < webLogCount; i++) {
    uint8_t idx = (inicio + i) % WEB_LOG_CAP;
    out += webLogs[idx] + "\n";
  }
  server.send(200, "text/plain", out);
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
