#include <Arduino.h>
#include "Motor.h"

// ========== CONFIGURACIÓN DE MOTORES ==========
// Crear instancias de los 3 motores con los parámetros especificados
// CONVENCIÓN (del código que funciona):
//   - mover(+1) = horario = SUELTA cable = longitud AUMENTA
//   - mover(-1) = antihorario = JALA cable = longitud DISMINUYE
Motor motor1(18, 5, 25, 26, 25200);   // Configuración del código que funciona
Motor motor2(17, 16, 32, 33, 25200);  // Pines invertidos para que siga misma convención
Motor motor3(23, 19, 27, 4, 25200);   // Configuración del código que funciona

// ========== CINEMÁTICA INVERSA - STEWART PLATFORM ==========
// Diámetro de la polea medido con calibrador: 17.5 mm
// Radio de la polea del cable: 8.75 mm
#define RADIO_POLEA 8.75

// Factor de reducción mecánica
// El encoder mide el eje del motor, pero la polea tiene reducción
// Calculado empíricamente: 207° motor / 2mm cable = 103.5 grados/mm
// Con radio 8.75mm esperamos: 6.55 grados/mm
// Factor = 103.5 / 6.55 = 15.8
#define FACTOR_REDUCCION 0.063

// Conversión: ΔL (mm) a grados de rotación del motor
// Si no hay reducción: MM_A_GRADOS = 180 / (π × radio) ≈ 6.55 grados/mm
// Con reducción: dividir por el factor
#define MM_A_GRADOS ((180.0 / (PI * RADIO_POLEA)) * FACTOR_REDUCCION)

// Altura de la plataforma (mm)
#define H 435.0

// Coordenadas de las juntas proximales (base) - Motor 1
const float A1x = 60.79;
const float A1y = 69.14;
const float A1z = 19.27;

// Coordenadas de las juntas proximales (base) - Motor 2
const float A2x = -91.59;
const float A2y = 19.82;
const float A2z = 19.27;

// Coordenadas de las juntas proximales (base) - Motor 3
const float A3x = 23.48;
const float A3y = -92.72;
const float A3z = 19.27;

// Coordenadas de las juntas distales (plataforma móvil) - Motor 1
const float B1x = 84.2;
const float B1y = 92.47;
const float B1z = 24.05;

// Coordenadas de las juntas distales (plataforma móvil) - Motor 2
const float B2x = -122.18;
const float B2y = 26.69;
const float B2z = 24.05;

// Coordenadas de las juntas distales (plataforma móvil) - Motor 3
const float B3x = 40.88;
const float B3y = -130.9;
const float B3z = 24.05;

// Longitudes iniciales de los cables (mm)
// IMPORTANTE: Estas deben calcularse con cinemática inversa en posición neutra (0°, 0°)
// Li = calcularLongitudCable(Ai, Bi, H, 0, 0)
// Calculadas automáticamente en setup()
volatile float Li1 = 0.0;  // Se calculará en setup()
volatile float Li2 = 0.0;  // Se calculará en setup()
volatile float Li3 = 0.0;  // Se calculará en setup()

// Longitudes actuales de los cables (mm)
volatile float L1 = 0.0;
volatile float L2 = 0.0;
volatile float L3 = 0.0;

// Longitudes deseadas calculadas por cinemática inversa (mm)
float dL1 = 0.0;
float dL2 = 0.0;
float dL3 = 0.0;

// Ángulos de orientación de la plataforma (radianes)
float tx = 0.0;  // Rotación en X
float ty = 0.0;  // Rotación en Y

// Contadores de pulsos desde el inicio
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;

// Tolerancia para considerar que se alcanzó la longitud deseada (mm)
const float TOLERANCIA = 0.5;

// Variables para detectar si el encoder está funcionando
unsigned long ultimaActualizacion1 = 0;
unsigned long ultimaActualizacion2 = 0;
unsigned long ultimaActualizacion3 = 0;

// Tiempo máximo sin cambio en encoder (ms) antes de detener el motor
const unsigned long TIMEOUT_ENCODER = 100;

// ========== FUNCIONES DE CINEMÁTICA INVERSA ==========

/**
 * @brief Calcula la longitud del cable usando cinemática inversa
 * @param Ax, Ay, Az Coordenadas de la junta proximal (base)
 * @param Bx, By, Bz Coordenadas de la junta distal (plataforma móvil)
 * @param h Altura de la plataforma
 * @param tx Ángulo de rotación en X (radianes)
 * @param ty Ángulo de rotación en Y (radianes)
 * @return Longitud del cable (mm)
 */
float calcularLongitudCable(float Ax, float Ay, float Az, float Bx, float By, float Bz, float h, float tx, float ty) {
  float comp1 = pow(Ay - By*cos(tx) + Bz*sin(tx), 2);
  float comp2 = pow(h - Az - Bx*sin(ty) + Bz*cos(tx)*cos(ty) + By*cos(ty)*sin(tx), 2);
  float comp3 = pow(Bx*cos(ty) - Ax + Bz*cos(tx)*sin(ty) + By*sin(tx)*sin(ty), 2);
  
  return sqrt(comp1 + comp2 + comp3);
}

/**
 * @brief Actualiza las longitudes actuales basándose en los encoders
 * Usa la conversión: grados de rotación → longitud de cable
 */
void actualizarLongitudesActuales() {
  // Convertir posición del encoder (pulsos) a grados, luego a longitud
  float grados1 = (motor1.getPosicion() * 360.0) / motor1.getPulsosPorRevolucion();
  float grados2 = (motor2.getPosicion() * 360.0) / motor2.getPulsosPorRevolucion();
  float grados3 = (motor3.getPosicion() * 360.0) / motor3.getPulsosPorRevolucion();
  
  // Convertir grados a longitud (mm)
  // Longitud = (grados / MM_A_GRADOS)
  // Positivo = suelta cable = longitud aumenta
  // Negativo = jala cable = longitud disminuye
  L1 = Li1 + (grados1 / MM_A_GRADOS);
  L2 = Li2 + (grados2 / MM_A_GRADOS);
  L3 = Li3 + (grados3 / MM_A_GRADOS);
}

/**
 * @brief Convierte cambio de longitud (mm) a grados de rotación
 */
float deltaL_a_grados(float deltaL_mm) {
  return deltaL_mm * MM_A_GRADOS;
}

// ========== FUNCIONES DE INTERRUPCIÓN ==========

// Funciones de interrupción para cada motor
void IRAM_ATTR encoderMotor1() {
  motor1.manejarEncoder();
  ultimaActualizacion1 = millis();
}

void IRAM_ATTR encoderMotor2() {
  motor2.manejarEncoder();
  ultimaActualizacion2 = millis();
}

void IRAM_ATTR encoderMotor3() {
  motor3.manejarEncoder();
  ultimaActualizacion3 = millis();
}

// ========== FUNCIONES DE CONTROL DE MOTORES ==========

/**
 * @brief Gira un motor con detección de encoder (del código que funciona)
 * @param motor Referencia al motor a girar
 * @param grados Grados a girar (positivo=suelta cable, negativo=jala cable)
 * @param ultimaActualizacion Referencia al tiempo de última actualización del encoder
 * @param nombreMotor Nombre del motor para mensajes de debug
 * @return true si el giro fue exitoso, false si se detectó falla del encoder
 */
bool girarGradosConDeteccion(Motor &motor, float grados, unsigned long &ultimaActualizacion, const char* nombreMotor) {
  // Calcular pulsos necesarios
  long pulsosObjetivo = (long)((grados / 360.0) * motor.getPulsosPorRevolucion());
  
  Serial.print(nombreMotor);
  Serial.print(": Girando ");
  Serial.print(grados, 2);
  Serial.print(" grados (");
  Serial.print(abs(pulsosObjetivo));
  Serial.println(" pulsos)");
  
  long posicionInicial = motor.getPosicion();
  int direccion = (pulsosObjetivo >= 0) ? 1 : -1;  // 1 para horario (suelta), -1 para antihorario (jala)
  pulsosObjetivo = abs(pulsosObjetivo);
  
  // Resetear tiempo de última actualización
  ultimaActualizacion = millis();
  
  // Comenzar movimiento
  motor.mover(direccion);
  
  // Continuar hasta alcanzar el objetivo
  while (abs(motor.getPosicion() - posicionInicial) < pulsosObjetivo) {
    // DETECCIÓN CRÍTICA: Verificar si el encoder está respondiendo
    if (millis() - ultimaActualizacion > TIMEOUT_ENCODER) {
      // ¡ENCODER NO DETECTADO! Detener inmediatamente
      motor.mover(0);
      Serial.print("❌ ERROR: ");
      Serial.print(nombreMotor);
      Serial.println(" - Encoder no detectado. Motor detenido.");
      Serial.print("Última actualización hace: ");
      Serial.print(millis() - ultimaActualizacion);
      Serial.println(" ms");
      return false;  // Indicar falla
    }
    
    delay(1);  // Delay mínimo para no saturar
  }
  
  // Detener motor al alcanzar el objetivo
  motor.mover(0);
  
  long pulsosReales = abs(motor.getPosicion() - posicionInicial);
  Serial.print("✓ ");
  Serial.print(nombreMotor);
  Serial.print(" completado - Objetivo: ");
  Serial.print(pulsosObjetivo);
  Serial.print(" pulsos, Real: ");
  Serial.print(pulsosReales);
  Serial.println(" pulsos");
  
  return true;  // Éxito
}

/**
 * @brief Mueve los 3 motores simultáneamente hacia sus objetivos
 * @param grados1 Grados a girar el motor 1
 * @param grados2 Grados a girar el motor 2
 * @param grados3 Grados a girar el motor 3
 * @return true si todos completaron exitosamente, false si hubo algún fallo
 */
bool moverMotoresSimultaneos(float grados1, float grados2, float grados3) {
  Serial.println("\n--- Moviendo 3 motores simultáneamente ---");
  
  // Calcular pulsos objetivo para cada motor
  long pulsos1 = (long)((grados1 / 360.0) * motor1.getPulsosPorRevolucion());
  long pulsos2 = (long)((grados2 / 360.0) * motor2.getPulsosPorRevolucion());
  long pulsos3 = (long)((grados3 / 360.0) * motor3.getPulsosPorRevolucion());
  
  Serial.print("Motor 1: ");
  Serial.print(grados1, 2);
  Serial.print("° (");
  Serial.print(abs(pulsos1));
  Serial.println(" pulsos)");
  
  Serial.print("Motor 2: ");
  Serial.print(grados2, 2);
  Serial.print("° (");
  Serial.print(abs(pulsos2));
  Serial.println(" pulsos)");
  
  Serial.print("Motor 3: ");
  Serial.print(grados3, 2);
  Serial.print("° (");
  Serial.print(abs(pulsos3));
  Serial.println(" pulsos)\n");
  
  // Guardar posiciones iniciales
  long pos1Inicial = motor1.getPosicion();
  long pos2Inicial = motor2.getPosicion();
  long pos3Inicial = motor3.getPosicion();
  
  // Calcular direcciones
  int dir1 = (pulsos1 >= 0) ? 1 : -1;
  int dir2 = (pulsos2 >= 0) ? 1 : -1;
  int dir3 = (pulsos3 >= 0) ? 1 : -1;
  
  // Convertir a valores absolutos
  pulsos1 = abs(pulsos1);
  pulsos2 = abs(pulsos2);
  pulsos3 = abs(pulsos3);
  
  // Resetear tiempos de última actualización
  ultimaActualizacion1 = millis();
  ultimaActualizacion2 = millis();
  ultimaActualizacion3 = millis();
  
  // Flags para saber qué motores ya terminaron
  bool motor1Completo = (pulsos1 == 0);
  bool motor2Completo = (pulsos2 == 0);
  bool motor3Completo = (pulsos3 == 0);
  
  // Iniciar movimiento de todos los motores
  if (!motor1Completo) motor1.mover(dir1);
  if (!motor2Completo) motor2.mover(dir2);
  if (!motor3Completo) motor3.mover(dir3);
  
  // Variables para detección de timeout
  unsigned long tiempoInicio = millis();
  const unsigned long TIMEOUT_MOVIMIENTO = 30000; // 30 segundos
  
  // Variables para reporte de progreso
  unsigned long ultimoReporte = millis();
  
  // Bucle principal - continuar hasta que todos terminen
  while (!(motor1Completo && motor2Completo && motor3Completo)) {
    // Verificar timeout general
    if (millis() - tiempoInicio > TIMEOUT_MOVIMIENTO) {
      Serial.println("\n❌ TIMEOUT: Movimiento excedió 30 segundos");
      motor1.mover(0);
      motor2.mover(0);
      motor3.mover(0);
      return false;
    }
    
    // Motor 1: Verificar si alcanzó objetivo
    if (!motor1Completo) {
      long pulsosActuales1 = abs(motor1.getPosicion() - pos1Inicial);
      if (pulsosActuales1 >= pulsos1) {
        motor1.mover(0);
        motor1Completo = true;
        Serial.println("✓ Motor 1 completado");
      } else {
        // Verificar encoder
        if (millis() - ultimaActualizacion1 > TIMEOUT_ENCODER) {
          Serial.println("❌ Motor 1: Encoder no responde");
          motor1.mover(0);
          motor2.mover(0);
          motor3.mover(0);
          return false;
        }
      }
    }
    
    // Motor 2: Verificar si alcanzó objetivo
    if (!motor2Completo) {
      long pulsosActuales2 = abs(motor2.getPosicion() - pos2Inicial);
      if (pulsosActuales2 >= pulsos2) {
        motor2.mover(0);
        motor2Completo = true;
        Serial.println("✓ Motor 2 completado");
      } else {
        // Verificar encoder
        if (millis() - ultimaActualizacion2 > TIMEOUT_ENCODER) {
          Serial.println("❌ Motor 2: Encoder no responde");
          motor1.mover(0);
          motor2.mover(0);
          motor3.mover(0);
          return false;
        }
      }
    }
    
    // Motor 3: Verificar si alcanzó objetivo
    if (!motor3Completo) {
      long pulsosActuales3 = abs(motor3.getPosicion() - pos3Inicial);
      if (pulsosActuales3 >= pulsos3) {
        motor3.mover(0);
        motor3Completo = true;
        Serial.println("✓ Motor 3 completado");
      } else {
        // Verificar encoder
        if (millis() - ultimaActualizacion3 > TIMEOUT_ENCODER) {
          Serial.println("❌ Motor 3: Encoder no responde");
          motor1.mover(0);
          motor2.mover(0);
          motor3.mover(0);
          return false;
        }
      }
    }
    
    // Reporte de progreso cada segundo
    if (millis() - ultimoReporte > 1000) {
      Serial.print("Progreso: M1=");
      Serial.print(abs(motor1.getPosicion() - pos1Inicial));
      Serial.print("/");
      Serial.print(pulsos1);
      Serial.print(", M2=");
      Serial.print(abs(motor2.getPosicion() - pos2Inicial));
      Serial.print("/");
      Serial.print(pulsos2);
      Serial.print(", M3=");
      Serial.print(abs(motor3.getPosicion() - pos3Inicial));
      Serial.print("/");
      Serial.println(pulsos3);
      ultimoReporte = millis();
    }
    
    delay(1); // Delay mínimo
  }
  
  // Asegurar que todos están detenidos
  motor1.mover(0);
  motor2.mover(0);
  motor3.mover(0);
  
  // Mostrar resultados finales
  Serial.println("\n✓ Todos los motores completados");
  Serial.print("  M1: ");
  Serial.print(abs(motor1.getPosicion() - pos1Inicial));
  Serial.print(" / ");
  Serial.print(pulsos1);
  Serial.println(" pulsos");
  Serial.print("  M2: ");
  Serial.print(abs(motor2.getPosicion() - pos2Inicial));
  Serial.print(" / ");
  Serial.print(pulsos2);
  Serial.println(" pulsos");
  Serial.print("  M3: ");
  Serial.print(abs(motor3.getPosicion() - pos3Inicial));
  Serial.print(" / ");
  Serial.print(pulsos3);
  Serial.println(" pulsos");
  
  return true;
}

// Variables para detección de movimiento de encoders (OLD - ya no se usa)
struct DeteccionEncoder {
  unsigned long ultimoCheck;
  long posicionAnterior;
  float errorAnterior;
  int checksConErrorCreciente;
};

DeteccionEncoder deteccion1 = {0, 0, 0.0, 0};
DeteccionEncoder deteccion2 = {0, 0, 0.0, 0};
DeteccionEncoder deteccion3 = {0, 0, 0.0, 0};

// Variable global para señalizar fallo de encoder (no se usa actualmente)
volatile bool encoderFallo = false;

/* FUNCIÓN ANTIGUA - Ya no se usa, reemplazada por girarGradosConDeteccion()
 * Se deja comentada por si se necesita como referencia
bool moverHaciaLongitud(...) {
  ...código complicado con direccionPositiva...
}
*/

// ========== SETUP Y LOOP ==========

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("  Stewart Platform - Cinemática Inversa");
  Serial.println("========================================\n");
  
  // Inicializar motores
  Serial.println("Inicializando motores...");
  motor1.inicializar();
  motor2.inicializar();
  motor3.inicializar();
  
  // Configurar interrupciones para los encoders
  Serial.println("Configurando interrupciones...");
  
  // Motor 1: Pines 25 y 26
  attachInterrupt(digitalPinToInterrupt(motor1.getPinEncA()), encoderMotor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor1.getPinEncB()), encoderMotor1, CHANGE);
  
  // Motor 2: Pines 32 y 33
  attachInterrupt(digitalPinToInterrupt(motor2.getPinEncA()), encoderMotor2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.getPinEncB()), encoderMotor2, CHANGE);
  
  // Motor 3: Pines 27 y 4
  attachInterrupt(digitalPinToInterrupt(motor3.getPinEncA()), encoderMotor3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor3.getPinEncB()), encoderMotor3, CHANGE);
  
  Serial.println("Sistema inicializado correctamente.\n");
  
  // ========== CALIBRACIÓN AUTOMÁTICA ==========
  Serial.println("========================================");
  Serial.println("  CALIBRACIÓN DE LONGITUDES INICIALES");
  Serial.println("========================================");
  Serial.println("Calculando longitudes en posición neutra (0°, 0°)...\n");
  
  // Calcular longitudes iniciales usando cinemática inversa
  Li1 = calcularLongitudCable(A1x, A1y, A1z, B1x, B1y, B1z, H, 0, 0);
  Li2 = calcularLongitudCable(A2x, A2y, A2z, B2x, B2y, B2z, H, 0, 0);
  Li3 = calcularLongitudCable(A3x, A3y, A3z, B3x, B3y, B3z, H, 0, 0);
  
  Serial.println("Longitudes iniciales calculadas:");
  Serial.print("  Li1 = ");
  Serial.print(Li1, 2);
  Serial.println(" mm");
  Serial.print("  Li2 = ");
  Serial.print(Li2, 2);
  Serial.println(" mm");
  Serial.print("  Li3 = ");
  Serial.print(Li3, 2);
  Serial.println(" mm");
  
  // Inicializar longitudes actuales (encoders en 0 al inicio)
  L1 = Li1;
  L2 = Li2;
  L3 = Li3;
  
  Serial.println("\n⚠️  IMPORTANTE: Asegúrate de que la plataforma");
  Serial.println("    esté en posición neutra (horizontal) al encender");
  Serial.println("    o usa el comando 'reset' para recalibrar.\n");
  Serial.println("========================================\n");
  
  // Instrucciones de uso
  Serial.println("========================================");
  Serial.println("COMANDOS DISPONIBLES:");
  Serial.println("========================================");
  Serial.println("• Control de orientación:");
  Serial.println("    ang [tx] [ty]  → Ángulos en grados");
  Serial.println("    Ejemplo: ang 5 -3");
  Serial.println("");
  Serial.println("• Diagnóstico:");
  Serial.println("    diag [tx] [ty] → Ver cálculo sin mover");
  Serial.println("    test           → Prueba encoders");
  Serial.println("    reset          → Recalibrar posición actual");
  Serial.println("    cal / cal1/2/3 → Calibrar radio de polea");
  Serial.println("");
  Serial.println("• Ver estado:");
  Serial.println("    pos      → Posiciones y longitudes");
  Serial.println("    ang      → Ángulos actuales");
  Serial.println("");
  Serial.println("• Control manual (modo anterior):");
  Serial.println("    M1 360   → Mover Motor 1");
  Serial.println("    M2 -180  → Mover Motor 2");
  Serial.println("    M3 90    → Mover Motor 3");
  Serial.println("========================================\n");
  Serial.println("Listo para recibir comandos...\n");
}

void loop() {
  // Leer y actualizar datos de los motores continuamente
  motor1.leer();
  motor2.leer();
  motor3.leer();
  
  // Actualizar longitudes actuales basadas en encoders
  actualizarLongitudesActuales();
  
  // Verificar si hay datos disponibles en el Serial
  if (Serial.available() > 0) {
    // Leer el comando completo
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    comando.toUpperCase();
    
    if (comando.length() > 0) {
      Serial.print("\n> Comando: ");
      Serial.println(comando);
      
      // Comando RESET para recalibrar posición
      if (comando.equals("RESET")) {
        Serial.println("\n========== RECALIBRANDO SISTEMA ==========");
        Serial.println("Reseteando encoders a 0...");
        
        // Resetear posiciones de encoders a 0
        motor1.resetPosicion();
        motor2.resetPosicion();
        motor3.resetPosicion();
        
        // Recalcular longitudes actuales (ahora serán iguales a Li)
        actualizarLongitudesActuales();
        
        // Resetear ángulos a 0
        tx = 0.0;
        ty = 0.0;
        
        Serial.println("✓ Encoders reseteados");
        Serial.println("✓ Posición actual definida como neutra (0°, 0°)");
        Serial.println("\nLongitudes actuales:");
        Serial.print("  L1 = ");
        Serial.print(L1, 2);
        Serial.println(" mm");
        Serial.print("  L2 = ");
        Serial.print(L2, 2);
        Serial.println(" mm");
        Serial.print("  L3 = ");
        Serial.print(L3, 2);
        Serial.println(" mm");
        Serial.println("========================================\n");
      }
      // Comando CAL para calibración empírica del radio
      else if (comando.equals("CAL")) {
        Serial.println("\n========== CALIBRACIÓN EMPÍRICA ==========");
        Serial.println("Este comando te ayudará a medir la relación");
        Serial.println("real entre pulsos del encoder y movimiento del cable.\n");
        Serial.println("INSTRUCCIONES:");
        Serial.println("1. Marca el cable con un marcador");
        Serial.println("2. Mide la posición inicial");
        Serial.println("3. El motor girará y se detendrá");
        Serial.println("4. Mide cuántos mm se movió el cable");
        Serial.println("5. Ingresa el valor medido\n");
        Serial.println("¿Motor a calibrar? (1, 2 o 3)");
        Serial.println("Escribe: cal1, cal2 o cal3\n");
        Serial.println("========================================\n");
      }
      // Comandos CAL1, CAL2, CAL3 para calibración individual
      else if (comando.equals("CAL1") || comando.equals("CAL2") || comando.equals("CAL3")) {
        int numMotor = comando.charAt(3) - '0';
        Motor* motorSel;
        unsigned long* ultAct;
        String nombreMotor;
        
        switch(numMotor) {
          case 1: motorSel = &motor1; ultAct = &ultimaActualizacion1; nombreMotor = "Motor 1"; break;
          case 2: motorSel = &motor2; ultAct = &ultimaActualizacion2; nombreMotor = "Motor 2"; break;
          case 3: motorSel = &motor3; ultAct = &ultimaActualizacion3; nombreMotor = "Motor 3"; break;
          default: Serial.println("❌ Motor no válido\n"); return;
        }
        
        Serial.println("\n========== CALIBRACIÓN " + nombreMotor + " ==========");
        Serial.println("Preparando prueba...\n");
        
        // Resetear posición
        motorSel->resetPosicion();
        
        Serial.println("✓ Encoder reseteado a 0");
        Serial.println("\n📏 Marca el cable con un marcador AHORA");
        Serial.println("   Presiona ENTER cuando estés listo para girar...");
        
        // Esperar entrada del usuario
        while (Serial.available() == 0) {
          delay(100);
        }
        Serial.readString(); // Limpiar buffer
        
        // Girar 1 vuelta completa (360 grados)
        Serial.println("\n⚙️  Girando 1 vuelta completa (360°)...");
        bool exito = girarGradosConDeteccion(*motorSel, 360.0, *ultAct, nombreMotor.c_str());
        
        if (exito) {
          long pulsosTotales = motorSel->getPosicion();
          
          Serial.println("\n✓ Movimiento completado");
          Serial.print("   Pulsos registrados: ");
          Serial.println(pulsosTotales);
          Serial.println("\n📏 Mide cuántos MILÍMETROS se movió el cable");
          Serial.println("   (usa una regla o cinta métrica)");
          Serial.println("\n   Escribe el valor en mm y presiona ENTER:");
          Serial.println("   Ejemplo: 5.5");
          
          // Esperar medición del usuario
          while (Serial.available() == 0) {
            delay(100);
          }
          
          String medicionStr = Serial.readStringUntil('\n');
          medicionStr.trim();
          float mmReales = medicionStr.toFloat();
          
          if (mmReales > 0) {
            Serial.println("\n========== RESULTADOS ==========");
            Serial.print("Medición: ");
            Serial.print(mmReales, 2);
            Serial.println(" mm");
            Serial.print("Pulsos: ");
            Serial.println(pulsosTotales);
            
            // Calcular radio efectivo de la polea
            // mmReales = 360° × (radio_efectivo/180°×π)
            // radio_efectivo = mmReales × π / 360° × 180°
            float circunferencia = mmReales; // 1 vuelta = circunferencia
            float radioEfectivo = circunferencia / (2.0 * PI);
            
            Serial.print("\n📐 Radio efectivo de la polea: ");
            Serial.print(radioEfectivo, 3);
            Serial.println(" mm");
            Serial.print("   Diámetro efectivo: ");
            Serial.print(radioEfectivo * 2.0, 3);
            Serial.println(" mm");
            
            // Calcular conversión mm/pulso
            float mmPorPulso = mmReales / (float)pulsosTotales;
            Serial.print("\n🔢 Conversión: ");
            Serial.print(mmPorPulso, 6);
            Serial.println(" mm/pulso");
            Serial.print("   O: ");
            Serial.print(1.0/mmPorPulso, 2);
            Serial.println(" pulsos/mm");
            
            // Calcular nuevo MM_A_GRADOS
            float nuevoMM_A_GRADOS = 360.0 / mmReales;
            Serial.print("\n✨ Valor sugerido para MM_A_GRADOS: ");
            Serial.print(nuevoMM_A_GRADOS, 2);
            Serial.println(" grados/mm");
            Serial.print("   (Valor actual: ");
            Serial.print(MM_A_GRADOS, 2);
            Serial.println(" grados/mm)");
            
            if (abs(nuevoMM_A_GRADOS - MM_A_GRADOS) > 5.0) {
              Serial.println("\n⚠️  RECOMENDACIÓN: Actualizar el valor en el código");
              Serial.println("   Cambia la línea:");
              Serial.print("   #define RADIO_POLEA ");
              Serial.println(RADIO_POLEA, 1);
              Serial.println("   por:");
              Serial.print("   #define RADIO_POLEA ");
              Serial.println(radioEfectivo, 3);
            } else {
              Serial.println("\n✓ El radio actual es correcto");
            }
            
            Serial.println("========================================\n");
          } else {
            Serial.println("❌ Valor inválido\n");
          }
        } else {
          Serial.println("❌ Error en el movimiento\n");
        }
      }
      // Comando TEST para probar encoders
      else if (comando.equals("TEST")) {
        Serial.println("\n========== PRUEBA DE ENCODERS ==========");
        Serial.println("Cada motor girará 1 segundo en cada dirección");
        Serial.println("Observa si los pulsos cambian correctamente\n");
        
        // Probar Motor 1
        Serial.println("--- Motor 1 ---");
        long pos1Inicial = motor1.getPosicion();
        Serial.print("Posición inicial: ");
        Serial.println(pos1Inicial);
        Serial.println("Girando horario 1 seg...");
        motor1.mover(1);
        delay(1000);
        motor1.mover(0);
        long pos1Horario = motor1.getPosicion();
        Serial.print("Posición después: ");
        Serial.print(pos1Horario);
        Serial.print(" (cambio: ");
        Serial.print(pos1Horario - pos1Inicial);
        Serial.println(" pulsos)");
        delay(500);
        
        Serial.println("Girando antihorario 1 seg...");
        motor1.mover(-1);
        delay(1000);
        motor1.mover(0);
        long pos1Final = motor1.getPosicion();
        Serial.print("Posición final: ");
        Serial.print(pos1Final);
        Serial.print(" (cambio: ");
        Serial.print(pos1Final - pos1Horario);
        Serial.println(" pulsos)\n");
        delay(1000);
        
        // Probar Motor 2
        Serial.println("--- Motor 2 ---");
        long pos2Inicial = motor2.getPosicion();
        Serial.print("Posición inicial: ");
        Serial.println(pos2Inicial);
        Serial.println("Girando horario 1 seg...");
        motor2.mover(1);
        delay(1000);
        motor2.mover(0);
        long pos2Horario = motor2.getPosicion();
        Serial.print("Posición después: ");
        Serial.print(pos2Horario);
        Serial.print(" (cambio: ");
        Serial.print(pos2Horario - pos2Inicial);
        Serial.println(" pulsos)");
        delay(500);
        
        Serial.println("Girando antihorario 1 seg...");
        motor2.mover(-1);
        delay(1000);
        motor2.mover(0);
        long pos2Final = motor2.getPosicion();
        Serial.print("Posición final: ");
        Serial.print(pos2Final);
        Serial.print(" (cambio: ");
        Serial.print(pos2Final - pos2Horario);
        Serial.println(" pulsos)\n");
        delay(1000);
        
        // Probar Motor 3
        Serial.println("--- Motor 3 ---");
        long pos3Inicial = motor3.getPosicion();
        Serial.print("Posición inicial: ");
        Serial.println(pos3Inicial);
        Serial.println("Girando horario 1 seg...");
        motor3.mover(1);
        delay(1000);
        motor3.mover(0);
        long pos3Horario = motor3.getPosicion();
        Serial.print("Posición después: ");
        Serial.print(pos3Horario);
        Serial.print(" (cambio: ");
        Serial.print(pos3Horario - pos3Inicial);
        Serial.println(" pulsos)");
        delay(500);
        
        Serial.println("Girando antihorario 1 seg...");
        motor3.mover(-1);
        delay(1000);
        motor3.mover(0);
        long pos3Final = motor3.getPosicion();
        Serial.print("Posición final: ");
        Serial.print(pos3Final);
        Serial.print(" (cambio: ");
        Serial.print(pos3Final - pos3Horario);
        Serial.println(" pulsos)\n");
        
        Serial.println("========== FIN DE PRUEBA ==========");
        Serial.println("ANÁLISIS:");
        Serial.println("✓ Si los pulsos cambian: Encoder funciona");
        Serial.println("✗ Si pulsos = 0: Verifica conexiones del encoder");
        Serial.println("========================================\n");
      }
      // Comando DIAG para diagnóstico sin mover los motores
      else if (comando.startsWith("DIAG")) {
        String parametros = comando.substring(4);
        parametros.trim();
        
        if (parametros.length() > 0) {
          int spaceIndex = parametros.indexOf(' ');
          if (spaceIndex > 0) {
            float txGrados = parametros.substring(0, spaceIndex).toFloat();
            float tyGrados = parametros.substring(spaceIndex + 1).toFloat();
            
            // Convertir a radianes
            float tx_test = txGrados * PI / 180.0;
            float ty_test = tyGrados * PI / 180.0;
            
            Serial.println("\n========== DIAGNÓSTICO DE CINEMÁTICA ==========");
            Serial.print("Ángulos solicitados: TX=");
            Serial.print(txGrados, 2);
            Serial.print("°, TY=");
            Serial.print(tyGrados, 2);
            Serial.println("°");
            
            // Calcular longitudes con posición actual (tx=0, ty=0)
            float L1_actual_calc = calcularLongitudCable(A1x, A1y, A1z, B1x, B1y, B1z, H, 0, 0);
            float L2_actual_calc = calcularLongitudCable(A2x, A2y, A2z, B2x, B2y, B2z, H, 0, 0);
            float L3_actual_calc = calcularLongitudCable(A3x, A3y, A3z, B3x, B3y, B3z, H, 0, 0);
            
            Serial.println("\nLongitudes en posición neutra (0°, 0°):");
            Serial.print("  L1: ");
            Serial.print(L1_actual_calc, 2);
            Serial.println(" mm");
            Serial.print("  L2: ");
            Serial.print(L2_actual_calc, 2);
            Serial.println(" mm");
            Serial.print("  L3: ");
            Serial.print(L3_actual_calc, 2);
            Serial.println(" mm");
            
            // Calcular longitudes deseadas
            float dL1_test = calcularLongitudCable(A1x, A1y, A1z, B1x, B1y, B1z, H, tx_test, ty_test);
            float dL2_test = calcularLongitudCable(A2x, A2y, A2z, B2x, B2y, B2z, H, tx_test, ty_test);
            float dL3_test = calcularLongitudCable(A3x, A3y, A3z, B3x, B3y, B3z, H, tx_test, ty_test);
            
            Serial.print("\nLongitudes deseadas para (");
            Serial.print(txGrados, 2);
            Serial.print("°, ");
            Serial.print(tyGrados, 2);
            Serial.println("°):");
            Serial.print("  L1: ");
            Serial.print(dL1_test, 2);
            Serial.println(" mm");
            Serial.print("  L2: ");
            Serial.print(dL2_test, 2);
            Serial.println(" mm");
            Serial.print("  L3: ");
            Serial.print(dL3_test, 2);
            Serial.println(" mm");
            
            Serial.println("\nCambios necesarios:");
            float deltaL1 = dL1_test - L1_actual_calc;
            float deltaL2 = dL2_test - L2_actual_calc;
            float deltaL3 = dL3_test - L3_actual_calc;
            
            Serial.print("  ΔL1: ");
            Serial.print(deltaL1, 2);
            Serial.print(" mm (");
            Serial.print(deltaL_a_grados(abs(deltaL1)), 2);
            Serial.println(" grados)");
            Serial.print("  ΔL2: ");
            Serial.print(deltaL2, 2);
            Serial.print(" mm (");
            Serial.print(deltaL_a_grados(abs(deltaL2)), 2);
            Serial.println(" grados)");
            Serial.print("  ΔL3: ");
            Serial.print(deltaL3, 2);
            Serial.print(" mm (");
            Serial.print(deltaL_a_grados(abs(deltaL3)), 2);
            Serial.println(" grados)");
            
            Serial.println("\nEstado actual de encoders:");
            Serial.print("  Motor 1: ");
            Serial.print(motor1.getPosicion());
            Serial.println(" pulsos");
            Serial.print("  Motor 2: ");
            Serial.print(motor2.getPosicion());
            Serial.println(" pulsos");
            Serial.print("  Motor 3: ");
            Serial.print(motor3.getPosicion());
            Serial.println(" pulsos");
            
            Serial.println("\nConversión:");
            Serial.print("  Radio polea: ");
            Serial.print(RADIO_POLEA, 1);
            Serial.println(" mm");
            Serial.print("  1 mm = ");
            Serial.print(MM_A_GRADOS, 2);
            Serial.println(" grados");
            Serial.print("  1 grado = ");
            Serial.print(1.0/MM_A_GRADOS, 4);
            Serial.println(" mm");
            
            Serial.println("==============================================\n");
          } else {
            Serial.println("❌ Error: Usa formato 'diag [tx] [ty]'");
            Serial.println("   Ejemplo: diag 5 -3\n");
          }
        } else {
          Serial.println("❌ Error: Especifica ángulos TX y TY");
          Serial.println("   Ejemplo: diag 1 0\n");
        }
      }
      // Comando ANG para establecer ángulos
      else if (comando.startsWith("ANG")) {
        String parametros = comando.substring(3);
        parametros.trim();
        
        if (parametros.length() == 0) {
          // Mostrar ángulos actuales
          Serial.println("\n=== ÁNGULOS ACTUALES ===");
          Serial.print("TX (rotación X): ");
          Serial.print(tx * 180.0 / PI, 2);
          Serial.println(" grados");
          Serial.print("TY (rotación Y): ");
          Serial.print(ty * 180.0 / PI, 2);
          Serial.println(" grados");
          Serial.println("========================\n");
        } else {
          // Parsear ángulos TX y TY
          int spaceIndex = parametros.indexOf(' ');
          if (spaceIndex > 0) {
            float txGrados = parametros.substring(0, spaceIndex).toFloat();
            float tyGrados = parametros.substring(spaceIndex + 1).toFloat();
            
            // Convertir a radianes
            tx = txGrados * PI / 180.0;
            ty = tyGrados * PI / 180.0;
            
            Serial.println("\n=== CALCULANDO CINEMÁTICA INVERSA ===");
            Serial.print("TX: ");
            Serial.print(txGrados, 2);
            Serial.print("° (");
            Serial.print(tx, 4);
            Serial.println(" rad)");
            Serial.print("TY: ");
            Serial.print(tyGrados, 2);
            Serial.print("° (");
            Serial.print(ty, 4);
            Serial.println(" rad)");
            
            // Calcular longitudes deseadas usando cinemática inversa
            dL1 = calcularLongitudCable(A1x, A1y, A1z, B1x, B1y, B1z, H, tx, ty);
            dL2 = calcularLongitudCable(A2x, A2y, A2z, B2x, B2y, B2z, H, tx, ty);
            dL3 = calcularLongitudCable(A3x, A3y, A3z, B3x, B3y, B3z, H, tx, ty);
            
            Serial.println("\nLongitudes deseadas:");
            Serial.print("  L1: ");
            Serial.print(dL1, 2);
            Serial.println(" mm");
            Serial.print("  L2: ");
            Serial.print(dL2, 2);
            Serial.println(" mm");
            Serial.print("  L3: ");
            Serial.print(dL3, 2);
            Serial.println(" mm");
            
            Serial.println("\nLongitudes actuales:");
            Serial.print("  L1: ");
            Serial.print(L1, 2);
            Serial.println(" mm");
            Serial.print("  L2: ");
            Serial.print(L2, 2);
            Serial.println(" mm");
            Serial.print("  L3: ");
            Serial.print(L3, 2);
            Serial.println(" mm");
            
            Serial.println("\n--- MOVIENDO A POSICIÓN DESEADA ---");
            
            // Calcular cambios de longitud necesarios (ΔL)
            float deltaL1 = dL1 - L1;
            float deltaL2 = dL2 - L2;
            float deltaL3 = dL3 - L3;
            
            Serial.println("\nCambios necesarios:");
            Serial.print("  ΔL1: ");
            Serial.print(deltaL1, 2);
            Serial.print(" mm → ");
            Serial.print(deltaL_a_grados(deltaL1), 2);
            Serial.println(" grados");
            Serial.print("  ΔL2: ");
            Serial.print(deltaL2, 2);
            Serial.print(" mm → ");
            Serial.print(deltaL_a_grados(deltaL2), 2);
            Serial.println(" grados");
            Serial.print("  ΔL3: ");
            Serial.print(deltaL3, 2);
            Serial.print(" mm → ");
            Serial.print(deltaL_a_grados(deltaL3), 2);
            Serial.println(" grados");
            
            // Convertir ΔL a grados de rotación
            float grados1 = deltaL_a_grados(deltaL1);
            float grados2 = deltaL_a_grados(deltaL2);
            float grados3 = deltaL_a_grados(deltaL3);
            
            Serial.println("\n--- Ejecutando movimientos simultáneos ---");
            
            // Mover los 3 motores simultáneamente
            bool exito = moverMotoresSimultaneos(grados1, grados2, grados3);
            
            // Actualizar longitudes finales
            actualizarLongitudesActuales();
            
            // Mostrar resultado
            Serial.println("\n========================================");
            if (exito) {
              Serial.println("✓ Movimiento completado exitosamente");
            } else {
              Serial.println("❌ Movimiento completado con errores");
            }
            
            Serial.println("Posiciones finales:");
            Serial.print("  L1: ");
            Serial.print(L1, 2);
            Serial.print(" mm (deseado: ");
            Serial.print(dL1, 2);
            Serial.println(")");
            Serial.print("  L2: ");
            Serial.print(L2, 2);
            Serial.print(" mm (deseado: ");
            Serial.print(dL2, 2);
            Serial.println(")");
            Serial.print("  L3: ");
            Serial.print(L3, 2);
            Serial.print(" mm (deseado: ");
            Serial.print(dL3, 2);
            Serial.println(")\n");
          } else {
            Serial.println("❌ Error: Usa formato 'ang [tx] [ty]'");
            Serial.println("   Ejemplo: ang 5 -3\n");
          }
        }
      }
      // Comando POS para ver posiciones
      else if (comando.equals("POS")) {
        Serial.println("\n========== ESTADO DEL SISTEMA ==========");
        Serial.println("Posiciones (pulsos):");
        Serial.print("  Motor 1: ");
        Serial.println(motor1.getPosicion());
        Serial.print("  Motor 2: ");
        Serial.println(motor2.getPosicion());
        Serial.print("  Motor 3: ");
        Serial.println(motor3.getPosicion());
        
        Serial.println("\nLongitudes de cables (mm):");
        Serial.print("  L1: ");
        Serial.print(L1, 2);
        Serial.print(" (inicial: ");
        Serial.print(Li1, 2);
        Serial.println(")");
        Serial.print("  L2: ");
        Serial.print(L2, 2);
        Serial.print(" (inicial: ");
        Serial.print(Li2, 2);
        Serial.println(")");
        Serial.print("  L3: ");
        Serial.print(L3, 2);
        Serial.print(" (inicial: ");
        Serial.print(Li3, 2);
        Serial.println(")");
        Serial.println("========================================\n");
      }
      // Control manual individual de motores (modo anterior)
      else if (comando.startsWith("M1") || comando.startsWith("M2") || comando.startsWith("M3")) {
        int numMotor = comando.charAt(1) - '0';
        String argumento = comando.substring(2);
        argumento.trim();
        
        Motor* motorSel;
        unsigned long* ultAct;
        String nombreMotor;
        
        switch(numMotor) {
          case 1: motorSel = &motor1; ultAct = &ultimaActualizacion1; nombreMotor = "Motor 1"; break;
          case 2: motorSel = &motor2; ultAct = &ultimaActualizacion2; nombreMotor = "Motor 2"; break;
          case 3: motorSel = &motor3; ultAct = &ultimaActualizacion3; nombreMotor = "Motor 3"; break;
          default: Serial.println("❌ Motor no válido\n"); return;
        }
        
        if (argumento.equals("POS")) {
          Serial.print(nombreMotor);
          Serial.print(": ");
          Serial.print(motorSel->getPosicion());
          Serial.println(" pulsos\n");
        } else if (argumento.length() > 0) {
          float grados = argumento.toFloat();
          // Usar la función girarGradosConDeteccion que ya existe
          // (necesitarías implementarla o usar un método simplificado)
          Serial.println("⚠️ Función en desarrollo - Usa comando 'ang'\n");
        }
      }
      else {
        Serial.println("❌ Comando no reconocido\n");
      }
      
      Serial.println("Listo...\n");
    }
  }
  
  delay(10);
}
