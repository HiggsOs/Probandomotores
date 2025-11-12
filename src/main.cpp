#include <Arduino.h>
#include "Motor.h"

// ========== CONFIGURACIÓN DE MOTORES ==========
// Crear instancias de los 3 motores con los parámetros especificados
// NOTA: Pines I1 e I2 INVERTIDOS en todos los motores para corregir dirección
// Horario = jalar cables, Antihorario = soltar cables
Motor motor1(5, 18, 25, 26, 25200);  // I1 e I2 invertidos
Motor motor2(16, 17, 32, 33, 25200);  // I1 e I2 invertidos
Motor motor3(19, 23, 27, 4, 25200);   // I1 e I2 invertidos

// ========== CINEMÁTICA INVERSA - STEWART PLATFORM ==========
// Constante Km: Relación entre número de encoder y longitud (mm)
// Diámetro de polea = 1.8 mm → Radio = 0.9 mm
// Km = (2 × π × radio) / pulsos_por_revolución
// Km = (2 × π × 0.9) / 25200 = 5.65 / 25200 = 0.000224 mm/pulso
#define Km 0.000224  // mm por pulso del encoder

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
volatile float Li1 = 430.0;
volatile float Li2 = 440.0;
volatile float Li3 = 435.0;

// Longitudes actuales de los cables (mm)
volatile float L1 = 430.0;
volatile float L2 = 440.0;
volatile float L3 = 415.0;

// Longitudes deseadas calculadas por cinemática inversa (mm)
float dL1 = 413.0;
float dL2 = 422.0;
float dL3 = 425.0;

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
 */
void actualizarLongitudesActuales() {
  // Convertir pulsos del encoder a longitud en mm
  L1 = Li1 - Km * motor1.getPosicion();
  L2 = Li2 + Km * motor2.getPosicion();
  L3 = Li3 - Km * motor3.getPosicion();
}

/**
 * @brief Convierte longitud (mm) a pulsos del encoder
 */
long longitudAPulsos(float longitud_mm) {
  return (long)(longitud_mm / Km);
}

/**
 * @brief Convierte pulsos del encoder a longitud (mm)
 */
float pulsosALongitud(long pulsos) {
  return pulsos * Km;
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

// Variables para detección de movimiento de encoders
struct DeteccionEncoder {
  unsigned long ultimoCheck;
  long posicionAnterior;
};

DeteccionEncoder deteccion1 = {0, 0};
DeteccionEncoder deteccion2 = {0, 0};
DeteccionEncoder deteccion3 = {0, 0};

// Variable global para señalizar fallo de encoder
volatile bool encoderFallo = false;

/**
 * @brief Mueve el motor hacia la longitud deseada con control proporcional y detección de encoder
 * @param motor Referencia al motor
 * @param longitudActual Longitud actual del cable (mm)
 * @param longitudDeseada Longitud deseada del cable (mm)
 * @param direccionPositiva Si true, aumentar longitud es dirección positiva (horario)
 * @param deteccion Referencia a la estructura de detección del encoder
 * @param nombreMotor Nombre del motor para debug
 * @return true si está dentro de tolerancia, false si se está moviendo
 */
bool moverHaciaLongitud(Motor &motor, float longitudActual, float longitudDeseada, bool direccionPositiva, DeteccionEncoder &deteccion, const char* nombreMotor) {
  // Si ya hubo un fallo, no mover
  if (encoderFallo) {
    motor.mover(0);
    return true;
  }
  
  float error = longitudDeseada - longitudActual;
  
  // DETECCIÓN CRÍTICA: Verificar si el encoder está respondiendo mientras el motor se mueve
  if (millis() - deteccion.ultimoCheck > 300) {  // Verificar cada 300ms
    long posicionActual = motor.getPosicion();
    
    // Si el motor debería estar moviéndose pero el encoder no cambia
    if (abs(error) > TOLERANCIA) {
      if (posicionActual == deteccion.posicionAnterior) {
        // ¡ENCODER NO RESPONDE!
        Serial.print("\n❌❌❌ FALLO CRÍTICO: ");
        Serial.print(nombreMotor);
        Serial.println(" - Encoder no detecta movimiento ❌❌❌");
        Serial.println(">>> DETENIENDO TODOS LOS MOTORES INMEDIATAMENTE <<<\n");
        
        // Marcar fallo global
        encoderFallo = true;
        
        // DETENER TODOS LOS MOTORES INMEDIATAMENTE
        motor1.mover(0);
        motor2.mover(0);
        motor3.mover(0);
        
        return true; // Forzar salida del bucle
      }
    }
    
    deteccion.posicionAnterior = posicionActual;
    deteccion.ultimoCheck = millis();
  }
  
  // Si está dentro de la tolerancia, detener
  if (abs(error) < TOLERANCIA) {
    motor.mover(0);
    return true;
  }
  
  // Determinar dirección
  int direccion;
  if (direccionPositiva) {
    // Para motor donde aumentar pulsos aumenta longitud
    direccion = (error > 0) ? 1 : -1;
  } else {
    // Para motor donde aumentar pulsos disminuye longitud
    direccion = (error > 0) ? -1 : 1;
  }
  
  motor.mover(direccion);
  return false;
}

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
  
  // Instrucciones de uso
  Serial.println("========================================");
  Serial.println("COMANDOS DISPONIBLES:");
  Serial.println("========================================");
  Serial.println("• Control de orientación:");
  Serial.println("    ang [tx] [ty]  → Ángulos en grados");
  Serial.println("    Ejemplo: ang 5 -3");
  Serial.println("");
  Serial.println("• Prueba de encoders:");
  Serial.println("    test          → Prueba encoders (1 seg c/motor)");
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
      
      // Comando TEST para probar encoders
      if (comando.equals("TEST")) {
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
            
            // Mostrar info de depuración inicial
            Serial.println("\nInfo de encoders:");
            Serial.print("Motor 1 posición: ");
            Serial.print(motor1.getPosicion());
            Serial.println(" pulsos");
            Serial.print("Motor 2 posición: ");
            Serial.print(motor2.getPosicion());
            Serial.println(" pulsos");
            Serial.print("Motor 3 posición: ");
            Serial.print(motor3.getPosicion());
            Serial.println(" pulsos");
            
            // Control en bucle hasta alcanzar posiciones
            unsigned long timeout = millis() + 30000; // 30 segundos timeout
            bool motor1Listo = false;
            bool motor2Listo = false;
            bool motor3Listo = false;
            
            unsigned long ultimoReporte = 0;
            
            // Resetear flag de fallo al inicio
            encoderFallo = false;
            
            // Inicializar estructuras de detección
            deteccion1.ultimoCheck = millis();
            deteccion1.posicionAnterior = motor1.getPosicion();
            deteccion2.ultimoCheck = millis();
            deteccion2.posicionAnterior = motor2.getPosicion();
            deteccion3.ultimoCheck = millis();
            deteccion3.posicionAnterior = motor3.getPosicion();
            
            while (!(motor1Listo && motor2Listo && motor3Listo) && millis() < timeout && !encoderFallo) {
              // Actualizar longitudes
              actualizarLongitudesActuales();
              
              // Mover cada motor hacia su objetivo con detección de encoder
              motor1Listo = moverHaciaLongitud(motor1, L1, dL1, false, deteccion1, "Motor 1");
              motor2Listo = moverHaciaLongitud(motor2, L2, dL2, true, deteccion2, "Motor 2");
              motor3Listo = moverHaciaLongitud(motor3, L3, dL3, false, deteccion3, "Motor 3");
              
              // Si hubo fallo de encoder, salir inmediatamente
              if (encoderFallo) {
                Serial.println("\n🛑 SISTEMA DETENIDO POR FALLO DE ENCODER 🛑\n");
                break;
              }
              
              // Reporte periódico cada 1 segundo
              if (millis() - ultimoReporte > 1000) {
                Serial.println("\n-- Estado actual --");
                Serial.print("M1: ");
                Serial.print(motor1.getPosicion());
                Serial.print(" pulsos, L=");
                Serial.print(L1, 2);
                Serial.print(" mm (obj: ");
                Serial.print(dL1, 2);
                Serial.println(" mm)");
                Serial.print("M2: ");
                Serial.print(motor2.getPosicion());
                Serial.print(" pulsos, L=");
                Serial.print(L2, 2);
                Serial.print(" mm (obj: ");
                Serial.print(dL2, 2);
                Serial.println(" mm)");
                Serial.print("M3: ");
                Serial.print(motor3.getPosicion());
                Serial.print(" pulsos, L=");
                Serial.print(L3, 2);
                Serial.print(" mm (obj: ");
                Serial.print(dL3, 2);
                Serial.println(" mm)");
                ultimoReporte = millis();
              }
              
              delay(10);
            }
            
            // Detener todos los motores al finalizar
            motor1.mover(0);
            motor2.mover(0);
            motor3.mover(0);
            
            if (encoderFallo) {
              Serial.println("\n❌ MOVIMIENTO ABORTADO POR FALLO DE ENCODER ❌");
              Serial.println("Verifica las conexiones de los encoders antes de continuar.\n");
            } else if (motor1Listo && motor2Listo && motor3Listo) {
              Serial.println("✓ Posición alcanzada\n");
            } else {
              Serial.println("⚠️ Timeout - Posición no alcanzada completamente\n");
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
