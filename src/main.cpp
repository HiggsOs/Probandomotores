#include <Arduino.h>
#include "Motor.h"

// ========== CONFIGURACIÓN DE MOTORES ==========
// Crear instancias de los 3 motores con los parámetros especificados
Motor motor1(18, 5, 25, 26, 25200);
Motor motor2(17, 16, 32, 33, 25200);  // Pines I1 e I2 invertidos para corregir dirección
Motor motor3(23, 19, 27, 4, 25200);

// ========== CINEMÁTICA INVERSA - STEWART PLATFORM ==========
// Constante Km: Relación entre número de encoder y longitud (mm)
// Km = 2*PI * (radio_polea_mm) / pulsos_por_revolucion
// Km = 2*PI * (1.8/2) / 25200 = 0.000224 mm/pulso
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
volatile float Li1 = 413.0;
volatile float Li2 = 422.0;
volatile float Li3 = 425.0;

// Longitudes actuales de los cables (mm)
volatile float L1 = 413.0;
volatile float L2 = 422.0;
volatile float L3 = 425.0;

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

/**
 * @brief Mueve el motor hacia la longitud deseada con control proporcional
 * @param motor Referencia al motor
 * @param longitudActual Longitud actual del cable (mm)
 * @param longitudDeseada Longitud deseada del cable (mm)
 * @param direccionPositiva Si true, aumentar longitud es dirección positiva (horario)
 * @return true si está dentro de tolerancia, false si se está moviendo
 */
bool moverHaciaLongitud(Motor &motor, float longitudActual, float longitudDeseada, bool direccionPositiva) {
  float error = longitudDeseada - longitudActual;
  
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
      
      // Comando ANG para establecer ángulos
      if (comando.startsWith("ANG")) {
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
            
            // Control en bucle hasta alcanzar posiciones
            unsigned long timeout = millis() + 30000; // 30 segundos timeout
            bool motor1Listo = false;
            bool motor2Listo = false;
            bool motor3Listo = false;
            
            while (!(motor1Listo && motor2Listo && motor3Listo) && millis() < timeout) {
              // Actualizar longitudes
              actualizarLongitudesActuales();
              
              // Mover cada motor hacia su objetivo
              motor1Listo = moverHaciaLongitud(motor1, L1, dL1, false); // Motor 1: pulsos negativos = aumentar L
              motor2Listo = moverHaciaLongitud(motor2, L2, dL2, true);  // Motor 2: pulsos positivos = aumentar L
              motor3Listo = moverHaciaLongitud(motor3, L3, dL3, false); // Motor 3: pulsos negativos = aumentar L
              
              delay(10);
            }
            
            // Detener todos los motores
            motor1.mover(0);
            motor2.mover(0);
            motor3.mover(0);
            
            if (motor1Listo && motor2Listo && motor3Listo) {
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
