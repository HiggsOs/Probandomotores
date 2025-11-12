#include <Arduino.h>
#include "Motor.h"

// Crear instancias de los 3 motores con los parámetros especificados
Motor motor1(18, 5, 25, 26, 25200);
Motor motor2(17, 16, 32, 33, 25200);  // Pines I1 e I2 invertidos para corregir dirección
Motor motor3(23, 19, 27, 4, 25200);

// Variables para detectar si el encoder está funcionando
unsigned long ultimaActualizacion1 = 0;
unsigned long ultimaActualizacion2 = 0;
unsigned long ultimaActualizacion3 = 0;

// Tiempo máximo sin cambio en encoder (ms) antes de detener el motor
const unsigned long TIMEOUT_ENCODER = 100;  // 100ms sin cambio = detener

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

/**
 * @brief Gira un motor con detección de encoder
 * @param motor Referencia al motor a girar
 * @param grados Grados a girar (positivo=horario, negativo=antihorario)
 * @param ultimaActualizacion Referencia al tiempo de última actualización del encoder
 * @param nombreMotor Nombre del motor para mensajes de debug
 * @return true si el giro fue exitoso, false si se detectó falla del encoder
 */
bool girarGradosConDeteccion(Motor &motor, float grados, unsigned long &ultimaActualizacion, const char* nombreMotor) {
  // Calcular pulsos necesarios
  long pulsosObjetivo = (long)((grados / 360.0) * motor.getPulsosPorRevolucion());
  
  Serial.print(nombreMotor);
  Serial.print(": Girando ");
  Serial.print(grados);
  Serial.print(" grados (");
  Serial.print(abs(pulsosObjetivo));
  Serial.println(" pulsos)");
  
  long posicionInicial = motor.getPosicion();
  int direccion = (pulsosObjetivo >= 0) ? 1 : -1;  // 1 para horario, -1 para antihorario
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("   Control Interactivo de 3 Motores");
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
  Serial.println("• Formato: M[motor] [grados]");
  Serial.println("  Ejemplos:");
  Serial.println("    M1 360    → Motor 1, 360° horario");
  Serial.println("    M2 -180   → Motor 2, 180° antihorario");
  Serial.println("    M3 90     → Motor 3, 90° horario");
  Serial.println("");
  Serial.println("• Comandos especiales:");
  Serial.println("    pos       → Ver posiciones de todos");
  Serial.println("    M1 pos    → Ver posición del Motor 1");
  Serial.println("    M2 pos    → Ver posición del Motor 2");
  Serial.println("    M3 pos    → Ver posición del Motor 3");
  Serial.println("========================================\n");
  Serial.println("Listo para recibir comandos...\n");
}

void loop() {
  // Leer y actualizar datos de los motores continuamente
  motor1.leer();
  motor2.leer();
  motor3.leer();
  
  // Verificar si hay datos disponibles en el Serial
  if (Serial.available() > 0) {
    // Leer el comando completo
    String comando = Serial.readStringUntil('\n');
    comando.trim();  // Eliminar espacios en blanco
    comando.toUpperCase();  // Convertir a mayúsculas para facilitar comparación
    
    if (comando.length() > 0) {
      Serial.print("\n> Comando recibido: ");
      Serial.println(comando);
      
      // Comando para ver todas las posiciones
      if (comando.equals("POS")) {
        Serial.println("\n========== POSICIONES ACTUALES ==========");
        
        Serial.print("Motor 1: ");
        Serial.print(motor1.getPosicion());
        Serial.print(" pulsos (");
        float grados1 = (motor1.getPosicion() * 360.0) / motor1.getPulsosPorRevolucion();
        Serial.print(grados1, 2);
        Serial.println(" grados)");
        
        Serial.print("Motor 2: ");
        Serial.print(motor2.getPosicion());
        Serial.print(" pulsos (");
        float grados2 = (motor2.getPosicion() * 360.0) / motor2.getPulsosPorRevolucion();
        Serial.print(grados2, 2);
        Serial.println(" grados)");
        
        Serial.print("Motor 3: ");
        Serial.print(motor3.getPosicion());
        Serial.print(" pulsos (");
        float grados3 = (motor3.getPosicion() * 360.0) / motor3.getPulsosPorRevolucion();
        Serial.print(grados3, 2);
        Serial.println(" grados)");
        
        Serial.println("=========================================\n");
      }
      // Comandos para motores individuales (M1, M2, M3)
      else if (comando.startsWith("M1") || comando.startsWith("M2") || comando.startsWith("M3")) {
        // Extraer número de motor
        int numMotor = comando.charAt(1) - '0';  // '1' -> 1, '2' -> 2, '3' -> 3
        
        // Extraer el argumento (grados o "pos")
        String argumento = comando.substring(2);
        argumento.trim();
        
        // Seleccionar motor, variable de actualización y nombre
        Motor* motorSeleccionado;
        unsigned long* ultimaActualizacion;
        String nombreMotor;
        
        switch(numMotor) {
          case 1:
            motorSeleccionado = &motor1;
            ultimaActualizacion = &ultimaActualizacion1;
            nombreMotor = "Motor 1";
            break;
          case 2:
            motorSeleccionado = &motor2;
            ultimaActualizacion = &ultimaActualizacion2;
            nombreMotor = "Motor 2";
            break;
          case 3:
            motorSeleccionado = &motor3;
            ultimaActualizacion = &ultimaActualizacion3;
            nombreMotor = "Motor 3";
            break;
          default:
            Serial.println("❌ Error: Motor no válido (usa M1, M2 o M3)\n");
            return;
        }
        
        // Verificar si el argumento es "pos"
        if (argumento.equals("POS")) {
          Serial.println("\n--- POSICIÓN ---");
          Serial.print(nombreMotor);
          Serial.print(": ");
          Serial.print(motorSeleccionado->getPosicion());
          Serial.print(" pulsos (");
          float grados = (motorSeleccionado->getPosicion() * 360.0) / motorSeleccionado->getPulsosPorRevolucion();
          Serial.print(grados, 2);
          Serial.println(" grados)");
          Serial.println("----------------\n");
        }
        // Si no es "pos", interpretar como número de grados
        else if (argumento.length() > 0) {
          float grados = argumento.toFloat();
          
          // Validar que sea un número válido
          if (grados == 0.0 && argumento != "0" && argumento != "0.0") {
            Serial.println("❌ Error: Valor no válido");
            Serial.println("   Usa formato: M1 360  o  M1 pos\n");
          } else {
            // Ejecutar movimiento
            Serial.println("\n--- INICIANDO MOVIMIENTO ---");
            bool exito = girarGradosConDeteccion(*motorSeleccionado, grados, *ultimaActualizacion, nombreMotor.c_str());
            
            if (exito) {
              Serial.println("✓ Movimiento completado\n");
            } else {
              Serial.println("❌ Movimiento fallido - Verifica encoder\n");
            }
            
            // Mostrar posición final
            Serial.print("Posición actual: ");
            Serial.print(motorSeleccionado->getPosicion());
            Serial.println(" pulsos\n");
          }
        } else {
          Serial.println("❌ Error: Falta especificar grados o 'pos'");
          Serial.println("   Ejemplos: M1 360  o  M1 pos\n");
        }
      }
      // Comando no reconocido
      else {
        Serial.println("❌ Comando no reconocido");
        Serial.println("   Usa: M1 [grados], M2 [grados], M3 [grados], o 'pos'\n");
      }
      
      Serial.println("Listo para siguiente comando...\n");
    }
  }
  
  delay(10);  // Pequeño delay para no saturar
}
