#include <Arduino.h>
#include "Motor.h"
#include "CinematicaInversa.h"

// Crear instancias de los 3 motores con los parámetros especificados
Motor motor1(18, 5, 25, 26, 25200);     // ENCA= 25, ENCB 26
Motor motor2(17, 16, 32, 33, 25200);  // ENCA= 32, ENCB 33
Motor motor3(23, 19, 27, 4, 25200);

// Crear instancia de cinemática inversa
CinematicaInversa cinematica;

// Variables para detectar si el encoder está funcionando
unsigned long ultimaActualizacion1 = 0;
unsigned long ultimaActualizacion2 = 0;
unsigned long ultimaActualizacion3 = 0;

// Tiempo máximo sin cambio en encoder (ms) antes de detener el motor
const unsigned long TIMEOUT_ENCODER = 100;  // 100ms sin cambio = detener

// Variable para parada de emergencia
volatile bool paradaEmergencia = false;

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
 * @brief Mueve los 3 motores simultáneamente a los ángulos especificados
 * @param grados1 Grados para Motor 1
 * @param grados2 Grados para Motor 2
 * @param grados3 Grados para Motor 3
 * @return true si todos completaron exitosamente, false si alguno falló
 */
bool moverMotoresSimultaneos(float grados1, float grados2, float grados3) {
  // Calcular pulsos objetivo para cada motor
  long pulsos1 = (long)((grados1 / 360.0) * motor1.getPulsosPorRevolucion());
  long pulsos2 = (long)((grados2 / 360.0) * motor2.getPulsosPorRevolucion());
  long pulsos3 = (long)((grados3 / 360.0) * motor3.getPulsosPorRevolucion());
  
  Serial.println("\n========== MOVIMIENTO SIMULTÁNEO ==========");
  Serial.print("Motor 1: ");
  Serial.print(grados1, 1);
  Serial.print("° (");
  Serial.print(abs(pulsos1));
  Serial.println(" pulsos)");
  
  Serial.print("Motor 2: ");
  Serial.print(grados2, 1);
  Serial.print("° (");
  Serial.print(abs(pulsos2));
  Serial.println(" pulsos)");
  
  Serial.print("Motor 3: ");
  Serial.print(grados3, 1);
  Serial.print("° (");
  Serial.print(abs(pulsos3));
  Serial.println(" pulsos)");
  Serial.println("Presiona 'S' o 'STOP' para parada de emergencia\n");
  
  // Guardar posiciones iniciales
  long pos1Inicial = motor1.getPosicion();
  long pos2Inicial = motor2.getPosicion();
  long pos3Inicial = motor3.getPosicion();
  
  // Determinar direcciones
  int dir1 = (pulsos1 >= 0) ? 1 : -1;
  int dir2 = (pulsos2 >= 0) ? 1 : -1;
  int dir3 = (pulsos3 >= 0) ? 1 : -1;
  
  pulsos1 = abs(pulsos1);
  pulsos2 = abs(pulsos2);
  pulsos3 = abs(pulsos3);
  
  // Resetear tiempos y parada de emergencia
  ultimaActualizacion1 = millis();
  ultimaActualizacion2 = millis();
  ultimaActualizacion3 = millis();
  paradaEmergencia = false;
  
  // Banderas para saber qué motores han terminado
  bool motor1Completado = (pulsos1 == 0);
  bool motor2Completado = (pulsos2 == 0);
  bool motor3Completado = (pulsos3 == 0);
  
  // Iniciar todos los motores
  if (!motor1Completado) motor1.mover(dir1);
  if (!motor2Completado) motor2.mover(dir2);
  if (!motor3Completado) motor3.mover(dir3);
  
  unsigned long ultimoReporte = millis();
  bool errorDetectado = false;
  
  // Continuar mientras al menos un motor esté activo
  while (!motor1Completado || !motor2Completado || !motor3Completado) {
    // Verificar parada de emergencia
    if (paradaEmergencia) {
      motor1.mover(0);
      motor2.mover(0);
      motor3.mover(0);
      Serial.println("\n PARADA DE EMERGENCIA ACTIVADA");
      Serial.println("Posiciones finales:");
      Serial.print("  Motor 1: ");
      Serial.print(motor1.getPosicion());
      Serial.println(" pulsos");
      Serial.print("  Motor 2: ");
      Serial.print(motor2.getPosicion());
      Serial.println(" pulsos");
      Serial.print("  Motor 3: ");
      Serial.print(motor3.getPosicion());
      Serial.println(" pulsos\n");
      paradaEmergencia = false;
      return false;
    }
    
    // Verificar comando de stop desde serial
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      cmd.toUpperCase();
      if (cmd == "S" || cmd == "STOP") {
        paradaEmergencia = true;
        continue;
      }
    }
    
    // Verificar y detener Motor 1 si alcanzó objetivo
    if (!motor1Completado) {
      if (abs(motor1.getPosicion() - pos1Inicial) >= pulsos1) {
        motor1.mover(0);
        motor1Completado = true;
        Serial.println("\nMotor 1 completado");
      }
      // Verificar timeout del encoder
      else if (millis() - ultimaActualizacion1 > TIMEOUT_ENCODER) {
        motor1.mover(0);
        Serial.println("\n ERROR Motor 1: Encoder no detectado");
        errorDetectado = true;
        motor1Completado = true;
      }
    }
    
    // Verificar y detener Motor 2 si alcanzó objetivo
    if (!motor2Completado) {
      if (abs(motor2.getPosicion() - pos2Inicial) >= pulsos2) {
        motor2.mover(0);
        motor2Completado = true;
        Serial.println("Motor 2 completado");
      }
      // Verificar timeout del encoder
      else if (millis() - ultimaActualizacion2 > TIMEOUT_ENCODER) {
        motor2.mover(0);
        Serial.println(" ERROR Motor 2: Encoder no detectado");
        errorDetectado = true;
        motor2Completado = true;
      }
    }
    
    // Verificar y detener Motor 3 si alcanzó objetivo
    if (!motor3Completado) {
      if (abs(motor3.getPosicion() - pos3Inicial) >= pulsos3) {
        motor3.mover(0);
        motor3Completado = true;
        Serial.println("Motor 3 completado");
      }
      // Verificar timeout del encoder
      else if (millis() - ultimaActualizacion3 > TIMEOUT_ENCODER) {
        motor3.mover(0);
        Serial.println(" ERROR Motor 3: Encoder no detectado");
        errorDetectado = true;
        motor3Completado = true;
      }
    }
    
    // Mostrar progreso cada 300ms
    if (millis() - ultimoReporte > 300) {
      Serial.print("\r");
      
      if (!motor1Completado) {
        long p1 = abs(motor1.getPosicion() - pos1Inicial);
        Serial.print("M1:");
        Serial.print((float)p1 / pulsos1 * 100.0, 0);
        Serial.print("% ");
      }
      
      if (!motor2Completado) {
        long p2 = abs(motor2.getPosicion() - pos2Inicial);
        Serial.print("M2:");
        Serial.print((float)p2 / pulsos2 * 100.0, 0);
        Serial.print("% ");
      }
      
      if (!motor3Completado) {
        long p3 = abs(motor3.getPosicion() - pos3Inicial);
        Serial.print("M3:");
        Serial.print((float)p3 / pulsos3 * 100.0, 0);
        Serial.print("%   ");
      }
      
      ultimoReporte = millis();
    }
    
    delay(1);
  }
  
  // Asegurar que todos estén detenidos
  motor1.mover(0);
  motor2.mover(0);
  motor3.mover(0);
  
  // Mostrar resultados finales
  Serial.println("\n\n========== MOVIMIENTO COMPLETADO ==========");
  
  Serial.print("Motor 1: Objetivo=");
  Serial.print(pulsos1);
  Serial.print(" Real=");
  Serial.print(abs(motor1.getPosicion() - pos1Inicial));
  Serial.print(" Pos=");
  Serial.println(motor1.getPosicion());
  
  Serial.print("Motor 2: Objetivo=");
  Serial.print(pulsos2);
  Serial.print(" Real=");
  Serial.print(abs(motor2.getPosicion() - pos2Inicial));
  Serial.print(" Pos=");
  Serial.println(motor2.getPosicion());
  
  Serial.print("Motor 3: Objetivo=");
  Serial.print(pulsos3);
  Serial.print(" Real=");
  Serial.print(abs(motor3.getPosicion() - pos3Inicial));
  Serial.print(" Pos=");
  Serial.println(motor3.getPosicion());
  
  Serial.println("===========================================\n");
  
  return !errorDetectado;
}

/**
 * @brief Gira un motor con detección de encoder y visualización en tiempo real
 * @param motor Referencia al motor a girar
 * @param grados Grados a girar (positivo=horario, negativo=antihorario)
 * @param ultimaActualizacion Referencia al tiempo de última actualización del encoder
 * @param nombreMotor Nombre del motor para mensajes de debug
 * @return true si el giro fue exitoso, false si se detectó falla del encoder
 */
bool girarGradosConDeteccion(Motor &motor, float grados, unsigned long &ultimaActualizacion, const char* nombreMotor) {
  // Calcular pulsos necesarios
  long pulsosObjetivo = (long)((grados / 360.0) * motor.getPulsosPorRevolucion());
  
  Serial.print("\n");
  Serial.print(nombreMotor);
  Serial.print(": Girando ");
  Serial.print(grados, 1);
  Serial.print(" grados (");
  Serial.print(abs(pulsosObjetivo));
  Serial.println(" pulsos)");
  Serial.println("Presiona 'S' o 'STOP' para parada de emergencia\n");
  
  long posicionInicial = motor.getPosicion();
  int direccion = (pulsosObjetivo >= 0) ? 1 : -1;  // 1 para horario, -1 para antihorario
  pulsosObjetivo = abs(pulsosObjetivo);
  
  // Resetear tiempo de última actualización y parada de emergencia
  ultimaActualizacion = millis();
  paradaEmergencia = false;
  
  // Comenzar movimiento
  motor.mover(direccion);
  
  unsigned long ultimoReporte = millis();
  
  // Continuar hasta alcanzar el objetivo
  while (abs(motor.getPosicion() - posicionInicial) < pulsosObjetivo) {
    // Verificar parada de emergencia
    if (paradaEmergencia) {
      motor.mover(0);
      Serial.println("\n PARADA DE EMERGENCIA ACTIVADA");
      Serial.print("Posición final: ");
      Serial.print(motor.getPosicion());
      Serial.println(" pulsos\n");
      paradaEmergencia = false;
      return false;
    }
    
    // Verificar comando de stop desde serial
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      cmd.toUpperCase();
      if (cmd == "S" || cmd == "STOP") {
        paradaEmergencia = true;
        continue;
      }
    }
    
    // DETECCIÓN CRÍTICA: Verificar si el encoder está respondiendo
    if (millis() - ultimaActualizacion > TIMEOUT_ENCODER) {
      motor.mover(0);
      Serial.print("\n ERROR: ");
      Serial.print(nombreMotor);
      Serial.println(" - Encoder no detectado. Motor detenido.");
      Serial.print("Última actualización hace: ");
      Serial.print(millis() - ultimaActualizacion);
      Serial.println(" ms\n");
      return false;
    }
    
    // Mostrar progreso cada 200ms
    if (millis() - ultimoReporte > 200) {
      long pulsosActuales = abs(motor.getPosicion() - posicionInicial);
      float porcentaje = (float)pulsosActuales / pulsosObjetivo * 100.0;
      
      Serial.print("\r");  // Retorno de carro para actualizar en la misma línea
      Serial.print("Progreso: ");
      Serial.print(pulsosActuales);
      Serial.print(" / ");
      Serial.print(pulsosObjetivo);
      Serial.print(" pulsos (");
      Serial.print(porcentaje, 1);
      Serial.print("%)   ");
      
      ultimoReporte = millis();
    }
    
    delay(1);
  }
  
  // Detener motor al alcanzar el objetivo
  motor.mover(0);
  
  long pulsosReales = abs(motor.getPosicion() - posicionInicial);
  Serial.print("\n\n");
  Serial.print(nombreMotor);
  Serial.print(" completado");
  Serial.print("\n  Objetivo: ");
  Serial.print(pulsosObjetivo);
  Serial.print(" pulsos");
  Serial.print("\n  Real: ");
  Serial.print(pulsosReales);
  Serial.print(" pulsos");
  Serial.print("\n  Posición total: ");
  Serial.print(motor.getPosicion());
  Serial.println(" pulsos\n");
  
  return true;
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
  Serial.println("• Control de motores:");
  Serial.println("    M1 [grados]  → Mover Motor 1 (ej: M1 360)");
  Serial.println("    M2 [grados]  → Mover Motor 2 (ej: M2 -180)");
  Serial.println("    M3 [grados]  → Mover Motor 3 (ej: M3 90)");
  Serial.println("    ALL [g1] [g2] [g3] → Mover los 3 motores simultáneamente");
  Serial.println("");
  Serial.println("• Cinemática inversa:");
  Serial.println("    CI [azimuth] [elevacion] → Calcular sin mover");
  Serial.println("    CIM [azimuth] [elevacion] → Calcular y mover");
  Serial.println("    POSCI        → Ver posición actual (azimuth/elevación)");
  Serial.println("    RESETCI      → Resetear posición a neutra (0°, 0°)");
  Serial.println("");
  Serial.println("• Seguridad:");
  Serial.println("    STOP o S     → Parada de emergencia");
  Serial.println("    r            → Resetear encoders a 0");
  Serial.println("");
  Serial.println("• Formato: M[motor] [grados]");
  Serial.println("  Ejemplos:");
  Serial.println("    M1 360      → Motor 1, 360° horario");
  Serial.println("    M2 -180     → Motor 2, 180° antihorario");
  Serial.println("    M3 90       → Motor 3, 90° horario");
  Serial.println("    ALL 90 180 -90 → M1=90°, M2=180°, M3=-90°");
  Serial.println("    CI 180 45   → Calcular para azimuth=180°, elev=45°");
  Serial.println("    CIM 180 45  → Calcular y mover a azimuth=180°, elev=45°");
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
      
      // Comando STOP / S - Parada de emergencia
      if (comando.equals("STOP") || comando.equals("S")) {
        paradaEmergencia = true;
        motor1.mover(0);
        motor2.mover(0);
        motor3.mover(0);
        Serial.println("\n PARADA DE EMERGENCIA ACTIVADA");
        Serial.println("Todos los motores detenidos.");
        Serial.println("Posiciones actuales:");
        Serial.print("  Motor 1: ");
        Serial.println(motor1.getPosicion());
        Serial.print("  Motor 2: ");
        Serial.println(motor2.getPosicion());
        Serial.print("  Motor 3: ");
        Serial.println(motor3.getPosicion());
        Serial.println("\nListo para nuevos comandos.\n");
        paradaEmergencia = false;  // Resetear para próximo movimiento
      }
      // Comando R - Reset de encoders
      else if (comando.equals("R")) {
        motor1.resetPosicion();
        motor2.resetPosicion();
        motor3.resetPosicion();
        Serial.println("\nTodos los encoders reseteados a 0");
        Serial.println("  Motor 1: 0 pulsos");
        Serial.println("  Motor 2: 0 pulsos");
        Serial.println("  Motor 3: 0 pulsos\n");
      }
      // Comandos R1, R2, R3 - Reset individual
      else if (comando.equals("R1")) {
        motor1.resetPosicion();
        Serial.println("\nMotor 1 reseteado a 0 pulsos\n");
      }
      else if (comando.equals("R2")) {
        motor2.resetPosicion();
        Serial.println("\nMotor 2 reseteado a 0 pulsos\n");
      }
      else if (comando.equals("R3")) {
        motor3.resetPosicion();
        Serial.println("\nMotor 3 reseteado a 0 pulsos\n");
      }
      // Comando POSCI - Ver posición actual de cinemática
      else if (comando.equals("POSCI")) {
        float azimuth, elevacion;
        cinematica.obtenerPosicionActual(azimuth, elevacion);
        
        float L1, L2, L3;
        cinematica.obtenerLongitudesActuales(L1, L2, L3);
        
        float Li1, Li2, Li3;
        cinematica.obtenerLongitudesIniciales(Li1, Li2, Li3);
        
        Serial.println("\n========== POSICIÓN ACTUAL SISTEMA ==========");
        Serial.print("Azimuth:   ");
        Serial.print(azimuth, 2);
        Serial.println("°");
        Serial.print("Elevación: ");
        Serial.print(elevacion, 2);
        Serial.println("°");
        
        Serial.println("\n--- Longitudes de cables ---");
        Serial.print("Cable 1: ");
        Serial.print(L1, 2);
        Serial.print(" mm  (Inicial: ");
        Serial.print(Li1, 2);
        Serial.print(" mm, Delta: ");
        Serial.print(L1 - Li1, 2);
        Serial.println(" mm)");
        
        Serial.print("Cable 2: ");
        Serial.print(L2, 2);
        Serial.print(" mm  (Inicial: ");
        Serial.print(Li2, 2);
        Serial.print(" mm, Delta: ");
        Serial.print(L2 - Li2, 2);
        Serial.println(" mm)");
        
        Serial.print("Cable 3: ");
        Serial.print(L3, 2);
        Serial.print(" mm  (Inicial: ");
        Serial.print(Li3, 2);
        Serial.print(" mm, Delta: ");
        Serial.print(L3 - Li3, 2);
        Serial.println(" mm)");
        
        Serial.println("============================================\n");
      }
      // Comando RESETCI - Resetear posición de cinemática
      else if (comando.equals("RESETCI")) {
        cinematica.resetearPosicion();
      }
      // Comando para ver todas las posiciones
      else if (comando.equals("POS")) {
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
            Serial.println(" Error: Motor no válido (usa M1, M2 o M3)\n");
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
            Serial.println(" Error: Valor no válido");
            Serial.println("   Usa formato: M1 360  o  M1 pos\n");
          } else {
            // Ejecutar movimiento
            Serial.println("\n--- INICIANDO MOVIMIENTO ---");
            bool exito = girarGradosConDeteccion(*motorSeleccionado, grados, *ultimaActualizacion, nombreMotor.c_str());
            
            if (exito) {
              Serial.println(" Movimiento completado\n");
            } else {
              Serial.println("Movimiento fallido - Verifica encoder\n");
            }
            
            // Mostrar posición final
            Serial.print("Posición actual: ");
            Serial.print(motorSeleccionado->getPosicion());
            Serial.println(" pulsos\n");
          }
        } else {
          Serial.println(" Error: Falta especificar grados o 'pos'");
          Serial.println("   Ejemplos: M1 360  o  M1 pos\n");
        }
      }
      // Comando ALL para mover los 3 motores simultáneamente
      else if (comando.startsWith("ALL ")) {
        // Extraer los 3 valores de grados
        String argumentos = comando.substring(4);
        argumentos.trim();
        
        // Parsear los 3 valores separados por espacios
        int espacio1 = argumentos.indexOf(' ');
        int espacio2 = argumentos.lastIndexOf(' ');
        
        if (espacio1 > 0 && espacio2 > espacio1) {
          String str1 = argumentos.substring(0, espacio1);
          String str2 = argumentos.substring(espacio1 + 1, espacio2);
          String str3 = argumentos.substring(espacio2 + 1);
          
          float grados1 = str1.toFloat();
          float grados2 = str2.toFloat();
          float grados3 = str3.toFloat();
          
          // Validar que sean números válidos
          bool validos = true;
          if (grados1 == 0.0 && str1 != "0" && str1 != "0.0") validos = false;
          if (grados2 == 0.0 && str2 != "0" && str2 != "0.0") validos = false;
          if (grados3 == 0.0 && str3 != "0" && str3 != "0.0") validos = false;
          
          if (validos) {
            // Ejecutar movimiento simultáneo
            bool exito = moverMotoresSimultaneos(grados1, grados2, grados3);
            
            if (exito) {
              Serial.println(" Todos los motores completados exitosamente\n");
            } else {
              Serial.println(" Movimiento terminado con errores\n");
            }
          } else {
            Serial.println(" Error: Valores no válidos");
            Serial.println("   Usa formato: ALL [g1] [g2] [g3]");
            Serial.println("   Ejemplo: ALL 90 180 -90\n");
          }
        } else {
          Serial.println(" Error: Debes especificar 3 valores");
          Serial.println("   Formato: ALL [grados1] [grados2] [grados3]");
          Serial.println("   Ejemplo: ALL 90 180 -90\n");
        }
      }
      // Comando CI - Cinemática Inversa (calcular sin mover)
      else if (comando.startsWith("CI ")) {
        // Extraer azimuth y elevación
        String argumentos = comando.substring(3);
        argumentos.trim();
        
        int espacio = argumentos.indexOf(' ');
        
        if (espacio > 0) {
          String strAzimuth = argumentos.substring(0, espacio);
          String strElevacion = argumentos.substring(espacio + 1);
          
          float azimuth = strAzimuth.toFloat();
          float elevacion = strElevacion.toFloat();
          
          // Validar rangos
          if (azimuth >= 0 && azimuth <= 360 && elevacion >= 0 && elevacion <= 90) {
            // Calcular cinemática inversa INCREMENTAL con salida detallada
            ResultadoMovimiento resultado = cinematica.calcularMovimientoIncremental(azimuth, elevacion, true);
            
            // Calcular pulsos necesarios
            long pulsos1 = motor1.gradosAPulsos(resultado.motor1);
            long pulsos2 = motor2.gradosAPulsos(resultado.motor2);
            long pulsos3 = motor3.gradosAPulsos(resultado.motor3);
            
            Serial.println("\n========== CONVERSIÓN A PULSOS ==========");
            Serial.print("Motor 1: ");
            Serial.print(pulsos1);
            Serial.println(" pulsos");
            Serial.print("Motor 2: ");
            Serial.print(pulsos2);
            Serial.println(" pulsos");
            Serial.print("Motor 3: ");
            Serial.print(pulsos3);
            Serial.println(" pulsos");
            Serial.println("=========================================");
            Serial.println("\nUsa CIM para calcular Y mover\n");
          } else {
            Serial.println("Error: Valores fuera de rango");
            Serial.println("   Azimuth: 0-360°");
            Serial.println("   Elevación: 0-90°\n");
          }
        } else {
          Serial.println("Error: Debes especificar azimuth y elevación");
          Serial.println("   Formato: CI [azimuth] [elevacion]");
          Serial.println("   Ejemplo: CI 180 45\n");
        }
      }
      // Comando CIM - Cinemática Inversa y Mover
      else if (comando.startsWith("CIM ")) {
        // Extraer azimuth y elevación
        String argumentos = comando.substring(4);
        argumentos.trim();
        
        int espacio = argumentos.indexOf(' ');
        
        if (espacio > 0) {
          String strAzimuth = argumentos.substring(0, espacio);
          String strElevacion = argumentos.substring(espacio + 1);
          
          float azimuth = strAzimuth.toFloat();
          float elevacion = strElevacion.toFloat();
          
          // Validar rangos
          if (azimuth >= 0 && azimuth <= 360 && elevacion >= 0 && elevacion <= 90) {
            // Calcular cinemática inversa INCREMENTAL con salida detallada
            ResultadoMovimiento resultado = cinematica.calcularMovimientoIncremental(azimuth, elevacion, true);
            
            Serial.println("\nIniciando movimiento basado en cinemática inversa...\n");
            
            // Ejecutar movimiento simultáneo de los 3 motores (movimiento incremental)
            bool exito = moverMotoresSimultaneos(resultado.motor1, resultado.motor2, resultado.motor3);
            
            if (exito) {
              // ACTUALIZAR la posición actual del sistema
              cinematica.actualizarPosicionActual(azimuth, elevacion);
              
              Serial.println("Sistema posicionado correctamente");
              Serial.print("   Azimuth objetivo: ");
              Serial.print(azimuth, 1);
              Serial.println("°");
              Serial.print("   Elevación objetivo: ");
              Serial.print(elevacion, 1);
              Serial.println("°\n");
            } else {
              Serial.println("Movimiento terminado con errores");
              Serial.println("   Posición actual NO actualizada\n");
            }
          } else {
            Serial.println("Error: Valores fuera de rango");
            Serial.println("   Azimuth: 0-360°");
            Serial.println("   Elevación: 0-90°\n");
          }
        } else {
          Serial.println("Error: Debes especificar azimuth y elevación");
          Serial.println("   Formato: CIM [azimuth] [elevacion]");
          Serial.println("   Ejemplo: CIM 180 45\n");
        }
      }
      // Comando no reconocido
      else {
        Serial.println("\nComando no reconocido\n");
        Serial.println("COMANDOS DISPONIBLES:");
        Serial.println("  M1 [grados]       → Mover Motor 1");
        Serial.println("  M2 [grados]       → Mover Motor 2");
        Serial.println("  M3 [grados]       → Mover Motor 3");
        Serial.println("  ALL [g1] [g2] [g3] → Mover los 3 simultáneamente");
        Serial.println("  CI [az] [el]      → Calcular cinemática inversa");
        Serial.println("  CIM [az] [el]     → Calcular y mover");
        Serial.println("  POSCI             → Ver posición actual sistema");
        Serial.println("  RESETCI           → Resetear posición a neutra");
        Serial.println("  STOP o S          → Parada de emergencia");
        Serial.println("  R                 → Reset todos los encoders");
        Serial.println("  R1/R2/R3          → Reset encoder individual");
        Serial.println("  POS               → Ver todas las posiciones");
        Serial.println("  M1/M2/M3 POS      → Ver posición de un motor");
        Serial.println("\nEjemplos:");
        Serial.println("  M1 360         → Motor 1, 360° horario");
        Serial.println("  M2 -180        → Motor 2, 180° antihorario");
        Serial.println("  ALL 90 180 -90 → M1=90°, M2=180°, M3=-90°");
        Serial.println("  CI 180 45      → Calcular para azimuth=180°, elev=45°");
        Serial.println("  CIM 180 45     → Calcular y mover a esa posición");
        Serial.println("  POSCI          → Ver posición actual del sistema");
        Serial.println("  S              → Parar todo\n");
      }
      
      Serial.println("Listo para siguiente comando...\n");
    }
  }
  
  delay(10);  // Pequeño delay para no saturar
}
