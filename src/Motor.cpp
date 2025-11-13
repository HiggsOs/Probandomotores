 #include "Motor.h"

// Constructor
Motor::Motor(int i1, int i2, int encA, int encB, long pulsosRev) {
  pinI1 = i1;
  pinI2 = i2;
  pinEncA = encA;
  pinEncB = encB;
  posicion = 0;
  posicionAnterior = 0;
  velocidad = 0;
  tiempoAnterior = 0;
  pulsosPorRevolucion = pulsosRev;  // Valor por defecto 14000, pero configurable
}

// Inicializar el motor
void Motor::inicializar() {
  // Configurar pines de motor como salidas
  pinMode(pinI1, OUTPUT);
  pinMode(pinI2, OUTPUT);
  
  // Configurar pines de encoder como entradas con pull-up
  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);
  
  // Detener motor al inicio
  mover(0);
}

// Mover motor: 1=horario, 0=detener, -1=antihorario
void Motor::mover(int direccion) {
  switch (direccion) {
    case 1:  // Horario
      digitalWrite(pinI1, LOW);
      digitalWrite(pinI2, HIGH);
      break;
    case -1: // Antihorario
      digitalWrite(pinI1, HIGH);
      digitalWrite(pinI2, LOW);
      break;
    case 0:  // Detener
    default:
      digitalWrite(pinI1, LOW);
      digitalWrite(pinI2, LOW);
      break;
  }
}

// Función para manejar interrupción del encoder (Quadrature Encoding)
void Motor::manejarEncoder() {
  // Leer estado actual de ambos canales
  bool estadoA = digitalRead(pinEncA);
  bool estadoB = digitalRead(pinEncB);
  
  // Variables estáticas para recordar el estado anterior
  static bool estadoA_anterior = false;
  static bool estadoB_anterior = false;
  
  // Detectar transición en canal A
  if (estadoA != estadoA_anterior) {
    if (estadoA == HIGH) {
      // Flanco ascendente en A
      if (estadoB == LOW) {
        posicion++;  // Giro horario
      } else {
        posicion--;  // Giro antihorario
      }
    } else {
      // Flanco descendente en A
      if (estadoB == HIGH) {
        posicion++;  // Giro horario
      } else {
        posicion--;  // Giro antihorario
      }
    }
  }
  
  // Detectar transición en canal B
  if (estadoB != estadoB_anterior) {
    if (estadoB == HIGH) {
      // Flanco ascendente en B
      if (estadoA == HIGH) {
        posicion++;  // Giro horario
      } else {
        posicion--;  // Giro antihorario
      }
    } else {
      // Flanco descendente en B
      if (estadoA == LOW) {
        posicion++;  // Giro horario
      } else {
        posicion--;  // Giro antihorario
      }
    }
  }
  
  // Actualizar estados anteriores
  estadoA_anterior = estadoA;
  estadoB_anterior = estadoB;
}

// Leer datos del encoder y calcular velocidad
void Motor::leer() {
  unsigned long tiempoActual = millis();
  
  if (tiempoActual - tiempoAnterior >= intervaloMedicion) {
    // Calcular velocidad (pulsos por intervalo)
    velocidad = posicion - posicionAnterior;
    posicionAnterior = posicion;
    tiempoAnterior = tiempoActual;
  }
}

// Calibrar pulsos por revolución completa
void Motor::calibrarRevolucion() {
  Serial.println("Iniciando calibración de revolución completa...");
  Serial.print("Valor actual de pulsos por revolución: ");
  Serial.println(pulsosPorRevolucion);
  
  // Resetear posición
  long posicionInicial = posicion;
  
  // Girar en una dirección hasta completar los pulsos esperados
  Serial.println("Girando en sentido horario...");
  mover(1);  // Horario
  
  while (abs(posicion - posicionInicial) < pulsosPorRevolucion) {
    delay(10);  // Pequeño delay para no saturar
    if (abs(posicion - posicionInicial) % 1000 == 0) {
      Serial.print("Pulsos: ");
      Serial.println(abs(posicion - posicionInicial));
    }
  }
  
  // Detener motor
  mover(0);
  long pulsosHorario = abs(posicion - posicionInicial);
  Serial.print("Pulsos detectados en sentido horario: ");
  Serial.println(pulsosHorario);
  
  delay(2000);  // Pausa
  
  // Resetear para la prueba en sentido contrario
  posicionInicial = posicion;
  
  // Girar en dirección contraria
  Serial.println("Girando en sentido antihorario...");
  mover(-1);  // Antihorario
  
  while (abs(posicion - posicionInicial) < pulsosPorRevolucion) {
    delay(10);
    if (abs(posicion - posicionInicial) % 1000 == 0) {
      Serial.print("Pulsos: ");
      Serial.println(abs(posicion - posicionInicial));
    }
  }
  
  // Detener motor
  mover(0);
  long pulsosAntihorario = abs(posicion - posicionInicial);
  Serial.print("Pulsos detectados en sentido antihorario: ");
  Serial.println(pulsosAntihorario);
  
  // Calcular nuevo valor y sugerir actualización
  long pulsosPromedio = (pulsosHorario + pulsosAntihorario) / 2;
  
  // Mostrar resultados
  Serial.println("=== RESULTADOS DE CALIBRACIÓN ===");
  Serial.print("Pulsos horario: ");
  Serial.println(pulsosHorario);
  Serial.print("Pulsos antihorario: ");
  Serial.println(pulsosAntihorario);
  Serial.print("Promedio medido: ");
  Serial.println(pulsosPromedio);
  Serial.print("Valor configurado: ");
  Serial.println(pulsosPorRevolucion);
  
  // Sugerir actualización si hay diferencia significativa
  if (abs(pulsosPromedio - pulsosPorRevolucion) > 100) {
    Serial.println("¡RECOMENDACIÓN: Actualizar pulsos por revolución!");
    Serial.print("Valor sugerido: ");
    Serial.println(pulsosPromedio);
  } else {
    Serial.println("Valor actual es correcto.");
  }
  Serial.println("================================");
}

// Girar un número específico de grados
void Motor::girarGrados(float grados) {
  // Calcular pulsos necesarios usando el valor configurable
  long pulsosObjetivo = (long)((grados / 360.0) * pulsosPorRevolucion);
  
  Serial.print("Girando ");
  Serial.print(grados);
  Serial.print(" grados (");
  Serial.print(pulsosObjetivo);
  Serial.print(" pulsos, usando ");
  Serial.print(pulsosPorRevolucion);
  Serial.println(" pulsos/rev)");
  
  long posicionInicial = posicion;
  int direccion = (pulsosObjetivo >= 0) ? 1 : -1;  // 1 para horario, -1 para antihorario
  pulsosObjetivo = abs(pulsosObjetivo);
  
  // Comenzar movimiento
  mover(direccion);
  
  // Continuar hasta alcanzar el objetivo
  while (abs(posicion - posicionInicial) < pulsosObjetivo) {
    delay(1);  // Delay mínimo para no saturar
  }
  
  // Detener motor al alcanzar el objetivo
  mover(0);
  
  long pulsosReales = abs(posicion - posicionInicial);
  Serial.print("Objetivo: ");
  Serial.print(pulsosObjetivo);
  Serial.print(" pulsos, Real: ");
  Serial.print(pulsosReales);
  Serial.println(" pulsos");
}

// Métodos para configurar y obtener pulsos por revolución
void Motor::setPulsosPorRevolucion(long pulsos) {
  pulsosPorRevolucion = pulsos;
  Serial.print("Pulsos por revolución actualizados a: ");
  Serial.println(pulsosPorRevolucion);
}

long Motor::getPulsosPorRevolucion() {
  return pulsosPorRevolucion;
}

// Getters para acceder a los datos
long Motor::getPosicion() { 
  return posicion; 
}

long Motor::getVelocidad() { 
  return velocidad; 
}

int Motor::getPinEncA() { 
  return pinEncA; 
}

int Motor::getPinEncB() { 
  return pinEncB; 
}

// Función estática para usar en attachInterrupt
volatile long* Motor::getPosicionPtr() { 
  return &posicion; 
}

// Resetear posición del encoder a cero
void Motor::resetPosicion() {
  posicion = 0;
  posicionAnterior = 0;
}