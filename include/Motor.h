#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/*
NOTAS SOBRE PINES ESP32 PARA INTERRUPCIONES:
✅ BUENOS PARA ENCODERS: GPIO25, GPIO26, GPIO27, GPIO14, GPIO12, GPIO13, GPIO15, GPIO2, GPIO32, GPIO33
⚠️ EVITAR: GPIO4 (problemas con boot), GPIO0 (boot), GPIO2 (boot LED), GPIO6-11 (flash)
🔧 CAMBIO REALIZADO: Motor 3 cambió de pines 27,4 a pines 25,26 (más confiables)
*/

/**
 * @brief Clase para controlar motores con encoders quadrature
 * 
 * Esta clase permite controlar motores paso a paso o DC con encoders quadrature,
 * proporcionando funciones para movimiento, calibración, y lectura de posición/velocidad.
 */
class Motor {
  private:
    int pinI1, pinI2;                    // Pines de control del motor
    int pinEncA, pinEncB;                // Pines del encoder
    volatile long posicion;              // Posición actual del encoder
    long posicionAnterior;               // Posición anterior para calcular velocidad
    long velocidad;                      // Velocidad calculada
    unsigned long tiempoAnterior;        // Tiempo anterior para mediciones
    static const unsigned long intervaloMedicion = 500; // Intervalo de medición en ms
    long pulsosPorRevolucion;            // Pulsos por revolución completa (configurable)
    
  public:
    /**
     * @brief Constructor de la clase Motor
     * @param i1 Pin de control I1 del motor
     * @param i2 Pin de control I2 del motor
     * @param encA Pin A del encoder
     * @param encB Pin B del encoder
     * @param pulsosRev Pulsos por revolución completa (por defecto 14000)
     */
    Motor(int i1, int i2, int encA, int encB, long pulsosRev = 14000);
    
    /**
     * @brief Inicializa el motor configurando pines y estado inicial
     */
    void inicializar();
    
    /**
     * @brief Mueve el motor en la dirección especificada
     * @param direccion 1=horario, 0=detener, -1=antihorario
     */
    void mover(int direccion);
    
    /**
     * @brief Maneja las interrupciones del encoder (Quadrature Encoding)
     * Esta función debe ser llamada en las interrupciones de los pines A y B del encoder
     */
    void manejarEncoder();
    
    /**
     * @brief Lee y actualiza los datos del encoder y calcula velocidad
     */
    void leer();
    
    /**
     * @brief Calibra los pulsos por revolución completa
     * Gira el motor una revolución completa en ambas direcciones y mide los pulsos
     */
    void calibrarRevolucion();
    
    /**
     * @brief Gira el motor un número específico de grados
     * @param grados Grados a girar (positivo=horario, negativo=antihorario)
     */
    void girarGrados(float grados);
    
    /**
     * @brief Establece el número de pulsos por revolución
     * @param pulsos Nuevo valor de pulsos por revolución
     */
    void setPulsosPorRevolucion(long pulsos);
    
    /**
     * @brief Obtiene el número de pulsos por revolución configurado
     * @return Pulsos por revolución
     */
    long getPulsosPorRevolucion();
    
    /**
     * @brief Obtiene la posición actual del encoder
     * @return Posición en pulsos
     */
    long getPosicion();
    
    /**
     * @brief Resetea la posición del encoder a 0
     * Útil para recalibrar el sistema en cualquier momento
     */
    void resetPosicion();
    
    /**
     * @brief Obtiene la velocidad calculada
     * @return Velocidad en pulsos por intervalo de medición
     */
    long getVelocidad();
    
    /**
     * @brief Obtiene el pin A del encoder
     * @return Número del pin A
     */
    int getPinEncA();
    
    /**
     * @brief Obtiene el pin B del encoder
     * @return Número del pin B
     */
    int getPinEncB();
    
    /**
     * @brief Obtiene un puntero a la variable de posición (para interrupciones)
     * @return Puntero volatile a la posición
     */
    volatile long* getPosicionPtr();
};

#endif // MOTOR_H