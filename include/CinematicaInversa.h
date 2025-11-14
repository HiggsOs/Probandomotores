/**
 * @file CinematicaInversa.h
 * @brief Cinemática Inversa - Sistema de Seguimiento Solar
 * 
 * Esta clase calcula los grados que debe mover cada motor para alcanzar
 * una posición deseada de azimuth y elevación.
 * 
 * Entrada: Azimuth y Elevación (en grados)
 * Salida: Grados a mover cada uno de los 3 motores
 */

#ifndef CINEMATICA_INVERSA_H
#define CINEMATICA_INVERSA_H

#include <Arduino.h>
#include <math.h>

/**
 * @struct Coordenada3D
 * @brief Estructura para almacenar coordenadas en 3D
 */
struct Coordenada3D {
    float x;
    float y;
    float z;
};

/**
 * @struct ResultadoMovimiento
 * @brief Estructura para almacenar el resultado del cálculo de movimiento
 */
struct ResultadoMovimiento {
    float motor1;   // Grados a mover motor 1
    float motor2;   // Grados a mover motor 2
    float motor3;   // Grados a mover motor 3
    float deltaL1;  // Cambio de longitud cable 1 (mm)
    float deltaL2;  // Cambio de longitud cable 2 (mm)
    float deltaL3;  // Cambio de longitud cable 3 (mm)
};

/**
 * @class CinematicaInversa
 * @brief Clase para calcular cinemática inversa del seguidor solar
 */
class CinematicaInversa {
private:
    // ========== PARÁMETROS GEOMÉTRICOS ==========
    float H;  // Altura de la plataforma (mm)
    
    // Coordenadas de las juntas proximales (base) - en mm
    Coordenada3D Base1;
    Coordenada3D Base2;
    Coordenada3D Base3;
    
    // Coordenadas de las juntas distales (plataforma móvil) - en mm
    Coordenada3D Plat1;
    Coordenada3D Plat2;
    Coordenada3D Plat3;
    
    // ========== PARÁMETROS DE CONVERSIÓN ==========
    float RADIO_POLEA;   
    float km;     // Radio de la polea (mm)
    float FACTOR_REDUCCION;   // Factor de reducción mecánica
    float MM_A_GRADOS;        // Conversión: ΔL (mm) a grados de rotación
    
    // Longitudes iniciales (posición neutra: 0°, 0°)
    float Li1;
    float Li2;
    float Li3;
    
    // Posición actual del sistema
    float azimuthActual;
    float elevacionActual;
    
    /**
     * @brief Calcula la longitud del cable usando cinemática inversa
     * @param A Coordenadas de junta proximal
     * @param B Coordenadas de junta distal
     * @param tx_rad Ángulo de rotación en X (radianes)
     * @param ty_rad Ángulo de rotación en Y (radianes)
     * @return Longitud del cable en mm
     */
    float calcularLongitudCable(const Coordenada3D& A, const Coordenada3D& B, 
                                float tx_rad, float ty_rad);
    
    /**
     * @brief Convierte cambio de longitud (mm) a grados de rotación del motor
     * @param deltaL_mm Cambio de longitud en milímetros
     * @return Grados de rotación necesarios
     */
    float deltaLAGrados(float deltaL_mm);
    
    /**
     * @brief Convierte azimuth y elevación del sol a ángulos de la plataforma
     * @param azimuth_deg Azimuth solar en grados (0-360°)
     * @param elevacion_deg Elevación solar en grados (0-90°)
     * @param tx_grados Salida: ángulo TX en grados
     * @param ty_grados Salida: ángulo TY en grados
     */
    void azimuthElevacionATxTy(float azimuth_deg, float elevacion_deg,
                               float& tx_grados, float& ty_grados);

public:
    /**
     * @brief Constructor - Inicializa los parámetros geométricos del sistema
     */
    CinematicaInversa();
    
    /**
     * @brief Calcula los grados que debe mover cada motor para alcanzar la posición solar (ABSOLUTO desde 0,0)
     * @param azimuth_deg Azimuth del sol en grados (0-360°)
     * @param elevacion_deg Elevación del sol en grados (0-90°)
     * @param verbose Si true, muestra información detallada por Serial
     * @return Estructura ResultadoMovimiento con los grados a mover cada motor
     */
    ResultadoMovimiento calcularMovimiento(float azimuth_deg, float elevacion_deg, 
                                          bool verbose = true);
    
    /**
     * @brief Calcula los grados que debe mover cada motor INCREMENTALMENTE desde la posición actual
     * @param azimuth_objetivo Azimuth objetivo en grados (0-360°)
     * @param elevacion_objetivo Elevación objetivo en grados (0-90°)
     * @param verbose Si true, muestra información detallada por Serial
     * @return Estructura ResultadoMovimiento con los grados INCREMENTALES a mover
     */
    ResultadoMovimiento calcularMovimientoIncremental(float azimuth_objetivo, 
                                                      float elevacion_objetivo,
                                                      bool verbose = true);
    
    /**
     * @brief Actualiza la posición actual del sistema
     * @param azimuth_deg Azimuth actual en grados
     * @param elevacion_deg Elevación actual en grados
     */
    void actualizarPosicionActual(float azimuth_deg, float elevacion_deg);
    
    /**
     * @brief Obtiene la posición actual del sistema
     * @param azimuth_deg Salida: azimuth actual
     * @param elevacion_deg Salida: elevación actual
     */
    void obtenerPosicionActual(float& azimuth_deg, float& elevacion_deg) const;
    
    /**
     * @brief Obtiene las longitudes actuales de los cables
     * @param L1 Salida: longitud actual cable 1
     * @param L2 Salida: longitud actual cable 2
     * @param L3 Salida: longitud actual cable 3
     */
    void obtenerLongitudesActuales(float& L1, float& L2, float& L3);
    
    /**
     * @brief Resetea la posición actual a la neutra (0, 0)
     */
    void resetearPosicion();
    
    /**
     * @brief Obtiene las longitudes iniciales de los cables
     * @param L1 Salida: longitud inicial cable 1
     * @param L2 Salida: longitud inicial cable 2
     * @param L3 Salida: longitud inicial cable 3
     */
    void obtenerLongitudesIniciales(float& L1, float& L2, float& L3) const;
    
    /**
     * @brief Imprime información del sistema por Serial
     */
    void imprimirInfo() const;
};

#endif // CINEMATICA_INVERSA_H
