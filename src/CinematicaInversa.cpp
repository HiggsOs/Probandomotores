/**
 * @file CinematicaInversa.cpp
 * @brief Implementación de la clase CinematicaInversa
 */

#include "CinematicaInversa.h"

CinematicaInversa::CinematicaInversa() {
    // ========== PARÁMETROS GEOMÉTRICOS ==========
    // Altura de la plataforma (mm)
    H = 435.0;
    
    // Coordenadas de las juntas proximales (base) - en mm
    A1 = {60.79, 69.14, 19.27};
    A2 = {-91.59, 19.82, 19.27};
    A3 = {23.48, -92.72, 19.27};
    
    // Coordenadas de las juntas distales (plataforma móvil) - en mm
    B1 = {84.2, 92.47, 24.05};
    B2 = {-122.18, 26.69, 24.05};
    B3 = {40.88, -130.9, 24.05};
    
    // ========== PARÁMETROS DE CONVERSIÓN ==========
    // Radio de la polea (mm)
    RADIO_POLEA = 8.75;
    
    // Factor de reducción mecánica
    FACTOR_REDUCCION = 0.063;
    
    // Conversión: ΔL (mm) a grados de rotación del motor
    // MM_A_GRADOS = (180 / (π × radio)) × factor_reducción
    MM_A_GRADOS = (180.0 / (PI * RADIO_POLEA)) * FACTOR_REDUCCION;
    
    // Calcular longitudes iniciales (posición neutra: 0°, 0°)
    Li1 = calcularLongitudCable(A1, B1, 0, 0);
    Li2 = calcularLongitudCable(A2, B2, 0, 0);
    Li3 = calcularLongitudCable(A3, B3, 0, 0);
    
    imprimirInfo();
}

float CinematicaInversa::calcularLongitudCable(const Coordenada3D& A, const Coordenada3D& B,
                                               float tx_rad, float ty_rad) {
    float comp1 = pow(A.y - B.y * cos(tx_rad) + B.z * sin(tx_rad), 2);
    
    float comp2 = pow(H - A.z - B.x * sin(ty_rad) + 
                     B.z * cos(tx_rad) * cos(ty_rad) + 
                     B.y * cos(ty_rad) * sin(tx_rad), 2);
    
    float comp3 = pow(B.x * cos(ty_rad) - A.x + 
                     B.z * cos(tx_rad) * sin(ty_rad) + 
                     B.y * sin(tx_rad) * sin(ty_rad), 2);
    
    return sqrt(comp1 + comp2 + comp3);
}

float CinematicaInversa::deltaLAGrados(float deltaL_mm) {
    return deltaL_mm * MM_A_GRADOS;
}

void CinematicaInversa::azimuthElevacionATxTy(float azimuth_deg, float elevacion_deg,
                                              float& tx_grados, float& ty_grados) {
    // Para seguimiento solar simple:
    // tx corresponde a la elevación
    // ty corresponde al azimuth (puede necesitar offset/transformación)
    
    // IMPORTANTE: Ajusta estos cálculos según tu configuración física
    // Esta es una aproximación simple
    tx_grados = elevacion_deg;
    ty_grados = azimuth_deg - 180.0;  // Centrar en 0° cuando el sol está al sur
}

ResultadoMovimiento CinematicaInversa::calcularMovimiento(float azimuth_deg, 
                                                         float elevacion_deg,
                                                         bool verbose) {
    if (verbose) {
        Serial.println();
        Serial.println(F("=================================================="));
        Serial.println(F("CALCULO DE CINEMATICA INVERSA"));
        Serial.println(F("=================================================="));
        Serial.print(F("Posicion solar solicitada:\n"));
        Serial.print(F("  Azimuth: "));
        Serial.print(azimuth_deg, 2);
        Serial.println(F("°"));
        Serial.print(F("  Elevacion: "));
        Serial.print(elevacion_deg, 2);
        Serial.println(F("°"));
    }
    
    // Convertir a ángulos de plataforma
    float tx_grados, ty_grados;
    azimuthElevacionATxTy(azimuth_deg, elevacion_deg, tx_grados, ty_grados);
    
    // Convertir a radianes
    float tx_rad = tx_grados * DEG_TO_RAD;
    float ty_rad = ty_grados * DEG_TO_RAD;
    
    if (verbose) {
        Serial.println(F("\nAngulos de plataforma:"));
        Serial.print(F("  TX (rotacion X): "));
        Serial.print(tx_grados, 2);
        Serial.print(F("° ("));
        Serial.print(tx_rad, 4);
        Serial.println(F(" rad)"));
        Serial.print(F("  TY (rotacion Y): "));
        Serial.print(ty_grados, 2);
        Serial.print(F("° ("));
        Serial.print(ty_rad, 4);
        Serial.println(F(" rad)"));
    }
    
    // Calcular longitudes deseadas usando cinemática inversa
    float dL1 = calcularLongitudCable(A1, B1, tx_rad, ty_rad);
    float dL2 = calcularLongitudCable(A2, B2, tx_rad, ty_rad);
    float dL3 = calcularLongitudCable(A3, B3, tx_rad, ty_rad);
    
    if (verbose) {
        Serial.println(F("\nLongitudes deseadas:"));
        Serial.print(F("  L1: "));
        Serial.print(dL1, 2);
        Serial.println(F(" mm"));
        Serial.print(F("  L2: "));
        Serial.print(dL2, 2);
        Serial.println(F(" mm"));
        Serial.print(F("  L3: "));
        Serial.print(dL3, 2);
        Serial.println(F(" mm"));
        
        Serial.println(F("\nLongitudes iniciales (referencia):"));
        Serial.print(F("  L1: "));
        Serial.print(Li1, 2);
        Serial.println(F(" mm"));
        Serial.print(F("  L2: "));
        Serial.print(Li2, 2);
        Serial.println(F(" mm"));
        Serial.print(F("  L3: "));
        Serial.print(Li3, 2);
        Serial.println(F(" mm"));
    }
    
    // Calcular cambios de longitud necesarios
    float deltaL1 = dL1 - Li1;
    float deltaL2 = dL2 - Li2;
    float deltaL3 = dL3 - Li3;
    
    if (verbose) {
        Serial.println(F("\nCambios de longitud necesarios:"));
        Serial.print(F("  ΔL1: "));
        Serial.print(deltaL1, 2);
        Serial.println(F(" mm"));
        Serial.print(F("  ΔL2: "));
        Serial.print(deltaL2, 2);
        Serial.println(F(" mm"));
        Serial.print(F("  ΔL3: "));
        Serial.print(deltaL3, 2);
        Serial.println(F(" mm"));
    }
    
    // Convertir a grados de rotación
    float grados1 = deltaLAGrados(deltaL1);
    float grados2 = deltaLAGrados(deltaL2);
    float grados3 = deltaLAGrados(deltaL3);
    
    if (verbose) {
        Serial.println(F("\n=================================================="));
        Serial.println(F("RESULTADO: GRADOS A MOVER CADA MOTOR"));
        Serial.println(F("=================================================="));
        Serial.print(F("  Motor 1: "));
        Serial.print(grados1, 2);
        Serial.println(F("°"));
        Serial.print(F("  Motor 2: "));
        Serial.print(grados2, 2);
        Serial.println(F("°"));
        Serial.print(F("  Motor 3: "));
        Serial.print(grados3, 2);
        Serial.println(F("°"));
        Serial.println(F("=================================================="));
        Serial.println(F("\nConvencion:"));
        Serial.println(F("  Positivo (+) = Suelta cable (longitud aumenta)"));
        Serial.println(F("  Negativo (-) = Jala cable (longitud disminuye)"));
        Serial.println(F("==================================================\n"));
    }
    
    ResultadoMovimiento resultado;
    resultado.motor1 = grados1;
    resultado.motor2 = grados2;
    resultado.motor3 = grados3;
    resultado.deltaL1 = deltaL1;
    resultado.deltaL2 = deltaL2;
    resultado.deltaL3 = deltaL3;
    
    return resultado;
}

void CinematicaInversa::obtenerLongitudesIniciales(float& L1, float& L2, float& L3) const {
    L1 = Li1;
    L2 = Li2;
    L3 = Li3;
}

void CinematicaInversa::imprimirInfo() const {
    Serial.println(F("========== SISTEMA INICIALIZADO =========="));
    Serial.print(F("Altura plataforma: "));
    Serial.print(H, 2);
    Serial.println(F(" mm"));
    Serial.print(F("Radio polea: "));
    Serial.print(RADIO_POLEA, 2);
    Serial.println(F(" mm"));
    Serial.print(F("Conversion: "));
    Serial.print(MM_A_GRADOS, 4);
    Serial.println(F(" grados/mm"));
    Serial.println(F("\nLongitudes iniciales (posicion neutra):"));
    Serial.print(F("  L1 = "));
    Serial.print(Li1, 2);
    Serial.println(F(" mm"));
    Serial.print(F("  L2 = "));
    Serial.print(Li2, 2);
    Serial.println(F(" mm"));
    Serial.print(F("  L3 = "));
    Serial.print(Li3, 2);
    Serial.println(F(" mm"));
    Serial.println(F("==========================================\n"));
}
