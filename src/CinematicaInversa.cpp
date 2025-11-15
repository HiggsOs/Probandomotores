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
    Base1 = {60.79, 69.14, 19.27};
    Base2 = {-91.59, 19.82, 19.27};
    Base3 = {23.48, -92.72, 19.27};
    
    // Coordenadas de las juntas distales (plataforma móvil) - en mm
    Plat1 = {84.2, 92.47, 24.05};
    Plat2 = {-122.18, 26.69, 24.05};
    Plat3 = {40.88, -130.9, 24.05};
    
    // ========== PARÁMETROS DE CONVERSIÓN ==========
    // Radio de la polea (mm)
    RADIO_POLEA = 8.75;
    km = (2*PI*RADIO_POLEA)/25200;  // Circunferencia de la polea
    
    
    // Conversión: ΔL (mm) a grados de rotación del motor
    // MM_A_GRADOS = (180 / (π × radio)) × factor_reducción
    MM_A_GRADOS = (180.0 / (PI * RADIO_POLEA)) * FACTOR_REDUCCION;
    
    // Calcular longitudes iniciales (posición neutra: 0°, 0°)
    Li1 = calcularLongitudCable(Base1, Plat1, 0, 0);
    Li2 = calcularLongitudCable(Base2, Plat2, 0, 0);
    Li3 = calcularLongitudCable(Base3, Plat3, 0, 0);
    
    // Inicializar posición actual en neutro
    azimuthActual = 0.0;
    elevacionActual = 0.0;
    
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
    // Convertir azimuth y elevación solar a ángulos de rotación de la plataforma
    // TX corresponde a la rotación alrededor del eje X (pitch)
    // TY corresponde a la rotación alrededor del eje Y (roll)
    
    // Elevación solar se mapea directamente a TX (inclinación)
    tx_grados = elevacion_deg;
    
    // Azimuth solar se mapea a TY (rotación horizontal)
    // Normalmente: TY = azimuth - 180 (para ajustar referencia)
    // Ajustar según la orientación de tu sistema
    ty_grados = azimuth_deg - 180.0;
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
    float dL1 = calcularLongitudCable(Base1, Plat1, tx_rad, ty_rad);
    float dL2 = calcularLongitudCable(Base2, Plat2, tx_rad, ty_rad);
    float dL3 = calcularLongitudCable(Base3, Plat3, tx_rad, ty_rad);
    
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

ResultadoMovimiento CinematicaInversa::calcularMovimientoIncremental(float azimuth_objetivo,
                                                                     float elevacion_objetivo,
                                                                     bool verbose) {
    if (verbose) {
        Serial.println();
        Serial.println(F("=================================================="));
        Serial.println(F("CALCULO DE CINEMATICA INVERSA INCREMENTAL"));
        Serial.println(F("=================================================="));
        Serial.print(F("Posicion actual:\n"));
        Serial.print(F("  Azimuth: "));
        Serial.print(azimuthActual, 2);
        Serial.println(F("°"));
        Serial.print(F("  Elevacion: "));
        Serial.print(elevacionActual, 2);
        Serial.println(F("°"));
        
        Serial.print(F("\nPosicion objetivo:\n"));
        Serial.print(F("  Azimuth: "));
        Serial.print(azimuth_objetivo, 2);
        Serial.println(F("°"));
        Serial.print(F("  Elevacion: "));
        Serial.print(elevacion_objetivo, 2);
        Serial.println(F("°"));
    }
    
    // Calcular movimiento desde posición actual (sin verbose para cálculo interno)
    ResultadoMovimiento movActual = calcularMovimiento(azimuthActual, elevacionActual, false);
    
    // Calcular movimiento hacia posición objetivo (sin verbose para cálculo interno)
    ResultadoMovimiento movObjetivo = calcularMovimiento(azimuth_objetivo, elevacion_objetivo, false);
    
    // Calcular movimiento incremental (diferencia)
    ResultadoMovimiento movIncremental;
    movIncremental.motor1 = movObjetivo.motor1 - movActual.motor1;
    movIncremental.motor2 = movObjetivo.motor2 - movActual.motor2;
    movIncremental.motor3 = movObjetivo.motor3 - movActual.motor3;
    movIncremental.deltaL1 = movObjetivo.deltaL1 - movActual.deltaL1;
    movIncremental.deltaL2 = movObjetivo.deltaL2 - movActual.deltaL2;
    movIncremental.deltaL3 = movObjetivo.deltaL3 - movActual.deltaL3;
    
    if (verbose) {
        Serial.println(F("\n=================================================="));
        Serial.println(F("RESULTADO: MOVIMIENTO INCREMENTAL"));
        Serial.println(F("=================================================="));
        Serial.print(F("  Motor 1: "));
        Serial.print(movIncremental.motor1, 2);
        Serial.print(F("° (de "));
        Serial.print(movActual.motor1, 2);
        Serial.print(F("° a "));
        Serial.print(movObjetivo.motor1, 2);
        Serial.println(F("°)"));
        
        Serial.print(F("  Motor 2: "));
        Serial.print(movIncremental.motor2, 2);
        Serial.print(F("° (de "));
        Serial.print(movActual.motor2, 2);
        Serial.print(F("° a "));
        Serial.print(movObjetivo.motor2, 2);
        Serial.println(F("°)"));
        
        Serial.print(F("  Motor 3: "));
        Serial.print(movIncremental.motor3, 2);
        Serial.print(F("° (de "));
        Serial.print(movActual.motor3, 2);
        Serial.print(F("° a "));
        Serial.print(movObjetivo.motor3, 2);
        Serial.println(F("°)"));
        
        Serial.println(F("=================================================="));
        Serial.println(F("\nConvencion:"));
        Serial.println(F("  Positivo (+) = Suelta cable (longitud aumenta)"));
        Serial.println(F("  Negativo (-) = Jala cable (longitud disminuye)"));
        Serial.println(F("==================================================\n"));
    }
    
    return movIncremental;
}

void CinematicaInversa::actualizarPosicionActual(float azimuth_deg, float elevacion_deg) {
    azimuthActual = azimuth_deg;
    elevacionActual = elevacion_deg;
    
    Serial.println(F("\nPosicion actual actualizada:"));
    Serial.print(F("  Azimuth: "));
    Serial.print(azimuthActual, 2);
    Serial.println(F("°"));
    Serial.print(F("  Elevacion: "));
    Serial.print(elevacionActual, 2);
    Serial.println(F("°\n"));
}

void CinematicaInversa::obtenerPosicionActual(float& azimuth_deg, float& elevacion_deg) const {
    azimuth_deg = azimuthActual;
    elevacion_deg = elevacionActual;
}

void CinematicaInversa::obtenerLongitudesActuales(float& L1, float& L2, float& L3) {
    // Convertir posición actual a ángulos de plataforma
    float tx_grados, ty_grados;
    azimuthElevacionATxTy(azimuthActual, elevacionActual, tx_grados, ty_grados);
    
    // Convertir a radianes
    float tx_rad = tx_grados * DEG_TO_RAD;
    float ty_rad = ty_grados * DEG_TO_RAD;
    
    // Calcular longitudes actuales
    L1 = calcularLongitudCable(Base1, Plat1, tx_rad, ty_rad);
    L2 = calcularLongitudCable(Base2, Plat2, tx_rad, ty_rad);
    L3 = calcularLongitudCable(Base3, Plat3, tx_rad, ty_rad);
}

void CinematicaInversa::resetearPosicion() {
    azimuthActual = 0.0;
    elevacionActual = 0.0;
    
    Serial.println(F("\nPosicion reseteada a neutra (0°, 0°)\n"));
}
