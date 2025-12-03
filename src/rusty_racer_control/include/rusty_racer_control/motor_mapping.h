/**
 * @file motor_mapping.h
 * @brief Mapping von Geschwindigkeitsbefehl zu Motor-Level
 * @author zx
 * @date 2025-12
 */

#pragma once
#include <algorithm>

/**
 * @brief Wandelt Geschwindigkeitsbefehl in Motor-Level um
 * @param v_cmd Geschwindigkeitsbefehl [m/s]
 * @param v_max Maximale Geschwindigkeit [m/s]
 * @return motor_level Motor-Antriebslevel [0, 1]
 * 
 * Methode 1: Einfache lineare Abbildung (aktuell verwendet)
 * Formel: motor_level = v_cmd / v_max
 */
inline double speed_to_motor_level(double v_cmd, double v_max) 
{
    // Lineare Abbildung
    double motor_level = v_cmd / v_max;
    
    // Begrenzung auf [0, 1]
    motor_level = std::max(0.0, std::min(motor_level, 1.0));
    
    return motor_level;
    
    // Methode 2: Stückweise lineare Abbildung (falls Methode 1 ungenau ist)
    // Benötigt experimentelle Daten für bessere Genauigkeit
    /*
    if (v_cmd <= 0.0) return 0.0;
    
    // Startbereich (hohe Haftreibung)
    if (v_cmd <= 0.2) {
        return 0.15 * (v_cmd / 0.2);
    }
    
    // Niedriger Geschwindigkeitsbereich
    if (v_cmd <= 0.5) {
        return 0.15 + 0.20 * ((v_cmd - 0.2) / 0.3);
    }
    
    // Mittlerer Geschwindigkeitsbereich
    if (v_cmd <= 1.0) {
        return 0.35 + 0.30 * ((v_cmd - 0.5) / 0.5);
    }
    
    // Hoher Geschwindigkeitsbereich
    if (v_cmd <= v_max) {
        return 0.65 + 0.35 * ((v_cmd - 1.0) / (v_max - 1.0));
    }
    
    return 1.0;
    */
}