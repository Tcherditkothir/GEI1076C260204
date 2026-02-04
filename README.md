# Mini-Sumo Robot - Exemple Complet C++ pour ESP32

## Description

Ce projet est un exemple d'un robot mini-sumo programmé en C++ pour ESP32, conçu pour enseigner les concepts avancés de C++ dans un contexte de robotique embarquée.

## Objectifs Pédagogiques

Le code illustre les concepts C++ essentiels pour la programmation embarquée :

1. **Namespace** - Organisation du code
2. **Références** - Passage de paramètres efficace
3. **Classes et Encapsulation** - Structure orientée objet
4. **Constructeurs/Destructeurs** - Gestion du cycle de vie
5. **Héritage Simple** - Réutilisation de code
6. **Polymorphisme (virtual)** - Flexibilité et extensibilité
7. **Operator Overloading** - Types personnalisés naturels
8. **Templates** - Fonctions génériques
9. **STL Containers** - Vector et Array
10. **Lambda Expressions** - Fonctions anonymes
11. **Const Correctness** - Garanties de non-modification
12. **RAII** - Gestion automatique des ressources

## Hardware Requis

- **Microcontrôleur** : ESP32
- **Moteurs** : 2x moteurs DC avec driver H-Bridge (ex: L298N)
- **Capteurs** : 4x capteurs de ligne infrarouge (détection bords du dohyo)
- **Interface** : 1x bouton poussoir de démarrage

## Configuration des Pins

Les pins peuvent être modifiées dans le namespace `RobotConfig` :

```cpp
namespace RobotConfig {
    constexpr uint8_t MOTOR_LEFT_PWM = 25;
    constexpr uint8_t MOTOR_LEFT_DIR = 26;
    constexpr uint8_t MOTOR_RIGHT_PWM = 27;
    constexpr uint8_t MOTOR_RIGHT_DIR = 14;
    
    constexpr uint8_t LINE_SENSOR_FRONT_LEFT = 32;
    constexpr uint8_t LINE_SENSOR_FRONT_RIGHT = 33;
    constexpr uint8_t LINE_SENSOR_BACK_LEFT = 34;
    constexpr uint8_t LINE_SENSOR_BACK_RIGHT = 35;
    
    constexpr uint8_t START_BUTTON = 15;
}
```

## Installation et Compilation

### Avec PlatformIO (VSCode)

1. Ouvrir le dossier dans VSCode
2. PlatformIO détectera automatiquement le projet
3. Compiler : `Ctrl+Alt+B` ou cliquer sur "Build"
4. Upload : `Ctrl+Alt+U` ou cliquer sur "Upload"
5. Monitor série : `Ctrl+Alt+S` ou cliquer sur "Serial Monitor"

### En ligne de commande

```bash
cd mini-sumo-robot
pio run                    # Compiler
pio run --target upload    # Upload vers ESP32
pio device monitor         # Moniteur série
```

## Utilisation

1. **Démarrage** : Presser le bouton START
2. **Arrêt** : Presser à nouveau le bouton START
3. **Changement de stratégie** : Automatique après 10 secondes (de Aggressive à Defensive)

## Structure du Code

### Classes Principales

- **`BaseSensor`** : Classe de base abstraite pour tous les capteurs (polymorphisme)
- **`LineSensor`** : Capteur de ligne dérivé de BaseSensor (héritage)
- **`Motor`** : Contrôle d'un moteur avec RAII pour la gestion PWM
- **`Strategy`** : Interface pour les stratégies de combat (polymorphisme)
- **`AggressiveStrategy`** : Stratégie offensive
- **`DefensiveStrategy`** : Stratégie défensive
- **`MiniSumoRobot`** : Classe principale orchestrant tout le robot

### Types Personnalisés

- **`MotorSpeed`** : Structure avec operator overloading (+, *, ==)

### Templates

- **`map_value<T>`** : Mapping de valeurs générique
- **`constrain_value<T>`** : Limitation dans un intervalle

## Concepts Clés Illustrés

### RAII (Resource Acquisition Is Initialization)

```cpp
class Motor {
    Motor(...) {
        // Acquisition de la ressource PWM
        ledcSetup(pwm_channel_, ...);
        ledcAttachPin(pwm_pin_, ...);
    }
    
    ~Motor() {
        // Libération automatique
        stop();
        ledcDetachPin(pwm_pin_);
    }
};
```

### Polymorphisme

```cpp
Strategy* current_strategy_;  // Pointeur vers classe de base

// Peut pointer vers différentes stratégies
current_strategy_ = new AggressiveStrategy();
// ou
current_strategy_ = new DefensiveStrategy();

// Appel polymorphique
MotorSpeed speed = current_strategy_->execute(...);
```

### Lambda Expressions

```cpp
auto button_handler = []() -> bool {
    static unsigned long last_press = 0;
    // ... logique anti-rebond ...
    return true;  // Bouton pressé
};
```

### STL Containers

```cpp
std::vector<LineSensor*> line_sensors_;  // Taille dynamique
std::array<bool, 4> line_detection_state_;  // Taille fixe
```

## Progression Pédagogique Suggérée

Pour présenter ce code dans un cours, suivre cet ordre :

1. **Namespace et configuration** - Organisation du code
2. **Classes de base** - BaseSensor, Motor
3. **Héritage** - LineSensor extends BaseSensor
4. **Constructeurs/Destructeurs** - Cycle de vie des objets
5. **Polymorphisme** - Strategy pattern
6. **Operator overloading** - MotorSpeed
7. **Templates** - Fonctions génériques
8. **STL** - Vector et Array
9. **Lambdas** - button_handler, print_sensor
10. **RAII** - Gestion automatique PWM
11. **Const correctness** - Fonctions const
12. **Intégration finale** - MiniSumoRobot

## Personnalisation

### Ajouter une Nouvelle Stratégie

```cpp
class CustomStrategy : public Strategy {
public:
    MotorSpeed execute(const std::array<bool, 4>& line_detected) override {
        // Votre logique ici
        return MotorSpeed(100, 100);
    }
    
    const char* getName() const override {
        return "Custom";
    }
};
```

### Ajouter un Nouveau Type de Capteur

```cpp
class UltrasonicSensor : public BaseSensor {
public:
    UltrasonicSensor(uint8_t trig_pin, uint8_t echo_pin)
        : BaseSensor(trig_pin), echo_pin_(echo_pin) {
        pinMode(echo_pin_, INPUT);
    }
    
    int read() override {
        // Logique de lecture ultrason
        return distance_cm;
    }
    
private:
    uint8_t echo_pin_;
};
```

## Dépannage

### Le robot ne démarre pas
- Vérifier la connexion du bouton START (pin 15)
- Vérifier le moniteur série pour les messages de debug

### Les moteurs ne tournent pas
- Vérifier les connexions du H-Bridge
- Vérifier l'alimentation des moteurs (séparée de l'ESP32)
- Vérifier les pins PWM et DIR dans RobotConfig

### Les capteurs ne détectent pas
- Calibrer le seuil (threshold) des LineSensor
- Vérifier le câblage des capteurs IR
- Observer les valeurs brutes dans le moniteur série


