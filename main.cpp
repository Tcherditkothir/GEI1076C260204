/*
 * ROBOT MINI-SUMO - EXEMPLE COMPLET C++ POUR ESP32
 * 
 * Ce code illustre TOUS les concepts C++ essentiels pour la programmation embarquée :
 * 1. Namespace
 * 2. Références
 * 3. Classes et encapsulation
 * 4. Constructeurs/Destructeurs
 * 5. Héritage simple
 * 6. Polymorphisme (virtual)
 * 7. Operator overloading
 * 8. Templates de base
 * 9. STL - Containers (vector, array)
 * 10. Lambda expressions
 * 11. Const correctness
 * 12. RAII léger
 * 
 * Hardware:
 * - 4 capteurs de ligne (détection bords du dohyo)
 * - 2 moteurs DC avec contrôle PWM
 * - 1 bouton de démarrage
 */

#include <Arduino.h>
#include <vector>      // STL Container
#include <array>       // STL Container pour taille fixe
#include <algorithm>   // Pour std::clamp

// ============================================================================
// CONCEPT 1: NAMESPACE
// ============================================================================
// Les namespaces permettent d'organiser le code et éviter les conflits de noms
// entre différentes bibliothèques (ex: capteurs de différents fabricants)

namespace RobotConfig {
    // Configuration des pins GPIO ESP32
    constexpr uint8_t MOTOR_LEFT_PWM = 25;
    constexpr uint8_t MOTOR_LEFT_DIR = 26;
    constexpr uint8_t MOTOR_RIGHT_PWM = 27;
    constexpr uint8_t MOTOR_RIGHT_DIR = 14;
    
    constexpr uint8_t LINE_SENSOR_FRONT_LEFT = 32;
    constexpr uint8_t LINE_SENSOR_FRONT_RIGHT = 33;
    constexpr uint8_t LINE_SENSOR_BACK_LEFT = 34;
    constexpr uint8_t LINE_SENSOR_BACK_RIGHT = 35;
    
    constexpr uint8_t START_BUTTON = 15;
    
    // Paramètres PWM
    constexpr int PWM_FREQUENCY = 5000;
    constexpr int PWM_RESOLUTION = 8;  // 0-255
    constexpr int PWM_CHANNEL_LEFT = 0;
    constexpr int PWM_CHANNEL_RIGHT = 1;
}

// ============================================================================
// CONCEPT 7: OPERATOR OVERLOADING
// ============================================================================
// Permet de créer des types personnalisés qui se comportent comme des types natifs
// Ici, un vecteur 2D pour représenter la vitesse des moteurs

struct MotorSpeed {
    int left;   // -255 à 255
    int right;  // -255 à 255
    
    // Constructeur par défaut
    MotorSpeed() : left(0), right(0) {}
    MotorSpeed(int l, int r) : left(l), right(r) {}
    
    // Surcharge de l'opérateur + pour combiner des vitesses
    MotorSpeed operator+(const MotorSpeed& other) const {
        return MotorSpeed(left + other.left, right + other.right);
    }
    
    // Surcharge de l'opérateur * pour scaling (multiplication scalaire)
    MotorSpeed operator*(float scale) const {
        return MotorSpeed(
            static_cast<int>(left * scale),
            static_cast<int>(right * scale)
        );
    }
    
    // Surcharge de l'opérateur == pour comparaison
    bool operator==(const MotorSpeed& other) const {
        return (left == other.left) && (right == other.right);
    }
};

// ============================================================================
// CONCEPT 8: TEMPLATES DE BASE
// ============================================================================
// Les templates permettent de créer des fonctions/classes génériques qui 
// fonctionnent avec différents types de données

// Template de fonction générique pour mapper des valeurs
template<typename T>
T map_value(T value, T in_min, T in_max, T out_min, T out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Template pour limiter une valeur dans un intervalle
template<typename T>
T constrain_value(T value, T min_val, T max_val) {
    // Utilisation de std::clamp (C++17)
    return std::clamp(value, min_val, max_val);
}

// ============================================================================
// CONCEPT 3 & 5: CLASSES, ENCAPSULATION & HÉRITAGE
// ============================================================================

// Classe de base abstraite pour tous les capteurs
// Démontre l'encapsulation (données privées) et l'héritage
class BaseSensor {
protected:  // Accessible aux classes dérivées
    uint8_t pin_;
    int last_reading_;
    
public:
    // CONCEPT 4: CONSTRUCTEUR
    // Initialise automatiquement le capteur lors de sa création
    explicit BaseSensor(uint8_t pin) : pin_(pin), last_reading_(0) {
        pinMode(pin_, INPUT);
    }
    
    // CONCEPT 4: DESTRUCTEUR VIRTUEL
    // Important pour le polymorphisme : assure la destruction correcte
    // des objets dérivés via un pointeur de base
    virtual ~BaseSensor() {
        // Nettoyage si nécessaire (ici, rien à faire)
    }
    
    // CONCEPT 6: FONCTION VIRTUELLE PURE (polymorphisme)
    // Force les classes dérivées à implémenter leur propre lecture
    virtual int read() = 0;
    
    // CONCEPT 11: CONST CORRECTNESS
    // Cette fonction garantit qu'elle ne modifie pas l'état de l'objet
    int getLastReading() const {
        return last_reading_;
    }
    
    uint8_t getPin() const {
        return pin_;
    }
};

// Classe dérivée pour capteur de ligne (IR)
// CONCEPT 5: HÉRITAGE SIMPLE
class LineSensor : public BaseSensor {
private:
    int threshold_;  // Seuil de détection ligne blanche/noire
    
public:
    // Constructeur qui appelle le constructeur de la classe de base
    LineSensor(uint8_t pin, int threshold = 512) 
        : BaseSensor(pin), threshold_(threshold) {
        // Initialisation spécifique au capteur de ligne
    }
    
    // CONCEPT 6: OVERRIDE de la fonction virtuelle
    // Implémentation spécifique pour la lecture d'un capteur IR
    int read() override {
        last_reading_ = analogRead(pin_);
        return last_reading_;
    }
    
    // Fonction spécifique au capteur de ligne
    // CONCEPT 11: const = ne modifie pas l'état
    bool detectsLine() const {
        return last_reading_ > threshold_;
    }
    
    // CONCEPT 2: RÉFÉRENCE
    // Passer par référence évite la copie et permet la modification
    void setThreshold(int& new_threshold) {
        threshold_ = new_threshold;
    }
};

// ============================================================================
// CONCEPT 12: RAII (Resource Acquisition Is Initialization)
// ============================================================================
// La classe Motor acquiert les ressources (PWM) dans le constructeur
// et les libère dans le destructeur automatiquement

class Motor {
private:
    uint8_t pwm_pin_;
    uint8_t dir_pin_;
    int pwm_channel_;
    int current_speed_;  // -255 à 255
    
public:
    // CONCEPT 4: CONSTRUCTEUR avec initializer list (efficace)
    Motor(uint8_t pwm_pin, uint8_t dir_pin, int pwm_channel)
        : pwm_pin_(pwm_pin), 
          dir_pin_(dir_pin), 
          pwm_channel_(pwm_channel),
          current_speed_(0) {
        
        // CONCEPT 12: RAII - Acquisition des ressources
        pinMode(dir_pin_, OUTPUT);
        ledcSetup(pwm_channel_, RobotConfig::PWM_FREQUENCY, RobotConfig::PWM_RESOLUTION);
        ledcAttachPin(pwm_pin_, pwm_channel_);
        
        // État initial sécuritaire
        stop();
    }
    
    // CONCEPT 4: DESTRUCTEUR
    // CONCEPT 12: RAII - Libération automatique des ressources
    ~Motor() {
        stop();
        ledcDetachPin(pwm_pin_);
    }
    
    // Définir la vitesse du moteur
    // CONCEPT 2: passage par valeur (petit type)
    void setSpeed(int speed) {
        // Limiter la vitesse dans la plage valide
        current_speed_ = constrain_value(speed, -255, 255);
        
        // Déterminer la direction
        bool direction = (current_speed_ >= 0);
        digitalWrite(dir_pin_, direction ? HIGH : LOW);
        
        // Appliquer la PWM (valeur absolue)
        int pwm_value = abs(current_speed_);
        ledcWrite(pwm_channel_, pwm_value);
    }
    
    void stop() {
        setSpeed(0);
    }
    
    // CONCEPT 11: const correctness
    int getSpeed() const {
        return current_speed_;
    }
};

// ============================================================================
// CONCEPT 6: POLYMORPHISME - Stratégies de combat interchangeables
// ============================================================================

// Classe de base abstraite pour les stratégies
class Strategy {
public:
    virtual ~Strategy() = default;
    
    // CONCEPT 6: Fonction virtuelle pure
    // Chaque stratégie doit implémenter son comportement
    virtual MotorSpeed execute(const std::array<bool, 4>& line_detected) = 0;
    
    // CONCEPT 11: const
    virtual const char* getName() const = 0;
};

// Stratégie agressive : avancer rapidement
class AggressiveStrategy : public Strategy {
public:
    MotorSpeed execute(const std::array<bool, 4>& line_detected) override {
        // Si détection de ligne devant : reculer
        if (line_detected[0] || line_detected[1]) {
            return MotorSpeed(-200, -200);  // Reculer
        }
        // Si détection de ligne derrière : avancer encore plus vite
        if (line_detected[2] || line_detected[3]) {
            return MotorSpeed(255, 255);  // Pleine vitesse avant
        }
        // Sinon : avancer normalement
        return MotorSpeed(200, 200);
    }
    
    const char* getName() const override {
        return "Aggressive";
    }
};

// Stratégie défensive : tourner pour chercher l'adversaire
class DefensiveStrategy : public Strategy {
public:
    MotorSpeed execute(const std::array<bool, 4>& line_detected) override {
        // Si détection de ligne : reculer et tourner
        if (line_detected[0] || line_detected[1]) {
            return MotorSpeed(-150, 150);  // Reculer en tournant
        }
        if (line_detected[2] || line_detected[3]) {
            return MotorSpeed(150, -150);  // Avancer en tournant
        }
        // Sinon : tourner sur place pour chercher
        return MotorSpeed(100, -100);
    }
    
    const char* getName() const override {
        return "Defensive";
    }
};

// ============================================================================
// CLASSE PRINCIPALE DU ROBOT
// ============================================================================

class MiniSumoRobot {
private:
    // CONCEPT 9: STL CONTAINERS
    // std::vector : taille dynamique, utile pour listes de capteurs
    std::vector<LineSensor*> line_sensors_;
    
    // std::array : taille fixe, plus efficace en mémoire
    std::array<bool, 4> line_detection_state_;
    
    Motor left_motor_;
    Motor right_motor_;
    
    // CONCEPT 6: POLYMORPHISME - pointeur vers stratégie de base
    Strategy* current_strategy_;
    
    bool is_running_;
    
public:
    // CONCEPT 4: CONSTRUCTEUR
    MiniSumoRobot()
        : left_motor_(RobotConfig::MOTOR_LEFT_PWM, 
                      RobotConfig::MOTOR_LEFT_DIR, 
                      RobotConfig::PWM_CHANNEL_LEFT),
          right_motor_(RobotConfig::MOTOR_RIGHT_PWM, 
                       RobotConfig::MOTOR_RIGHT_DIR, 
                       RobotConfig::PWM_CHANNEL_RIGHT),
          current_strategy_(nullptr),
          is_running_(false) {
        
        // Initialiser les capteurs de ligne
        // CONCEPT 9: Utilisation de vector (push_back)
        line_sensors_.push_back(new LineSensor(RobotConfig::LINE_SENSOR_FRONT_LEFT));
        line_sensors_.push_back(new LineSensor(RobotConfig::LINE_SENSOR_FRONT_RIGHT));
        line_sensors_.push_back(new LineSensor(RobotConfig::LINE_SENSOR_BACK_LEFT));
        line_sensors_.push_back(new LineSensor(RobotConfig::LINE_SENSOR_BACK_RIGHT));
        
        // Initialiser l'array de détection
        line_detection_state_.fill(false);
        
        // Bouton de démarrage
        pinMode(RobotConfig::START_BUTTON, INPUT_PULLUP);
        
        // Stratégie par défaut
        current_strategy_ = new AggressiveStrategy();
        
        Serial.begin(115200);
        Serial.println("Mini-Sumo Robot initialized!");
    }
    
    // CONCEPT 4: DESTRUCTEUR
    // CONCEPT 12: RAII - Nettoyage automatique
    ~MiniSumoRobot() {
        // Libérer la mémoire des capteurs
        for (auto* sensor : line_sensors_) {
            delete sensor;
        }
        line_sensors_.clear();
        
        // Libérer la stratégie
        delete current_strategy_;
        
        Serial.println("Robot destroyed cleanly");
    }
    
    // CONCEPT 2: RÉFÉRENCE
    // Passer la nouvelle stratégie par référence (pas de copie)
    void setStrategy(Strategy& new_strategy) {
        if (current_strategy_ != nullptr) {
            delete current_strategy_;
        }
        current_strategy_ = &new_strategy;
        Serial.printf("Strategy changed to: %s\n", current_strategy_->getName());
    }
    
    // Lire tous les capteurs de ligne
    void updateSensors() {
        // CONCEPT 9: Itération avec index sur vector
        for (size_t i = 0; i < line_sensors_.size(); i++) {
            line_sensors_[i]->read();
            line_detection_state_[i] = line_sensors_[i]->detectsLine();
        }
    }
    
    // CONCEPT 10: LAMBDA EXPRESSION
    // Les lambdas sont des fonctions anonymes utiles pour les callbacks
    void printSensorStatus() {
        // Lambda capturant 'this' pour accéder aux membres
        auto print_sensor = [this](size_t index, const char* position) {
            Serial.printf("%s: %d (Line: %s) | ",
                position,
                line_sensors_[index]->getLastReading(),
                line_detection_state_[index] ? "YES" : "NO");
        };
        
        print_sensor(0, "FL");
        print_sensor(1, "FR");
        print_sensor(2, "BL");
        print_sensor(3, "BR");
        Serial.println();
    }
    
    void start() {
        is_running_ = true;
        Serial.println("Robot STARTED!");
    }
    
    void stop() {
        is_running_ = false;
        left_motor_.stop();
        right_motor_.stop();
        Serial.println("Robot STOPPED!");
    }
    
    // CONCEPT 11: const - fonction qui ne modifie pas l'état
    bool isRunning() const {
        return is_running_;
    }
    
    void update() {
        if (!is_running_) return;
        
        // Mettre à jour les capteurs
        updateSensors();
        
        // CONCEPT 6: POLYMORPHISME
        // Appel de la méthode virtuelle de la stratégie courante
        MotorSpeed speed = current_strategy_->execute(line_detection_state_);
        
        // CONCEPT 7: OPERATOR OVERLOADING
        // On pourrait combiner des vitesses avec l'opérateur +
        MotorSpeed safety_limit = speed * 0.9f;  // Utilisation de operator*
        
        // Appliquer les vitesses aux moteurs
        left_motor_.setSpeed(safety_limit.left);
        right_motor_.setSpeed(safety_limit.right);
    }
};

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

MiniSumoRobot* robot = nullptr;

// CONCEPT 10: LAMBDA pour gestion du bouton avec anti-rebond
auto button_handler = []() -> bool {
    static unsigned long last_press = 0;
    static bool last_state = HIGH;
    
    bool current_state = digitalRead(RobotConfig::START_BUTTON);
    unsigned long now = millis();
    
    // Détection du front descendant avec anti-rebond (50ms)
    if (current_state == LOW && last_state == HIGH && (now - last_press > 50)) {
        last_press = now;
        last_state = current_state;
        return true;  // Bouton pressé
    }
    
    last_state = current_state;
    return false;
};

// ============================================================================
// FONCTIONS ARDUINO
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n========================================");
    Serial.println("   MINI-SUMO ROBOT - C++ DEMO");
    Serial.println("========================================\n");
    
    // CONCEPT 12: RAII - Le constructeur initialise tout automatiquement
    robot = new MiniSumoRobot();
    
    Serial.println("\nPress the START button to begin...");
}

void loop() {
    // Gestion du bouton avec lambda
    if (button_handler()) {
        if (robot->isRunning()) {
            robot->stop();
        } else {
            robot->start();
            
            // CONCEPT 8: TEMPLATE avec différents types
            int demo_value = 512;
            int mapped = map_value<int>(demo_value, 0, 1023, 0, 100);
            Serial.printf("Template demo - mapped value: %d%%\n", mapped);
        }
    }
    
    // Mise à jour du robot
    robot->update();
    
    // Affichage périodique des capteurs
    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        if (robot->isRunning()) {
            robot->printSensorStatus();
        }
        last_print = millis();
    }
    
    // CONCEPT 10: Lambda pour changer de stratégie après 10 secondes
    static unsigned long start_time = millis();
    static bool strategy_changed = false;
    
    if (robot->isRunning() && !strategy_changed && (millis() - start_time > 10000)) {
        auto change_strategy = [&]() {
            Serial.println("\n>>> Changing to Defensive Strategy <<<\n");
            DefensiveStrategy* defensive = new DefensiveStrategy();
            robot->setStrategy(*defensive);
            strategy_changed = true;
        };
        change_strategy();
    }
    
    delay(10);  // Petite pause pour éviter la surcharge CPU
}

/*
 * ============================================================================
 * RÉSUMÉ DES CONCEPTS DÉMONTRÉS :
 * ============================================================================
 * 
 * 1. NAMESPACE (RobotConfig) : Organisation du code, évite conflits de noms
 * 
 * 2. RÉFÉRENCES (&) : Passage de paramètres sans copie (setThreshold, setStrategy)
 * 
 * 3. CLASSES & ENCAPSULATION : BaseSensor, LineSensor, Motor, Strategy
 *    - Données privées (pin_, threshold_)
 *    - Méthodes publiques (read(), setSpeed())
 * 
 * 4. CONSTRUCTEURS/DESTRUCTEURS : 
 *    - Initialisation automatique (Motor, MiniSumoRobot)
 *    - Nettoyage automatique (~MiniSumoRobot)
 * 
 * 5. HÉRITAGE : LineSensor hérite de BaseSensor
 * 
 * 6. POLYMORPHISME : 
 *    - Fonctions virtuelles (read(), execute())
 *    - Strategy* peut pointer vers AggressiveStrategy ou DefensiveStrategy
 * 
 * 7. OPERATOR OVERLOADING : 
 *    - MotorSpeed : +, *, ==
 *    - Manipulation naturelle des vitesses
 * 
 * 8. TEMPLATES : 
 *    - map_value<T>, constrain_value<T>
 *    - Fonctions génériques pour différents types
 * 
 * 9. STL CONTAINERS :
 *    - std::vector pour liste dynamique de capteurs
 *    - std::array pour état de détection (taille fixe)
 *    - Méthodes: push_back(), fill(), size()
 * 
 * 10. LAMBDA EXPRESSIONS :
 *     - button_handler : gestion anti-rebond
 *     - print_sensor : affichage capteurs
 *     - change_strategy : changement de stratégie
 * 
 * 11. CONST CORRECTNESS :
 *     - Fonctions const (getLastReading(), isRunning())
 *     - Garantit qu'elles ne modifient pas l'état
 * 
 * 12. RAII (Resource Acquisition Is Initialization) :
 *     - Motor acquiert PWM dans constructeur
 *     - Libération automatique dans destructeur
 *     - Garantit la sécurité des ressources
 * 
 * ============================================================================
 * UTILISATION PRATIQUE POUR LE COURS :
 * ============================================================================
 * 
 * Ce code peut être présenté de manière incrémentale :
 * 
 * 1. Commencer avec les classes de base (BaseSensor, Motor)
 * 2. Introduire l'héritage (LineSensor)
 * 3. Ajouter le polymorphisme (Strategy)
 * 4. Montrer les templates et STL
 * 5. Finaliser avec les lambdas et RAII
 * 
 * Chaque concept est isolable et peut être expliqué individuellement
 * tout en montrant son utilité dans un contexte réel de robotique.
 * 
 * ============================================================================
 */
