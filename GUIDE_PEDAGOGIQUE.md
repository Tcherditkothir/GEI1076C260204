# Guide Pédagogique - Concepts C++ par Ordre de Présentation

Ce document guide l'enseignant à travers chaque concept C++ du code, avec des références précises aux lignes de code et des exercices suggérés.

---

## 1. NAMESPACE (Lignes 32-52)

### Concept
Les namespaces permettent d'organiser le code et d'éviter les conflits de noms entre bibliothèques.

### Localisation dans le code
```cpp
namespace RobotConfig {
    constexpr uint8_t MOTOR_LEFT_PWM = 25;
    // ...
}
```

### Pourquoi c'est important
- Organisation du code par contexte logique
- Évite les collisions de noms (ex: deux bibliothèques ayant une variable `threshold`)
- Facilite la maintenance et la configuration

### Points à enseigner
- Syntaxe : `namespace Name { ... }`
- Utilisation : `RobotConfig::MOTOR_LEFT_PWM`
- `constexpr` : évalué à la compilation, économise la RAM

### Exercice suggéré
Créer un namespace `SensorConfig` avec des seuils de détection pour différents types de capteurs.

---

## 2. RÉFÉRENCES (Lignes 221, 279)

### Concept
Les références permettent de passer des paramètres sans copie, avec ou sans modification possible.

### Localisation dans le code
```cpp
void setThreshold(int& new_threshold) {  // Ligne 221
    threshold_ = new_threshold;
}

void setStrategy(Strategy& new_strategy) {  // Ligne 279
    // ...
}
```

### Pourquoi c'est important
- Évite la copie de données (économie mémoire/temps)
- Permet la modification de l'original (`int&`)
- Alternative plus sûre aux pointeurs

### Points à enseigner
- Syntaxe : `Type&` pour référence
- Différence avec pointeur : pas de nullptr, pas de réassignation
- `const Type&` : référence en lecture seule

### Exercice suggéré
Comparer performance : passer un objet `Motor` par valeur vs par référence.

---

## 3. CLASSES & ENCAPSULATION (Lignes 120-360)

### Concept
Les classes regroupent données et fonctions, avec contrôle d'accès (private/public/protected).

### Localisation dans le code
```cpp
class BaseSensor {
protected:
    uint8_t pin_;           // Données protégées
    int last_reading_;
    
public:
    explicit BaseSensor(uint8_t pin) : pin_(pin), last_reading_(0) {
        pinMode(pin_, INPUT);
    }
    // ...
};
```

### Pourquoi c'est important
- Encapsulation : cacher les détails d'implémentation
- Interface publique : contrôler comment les données sont modifiées
- Sécurité : empêcher les modifications accidentelles

### Points à enseigner
- `private` : accessible uniquement dans la classe
- `protected` : accessible dans les classes dérivées
- `public` : accessible partout
- `explicit` : empêche les conversions implicites

### Exercice suggéré
Créer une classe `Battery` avec voltage privé et méthode publique `getPercentage()`.

---

## 4. CONSTRUCTEURS & DESTRUCTEURS (Lignes 126-133, 253-269, 296-311, 313-325)

### Concept
Constructeurs initialisent les objets, destructeurs les nettoient automatiquement.

### Localisation dans le code
```cpp
// Constructeur avec initializer list
Motor(uint8_t pwm_pin, uint8_t dir_pin, int pwm_channel)
    : pwm_pin_(pwm_pin), 
      dir_pin_(dir_pin), 
      pwm_channel_(pwm_channel),
      current_speed_(0) {
    // RAII : acquisition de ressources
    ledcSetup(pwm_channel_, ...);
}

// Destructeur
~Motor() {
    stop();
    ledcDetachPin(pwm_pin_);
}
```

### Pourquoi c'est important
- Initialisation garantie et sécurisée
- Nettoyage automatique (pas de fuite mémoire)
- Base du RAII (voir concept 12)

### Points à enseigner
- Initializer list : `Member(value)` plus efficace que `member = value`
- Destructeur virtuel nécessaire pour polymorphisme
- Ordre d'appel : construction (base→dérivée), destruction (dérivée→base)

### Exercice suggéré
Tracer l'ordre de construction/destruction des objets lors du démarrage du robot.

---

## 5. HÉRITAGE SIMPLE (Lignes 181-227)

### Concept
Une classe peut hériter des propriétés et méthodes d'une autre classe.

### Localisation dans le code
```cpp
class LineSensor : public BaseSensor {
public:
    LineSensor(uint8_t pin, int threshold = 512) 
        : BaseSensor(pin),  // Appel constructeur base
          threshold_(threshold) {
    }
    
    int read() override {  // Redéfinition de la méthode virtuelle
        last_reading_ = analogRead(pin_);  // pin_ hérité de BaseSensor
        return last_reading_;
    }
};
```

### Pourquoi c'est important
- Réutilisation de code (pas de duplication)
- Hiérarchie logique (Sensor → LineSensor, UltrasonicSensor...)
- Extension facile (ajouter nouveaux types de capteurs)

### Points à enseigner
- Syntaxe : `class Derived : public Base`
- Accès aux membres protégés de la classe de base
- `override` : documentation et vérification du compilateur

### Exercice suggéré
Créer une classe `UltrasonicSensor` qui hérite de `BaseSensor`.

---

## 6. POLYMORPHISME (Lignes 135-143, 355-395)

### Concept
Un pointeur de classe de base peut pointer vers des objets de classes dérivées, permettant un comportement dynamique.

### Localisation dans le code
```cpp
// Classe de base avec fonction virtuelle pure
class Strategy {
public:
    virtual ~Strategy() = default;
    virtual MotorSpeed execute(...) = 0;  // Fonction pure
};

// Classes dérivées
class AggressiveStrategy : public Strategy {
    MotorSpeed execute(...) override { /* comportement agressif */ }
};

class DefensiveStrategy : public Strategy {
    MotorSpeed execute(...) override { /* comportement défensif */ }
};

// Utilisation polymorphique
Strategy* current_strategy_;
current_strategy_ = new AggressiveStrategy();
MotorSpeed speed = current_strategy_->execute(...);  // Appel dynamique
```

### Pourquoi c'est important
- Flexibilité : changer de comportement à l'exécution
- Extensibilité : ajouter nouvelles stratégies sans modifier le code existant
- Design patterns : Strategy, Factory, etc.

### Points à enseigner
- `virtual` : permet la redéfinition dans classes dérivées
- `= 0` : fonction virtuelle pure (classe abstraite)
- Late binding : décision à l'exécution, pas à la compilation
- Importance du destructeur virtuel

### Exercice suggéré
Implémenter une `SearchStrategy` qui fait tourner le robot pour chercher l'adversaire.

---

## 7. OPERATOR OVERLOADING (Lignes 57-89)

### Concept
Définir le comportement des opérateurs (+, -, *, ==, etc.) pour les types personnalisés.

### Localisation dans le code
```cpp
struct MotorSpeed {
    int left, right;
    
    // Addition de deux vitesses
    MotorSpeed operator+(const MotorSpeed& other) const {
        return MotorSpeed(left + other.left, right + other.right);
    }
    
    // Multiplication par un scalaire
    MotorSpeed operator*(float scale) const {
        return MotorSpeed(
            static_cast<int>(left * scale),
            static_cast<int>(right * scale)
        );
    }
    
    // Comparaison
    bool operator==(const MotorSpeed& other) const {
        return (left == other.left) && (right == other.right);
    }
};

// Utilisation naturelle
MotorSpeed speed1(100, 100);
MotorSpeed speed2(50, 50);
MotorSpeed combined = speed1 + speed2;      // operator+
MotorSpeed scaled = speed1 * 0.5f;          // operator*
bool same = (speed1 == speed2);             // operator==
```

### Pourquoi c'est important
- Code plus lisible et intuitif
- Types personnalisés se comportent comme types natifs
- Facilite les calculs mathématiques (vecteurs, matrices)

### Points à enseigner
- Syntaxe : `ReturnType operator@(params) const`
- Choix des opérateurs à surcharger (logique et intuitivité)
- `const` : l'opération ne modifie pas l'objet

### Exercice suggéré
Ajouter `operator-` pour soustraire des vitesses et `operator<<` pour affichage Serial.

---

## 8. TEMPLATES (Lignes 94-109)

### Concept
Créer des fonctions ou classes génériques qui fonctionnent avec différents types.

### Localisation dans le code
```cpp
// Template de fonction
template<typename T>
T map_value(T value, T in_min, T in_max, T out_min, T out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template<typename T>
T constrain_value(T value, T min_val, T max_val) {
    return std::clamp(value, min_val, max_val);
}

// Utilisation avec différents types
int mapped_int = map_value<int>(512, 0, 1023, 0, 255);
float mapped_float = map_value<float>(0.5f, 0.0f, 1.0f, -1.0f, 1.0f);
```

### Pourquoi c'est important
- Réutilisation de code sans duplication
- Type-safe (vérification à la compilation)
- Performance : pas de coût à l'exécution (instantiation à la compilation)

### Points à enseigner
- Syntaxe : `template<typename T>`
- Instanciation automatique ou explicite `<int>`
- STL utilise massivement les templates (vector<T>, array<T, N>)

### Exercice suggéré
Créer un template `clamp_and_map<T>` combinant les deux fonctions.

---

## 9. STL CONTAINERS (Lignes 414-423)

### Concept
La bibliothèque standard fournit des structures de données prêtes à l'emploi.

### Localisation dans le code
```cpp
#include <vector>
#include <array>

class MiniSumoRobot {
private:
    // Vector : taille dynamique
    std::vector<LineSensor*> line_sensors_;
    
    // Array : taille fixe, plus efficace
    std::array<bool, 4> line_detection_state_;
    
public:
    MiniSumoRobot() {
        // Vector : ajout dynamique
        line_sensors_.push_back(new LineSensor(...));
        
        // Array : initialisation
        line_detection_state_.fill(false);
    }
    
    void updateSensors() {
        // Itération
        for (size_t i = 0; i < line_sensors_.size(); i++) {
            line_sensors_[i]->read();
            line_detection_state_[i] = line_sensors_[i]->detectsLine();
        }
    }
};
```

### Pourquoi c'est important
- Gain de temps : pas besoin de réinventer la roue
- Sécurité : gestion automatique de la mémoire
- Performance : implémentations optimisées

### Points à enseigner
- `std::vector<T>` : tableau dynamique (push_back, size, clear)
- `std::array<T, N>` : tableau fixe (fill, size, accès [])
- Différence : vector = heap, array = stack
- Autres containers : map, set, queue, etc.

### Exercice suggéré
Remplacer le vector par un `std::array<LineSensor*, 4>` et comparer l'utilisation mémoire.

---

## 10. LAMBDA EXPRESSIONS (Lignes 443-466, 571-591)

### Concept
Fonctions anonymes définies inline, utiles pour callbacks et courtes fonctions.

### Localisation dans le code
```cpp
// Lambda simple avec capture
auto button_handler = []() -> bool {
    static unsigned long last_press = 0;
    // ... logique ...
    return true;
};

// Lambda capturant 'this'
void printSensorStatus() {
    auto print_sensor = [this](size_t index, const char* position) {
        Serial.printf("%s: %d\n", 
            position, 
            line_sensors_[index]->getLastReading());
    };
    
    print_sensor(0, "FL");
    print_sensor(1, "FR");
}

// Lambda capturant par référence
auto change_strategy = [&]() {
    DefensiveStrategy* defensive = new DefensiveStrategy();
    robot->setStrategy(*defensive);
};
```

### Pourquoi c'est important
- Code concis pour petites fonctions
- Capture du contexte environnant
- Idéal pour callbacks, tri personnalisé, événements

### Points à enseigner
- Syntaxe : `[capture](params) -> return_type { body }`
- Captures : `[]` rien, `[=]` par valeur, `[&]` par référence, `[this]` membres
- `auto` : type inféré automatiquement
- Utilisation avec algorithmes STL (sort, find_if, etc.)

### Exercice suggéré
Créer une lambda pour trier les capteurs par intensité de lecture.

---

## 11. CONST CORRECTNESS (Lignes 145-152, 274-276, 489-491)

### Concept
Utiliser `const` pour garantir qu'une fonction ou variable ne sera pas modifiée.

### Localisation dans le code
```cpp
class BaseSensor {
public:
    // Fonction const : ne modifie pas l'état de l'objet
    int getLastReading() const {
        return last_reading_;  // OK : lecture seule
        // last_reading_ = 0;  // ERREUR : modification interdite
    }
    
    uint8_t getPin() const {
        return pin_;
    }
};

class MiniSumoRobot {
public:
    bool isRunning() const {
        return is_running_;
    }
};

// Paramètre const : ne peut pas être modifié
MotorSpeed operator+(const MotorSpeed& other) const {
    // other.left = 0;  // ERREUR : other est const
    return MotorSpeed(left + other.left, right + other.right);
}
```

### Pourquoi c'est important
- Documentation : indique les intentions (lecture seule)
- Sécurité : empêche les modifications accidentelles
- Optimisation : permet au compilateur d'optimiser
- Interface : objet const ne peut appeler que méthodes const

### Points à enseigner
- `const` après fonction membre : `void foo() const`
- `const` paramètre : `void bar(const Type& param)`
- `const` variable : `const int x = 10;`
- `constexpr` : évaluation à la compilation

### Exercice suggéré
Identifier toutes les méthodes qui devraient être const mais ne le sont pas.

---

## 12. RAII (Resource Acquisition Is Initialization) (Lignes 253-276)

### Concept
Lier la durée de vie d'une ressource à la durée de vie d'un objet.

### Localisation dans le code
```cpp
class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t dir_pin, int pwm_channel)
        : pwm_pin_(pwm_pin), dir_pin_(dir_pin), pwm_channel_(pwm_channel) {
        
        // ACQUISITION de la ressource dans le constructeur
        pinMode(dir_pin_, OUTPUT);
        ledcSetup(pwm_channel_, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttachPin(pwm_pin_, pwm_channel_);
        stop();  // État sécuritaire
    }
    
    ~Motor() {
        // LIBÉRATION automatique dans le destructeur
        stop();
        ledcDetachPin(pwm_pin_);
    }
};

// Utilisation : aucune gestion manuelle nécessaire
{
    Motor my_motor(25, 26, 0);  // Ressource acquise automatiquement
    my_motor.setSpeed(200);
    // ...
}  // Ressource libérée automatiquement (destructeur appelé)
```

### Pourquoi c'est important
- Pas de fuites de ressources (mémoire, fichiers, hardware)
- Exception-safe : même si exception, destructeur est appelé
- Code plus propre : pas de cleanup manuel partout
- Fondamental en C++ moderne (smart pointers)

### Points à enseigner
- Pattern : acquérir dans constructeur, libérer dans destructeur
- Garantie : destructeur toujours appelé (fin de scope, exception, delete)
- Exemples : fichiers, mutex, connexions réseau, PWM
- Smart pointers : unique_ptr, shared_ptr appliquent RAII

### Exercice suggéré
Créer une classe `Timer` qui démarre dans le constructeur et affiche le temps écoulé dans le destructeur.

---

## PROGRESSION D'ENSEIGNEMENT RECOMMANDÉE

### Séance 1 : Bases
1. Namespace (organisation)
2. Classes et encapsulation (structure)
3. Constructeurs/Destructeurs (cycle de vie)

### Séance 2 : Orienté Objet
4. Héritage (réutilisation)
5. Polymorphisme (flexibilité)
6. Const correctness (sécurité)

### Séance 3 : Fonctionnalités Avancées
7. Operator overloading (lisibilité)
8. Templates (généricité)
9. Références (efficacité)

### Séance 4 : STL et Moderne
10. STL Containers (productivité)
11. Lambda expressions (concision)
12. RAII (gestion ressources)

---

## EXERCICES INTÉGRÉS

### Niveau Débutant
1. Modifier les pins dans le namespace
2. Changer le seuil des capteurs de ligne
3. Ajouter un message Serial dans une lambda

### Niveau Intermédiaire
1. Créer une nouvelle stratégie (ex: SpinStrategy)
2. Implémenter un capteur ultrasonique héritant de BaseSensor
3. Ajouter operator- à MotorSpeed

### Niveau Avancé
1. Implémenter un système de logging avec RAII (ouverture/fermeture fichier)
2. Créer un template de filtre (moyenne mobile) pour lisser les lectures capteurs
3. Utiliser std::algorithm avec lambdas pour trouver le capteur avec lecture max

---

## POINTS D'ATTENTION POUR L'ENSEIGNANT

### Erreurs Courantes à Anticiper
- Oublier `virtual` dans destructeur de classe de base
- Confusion entre référence et pointeur
- Oublier `const` dans méthodes qui ne modifient pas
- Fuites mémoire avec `new` sans `delete` correspondant

### Démonstrations Visuelles Suggérées
- Debugger pour voir l'ordre constructeur/destructeur
- Moniteur série pour observer le polymorphisme en action
- Mesure mémoire : vector vs array
- Benchmark : passage par valeur vs référence

### Extensions Possibles
- Ajouter capteur ultrasonique (nouveau type dérivé)
- Implémenter machine à états (enum + switch)
- Logger avec timestamps (RAII + fichier SD)
- Communication Bluetooth (callbacks avec lambdas)

---

Ce guide permet de présenter chaque concept de manière isolée tout en montrant son utilité dans un contexte pratique et cohérent.
