# Mini-Sumo Robot - Exemple Complet C++ pour Arduino IDE

## Description

Ce projet est un exemple pédagogique complet d'un robot mini-sumo programmé en C++ pour ESP32, conçu spécifiquement pour enseigner les concepts avancés de C++ dans un contexte de robotique embarquée.

**Version Arduino IDE** - Fichier unique .ino prêt à téléverser.

## Objectifs Pédagogiques

Le code illustre **tous** les concepts C++ essentiels pour la programmation embarquée :

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

- **Microcontrôleur** : ESP32 (n'importe quel modèle compatible Arduino)
- **Moteurs** : 2x moteurs DC avec driver H-Bridge (ex: L298N, DRV8833)
- **Capteurs** : 4x capteurs de ligne infrarouge (détection bords du dohyo)
- **Interface** : 1x bouton poussoir de démarrage
- **Alimentation** : Batterie adaptée pour moteurs (séparée de l'ESP32 si nécessaire)

## Configuration des Pins

Les pins peuvent être modifiées au début du fichier .ino dans le namespace `RobotConfig` :

```cpp
namespace RobotConfig {
    constexpr uint8_t MOTOR_LEFT_PWM = 25;      // PWM moteur gauche
    constexpr uint8_t MOTOR_LEFT_DIR = 26;      // Direction moteur gauche
    constexpr uint8_t MOTOR_RIGHT_PWM = 27;     // PWM moteur droit
    constexpr uint8_t MOTOR_RIGHT_DIR = 14;     // Direction moteur droit
    
    constexpr uint8_t LINE_SENSOR_FRONT_LEFT = 32;   // Capteur ligne avant gauche
    constexpr uint8_t LINE_SENSOR_FRONT_RIGHT = 33;  // Capteur ligne avant droit
    constexpr uint8_t LINE_SENSOR_BACK_LEFT = 34;    // Capteur ligne arrière gauche
    constexpr uint8_t LINE_SENSOR_BACK_RIGHT = 35;   // Capteur ligne arrière droit
    
    constexpr uint8_t START_BUTTON = 15;        // Bouton de démarrage
}
```

## Installation Arduino IDE

### 1. Installer le support ESP32

**Méthode rapide :**
1. Ouvrir Arduino IDE
2. Aller dans `Fichier` → `Préférences`
3. Dans "URL de gestionnaire de cartes additionnelles", ajouter :
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Cliquer `OK`
5. Aller dans `Outils` → `Type de carte` → `Gestionnaire de cartes`
6. Rechercher "esp32" et installer "esp32 by Espressif Systems"

### 2. Configurer la carte

1. `Outils` → `Type de carte` → `ESP32 Arduino` → `ESP32 Dev Module`
2. `Outils` → `Port` → Sélectionner le port COM de votre ESP32
3. Autres paramètres par défaut conviennent

### 3. Téléverser le code

1. Ouvrir le fichier `mini_sumo_robot.ino`
2. Cliquer sur le bouton "Téléverser" (→)
3. Attendre la compilation et le téléversement
4. Ouvrir le moniteur série (`Outils` → `Moniteur série`, 115200 bauds)

## Utilisation

### Démarrage du Robot

1. **Alimenter** l'ESP32 et les moteurs
2. **Moniteur série** : Ouvrir à 115200 bauds pour voir les messages de debug
3. **Presser le bouton START** pour démarrer le robot
4. **Presser à nouveau** pour l'arrêter

### Comportement

- **0-10 secondes** : Stratégie agressive (avance rapidement)
  - Recule si détection de ligne devant
  - Accélère si détection de ligne derrière
  
- **Après 10 secondes** : Change automatiquement vers stratégie défensive
  - Tourne pour chercher l'adversaire
  - Recule en tournant si ligne détectée

### Moniteur Série

Le moniteur série affiche :
```
========================================
   MINI-SUMO ROBOT - C++ DEMO
========================================

Mini-Sumo Robot initialized!

Press the START button to begin...

[Après pression du bouton]
Robot STARTED!
Template demo - mapped value: 50%
FL: 234 (Line: NO) | FR: 456 (Line: NO) | BL: 123 (Line: NO) | BR: 890 (Line: YES) |
...

>>> Changing to Defensive Strategy <<<
Strategy changed to: Defensive
```

## Structure du Code

Le fichier .ino contient tout dans un seul fichier, organisé ainsi :

1. **Includes et Namespace** (lignes 1-60)
2. **Operator Overloading** - Struct `MotorSpeed` (lignes 62-89)
3. **Templates** - Fonctions génériques (lignes 91-109)
4. **Classes de Capteurs** - `BaseSensor`, `LineSensor` (lignes 111-227)
5. **Classe Motor** - Contrôle moteur avec RAII (lignes 229-276)
6. **Stratégies** - `Strategy`, `AggressiveStrategy`, `DefensiveStrategy` (lignes 278-320)
7. **Classe Robot** - `MiniSumoRobot` (lignes 322-505)
8. **Fonctions Arduino** - `setup()`, `loop()` (lignes 507-fin)

## Calibration des Capteurs

### Régler le Seuil de Détection

Les capteurs de ligne ont un seuil par défaut de 512 (sur une échelle 0-1023). Pour calibrer :

1. **Observer les valeurs** dans le moniteur série
2. **Identifier** la valeur typique sur surface blanche vs noire
3. **Modifier** dans le code :

```cpp
// Dans MiniSumoRobot constructor
line_sensors_.push_back(new LineSensor(RobotConfig::LINE_SENSOR_FRONT_LEFT, 600));  // Seuil personnalisé
```

Ou modifier dynamiquement :
```cpp
int new_threshold = 700;
line_sensors_[0]->setThreshold(new_threshold);
```

## Personnalisation

### Ajouter une Nouvelle Stratégie

```cpp
class SpinStrategy : public Strategy {
public:
    MotorSpeed execute(const std::array<bool, 4>& line_detected) override {
        // Si ligne détectée : reculer
        if (line_detected[0] || line_detected[1] || line_detected[2] || line_detected[3]) {
            return MotorSpeed(-150, -150);
        }
        // Sinon : tourner rapidement
        return MotorSpeed(255, -255);
    }
    
    const char* getName() const override {
        return "Spin";
    }
};

// Dans loop(), changer la stratégie
robot->setStrategy(*(new SpinStrategy()));
```

### Modifier les Vitesses

Ajuster les valeurs dans les stratégies :
- `MotorSpeed(200, 200)` : Plus lent = plus de contrôle
- `MotorSpeed(255, 255)` : Plus rapide = plus agressif

### Changer le Délai de Changement de Stratégie

Dans `loop()`, modifier :
```cpp
if (robot->isRunning() && !strategy_changed && (millis() - start_time > 15000)) {  // 15 secondes au lieu de 10
```

## Dépannage

### Le code ne compile pas

**Erreur : "vector: No such file or directory"**
- Solution : Vérifier que le support ESP32 est bien installé
- Le C++11/14/17 est nécessaire (inclus dans ESP32 Arduino)

**Erreur : "override does not override"**
- Solution : Vérifier la signature de la fonction virtuelle
- S'assurer que `const` est au bon endroit

### Le robot ne démarre pas

1. **Vérifier l'alimentation** de l'ESP32 et des moteurs
2. **Moniteur série** : Y a-t-il des messages ?
3. **Bouton START** : Vérifier le câblage (GPIO 15, avec pull-up interne)
4. **LED ESP32** : Clignote-t-elle ? (indique l'exécution du programme)

### Les moteurs ne tournent pas

1. **Alimentation moteurs** : Vérifier la batterie/source d'alimentation
2. **H-Bridge** : Vérifier les connexions
   - Enable/VCC du driver
   - IN1, IN2 du driver → DIR pins ESP32
   - PWM du driver → PWM pins ESP32
3. **Moniteur série** : Observer les valeurs de vitesse
4. **Test manuel** : Appeler `motor.setSpeed(100)` dans `setup()`

### Les capteurs ne détectent pas

1. **Alimentation capteurs** : Vérifier VCC/GND
2. **Câblage** : Signal → GPIO ESP32
3. **Calibration** : Observer les valeurs brutes dans le moniteur série
4. **Seuil** : Ajuster le threshold (défaut 512)
5. **Type de capteur** : Analogique (0-1023) ou numérique ?

### Moniteur série affiche des caractères bizarres

- **Bauds** : Vérifier que le moniteur est à 115200 bauds
- **Reset** : Presser le bouton RESET de l'ESP32
- **USB** : Essayer un autre câble/port USB

## Concepts Pédagogiques - Référence Rapide

| Concept | Ligne(s) | Description |
|---------|----------|-------------|
| Namespace | 32-52 | Organisation config avec `RobotConfig::` |
| Références | 221, 279 | `int&`, `Strategy&` pour éviter copies |
| Classes | 120-505 | Encapsulation : private/public/protected |
| Constructeurs | 126, 253, 296 | Initializer lists `: member_(value)` |
| Destructeurs | 133, 269, 313 | Nettoyage automatique avec `~` |
| Héritage | 181 | `class LineSensor : public BaseSensor` |
| Polymorphisme | 135, 355 | `virtual`, fonctions pures `= 0` |
| Operator overload | 68-86 | `operator+`, `operator*`, `operator==` |
| Templates | 97-109 | `template<typename T>` génériques |
| STL vector | 414 | `std::vector<LineSensor*>` dynamique |
| STL array | 417 | `std::array<bool, 4>` taille fixe |
| Lambda | 443, 571 | `auto func = []() { ... }` anonyme |
| Const | 145-152 | `const` après fonction membre |
| RAII | 253-276 | Ressources acquises/libérées auto |

## Progression d'Enseignement Suggérée

### Séance 1 : Fondamentaux (1-2h)
1. Montrer le namespace et la configuration
2. Expliquer les classes `Motor` et `BaseSensor`
3. Démontrer constructeurs/destructeurs avec Serial.print

### Séance 2 : Orienté Objet (1-2h)
1. Héritage : `LineSensor` extends `BaseSensor`
2. Polymorphisme : créer une nouvelle `Strategy`
3. Exercice : implémenter `UltrasonicSensor`

### Séance 3 : Avancé (1-2h)
1. Operator overloading avec `MotorSpeed`
2. Templates : créer `average<T>()` pour filtrer capteurs
3. STL : utiliser `std::find_if` avec lambda

### Séance 4 : Moderne et Pratique (1-2h)
1. Lambdas : callbacks pour événements
2. RAII : gérer fichier SD ou LED
3. Projet final : robot autonome complet

## Ressources Complémentaires

### Documentation
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/)
- [C++ Reference](https://en.cppreference.com/)
- [Arduino Language Reference](https://www.arduino.cc/reference/en/)

### Tutoriels Recommandés
- Polymorphisme : chercher "C++ virtual functions tutorial"
- STL : chercher "C++ STL containers tutorial"
- RAII : chercher "C++ RAII pattern"

## Licence

Code pédagogique libre d'utilisation pour l'enseignement du C++.

## Support

Pour questions ou problèmes :
1. Vérifier la section Dépannage ci-dessus
2. Observer le moniteur série pour diagnostics
3. Tester chaque composant individuellement

## Auteur

Créé comme exemple pédagogique pour l'enseignement du C++ en robotique embarquée avec Arduino IDE.

---

**Bon apprentissage du C++ avec votre robot mini-sumo ! 🤖**
