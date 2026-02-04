# Guide d'Installation Arduino IDE pour ESP32

## Installation Complète Pas-à-Pas

### Étape 1 : Télécharger Arduino IDE

1. Aller sur https://www.arduino.cc/en/software
2. Télécharger la version pour votre système d'exploitation :
   - Windows : Fichier .exe ou Windows App
   - macOS : Fichier .dmg
   - Linux : AppImage ou package
3. Installer Arduino IDE

**Version recommandée** : Arduino IDE 2.x (plus moderne) ou 1.8.x (classique)

### Étape 2 : Installer le Support ESP32

#### Arduino IDE 2.x (Méthode Moderne)

1. **Ouvrir Arduino IDE**

2. **Ajouter l'URL du gestionnaire de cartes** :
   - Cliquer sur `File` (Fichier) → `Preferences` (Préférences)
   - Dans le champ "Additional Boards Manager URLs" (URL de gestionnaire de cartes additionnelles)
   - Coller cette URL :
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Si vous avez déjà d'autres URLs, séparez-les avec une virgule
   - Cliquer `OK`

3. **Installer la plateforme ESP32** :
   - Cliquer sur l'icône "Boards Manager" (Gestionnaire de cartes) dans la barre latérale gauche
   - Ou aller dans `Tools` → `Board` → `Boards Manager`
   - Dans la barre de recherche, taper "esp32"
   - Trouver "esp32 by Espressif Systems"
   - Cliquer `INSTALL`
   - Attendre la fin du téléchargement (peut prendre plusieurs minutes)

#### Arduino IDE 1.8.x (Méthode Classique)

1. **Ouvrir Arduino IDE**

2. **Ajouter l'URL** :
   - `File` → `Preferences`
   - "Additional Boards Manager URLs"
   - Ajouter : `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Cliquer `OK`

3. **Installer ESP32** :
   - `Tools` → `Board` → `Boards Manager...`
   - Rechercher "esp32"
   - Installer "esp32 by Espressif Systems"

### Étape 3 : Installer les Drivers USB (Si Nécessaire)

#### Windows

La plupart des ESP32 utilisent un des chips suivants :
- **CP210x** (Silicon Labs)
- **CH340** (WCH)
- **FTDI**

**CP210x (le plus commun)** :
1. Télécharger depuis https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
2. Installer le driver
3. Redémarrer l'ordinateur

**CH340** :
1. Télécharger depuis http://www.wch.cn/download/CH341SER_EXE.html
2. Installer
3. Redémarrer

#### macOS

- CP210x : Télécharger depuis Silicon Labs (lien ci-dessus)
- CH340 : Souvent détecté automatiquement, sinon chercher "CH340 macOS driver"

#### Linux

Généralement les drivers sont déjà inclus. Si problème :
```bash
# Ajouter votre utilisateur au groupe dialout
sudo usermod -a -G dialout $USER
# Déconnexion/reconnexion nécessaire
```

### Étape 4 : Configurer la Carte ESP32

1. **Connecter l'ESP32** à l'ordinateur via USB

2. **Sélectionner la carte** :
   - `Tools` → `Board` → `ESP32 Arduino`
   - Choisir votre modèle (si incertain, choisir "ESP32 Dev Module")
   - Options communes :
     - ESP32 Dev Module (générique)
     - ESP32-WROOM-DA Module
     - NodeMCU-32S
     - DOIT ESP32 DEVKIT V1

3. **Sélectionner le port** :
   - `Tools` → `Port`
   - Windows : COMx (ex: COM3, COM4)
   - macOS : /dev/cu.usbserial-xxxx ou /dev/cu.SLAB_USBtoUART
   - Linux : /dev/ttyUSB0 ou /dev/ttyACM0

4. **Configurer les paramètres** (optionnel, les défauts fonctionnent) :
   - `Tools` → `Upload Speed` : 115200 ou 921600
   - `Tools` → `Flash Frequency` : 80MHz
   - `Tools` → `Partition Scheme` : Default 4MB with spiffs

### Étape 5 : Tester l'Installation

#### Test Simple : Blink

1. **Ouvrir l'exemple Blink** :
   - `File` → `Examples` → `01.Basics` → `Blink`

2. **Modifier le code** pour ESP32 :
   ```cpp
   void setup() {
     pinMode(2, OUTPUT);  // LED interne sur GPIO 2 pour la plupart des ESP32
   }
   
   void loop() {
     digitalWrite(2, HIGH);
     delay(1000);
     digitalWrite(2, LOW);
     delay(1000);
   }
   ```

3. **Téléverser** :
   - Cliquer sur le bouton "Upload" (→)
   - Attendre "Done uploading"
   - La LED interne devrait clignoter

### Étape 6 : Ouvrir le Projet Mini-Sumo

1. **Télécharger** le fichier `mini_sumo_robot.ino`

2. **Ouvrir avec Arduino IDE** :
   - Double-cliquer sur le fichier .ino
   - OU `File` → `Open` et sélectionner le fichier

3. **Vérifier la compilation** :
   - Cliquer sur le bouton "Verify" (✓)
   - Devrait compiler sans erreur

4. **Téléverser** :
   - Connecter l'ESP32
   - Cliquer "Upload" (→)
   - Attendre "Done uploading"

5. **Moniteur Série** :
   - Cliquer sur l'icône "Serial Monitor" (loupe) en haut à droite
   - OU `Tools` → `Serial Monitor`
   - Sélectionner 115200 bauds dans le menu déroulant

## Dépannage Installation

### Problème : Port COM n'apparaît pas

**Solutions** :
1. Vérifier que l'ESP32 est bien connecté (LED power allumée)
2. Essayer un autre câble USB (certains câbles sont charge-only)
3. Installer les drivers USB (voir Étape 3)
4. Redémarrer Arduino IDE
5. Redémarrer l'ordinateur

### Problème : "espcomm_upload_mem failed"

**Solutions** :
1. Maintenir le bouton BOOT de l'ESP32 pendant le téléversement
2. Réduire l'Upload Speed : `Tools` → `Upload Speed` → 115200
3. Vérifier que le bon port est sélectionné
4. Désactiver antivirus temporairement

### Problème : Compilation échoue - "vector: No such file or directory"

**Solutions** :
1. Vérifier que la carte ESP32 est bien sélectionnée (pas Arduino Uno)
2. Réinstaller le support ESP32 via Boards Manager
3. Mettre à jour Arduino IDE vers version récente

### Problème : "A fatal error occurred: Failed to connect"

**Solutions** :
1. Presser et maintenir le bouton BOOT pendant le téléversement
2. Presser RESET puis immédiatement BOOT
3. Vérifier les drivers USB
4. Essayer un autre port USB

### Problème : Code compile mais ne fait rien

**Solutions** :
1. Ouvrir le moniteur série pour voir les messages
2. Vérifier les bauds (115200)
3. Presser le bouton RESET de l'ESP32
4. Vérifier l'alimentation (USB doit fournir assez de courant)

## Configuration Avancée

### Augmenter la Vitesse de Téléversement

Pour téléversements plus rapides :
```
Tools → Upload Speed → 921600
```

Note : Peut causer des erreurs sur certains systèmes, revenir à 115200 si problème.

### Activer Plus de Messages de Debug

```
Tools → Core Debug Level → Info ou Debug ou Verbose
```

Utile pour diagnostiquer des problèmes.

### Partition Scheme pour Plus de Mémoire

Si vous ajoutez beaucoup de fonctionnalités :
```
Tools → Partition Scheme → Huge APP (3MB No OTA/1MB SPIFFS)
```

## Vérification de l'Installation

### Checklist Finale

- [ ] Arduino IDE installé et fonctionne
- [ ] Support ESP32 installé via Boards Manager
- [ ] Drivers USB installés (si nécessaire)
- [ ] ESP32 détecté (port COM visible)
- [ ] Carte ESP32 sélectionnée dans Tools → Board
- [ ] Port correct sélectionné dans Tools → Port
- [ ] Test Blink réussi (LED clignote)
- [ ] Moniteur série fonctionne à 115200 bauds
- [ ] mini_sumo_robot.ino compile sans erreur

### Informations Système

Pour vérifier votre configuration :
1. `File` → `Preferences` → Voir "Sketchbook location"
2. `Tools` → `Board` → Noter le modèle ESP32
3. `Tools` → `Port` → Noter le port
4. `Help` → `About Arduino IDE` → Noter la version

## Ressources Utiles

### Documentation Officielle
- Arduino IDE : https://www.arduino.cc/en/Guide
- ESP32 Arduino : https://docs.espressif.com/projects/arduino-esp32/
- Espressif : https://www.espressif.com/en/support/download/documents

### Forums et Support
- Arduino Forum : https://forum.arduino.cc/
- ESP32 Forum : https://esp32.com/
- Reddit r/esp32 : https://www.reddit.com/r/esp32/

### Tutoriels
- Random Nerd Tutorials (ESP32) : https://randomnerdtutorials.com/projects-esp32/
- DroneBot Workshop : https://dronebotworkshop.com/

## Commandes Utiles

### Via Moniteur Série

Le robot répond à certaines entrées (si vous ajoutez du code) :
```
s - start/stop
r - reset
d - debug info
```

### Raccourcis Arduino IDE

- `Ctrl + R` (Cmd + R sur Mac) : Vérifier/Compiler
- `Ctrl + U` (Cmd + U sur Mac) : Téléverser
- `Ctrl + Shift + M` (Cmd + Shift + M sur Mac) : Moniteur Série
- `Ctrl + T` (Cmd + T sur Mac) : Auto-format du code

## Prochaines Étapes

Après installation réussie :
1. Lire le README.md du projet
2. Comprendre la structure du code
3. Modifier les pins selon votre hardware
4. Calibrer les capteurs de ligne
5. Tester chaque fonction individuellement
6. Assembler le robot complet

---

**Félicitations ! Vous êtes prêt à programmer votre robot mini-sumo ! 🎉**
