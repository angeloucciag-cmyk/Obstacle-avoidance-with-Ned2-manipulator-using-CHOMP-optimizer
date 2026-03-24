# Obstacle Avoidance for Niryo Ned2 using CHOMP Algorithm

Questo progetto implementa un sistema di pianificazione del moto e prevenzione delle collisioni per il manipolatore industriale **Niryo Ned2**, focalizzandosi su task di *pick and place* in ambienti semi-strutturati.

## 📌 Descrizione del Progetto
L'obiettivo è generare traiettorie fluide che evitino ostacoli statici. Il cuore del sistema è l'algoritmo **CHOMP** (Covariant Hamiltonian Optimization for Motion Planning), che ottimizza una traiettoria iniziale minimizzando un funzionale di costo basato su:
1.  **Smoothness:** regolarità del movimento.
2.  **Obstacle Cost:** distanza di sicurezza dalle collisioni.

## 🛠 Tecnologie e Tool
* **Linguaggio:** MATLAB
* **Toolbox:** Robotics System Toolbox
* **Algoritmo:** CHOMP (Covariant Hamiltonian Optimization for Motion Planning)
* **Modellazione:** Primitive geometriche (sfere) per la rappresentazione degli ostacoli.

## 🚀 Key Features
* **Pianificazione nello spazio dei giunti:** Generazione di traiettorie sicure per robot a 6 gradi di libertà.
* **Gestione degli Ostacoli:** Gli ostacoli sono modellati come campi di potenziale per influenzare l'ottimizzazione della traiettoria senza collisioni.
* **Ottimizzazione Iterativa:** Partendo da una traiettoria "naive" (lineare), l'algoritmo converge verso una soluzione fluida e sicura.

## 📊 Risultati
Il sistema è stato validato in ambiente simulato, dimostrando:
* Assenza di collisioni anche con ostacoli posizionati lungo la linea retta tra Start e Goal.
* Rispetto dei limiti dei giunti del robot Niryo Ned2.
* Traiettorie pronte per l'implementazione su hardware reale tramite ROS/MATLAB Bridge.

## 📂 Struttura del Repository
* `/src`: Script MATLAB principali.
* `/models`: File di descrizione del robot (URDF/Mesh).
* `/docs`: Report tecnico dettagliato del progetto.

---
*Progetto sviluppato nell'ambito del corso di DYNAMICAL SYSTEM THEORY, Laurea Magistrale in Automazione-robotics.*
