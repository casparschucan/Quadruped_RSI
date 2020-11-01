# Quadruped_RSI

Anleitung um die trainierten Modelle auszuprobieren:   
1.benötigte Bibliotheken installieren\
2.In tests Ordner die gewünschte Simulation herunterladen\
3.gewünschtes und zu Simulation passendes Modell als .zip Datei in gleichen Ordner wie Simulatonsdatei herunterladen\
4.gewünschter und zu Simulations passender Roboter als .urdf Datei in gleichen Ordner wie Simulatonsdatei herunterladen\
5.An richtigen, auskommenierten Stellen Dateinamen von Roboter und Modell in Simulatonsdatei einfügen  
6.(Nur nötig bei trainierten Modellen für version2/3 oder robotY) Pfad der Python Bibliotheken auf dem Computer finden. Bei mir Programme/Python/Python37/Lib/site-packages. Dort wähle stable_baselines/sac. Im Ordner sac die Datei policies.py mit Code Editor öffnen und Zeile 176 von layers = [64, 64] zu layers = [256, 256] ändern. Diese Änderung muss wieder rückgängig gemacht werden um danach Modelle für version0/1 laufen zu lassen

benötigte Bibliotheken:\
openAI Gym\
stable baselines\
pybullet\
numpy

andere Notwendigkeiten:\
Python 3.5 oder neuer
