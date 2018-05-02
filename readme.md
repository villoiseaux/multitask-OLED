<h1>Descrption</h1>

Ce programme fait l'acquisition du signal analogique présent sur A3 (ESP32) et en calcule le spectre (FFT).
Ce spectre est envoyé sur un topic MQTT.

Pour le moment on gère des Brokers MQTT authentifiés mais en clair. Pas de support de MQTTS

<h1>ATTENTION</h1>

Avant la compilation de ce code il faut "patcher" la librairie <b>PubSubClient</b>.

Dans PubSubClient.h

<pre>
// MQTT_MAX_PACKET_SIZE : Maximum packet size
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 1024
#endif
</pre>
