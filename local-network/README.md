**Guida alla creazione di una rete locale**

1. Installare i pacchetti necessari: hostapd, dnsmasq, iw, tcpdump (per debug).
2. Verificare che la scheda wifi supporti la modalità AP (Access Point) con `iw list`
3. Identificare il nome dell'interfaccia wifi (nel mio caso wlp4s0) con `ip link`
4. Creare la configurazione di hostapd in `/etc/hostapd/hostapd.conf` specificando SSID, password WPA2 e interfaccia (2.4 GHz per Crazyflie)
5. Configurare dnsmasq in `/etc/dsnmasq.d/crazy-ap.conf` per fornire DHCP e assegnare IP statici ai vari CF in base al MAC address.
6. Assegnare un IP statico al PC sulla stessa interfaccia e disabilitare NetworkManager su quella scheda.
7. In caso di problemi durante l'assegnazione DHCP usare `journalctl -fu hostapd` e `journalctl -fu dnsmasq` per analizzare le richieste DHCP.

**Script per comodità**

Sono stati creati due script bash (`start_ap.sh` e `stop_ap.sh`) per attivare e disattivare velocemente
l’Access Point:

```
./start_ap.sh
```

- Disabilita NetworkManager sull’interfaccia Wi-Fi. - Imposta IP statico 10.0.0.1/24. - Avvia
  hostapd e dnsmasq. Risultato: la rete 'crazy-net' è attiva.

```
stop_ap.sh
```

- Ferma hostapd e dnsmasq. - Riporta la scheda sotto il controllo di NetworkManager. - Pulizia
  indirizzi IP. Risultato: il PC torna a collegarsi al Wi-Fi di casa normalmente.

**Note importanti**

- Crazyflie AI-deck supporta solo Wi-Fi a 2.4 GHz, quindi in hostapd bisogna usare `hw_mode=g` e un canale 2.4GHz (es. 6).

- Per ogni Crazyflie va aggiunta in dnsmasq la regola `dhcp-host=,,,infinite`.

- Se la scheda Wi-Fi del PC non supporta AP, usare un dongle USB compatibile (chipsetAtheros/Realtek).

- Il firewall (ufw) deve permettere traffico dalla subnet 10.0.0.0/24 e sulle porte socket usate (es. 5000/tcp).
