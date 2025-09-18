#!/bin/bash
# Avvia Access Point "crazy-net" su wlp4s0

IFACE="wlp4s0"
SSID="crazy-net"
PASS="SuperSegreta123"

echo "[*] Disabilito NetworkManager su $IFACE..."
sudo nmcli dev set $IFACE managed no

echo "[*] Pulisco indirizzi e assegno IP statico 10.0.0.1..."
sudo ip addr flush dev $IFACE
sudo ip addr add 10.0.0.1/24 dev $IFACE
sudo ip link set $IFACE up

echo "[*] Sblocco hostapd e avvio servizi..."
sudo systemctl unmask hostapd
sudo systemctl start hostapd
sudo dnsmasq --test && sudo systemctl restart dnsmasq

echo "[✓] AP '$SSID' attivo su $IFACE (IP 10.0.0.1)"
