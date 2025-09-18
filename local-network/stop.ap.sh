#!/bin/bash
# Ferma Access Point e ripristina la scheda per il Wi-Fi di casa

IFACE="wlp4s0"

echo "[*] Spengo dnsmasq e hostapd..."
sudo systemctl stop dnsmasq
sudo systemctl stop hostapd

echo "[*] Rendo di nuovo gestita da NetworkManager l'interfaccia $IFACE..."
sudo nmcli dev set $IFACE managed yes

echo "[*] Pulisco indirizzi e riporto su l'interfaccia..."
sudo ip addr flush dev $IFACE
sudo ip link set $IFACE up

echo "[✓] AP fermato. Wifi ripristinato"
