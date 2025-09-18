#!/usr/bin/env python3
import socket
import json
import time
from typing import Optional, Tuple

HOST = "10.64.39.88"  # AI-deck IP
PORT = 5000           # CPX TCP port

# === CPX endpoints (coerenti con i log) ===
CPX_T_STM32 = 1   # Crazyflie MCU
CPX_T_ESP32 = 2   # AI-deck ESP32
CPX_T_HOST  = 3   # Host via Wi-Fi/TCP
CPX_T_GAP8  = 4   # AI-deck GAP8

# === CPX functions ===
CPX_F_WIFI_CTRL = 4
CPX_F_APP       = 5

# === CPX version ===
CPX_VERSION = 0

# === Framing TCP ===
# [ length:2 LITTLE ][ header:2 BIG ][ payload ]
LEN_ENDIAN = "little"  # SOLO per la length TCP

# ---------------- CPX framing ----------------

def build_cpx_header(source: int, destination: int, version: int, function: int, lp: int = 1, rsv: int = 0) -> bytes:
    """
    Header CPX a 16 bit (bitfield):
    [rsv:1][lp:1][src:3][dst:3][ver:2][fun:6]
    >>> L'HEADER CPX è in BIG-ENDIAN <<<
    """
    h = (
        ((rsv         & 0x1) << 15) |
        ((lp          & 0x1) << 14) |
        ((source      & 0x7) << 11) |
        ((destination & 0x7) <<  8) |
        ((version     & 0x3) <<  6) |
        ((function    & 0x3F))
    )
    return h.to_bytes(2, "big")  # HEADER BIG-ENDIAN

def parse_cpx_header(hdr_bytes: bytes) -> dict:
    h = int.from_bytes(hdr_bytes, "big")  # HEADER BIG-ENDIAN
    return {
        "rsv": (h >> 15) & 0x1,
        "lp":  (h >> 14) & 0x1,
        "src": (h >> 11) & 0x7,
        "dst": (h >>  8) & 0x7,
        "ver": (h >>  6) & 0x3,
        "fun":  h        & 0x3F,
    }

def build_cpx_packet(source: int, destination: int, function: int, data_bytes: bytes, lp: int = 1) -> bytes:
    hdr = build_cpx_header(source, destination, version=CPX_VERSION, function=function, lp=lp)
    return hdr + data_bytes

def send_cpx(sock: socket.socket, dst: int, fun: int, payload: bytes, lp: int = 1, src: int = CPX_T_HOST) -> bytes:
    pkt = build_cpx_packet(source=src, destination=dst, function=fun, data_bytes=payload, lp=lp)
    # framing TCP: 2 byte length LITTLE-ENDIAN + (header BIG + payload)
    sock.sendall(len(pkt).to_bytes(2, LEN_ENDIAN) + pkt)
    return pkt

def _recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
    """Legge esattamente n byte (o None se connessione chiusa)."""
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def recv_cpx(sock: socket.socket) -> Optional[Tuple[dict, bytes]]:
    """
    Riceve un frame CPX: [len:2 little][hdr:2 big][data:len-2]
    Ritorna (header_dict, payload) oppure None se la connessione si chiude.
    """
    length_bytes = _recv_exact(sock, 2)
    if not length_bytes:
        return None
    total_len = int.from_bytes(length_bytes, LEN_ENDIAN)
    if total_len < 2:
        _ = _recv_exact(sock, max(0, total_len))
        return None

    buf = _recv_exact(sock, total_len)
    if not buf or len(buf) < 2:
        return None

    hdr = buf[:2]          # header BIG-ENDIAN
    data = buf[2:]
    return (parse_cpx_header(hdr), data)

# ------------- Helpers di comodo -------------

def test_router_ping(sock, text: str = "ping"):
    # HOST -> ESP32 (solo per far apparire un log nel router)
    send_cpx(sock, dst=CPX_T_ESP32, fun=CPX_F_WIFI_CTRL, payload=text.encode("utf-8"), lp=1)
    print(f"📤 Ping ESP32 (WIFI_CTRL): {text}")

def send_to_gap8(sock: socket.socket, text: str):
    # HOST -> GAP8 sulla function APP (quello che ti serve)
    send_cpx(sock, dst=CPX_T_GAP8, fun=CPX_F_APP, payload=text.encode("utf-8"), lp=1)
    print(f"📤 Sent to GAP8 (APP): {text}")

def pretty_print_payload(payload: bytes):
    """
    Stampa in modo 'carino' gli ACK/ALERT/CMD_RESPONSE in JSON (una riga = un JSON).
    """
    try:
        s = payload.decode("utf-8", errors="replace").strip()
        if not s:
            print("ℹ️ Payload vuoto.")
            return
        for line in s.splitlines():
            if not line:
                continue
            obj = json.loads(line)
            drone = obj.get("drone_id", "?")
            mtype = obj.get("type", "unknown")
            msg   = obj.get("message", "")
            count = obj.get("count", None)

            if mtype == "alert":
                if isinstance(count, int):
                    testo = "1 faccia" if count == 1 else f"{count} facce"
                    print(f"\n🚨 ALERT da {drone}: {testo} ({msg})\n")
                else:
                    print(f"\n🚨 ALERT da {drone}: {msg}\n")
            elif mtype == "ack":
                print(f"✅ ACK da {drone}: {msg}")
            elif mtype == "cmd_response":
                print(f"🟦 CMD_RESPONSE da {drone}: {msg}")
            else:
                print(f"ℹ️ [{mtype}] da {drone}: {msg}  raw={line}")
    except Exception:
        print("ℹ️ Payload non-JSON:", payload[:120], "..." if len(payload) > 120 else "")

# ------------- Modalità operative -------------

def receive_only(sock: socket.socket):
    print("🔊 In ascolto pacchetti CPX… (Ctrl-C per stop)\n")
    try:
        while True:
            res = recv_cpx(sock)
            if res is None:
                print("❌ Connessione chiusa.")
                break
            hdr, data = res
            src, dst, fun = hdr["src"], hdr["dst"], hdr["fun"]
            print(f"CPX: src={src} dst={dst} fun={fun} len={len(data)}")
            if src == CPX_T_GAP8 and dst == CPX_T_HOST and fun == CPX_F_APP:
                pretty_print_payload(data)
    except KeyboardInterrupt:
        print("\n👋 Ricezione terminata.")

def send_and_receive(sock: socket.socket):
    print("✏️  Invia e ricevi (exit per uscire)\n")
    try:
        while True:
            msg = input("> ").strip()
            if msg.lower() == "exit":
                return
            if msg.lower() == "ping":
                test_router_ping(sock)
            else:
                send_to_gap8(sock, msg)

            # finestra breve per leggere eventuali risposte/alert
            deadline = time.time() + 2.0
            while time.time() < deadline:
                remaining = max(0.0, deadline - time.time())
                sock.settimeout(remaining)
                try:
                    res = recv_cpx(sock)
                except socket.timeout:
                    break
                if res is None:
                    print("❌ Connessione chiusa.")
                    return
                hdr, data = res
                src, dst, fun = hdr["src"], hdr["dst"], hdr["fun"]
                print(f"CPX: src={src} dst={dst} fun={fun} len={len(data)}")
                if src == CPX_T_GAP8 and dst == CPX_T_HOST and fun == CPX_F_APP:
                    pretty_print_payload(data)
    except KeyboardInterrupt:
        print("\n👋 Stop.")

# ------------------- main --------------------

if __name__ == "__main__":
    mode = input("Scegli: [1] Solo ricevi  [2] Invia e ricevi: ").strip()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"✅ Connesso a {HOST}:{PORT}\n")
        if mode == "1":
            receive_only(s)
        elif mode == "2":
            send_and_receive(s)
        else:
            print("⚠️  Modalità non valida.")

    print("👋 Fine."  )