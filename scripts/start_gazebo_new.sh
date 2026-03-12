#!/bin/bash

echo "Starting PX4 Gazebo Simulator..."

# ── Percorso agli SDF (cartella models/ nella root del progetto) ──────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MODELS_DIR="$PROJECT_DIR/models"

# ── World usato da gz_x500 ────────────────────────────────────────────────────
GZ_WORLD="lawn"

# ── Trap per chiudere PX4 se lo script viene interrotto (Ctrl+C) ──────────────
cleanup() {
    echo "Chiusura in corso... Killing PX4 ($PX4_PID)"
    kill -TERM "$PX4_PID" 2>/dev/null
}
trap cleanup SIGINT SIGTERM EXIT

# ── Trova PX4 ─────────────────────────────────────────────────────────────────
if [ -d "$HOME/PX4-Autopilot" ]; then
    PX4_DIR="$HOME/PX4-Autopilot"
elif [ -d "/opt/px4" ]; then
    PX4_DIR="/opt/px4"
else
    echo "Errore: directory PX4-Autopilot non trovata!"
    echo "Aggiorna il path in questo script."
    exit 1
fi

cd "$PX4_DIR"

# ── Avvia PX4+Gazebo in background ───────────────────────────────────────────
echo "Avvio PX4 SITL con Gazebo (gz_x500) nel world '$GZ_WORLD'..."
PX4_GZ_WORLD=$GZ_WORLD make px4_sitl gz_x500 &
PX4_PID=$!

# ── Attendi che gz-sim sia pronto (servizio /world/default/create disponibile) -
echo "Attesa avvio Gazebo..."
MAX_WAIT=60   # secondi massimi di attesa
ELAPSED=0

# Verifica se 'gz' è installato
if ! command -v gz &> /dev/null; then
    echo "ERRORE: Comando 'gz' non trovato. Assicurati che Gazebo sia installato e nel PATH."
    exit 1
fi

until gz service --list 2>/dev/null | grep -q "/world/${GZ_WORLD}/create"; do
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    echo "  ...in attesa di Gazebo ($ELAPSED/${MAX_WAIT}s)"
    
    # Se PX4 muore durante l'attesa, esci
    if ! ps -p $PX4_PID > /dev/null; then
        echo "ERRORE: Il processo PX4 (PID $PX4_PID) è terminato inaspettatamente."
        exit 1
    fi

    if [ $ELAPSED -ge $MAX_WAIT ]; then
        echo "Timeout: Gazebo non ha risposto entro ${MAX_WAIT}s. Spawn saltato."
        exit 1
    fi
done

echo "Gazebo pronto. Spawning modelli..."

# Verifica esistenza cartella models
if [ ! -d "$MODELS_DIR" ]; then
    echo "ATTENZIONE: Cartella models non trovata in $MODELS_DIR. Salto lo spawn."
else

# ── Spawn geofence (cilindro statico rosso trasparente, r=5m) ─────────────────
    if [ -f "${MODELS_DIR}/geofence_circle.sdf" ]; then
        echo "  → Spawn geofence_circle..."
        gz service -s /world/${GZ_WORLD}/create \
            --reqtype gz.msgs.EntityFactory \
            --reptype gz.msgs.Boolean \
            --timeout 5000 \
            --req "sdf_filename: \"${MODELS_DIR}/geofence_circle.sdf\" name: \"geofence\""
    else
        echo "  ⚠ File ${MODELS_DIR}/geofence_circle.sdf non trovato."
    fi

# ── Spawn macchinina (box con ruote, verrà mosso dal nodo ROS2) ───────────────
    if [ -f "${MODELS_DIR}/ground_vehicle.sdf" ]; then
        echo "  → Spawn ground_vehicle..."
        gz service -s /world/${GZ_WORLD}/create \
            --reqtype gz.msgs.EntityFactory \
            --reptype gz.msgs.Boolean \
            --timeout 5000 \
            --req "sdf_filename: \"${MODELS_DIR}/ground_vehicle.sdf\" name: \"ground_vehicle\""
    else
        echo "  ⚠ File ${MODELS_DIR}/ground_vehicle.sdf non trovato."
    fi
fi

echo ""
echo "Modelli spawnati. Gazebo in esecuzione (PID: $PX4_PID)."
echo "Premi Ctrl+C per fermare PX4+Gazebo."

# ── Attendi la fine di PX4 ───────────────────────────────────────────────────
wait $PX4_PID
echo "Gazebo simulator fermato."
