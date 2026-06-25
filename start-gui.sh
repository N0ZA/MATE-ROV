#!/bin/bash
export DISPLAY=:0

# Wait up to 30 seconds for backend to be ready
for i in $(seq 1 30); do
    curl -sf http://localhost:3000 > /dev/null 2>&1 && break
    sleep 1
done

# Parse monitor geometry from xrandr (e.g. 1920x1080+0+0)
MONITORS=$(xrandr --query | grep " connected" | grep -oP '\d+x\d+\+\d+\+\d+')
MON1=$(echo "$MONITORS" | sed -n '1p')
MON2=$(echo "$MONITORS" | sed -n '2p')

parse_geom() {
    local g=$1
    W=$(echo $g | grep -oP '^\d+(?=x)')
    H=$(echo $g | grep -oP '(?<=x)\d+(?=\+)')
    X=$(echo $g | grep -oP '(?<=\+)\d+' | sed -n '1p')
    Y=$(echo $g | grep -oP '(?<=\+)\d+' | sed -n '2p')
    echo "$W $H $X $Y"
}

read W1 H1 X1 Y1 <<< $(parse_geom "$MON1")
read W2 H2 X2 Y2 <<< $(parse_geom "$MON2")

# Control UI on monitor 2
chromium-browser --app=http://localhost:3000 \
    --user-data-dir=/tmp/chrome-control \
    --no-first-run \
    --window-position=${X2},${Y2} \
    --window-size=${W2},${H2} &

sleep 2

# Camera UI on monitor 1
chromium-browser --app=http://localhost:3001 \
    --user-data-dir=/tmp/chrome-camera \
    --no-first-run \
    --window-position=${X1},${Y1} \
    --window-size=${W1},${H1} &
