#!/usr/bin/env python3
import requests
import notify2
import time
import sys
import os

# plik umieszczony w /usr/local/bin/robotino_battery.py

URL = 'http://192.168.0.12'

LOW_THRESHOLD = 25
VERY_LOW_THRESHOLD = 15

NORMAL_PERIOD = 30.0
LOW_PERIOD = 10.0
VERY_LOW_PERIOD = 5.0

SHUTDOWN_START = 60
SHUTDOWN_WARNINGS = {60, 40, 20}

APP_NAME = "Robotino Battery Monitor"

if not os.environ.get("DISPLAY"):
    time.sleep(2)

notify2.init(APP_NAME)

warned_low = set()
warned_very_low = set()

low_battery_active = False
any_low = False
any_very_low = False

shutdown_active = False
shutdown_time_left = SHUTDOWN_START
shutdown_warned = set()

current_period = NORMAL_PERIOD

def notify(text, icon):
    n = notify2.Notification(
        "Robotino - stan baterii",
        text,
        icon
    )
    n.set_timeout(5000)
    n.show()


def force_stop():
    notify("Wymuszone zatrzymanie (very low battery)", "process-stop")

def read_powermanagement():
    global low_battery_active

    try:
        r = requests.get(URL + '/data/powermanagement', timeout=2)
        if r.status_code != 200:
            return

        data = r.json()
        lowbatt = data.get('batteryLow')

        if isinstance(lowbatt, bool):
            low_battery_active = lowbatt

    except Exception as e:
        print(f'powermanagement error: {e}', file=sys.stderr)


def read_festoolcharger():
    global any_low, any_very_low

    try:
        r = requests.get(URL + '/data/festoolcharger', timeout=2)
        if r.status_code != 200:
            return

        payload = r.json().get('payload', {})
        capacities = payload.get('capacities', [])

        any_low = False
        any_very_low = False

        for i, cap in enumerate(capacities):
            if cap < 0:
                continue

            if cap <= VERY_LOW_THRESHOLD:
                any_very_low = True
                if i not in warned_very_low:
                    notify(
                        f'Akumulator {i+1}: {cap}%',
                        'battery-empty'
                    )
                    warned_very_low.add(i)

            elif cap <= LOW_THRESHOLD:
                any_low = True
                if i not in warned_low:
                    notify(
                        f'Akumulator {i+1}: {cap}%',
                        'battery-caution'
                    )
                    warned_low.add(i)

    except Exception as e:
        print(f'festoolcharger error: {e}', file=sys.stderr)

def shutdown_logic():
    global shutdown_active, shutdown_time_left

    if low_battery_active:
        if not shutdown_active:
            shutdown_active = True
            shutdown_time_left = SHUTDOWN_START
            shutdown_warned.clear()

        if any_very_low:
            force_stop()

        shutdown_time_left -= current_period

        for sec in SHUTDOWN_WARNINGS:
            if shutdown_time_left <= sec and sec not in shutdown_warned:
                notify(
                    f'Wyłączenie za {sec} sekund!',
                    'system-shutdown'
                )
                shutdown_warned.add(sec)

    else:
        shutdown_active = False
        shutdown_time_left = SHUTDOWN_START
        shutdown_warned.clear()

def update_period():
    global current_period

    if any_very_low:
        current_period = VERY_LOW_PERIOD
    elif any_low:
        current_period = LOW_PERIOD
    else:
        current_period = NORMAL_PERIOD

def main():
    global current_period

    notify("Monitor baterii uruchomiony", "battery")

    while True:
        read_powermanagement()
        read_festoolcharger()
        shutdown_logic()
        update_period()
        time.sleep(current_period)


if __name__ == "__main__":
    main()
