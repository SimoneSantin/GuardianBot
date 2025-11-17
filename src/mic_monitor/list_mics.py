# mic_monitor/list_mics.py
import sounddevice as sd

def main():
    try:
        devices = sd.query_devices()
    except Exception as e:
        print(f"Cannot query audio devices: {e}")
        return

    print("=== Input-capable audio devices ===")
    found = False
    for i, d in enumerate(devices):
        if d.get('max_input_channels', 0) > 0:
            found = True
            print(f"[{i}] {d['name']}  (inputs={d['max_input_channels']})")
    if not found:
        print("(No input devices found. Plug in your mic and try again.)")
