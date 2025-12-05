import odrive
from odrive.enums import *

odrv0 = odrive.find_any()
print(f"Connected to ODrive: {odrv0.serial_number}")
print(f"Firmware: v{odrv0.fw_version_major}.{odrv0.fw_version_minor}.{odrv0.fw_version_revision}\n")

def print_attributes(obj, name, exclude_prefixes=['_', 'clear', 'get', 'set']):
    print(f"\n=== {name} ===")
    attrs = [attr for attr in dir(obj) if not any(attr.startswith(p) for p in exclude_prefixes)]
    for attr in sorted(attrs):
        try:
            value = getattr(obj, attr)
            if not callable(value):
                print(f"{attr}: {value}")
        except Exception as e:
            print(f"{attr}: <error reading: {e}>")

print_attributes(odrv0.config, "Main Config")

print_attributes(odrv0.axis0.config, "Axis 0 Config")
print_attributes(odrv0.axis0.motor.config, "Motor 0 Config")
print_attributes(odrv0.axis0.controller.config, "Controller 0 Config")
print_attributes(odrv0.axis0.encoder.config, "Encoder 0 Config")

print_attributes(odrv0.axis1.config, "Axis 1 Config")
print_attributes(odrv0.axis1.motor.config, "Motor 1 Config")
print_attributes(odrv0.axis1.controller.config, "Controller 1 Config")
print_attributes(odrv0.axis1.encoder.config, "Encoder 1 Config")