"""Read live state without reset, halt, or motor commands."""
import json
import re
import time
from pathlib import Path
from pyocd.core.helpers import ConnectHelper

base = Path(__file__).resolve().parent
source = (base / 'competition.map').read_text(encoding='utf-8', errors='replace')

def address(name):
    return int(re.search(r'^\s*' + name + r'\s+(0x[0-9a-fA-F]+)\s+Data', source, re.M)[1], 16)

options = {'target_override': 'stm32f407vetx', 'frequency': 1000000,
           'connect_mode': 'attach', 'resume_on_disconnect': False,
           'pack': ['C:/work/智能车/.Download/Keil.STM32F4xx_DFP.2.16.0.pack'],
           'no_config': True}
session = ConnectHelper.session_with_chosen_probe(blocking=False, unique_id='LU_2022_8888', options=options)
if session is None:
    raise SystemExit('CMSIS-DAP is disconnected; reconnect it for read-only diagnosis.')
with session:
    target = session.target
    print('Core:', target.get_state().name, flush=True)
    print('UARTs:', {hex(a): hex(target.read32(a)) for a in
                    [0x40004400, 0x40004408, 0x4000440c, 0x40004c0c, 0x40020020, 0xe000e104]}, flush=True)
    previous = None
    for i in range(32):
        names = ['ax_control_mode', 'ax_robot_move_enable', 'ax_motion_fault',
                 'ax_feedback_fault_mask', 'uart4_rx_con', 'uart4_rx_ok']
        snapshot = {name: target.read8(address(name)) for name in names}
        for name in ('ax_bluetooth_rx_bytes', 'ax_bluetooth_rx_frames'):
            snapshot[name] = target.read32(address(name))
        for name, size in [('uart4_rx_buf', 20), ('uart4_rx_ready', 16), ('R_Vel', 36)]:
            snapshot[name] = bytes(target.read_memory_block8(address(name), size)).hex(' ')
        if snapshot != previous:
            print(json.dumps(snapshot), flush=True)
            previous = snapshot
        time.sleep(0.25)
