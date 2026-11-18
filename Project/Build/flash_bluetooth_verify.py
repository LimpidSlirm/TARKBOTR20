"""One-session programming and full readback for this explicitly authorized flash."""
from pathlib import Path
import hashlib
import json
import re
import time
from datetime import datetime, timezone
from intelhex import IntelHex
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer

HERE = Path(__file__).resolve().parent
image = HERE / 'competition.hex'
firmware = IntelHex(str(image))
segments = firmware.segments()
assert segments and all(0x08000000 <= a < b <= 0x08080000 for a, b in segments)
stack_top = int.from_bytes(firmware.gets(0x08000000, 4), 'little')
reset_vector = int.from_bytes(firmware.gets(0x08000004, 4), 'little')
assert 0x20000000 < stack_top <= 0x20020000 and stack_top % 8 == 0
assert reset_vector & 1 and 0x08000000 <= reset_vector < 0x08080000
report = {'utc': datetime.now(timezone.utc).isoformat(),
          'hex_sha256': hashlib.sha256(image.read_bytes()).hexdigest(),
          'probe_uid': 'LU_2022_8888', 'readback_verified': False}
session = ConnectHelper.session_with_chosen_probe(
    blocking=False, unique_id=report['probe_uid'],
    options={'target_override': 'stm32f407vetx', 'frequency': 1000000,
             'connect_mode': 'under-reset', 'resume_on_disconnect': False,
             'pack': ['C:/work/智能车/.Download/Keil.STM32F4xx_DFP.2.16.0.pack'],
             'hide_programming_progress': True, 'no_config': True})
assert session is not None, 'No matching probe'
try:
    with session:
        target = session.target
        chip_id = target.read32(0xE0042000)
        flash_kib = target.read16(0x1FFF7A22)
        report.update(chip_id=hex(chip_id), flash_kib=flash_kib)
        assert chip_id & 0xFFF == 0x413 and flash_kib == 512, report
        print('Target confirmed:', report['chip_id'], flash_kib, 'KiB', flush=True)
        FileProgrammer(session, chip_erase='sector', keep_unwritten=True).program(str(image))
        target.reset_and_halt()
        checked = 0
        digest = hashlib.sha256()
        for start, end in segments:
            for address in range(start, end, 1024):
                size = min(1024, end - address)
                actual = bytes(target.read_memory_block8(address, size))
                expected = firmware.gets(address, size)
                assert actual == expected, f'Readback mismatch at {address:#x}'
                digest.update(actual)
                checked += size
        report.update(readback_verified=True, verified_bytes=checked,
                      readback_data_sha256=digest.hexdigest())
        print('PASS: full readback:', checked, 'bytes', flush=True)
        target.reset()
        time.sleep(9)
        symbols = (HERE / 'competition.map').read_text(encoding='utf-8', errors='replace')
        for name in ('ax_motion_fault', 'ax_robot_move_enable', 'ax_control_mode'):
            address = int(re.search(r'^\s*' + name + r'\s+(0x[0-9a-fA-F]+)\s+Data', symbols, re.M)[1], 16)
            report[name] = target.read8(address)
        report['bluetooth_uart'] = 'USART2 (PA2/PA3)'
        report['uart2_cr1'] = hex(target.read32(0x4000440C))
        report['uart2_brr'] = hex(target.read32(0x40004408))
        report['uart2_clock_enabled'] = bool(target.read32(0x40023840) & (1 << 17))
        report['uart2_ready'] = (int(report['uart2_cr1'], 16) & 0x202C) == 0x202C
        for name in ('ax_bluetooth_rx_bytes', 'ax_bluetooth_rx_frames'):
            address = int(re.search(r'^\s*' + name + r'\s+(0x[0-9a-fA-F]+)\s+Data', symbols, re.M)[1], 16)
            report[name] = target.read32(address)
        report['core_state'] = target.get_state().name
        print('Runtime:', json.dumps(report, ensure_ascii=False), flush=True)
finally:
    (HERE / 'bluetooth-flash-verification.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
