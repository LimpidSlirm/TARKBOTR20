"""Host checks for production control code; no probe or hardware access."""
from pathlib import Path
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'tests/.build/manual'
OUT.mkdir(parents=True, exist_ok=True)
STUBS = OUT / 'include'
STUBS.mkdir(exist_ok=True)

header = (ROOT / 'Robot/app/ax_robot.h').read_text(encoding='utf-8')
standard = {'stdio.h', 'stdint.h', 'stdlib.h', 'string.h', 'math.h'}
for name in re.findall(r'#include "([^"]+)"', header):
    if name in standard or name == 'ax_chassis_config.h':
        continue
    (STUBS / name).write_text('/* Host hardware interface only. */\n', encoding='utf-8')
(STUBS / 'ax_mpu6050.h').write_text(
    '#include "../../../../Driver/ax_mpu6050.h"\n', encoding='utf-8')
for name, path in [('main.h', 'Robot/app/main.h'), ('ax_uart4.h', 'Driver/ax_uart4.h'),
                   ('ax_board_config.h', 'Robot/ax_board_config.h')]:
    (STUBS / name).write_text('#include "' + (ROOT / path).as_posix() + '"\n', encoding='utf-8')


def function(source, signature):
    start = source.index(signature)
    pos = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos]


source = (ROOT / 'Robot/app/ax_robot.c').read_text(encoding='utf-8')
main = (ROOT / 'Robot/app/main.c').read_text(encoding='utf-8')
uart = (ROOT / 'Driver/ax_uart4.c').read_text(encoding='utf-8')
uart2 = (ROOT / 'Driver/ax_uart2.c').read_text(encoding='utf-8')
control = (ROOT / 'Robot/control/ax_control.c').read_text(encoding='utf-8')
platform = (ROOT / 'Robot/platform/FreeRTOSConfig.h').read_text(encoding='utf-8')
max_priorities = int(re.search(r'#define\s+configMAX_PRIORITIES\s+\(\s*(\d+)\s*\)', platform)[1])
priorities = re.findall(r'#define\s+(\w*TASK_PRIO)\s+(\d+)', main)
assert len(priorities) == 7
assert all(int(value) < max_priorities for _, value in priorities)
assert 'AX_CreateTaskChecked(' in main and 'int32_t gyro_sum[3]' in main
assert 'Key_Motion_Init();' in main and 'Key_Motion_Step();' in source
steps = '\n'.join((
    'static volatile int key_motion_request;',
    function(source, 'void AX_ROBOT_LatchFault(uint8_t code)'),
    function(main, 'static void Key_Motion_Init(void)'),
    function(main, 'void Key_Motion_Step(void)'),
    uart[uart.index('static volatile uint8_t uart4_rx_ok;'):uart.index('static uint8_t uart4_tx_buf')],
    function(uart, 'void AX_BLUETOOTH_ReceiveByte(uint8_t byte)'),
    function(uart, 'void UART4_IRQHandler(void)'),
    function(uart2, 'void USART2_IRQHandler(void)'),
    function(uart, 'uint8_t AX_UART4_GetData(uint8_t *pbuf)'),
    function(control, 'void AX_CTL_App(void)'),
    function(source, 'void Robot_Task(void* parameter)'),
    function(main, 'static int16_t AX_StartupGyroOffset(int32_t sum)'),
))
(STUBS / 'production_steps.inc').write_text(steps, encoding='utf-8')
vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
install = subprocess.check_output([str(vswhere), '-latest', '-products', '*', '-requires',
    'Microsoft.VisualStudio.Component.VC.Tools.x86.x64', '-property', 'installationPath'], text=True).strip()
assert install, 'MSVC is required'
vcvars = Path(install) / 'VC/Auxiliary/Build/vcvars64.bat'
units = ['Robot/modules/turn/turn_control.c', 'Robot/modules/turn/ax_turn.c',
         'Robot/control/ax_kinematics.c', 'Robot/control/ax_speed.c',
         'Robot/modules/ramp/ax_motion_sequence.c', 'Robot/modules/ramp/ax_ramp.c',
         'Driver/ax_mpu6050.c', 'Driver/ax_encoder_extended.c', 'tests/test_manual_control.c']
includes = [STUBS, ROOT / 'tests', ROOT / 'Robot/app', ROOT / 'Robot/control',
            ROOT / 'Robot/modules/turn', ROOT / 'Robot/modules/ramp', ROOT / 'Robot', ROOT / 'Driver']
command = 'cl /nologo /utf-8 /W4 /WX /wd4244 /wd4100 /std:c11 /Od /RTC1 /fp:strict /DAPP_KEY_TURN_TEST=1 '
command += '/FIhost_platform_manual.h '
command += ' '.join('/I"' + os.path.relpath(p, OUT) + '"' for p in includes) + ' '
command += ' '.join('"' + os.path.relpath(ROOT / u, OUT) + '"' for u in units) + ' /Fe:test_manual_control.exe\n'
batch = OUT / 'compile.cmd'
batch.write_text('@echo off\ncall "' + str(vcvars) + '" >nul\nif errorlevel 1 exit /b 1\n' +
                 command + 'exit /b %errorlevel%\n', encoding='mbcs')
subprocess.run(['cmd.exe', '/d', '/c', str(batch)], cwd=OUT, check=True)
result = subprocess.run([str(OUT / 'test_manual_control.exe')], cwd=OUT, capture_output=True, text=True)
print(result.stdout, end='')
print(result.stderr, end='')
(OUT / 'results.txt').write_text(result.stdout + result.stderr, encoding='utf-8')
result.check_returncode()
print('PASS: task priorities and A/B main.c integration')
