"""Build and run the real C implementation with MSVC; never access hardware."""
from pathlib import Path
import argparse
import os
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--armcc', type=Path, help='Optional AC5 armcc.exe, compile only')
    args = parser.parse_args()
    out = ROOT / '.build'
    out.mkdir(exist_ok=True)
    vswhere = Path(os.environ.get('ProgramFiles(x86)', 'C:/Program Files (x86)')) / 'Microsoft Visual Studio/Installer/vswhere.exe'
    install = subprocess.check_output(
        [str(vswhere), '-latest', '-products', '*', '-requires',
         'Microsoft.VisualStudio.Component.VC.Tools.x86.x64', '-property', 'installationPath'],
        text=True,
    ).strip()
    if not install:
        raise SystemExit('MSVC C tools not found')
    vcvars = Path(install) / 'VC/Auxiliary/Build/vcvars64.bat'
    batch = out / 'compile.cmd'
    batch.write_text(
        '@echo off\ncall "' + str(vcvars) + '" >nul\n'
        'if errorlevel 1 exit /b 1\n'
        'cl /nologo /W4 /WX /std:c11 /Od /RTC1 /Zi /fp:strict '
        '/I".." "../turn_control.c" "../tests/test_turn_control.c" '
        '/Fe:"test_turn_control.exe"\nexit /b %errorlevel%\n',
        encoding='mbcs',
    )
    subprocess.run(['cmd.exe', '/d', '/c', str(batch)], cwd=out, check=True)
    result = subprocess.run([str(out / 'test_turn_control.exe')], cwd=out,
                            text=True, capture_output=True, check=True)
    print(result.stdout, end='')
    (out / 'host-results.txt').write_text(result.stdout, encoding='utf-8')
    if args.armcc:
        subprocess.run([str(args.armcc.resolve()), '--c90', '--cpu=Cortex-M4',
                        '--fpu=FPv4-SP', '--strict', '-c', str(ROOT / 'turn_control.c'),
                        '-o', str(out / 'turn_control_arm.o')], cwd=out, check=True)
        print('PASS: AC5 Cortex-M4 C90 compile only (no linking/download)')


if __name__ == '__main__':
    main()
