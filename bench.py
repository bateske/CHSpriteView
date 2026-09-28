"""Build a variant, upload it, and capture the device's serial report.
usage: python bench.py <label> <seconds> [extra -D flags...] [--opt o2std]"""
import subprocess, sys, time, serial, os
label, secs = sys.argv[1], float(sys.argv[2])
args = sys.argv[3:]
opt = 'o2std'
if '--opt' in args:
    i = args.index('--opt'); opt = args[i+1]; del args[i:i+2]
flags = ' '.join(args)
sp = os.environ.get('BENCH_BUILD', 'build_bench')
bp = os.path.join(sp, label)
fq = f'CHGame:ch32v:CHGame:opt={opt},rtlib=nano'
cmd = ['arduino-cli', 'compile', '-b', fq, '--build-path', bp]
if flags:   # an override, even an empty one, changes what the core links
    cmd += ['--build-property', f'compiler.cpp.extra_flags={flags}',
            '--build-property', f'compiler.c.extra_flags={flags}']
cmd += [os.environ.get('BENCH_SKETCH', '.')]
r = subprocess.run(cmd, capture_output=True, text=True)
size = [l for l in r.stdout.splitlines() if 'Sketch uses' in l or 'Global' in l]
if r.returncode: print(r.stdout[-3000:], r.stderr[-3000:]); sys.exit(1)
print(f'== {label}  opt={opt}  flags={flags or "(none)"}'); [print('  ' + l) for l in size]
r = subprocess.run(['arduino-cli', 'upload', '-b', 'CHGame:ch32v:CHGame', '-p', 'COM8',
                    '--input-dir', bp, os.environ.get('BENCH_SKETCH', '.')], capture_output=True, text=True)
if 'application is up' not in r.stdout: print(r.stdout[-2000:], r.stderr[-2000:]); sys.exit(1)
t = time.time(); s = None
while time.time() - t < 8:
    try: s = serial.Serial('COM8', 115200, timeout=0.2); break
    except Exception: time.sleep(0.05)
s.dtr = True; buf = b''; t = time.time()
while time.time() - t < secs: buf += s.read(4096)
s.close()
print(buf.decode(errors='replace'))
