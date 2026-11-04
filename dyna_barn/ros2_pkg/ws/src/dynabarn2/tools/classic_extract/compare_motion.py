import re, sys, math, bisect
classic_file, harmonic_file = sys.argv[1:3]
OFF = 0.001  # pose commands take effect on the following physics step
C = {}; cur = None
for l in open(classic_file):
    if l.startswith('PLUGIN'): cur = l.split()[1].split('/')[-1][:-3]; C[cur] = [0, [], [], []]
    elif l.startswith('ANIM'): C[cur][0] = float(l.split()[2])
    elif l.startswith('SAMPLE'):
        f = l.split(); t = float(f[1])
        if t < C[cur][0] - 1e-6:  # the sample at exactly t=length has already wrapped to the start
            C[cur][1].append(t); C[cur][2].append(float(f[2])); C[cur][3].append(float(f[3]))
def at(n, t):
    d, ts, xs, ys = C[n]; t = math.fmod(t, d)
    i = bisect.bisect_right(ts, t)
    if i == 0: return xs[0], ys[0]
    if i >= len(ts): return xs[-1], ys[-1]
    u = (t-ts[i-1])/(ts[i]-ts[i-1])
    return xs[i-1]+u*(xs[i]-xs[i-1]), ys[i-1]+u*(ys[i]-ys[i-1])
text = open(harmonic_file).read()
errs = []; tmax = 0; nsnap = 0
for blk in re.split(r'(?m)^header \{', text)[1:]:
    s = re.search(r'stamp \{\s*(?:sec: (\d+)\s*)?(?:nsec: (\d+))?', blk)
    t = int(s.group(1) or 0) + int(s.group(2) or 0)*1e-9; tmax = max(tmax, t); nsnap += 1
    for m in re.finditer(r'name: "(\w+)".*?position \{(.*?)\}', blk, re.S):
        n = m.group(1)
        if n not in C: continue
        g = lambda k: float((re.search(k+r': (\S+)', m.group(2)) or [0, 0])[1])
        cx, cy = at(n, t+OFF)
        errs.append((math.hypot(g('x')-cx, g('y')-cy), n, t))
v = sorted(e[0] for e in errs)
pct = lambda p: v[min(len(v)-1, int(p*len(v)))]
vmax = 0; rmax = 0
for n, (d, ts, xs, ys) in C.items():
    for i in range(1, len(ts)):
        vmax = max(vmax, math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1])/(ts[i]-ts[i-1]))
    rmax = max(rmax, max(max(abs(a) for a in xs), max(abs(a) for a in ys)))
print('snapshots %d, sim time up to %.2f s, %d pose comparisons' % (nsnap, tmax, len(v)))
print('error: mean %.6f  median %.6f  p99 %.6f  p99.9 %.6f  max %.6f m' % (sum(v)/len(v), pct(.5), pct(.99), pct(.999), v[-1]))
print('classic obstacles: peak speed %.1f m/s, furthest coordinate %.2f m' % (vmax, rmax))
for e in sorted(errs, reverse=True)[:4]: print('  err %.4f %s t=%.3f' % e)
