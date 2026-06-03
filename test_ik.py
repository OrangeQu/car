import math

AL = [0.253, 0.155, 0.135, 0.081, 0.105]

def ik(f, l, u):
    y1 = math.sqrt(l*l + f*f)
    z1 = u + AL[3] + AL[4] - AL[0]
    a = AL[1]
    b = AL[2]
    c = math.sqrt(y1*y1 + z1*z1)
    if y1 < 0.001:
        y1 = 0.001
    if c > a + b or c < abs(a - b):
        return None
    be = -(math.pi/2 - math.acos((a*a + c*c - b*b)/(2*a*c)) - math.atan2(z1, y1))
    ga = -(math.pi - math.acos((a*a + b*b - c*c)/(2*a*b)))
    de = -(math.pi + (be + ga))
    be = max(0.01, min(3.14, be))
    ga = max(-3.14, min(-0.01, ga))
    de = max(-1.75, min(1.75, de))
    return (be, ga, de)

def fk(a2, a3, a4):
    L = AL
    ex = L[1] * math.sin(a2)
    ez = L[0] + L[1] * math.cos(a2)
    wx = ex + L[2] * math.sin(a2 + a3)
    wz = ez + L[2] * math.cos(a2 + a3)
    tx = wx + (L[3] + L[4]) * math.sin(a2 + a3 + a4)
    tz = wz + (L[3] + L[4]) * math.cos(a2 + a3 + a4)
    return (tx, tz)

print("IK Test Results:")
print("=" * 60)
for f, u in [(0.15, 0.05), (0.15, 0.10), (0.15, 0.15), (0.15, 0.20), 
             (0.15, 0.25), (0.15, 0.30), (0.10, 0.05), (0.10, 0.10),
             (0.10, 0.15), (0.10, 0.20), (0.08, 0.05), (0.08, 0.10), (0.05, 0.05)]:
    r = ik(f, 0.0, u)
    if r:
        tx, tz = fk(*r)
        print(f"目标(前={f:.2f},高={u:.2f}) -> a2={r[0]:.3f}, a3={r[1]:.3f}, a4={r[2]:.3f} -> 验证(前={tx:.3f},高={tz:.3f})")
    else:
        print(f"目标(前={f:.2f},高={u:.2f}) -> IK无解")
