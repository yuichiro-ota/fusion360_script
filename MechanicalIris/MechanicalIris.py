# -*- coding: utf-8 -*-
# ============================================================================
#  MechanicalIris.py  -  Fusion 360 Script
#
#  パラメトリック メカニカルアイリス（虹彩絞り）ジェネレータ
#
#  ・開口直径（全開時）を入力すると、機構全体がその寸法に合わせてスケールします
#  ・サーボモータを選択すると、ブラケット（切欠き・取付穴）とクランクアーム長が
#    そのサーボに合わせて自動計算されます
#
#  生成される部品（それぞれ別コンポーネント）
#     Base       : ベースプレート（ピボットピン + サーボブラケット一体）
#     Blade x N  : 羽根（ドライブピン一体）
#     DriveRing  : ドライブリング（放射状スロット）
#     Cover      : カバープレート
#     Crank      : サーボクランクアーム（ドライブピン一体）
#
#  ---------------------------------------------------------------------------
#  設計の考え方
#  ---------------------------------------------------------------------------
#  ・羽根の内側刃は半径 Rb の円弧。その中心 C はピボット P から距離 d 離れている。
#    Rb = a（最大開口半径）, d = Rp（ピボット円半径）とすると
#        C = Rp*(u(α) + u(α+θ))  →  |OC| = 2*Rp*cos(θ/2),  ∠C = α + θ/2
#    となり、θ=180° で C が中心に一致 → 開口はきれいな真円 半径 a。
#    開口半径 = Rb - |OC|、|OC| >= Rb で全閉。
#  ・羽根のドライブピン Q はピボットから距離 e（刃の反対側）。
#    リングを Φ(θ) = ∠Q(θ) だけ回すと、リング座標系でのピン軌跡が
#    完全な放射方向の直線になる → スロットは全てまっすぐな放射状スロット。
#  ・羽根の外周半径 Lb は「どの絞り位置でも羽根同士の隙間から光が漏れない」
#    条件を解いて自動決定（羽根枚数が多いほど小さく＝機構がコンパクトになる）。
#  ============================================================================

import adsk.core
import adsk.fusion
import traceback
import math

CMD_ID = 'MechanicalIrisGeneratorCmd'

_app = None
_ui = None
_handlers = []
_component_mode = True

MM = 0.1          # 1 mm = 0.1 cm （Fusion の内部単位は cm）

# ---------------------------------------------------------------------------
#  調整用の定数（必要ならここを書き換えてください）
# ---------------------------------------------------------------------------
PIN_POS_RATIO   = 0.72   # ドライブピン位置 e = 羽根外周半径 Lb * これ
BLADE_MARGIN    = 1.5    # 羽根外周半径に足す余裕 [mm]
RIM_WIDTH_RATIO = 0.11   # ベース外周リブ幅 = 開口半径 * これ（最小 6mm）
WALL_MIN        = 1.5    # 最小肉厚 [mm]

# ---------------------------------------------------------------------------
#  サーボモータ諸元
#    body_l    : 本体長さ（長辺）[mm]        body_w : 本体幅（短辺）[mm]
#    hole_pitch: 取付穴ピッチ（長辺方向）    hole_d : 取付穴径
#    holes_per_side / row_pitch : 片側の穴数と、2穴のときの幅方向ピッチ
#    tab_span  : 取付タブ込みの全長
#    shaft_off : 本体中心から出力軸中心までのオフセット
#    horn_seat : 取付タブ上面から、サーボホーン取付面までの高さ
#    spline_d  : セレーション（スプライン）外径
#  ※ 代表値です。手元の個体に合わせて書き換えるか「カスタム」を使ってください。
# ---------------------------------------------------------------------------
SERVO_TABLE = [
    ('SG90 (9g マイクロ)', dict(
        body_l=22.8, body_w=12.2, hole_pitch=27.8, hole_d=2.3, holes_per_side=1,
        row_pitch=0.0, tab_span=32.2, shaft_off=5.5, horn_seat=6.5, spline_d=4.9)),
    ('MG90S (9g 金属ギア)', dict(
        body_l=22.8, body_w=12.2, hole_pitch=27.8, hole_d=2.3, holes_per_side=1,
        row_pitch=0.0, tab_span=32.2, shaft_off=5.5, horn_seat=6.5, spline_d=4.9)),
    ('MG996R (標準サイズ)', dict(
        body_l=40.7, body_w=19.7, hole_pitch=49.5, hole_d=4.3, holes_per_side=2,
        row_pitch=10.0, tab_span=54.0, shaft_off=10.3, horn_seat=8.0, spline_d=5.9)),
    ('DS3218 (20kg 標準)', dict(
        body_l=40.0, body_w=20.0, hole_pitch=49.5, hole_d=4.3, holes_per_side=2,
        row_pitch=10.0, tab_span=54.0, shaft_off=10.0, horn_seat=8.5, spline_d=5.9)),
    ('Futaba S3003 (標準)', dict(
        body_l=40.4, body_w=19.8, hole_pitch=49.0, hole_d=4.0, holes_per_side=2,
        row_pitch=10.0, tab_span=54.4, shaft_off=10.0, horn_seat=7.5, spline_d=5.9)),
    ('カスタム (下欄で指定)', None),
]


# ===========================================================================
#  低レベルユーティリティ
# ===========================================================================
def pt(x_mm, y_mm, z_mm=0.0):
    return adsk.core.Point3D.create(x_mm * MM, y_mm * MM, z_mm * MM)


def val(x_mm):
    return adsk.core.ValueInput.createByReal(x_mm * MM)


def u(deg):
    r = math.radians(deg)
    return (math.cos(r), math.sin(r))


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


class Builder(object):
    """1 コンポーネント分の作図ヘルパ（スケッチ平面をキャッシュ）"""

    def __init__(self, comp):
        self.comp = comp
        self._planes = {}
        # パーツ ドキュメントでは全パーツをルートに作るため、この Builder が
        # 作成したボディだけを JOIN/CUT の対象にする。
        self._body_start = comp.bRepBodies.count

    def sketch(self, z_mm):
        key = round(z_mm, 6)
        if abs(z_mm) < 1e-9:
            plane = self.comp.xYConstructionPlane
        elif key in self._planes:
            plane = self._planes[key]
        else:
            pin = self.comp.constructionPlanes.createInput()
            pin.setByOffset(self.comp.xYConstructionPlane, val(z_mm))
            plane = self.comp.constructionPlanes.add(pin)
            plane.isLightBulbOn = False
            self._planes[key] = plane
        return self.comp.sketches.add(plane)

    def _extrude(self, sk, profiles, h_mm, op):
        if isinstance(profiles, list):
            col = adsk.core.ObjectCollection.create()
            for p in profiles:
                col.add(p)
            target = col
        else:
            target = profiles
        extrudes = self.comp.features.extrudeFeatures
        inp = extrudes.createInput(target, op)
        inp.setDistanceExtent(False, val(h_mm))
        if op != NEW:
            # 同じルート内にある別パーツを誤って結合・切削しないようにする。
            bodies = self.comp.bRepBodies
            participants = [
                bodies.item(i) for i in range(self._body_start, bodies.count)
            ]
            if participants:
                try:
                    inp.participantBodies = participants
                except:
                    pass
        ext = extrudes.add(inp)
        sk.isVisible = False
        return ext

    def circle(self, cx, cy, r, z0, h, op):
        sk = self.sketch(z0)
        sk.sketchCurves.sketchCircles.addByCenterRadius(pt(cx, cy), r * MM)
        return self._extrude(sk, sk.profiles.item(0), h, op)

    def circles(self, centers, r, z0, h, op):
        sk = self.sketch(z0)
        for (cx, cy) in centers:
            sk.sketchCurves.sketchCircles.addByCenterRadius(pt(cx, cy), r * MM)
        profs = [sk.profiles.item(i) for i in range(sk.profiles.count)]
        return self._extrude(sk, profs, h, op)

    def rect(self, x1, y1, x2, y2, z0, h, op):
        sk = self.sketch(z0)
        sk.sketchCurves.sketchLines.addTwoPointRectangle(pt(x1, y1), pt(x2, y2))
        return self._extrude(sk, sk.profiles.item(0), h, op)

    def polygons(self, loops, z0, h, op):
        sk = self.sketch(z0)
        lines = sk.sketchCurves.sketchLines
        for poly in loops:
            first = None
            prev = None
            for i in range(1, len(poly)):
                p = pt(poly[i][0], poly[i][1])
                if prev is None:
                    ln = lines.addByTwoPoints(pt(poly[0][0], poly[0][1]), p)
                    first = ln
                else:
                    ln = lines.addByTwoPoints(prev.endSketchPoint, p)
                prev = ln
            lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
        profs = [sk.profiles.item(i) for i in range(sk.profiles.count)]
        return self._extrude(sk, profs, h, op)


def slot_outline(center_pts, width, cap_seg=10):
    """中心線の点列からスロット（長穴）の閉ポリゴンを作る"""
    n = len(center_pts)
    hw = width / 2.0
    normals = []
    for i in range(n):
        if i == 0:
            dx = center_pts[1][0] - center_pts[0][0]
            dy = center_pts[1][1] - center_pts[0][1]
        elif i == n - 1:
            dx = center_pts[-1][0] - center_pts[-2][0]
            dy = center_pts[-1][1] - center_pts[-2][1]
        else:
            dx = center_pts[i + 1][0] - center_pts[i - 1][0]
            dy = center_pts[i + 1][1] - center_pts[i - 1][1]
        L = math.hypot(dx, dy) or 1.0
        normals.append((-dy / L, dx / L))

    left = [(center_pts[i][0] + normals[i][0] * hw,
             center_pts[i][1] + normals[i][1] * hw) for i in range(n)]
    right = [(center_pts[i][0] - normals[i][0] * hw,
              center_pts[i][1] - normals[i][1] * hw) for i in range(n)]

    def cap(center, p_from, p_to, forward):
        a0 = math.atan2(p_from[1] - center[1], p_from[0] - center[0])
        a1 = math.atan2(p_to[1] - center[1], p_to[0] - center[0])
        for sign in (1.0, -1.0):
            d = a1 - a0
            while d * sign <= 0:
                d += sign * 2 * math.pi
            pts = []
            for k in range(1, cap_seg):
                ang = a0 + d * k / float(cap_seg)
                pts.append((center[0] + hw * math.cos(ang),
                            center[1] + hw * math.sin(ang)))
            mid = pts[len(pts) // 2]
            if ((mid[0] - center[0]) * forward[0] +
                    (mid[1] - center[1]) * forward[1]) >= 0:
                return pts
        return pts

    f_end = (center_pts[-1][0] - center_pts[-2][0],
             center_pts[-1][1] - center_pts[-2][1])
    f_st = (center_pts[0][0] - center_pts[1][0],
            center_pts[0][1] - center_pts[1][1])

    poly = list(left)
    poly.extend(cap(center_pts[-1], left[-1], right[-1], f_end))
    poly.extend(reversed(right))
    poly.extend(cap(center_pts[0], right[0], left[0], f_st))
    return poly


# ===========================================================================
#  羽根の必要リーチ（外周半径 Lb）を解く
# ===========================================================================
def required_blade_reach(Rp, Rb, r_outer, N, th_close):
    """
    どの絞り位置でも「開口（= 全羽根の刃円弧の共通内部）の外側」が
    必ずどれかの羽根で覆われるために必要な、羽根外周半径 Lb を求める。

    点 X(r, ψ)、羽根 i の角度を α_i、β = ψ - α_i、c = θ/2、
    L = |OC| = 2 Rp cos c とすると
      ・羽根 i の刃より外側（= 覆う資格がある）
            cos(β - c) <= κ,  κ = (r² + L² - Rb²) / (2 r L)
        すなわち |β - c| >= B = acos(κ)。
        許容される β は中心 μ = c + 180、半角 h = 180 - B の区間になる。
      ・その羽根が X に届く条件
            r² + Rp² - 2 r Rp cos β <= Lb²      （|β| について単調増加）
    実際の羽根は β が s = 360/N 刻みの離散値しか取らないので、区間内に
    入る格子点のうち |β| が最小のものが「その点を覆う羽根」になる。
    位相を最悪にしたときのその値は
            β_max = min(|μ| + h,  |μ| - h + s)
    となる（前者は格子点が区間の外端にしか無い場合、後者は区間が広く
    格子間隔で刻まれる場合）。h は r と 1 対 1 に対応し
            r = sqrt(Rb² - L² sin²h) - L cos h
    なので、(θ, h) の 2 変数を走査すれば必要リーチが厳密に求まる。
    """
    s = 360.0 / N
    need = 0.0
    for it in range(241):
        th = th_close + (180.0 - th_close) * it / 240.0
        c = th / 2.0
        L = 2.0 * Rp * math.cos(math.radians(c))
        mu = 180.0 - c                     # |μ|（μ = c - 180 なのでその絶対値）
        for ih in range(361):
            h = 180.0 * ih / 360.0
            sh = math.sin(math.radians(h))
            ch = math.cos(math.radians(h))
            disc = Rb * Rb - (L * sh) ** 2
            if disc < 0.0:
                continue
            r = math.sqrt(disc) - L * ch
            if r <= 0.05 or r > r_outer:
                continue
            b = min(mu + h, mu - h + s)
            if b <= 0.0:
                continue
            if b > 180.0:
                b = 180.0
            d = math.sqrt(r * r + Rp * Rp -
                          2.0 * r * Rp * math.cos(math.radians(b)))
            if d > need:
                need = d
    return need


# ===========================================================================
#  ジオメトリ計算
# ===========================================================================
def compute(p):
    g = {}
    warn = []

    a = p['ap_d'] / 2.0
    cl = p['clearance']
    r_pin = p['pin_d'] / 2.0
    wall = max(WALL_MIN, 0.04 * a)
    N = p['n_blade']

    Rp = a + r_pin + cl + wall           # ピボット円半径
    Rb = a                               # 羽根内側刃の円弧半径
    d = Rp                               # 刃円弧中心のピボットからのオフセット

    close_target = min(1.0, (Rb + p['over_close']) / (2.0 * Rp))
    th_close = 2.0 * math.degrees(math.acos(close_target))
    th_open = 180.0
    d_theta = th_open - th_close
    if d_theta < 5.0:
        raise ValueError('羽根の回転角が小さすぎます。開口直径を大きくするか、'
                         'ピン径・クリアランスを小さくしてください。')

    ring_in = a + 0.5
    Lb = required_blade_reach(Rp, Rb, ring_in, N, th_close) + BLADE_MARGIN
    if Lb <= Rp:
        Lb = Rp + BLADE_MARGIN
    e = Lb * PIN_POS_RATIO               # ドライブピン位置

    def Q(theta_deg, alpha_deg=0.0):
        al = math.radians(alpha_deg)
        th = math.radians(alpha_deg + theta_deg)
        return (Rp * math.cos(al) - e * math.cos(th),
                Rp * math.sin(al) - e * math.sin(th))

    def phi(theta_deg):
        q = Q(theta_deg)
        return math.degrees(math.atan2(q[1], q[0]))

    d_phi = abs(phi(th_close))
    r_pin_min = math.hypot(*Q(th_close))
    r_pin_max = math.hypot(*Q(th_open))

    # --- 板厚・高さ方向 --------------------------------------------------
    t_base = p['t_base']
    t_blade = p['t_blade']
    t_ring = p['t_ring']
    t_cover = p['t_cover']
    t_crank = p['t_crank']
    chamber = t_blade * p['blade_room']       # 羽根室の深さ
    rim_h = chamber + t_ring + cl

    ring_out = Rp + Lb
    rim_w = max(6.0, RIM_WIDTH_RATIO * a)
    R_plate = ring_out + cl + rim_w

    z_blade = t_base
    z_ring = t_base + chamber
    z_cover = t_base + rim_h
    z_cover_top = z_cover + t_cover

    # --- サーボ -----------------------------------------------------------
    sv = p['servo']
    R_s = R_plate + sv['body_w'] / 2.0 + 4.0
    z_crank = z_cover_top + cl
    bracket_top = z_crank - sv['horn_seat']
    if bracket_top < t_base:
        bracket_top = t_base
        z_crank = bracket_top + sv['horn_seat']

    def crank_len(sweep_deg):
        dlt = math.radians(sweep_deg / 2.0)
        t = math.tan(math.radians(d_phi / 2.0))
        return R_s * t / (math.sin(dlt) + t * math.cos(dlt))

    def pin_radius(omega_deg, Lc):
        w = math.radians(omega_deg)
        disc = (R_s * math.cos(w)) ** 2 - R_s ** 2 + Lc ** 2
        if disc < 0:
            return None
        return R_s * math.cos(w) - math.sqrt(disc)

    # ドライブスロットの角度基準（サーボ諸元には依存しない）
    phi_mid = phi(th_open - d_theta * 0.5)
    lam_drive = -phi_mid                       # ドライブスロットのリング座標角
    alpha0 = lam_drive + 180.0 / N             # 羽根スロットと最大に離す
    om_a = lam_drive + phi(th_open)
    om_b = lam_drive + phi(th_close)
    om_max = max(abs(om_a), abs(om_b))
    om_min = 0.0 if om_a * om_b <= 0.0 else min(abs(om_a), abs(om_b))

    def band(sweep_deg):
        Lc = crank_len(sweep_deg)
        if Lc <= 2.0:
            return None
        r0 = pin_radius(om_min, Lc)
        r1 = pin_radius(om_max, Lc)
        if r0 is None or r1 is None:
            return None
        return Lc, min(r0, r1), max(r0, r1)

    w_drive = p['pin_d'] + 2.0 * cl
    lo_limit = ring_in + w_drive / 2.0 + 2.0
    hi_limit = ring_out - w_drive / 2.0 - 2.5

    sweep = p['servo_sweep']
    res = band(sweep)
    if not (res and res[1] >= lo_limit and res[2] <= hi_limit):
        best = None
        for si in range(20, 681):
            cand = si / 4.0                     # 5° 〜 170° を 0.25° 刻み
            r = band(cand)
            if r and r[1] >= lo_limit and r[2] <= hi_limit:
                if best is None or abs(cand - sweep) < abs(best - sweep):
                    best = cand
        if best is None:
            raise ValueError('このサイズではサーボクランクがドライブリング上に'
                             '収まりません。開口直径を大きくするか、小型の'
                             'サーボを選んでください。')
        warn.append('サーボ動作角を {:.0f}° → {:.0f}° に自動調整しました'
                    '（クランクピンをリング内に収めるため）。'.format(sweep, best))
        sweep = best
        res = band(sweep)
    L_c, r_dr_min, r_dr_max = res

    # --- モデル化する状態 --------------------------------------------------
    t = p['state']
    theta_t = th_open - d_theta * t
    phi_t = phi(theta_t)
    omega_t = lam_drive + phi_t
    r_k = pin_radius(omega_t, L_c)
    Kx = r_k * math.cos(math.radians(omega_t))
    Ky = r_k * math.sin(math.radians(omega_t))
    gamma_t = math.degrees(math.atan2(Ky, Kx - R_s))

    # クランク角は ±180° をまたぐことがあるので、連続になるよう展開して
    # 実際に通る側の範囲を求める（カバーの逃げスロットに使用）
    g_ends = []
    prev = None
    for k in range(21):
        om = lam_drive + phi(th_open - d_theta * k / 20.0)
        rr = pin_radius(om, L_c)
        gm = math.degrees(math.atan2(
            rr * math.sin(math.radians(om)),
            rr * math.cos(math.radians(om)) - R_s))
        if prev is not None:
            while gm - prev > 180.0:
                gm -= 360.0
            while gm - prev < -180.0:
                gm += 360.0
        prev = gm
        g_ends.append(gm)

    g.update(dict(
        a=a, N=N, cl=cl, r_pin=r_pin, pin_d=p['pin_d'],
        Rp=Rp, Rb=Rb, d=d, Lb=Lb, e=e,
        th_open=th_open, th_close=th_close, d_theta=d_theta, d_phi=d_phi,
        r_pin_min=r_pin_min, r_pin_max=r_pin_max,
        t_base=t_base, t_blade=t_blade, t_ring=t_ring, t_cover=t_cover,
        t_crank=t_crank, chamber=chamber, rim_h=rim_h, rim_w=rim_w,
        ring_in=ring_in, ring_out=ring_out, R_plate=R_plate,
        z_blade=z_blade, z_ring=z_ring, z_cover=z_cover, z_cover_top=z_cover_top,
        z_crank=z_crank, bracket_top=bracket_top,
        servo=sv, R_s=R_s, L_c=L_c, sweep=sweep,
        r_dr_min=r_dr_min, r_dr_max=r_dr_max, w_drive=w_drive,
        lam_drive=lam_drive, alpha0=alpha0,
        theta_t=theta_t, phi_t=phi_t, gamma_t=gamma_t, state_t=t,
        g_start=min(g_ends), g_end=max(g_ends),
    ))

    if Rp - Rb < r_pin + 1.0:
        warn.append('ピボット部の肉厚が薄めです。ピン径を細くすると改善します。')
    return g, warn


# ===========================================================================
#  パーツ作成
# ===========================================================================
def new_comp(root, name):
    global _component_mode
    if _component_mode:
        try:
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = name
            return occ.component
        except RuntimeError:
            # Fusion の「パーツ」ドキュメントはコンポーネントを追加できない。
            # 以降はルート内の独立ボディとして各パーツを生成する。
            _component_mode = False
    return root


def screw_positions(g):
    n = 6 if g['R_plate'] > 45 else 4
    r = g['R_plate'] - g['rim_w'] / 2.0
    out = []
    for i in range(n):
        c, s = u(360.0 * i / n + 90.0)
        out.append((r * c, r * s))
    return out


def build_base(root, g):
    comp = new_comp(root, 'Base')
    b = Builder(comp)
    sv = g['servo']
    top_h = max(g['bracket_top'], g['t_base'] + g['rim_h'])

    ext = b.circle(0, 0, g['R_plate'], 0, g['t_base'] + g['rim_h'], NEW)
    ext.bodies.item(0).name = 'Base'

    # サーボブラケット
    yc = -sv['shaft_off']
    y_half = sv['tab_span'] / 2.0 + 3.0
    x_in = math.sqrt(max(g['R_plate'] ** 2 -
                         min(y_half, g['R_plate']) ** 2, 0.0)) - 6.0
    x_in = max(x_in, 0.25 * g['R_plate'])      # 中心を跨がないように
    x_out = g['R_s'] + sv['body_w'] / 2.0 + 4.0
    b.rect(x_in, yc - y_half, x_out, yc + y_half, 0, g['bracket_top'], JOIN)

    # 羽根室（ブラケットのはみ出しもここで削られる）
    b.circle(0, 0, g['ring_out'] + g['cl'], g['t_base'],
             top_h - g['t_base'] + 5.0, CUT)

    # 開口穴
    b.circle(0, 0, g['ring_in'], -1.0, top_h + 2.0, CUT)

    # サーボ本体の逃げ
    b.rect(g['R_s'] - sv['body_w'] / 2.0 - 0.3, yc - sv['body_l'] / 2.0 - 0.3,
           g['R_s'] + sv['body_w'] / 2.0 + 0.3, yc + sv['body_l'] / 2.0 + 0.3,
           -1.0, top_h + 2.0, CUT)

    # サーボ取付穴
    holes = []
    for sgn in (-1.0, 1.0):
        y = yc + sgn * sv['hole_pitch'] / 2.0
        if sv['holes_per_side'] >= 2:
            for s2 in (-1.0, 1.0):
                holes.append((g['R_s'] + s2 * sv['row_pitch'] / 2.0, y))
        else:
            holes.append((g['R_s'], y))
    b.circles(holes, sv['hole_d'] / 2.0, -1.0, top_h + 2.0, CUT)

    # カバー固定用 下穴（M3 タップ想定 φ2.5）
    b.circles(screw_positions(g), 1.25, g['t_base'], g['rim_h'] + 1.0, CUT)

    # 羽根のピボットピン
    posts = []
    for i in range(g['N']):
        c, s = u(g['alpha0'] + 360.0 * i / g['N'])
        posts.append((g['Rp'] * c, g['Rp'] * s))
    b.circles(posts, g['r_pin'], g['t_base'], g['chamber'] - 0.2, JOIN)
    return comp


def build_blades(root, g):
    comp = new_comp(root, 'Blade')
    b = Builder(comp)

    alpha = g['alpha0']
    th = g['theta_t']
    ca, sa = u(alpha)
    cd, sd = u(alpha + th)
    P = (g['Rp'] * ca, g['Rp'] * sa)
    C = (P[0] + g['d'] * cd, P[1] + g['d'] * sd)
    Q = (P[0] - g['e'] * cd, P[1] - g['e'] * sd)

    ext = b.circle(P[0], P[1], g['Lb'], g['z_blade'], g['t_blade'], NEW)
    body = ext.bodies.item(0)
    body.name = 'Blade'

    b.circle(C[0], C[1], g['Rb'], g['z_blade'] - 1.0, g['t_blade'] + 2.0, CUT)

    # ドライブピン（リングのスロットまで届かせる）
    post_h = g['chamber'] - g['t_blade'] + g['t_ring'] - 0.3
    b.circle(Q[0], Q[1], g['pin_d'] / 2.0,
             g['z_blade'] + g['t_blade'], post_h, JOIN)

    # ピボット穴
    b.circle(P[0], P[1], g['r_pin'] + g['cl'],
             g['z_blade'] - 1.0, g['t_blade'] + 2.0, CUT)

    col = adsk.core.ObjectCollection.create()
    col.add(body)
    pin = comp.features.circularPatternFeatures.createInput(
        col, comp.zConstructionAxis)
    pin.quantity = adsk.core.ValueInput.createByReal(g['N'])
    pin.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    pin.isSymmetric = False
    comp.features.circularPatternFeatures.add(pin)
    return comp


def build_ring(root, g):
    comp = new_comp(root, 'DriveRing')
    b = Builder(comp)

    ext = b.circle(0, 0, g['ring_out'], g['z_ring'], g['t_ring'], NEW)
    ext.bodies.item(0).name = 'DriveRing'
    b.circle(0, 0, g['ring_in'], g['z_ring'] - 1.0, g['t_ring'] + 2.0, CUT)

    loops = []
    w_blade = g['pin_d'] + 2.0 * g['cl']
    r1 = g['r_pin_min'] - 0.4
    r2 = g['r_pin_max'] + 0.4
    for i in range(g['N']):
        c, s = u(g['alpha0'] + 360.0 * i / g['N'] + g['phi_t'])
        loops.append(slot_outline([(r1 * c, r1 * s), (r2 * c, r2 * s)], w_blade))

    c, s = u(g['lam_drive'] + g['phi_t'])
    d1 = g['r_dr_min'] - 0.4
    d2 = g['r_dr_max'] + 0.4
    loops.append(slot_outline([(d1 * c, d1 * s), (d2 * c, d2 * s)], g['w_drive']))

    b.polygons(loops, g['z_ring'] - 1.0, g['t_ring'] + 2.0, CUT)
    return comp


def build_cover(root, g):
    comp = new_comp(root, 'Cover')
    b = Builder(comp)

    ext = b.circle(0, 0, g['R_plate'], g['z_cover'], g['t_cover'], NEW)
    ext.bodies.item(0).name = 'Cover'
    b.circle(0, 0, g['ring_in'], g['z_cover'] - 1.0, g['t_cover'] + 2.0, CUT)
    b.circles(screw_positions(g), 1.7, g['z_cover'] - 1.0, g['t_cover'] + 2.0, CUT)

    # クランクピンの逃げ（サーボ軸中心・半径 L_c の円弧スロット）
    pts = []
    ga, gb = g['g_start'] - 4.0, g['g_end'] + 4.0
    for i in range(25):
        c, s = u(ga + (gb - ga) * i / 24.0)
        pts.append((g['R_s'] + g['L_c'] * c, g['L_c'] * s))
    b.polygons([slot_outline(pts, g['pin_d'] + 2.0)],
               g['z_cover'] - 1.0, g['t_cover'] + 2.0, CUT)
    return comp


def build_crank(root, g):
    comp = new_comp(root, 'Crank')
    b = Builder(comp)
    sv = g['servo']

    c, s = u(g['gamma_t'])
    S = (g['R_s'], 0.0)
    K = (S[0] + g['L_c'] * c, S[1] + g['L_c'] * s)

    r_hub = max(sv['spline_d'] / 2.0 + 3.5, 6.0)
    r_end = g['pin_d'] / 2.0 + 2.5

    ext = b.circle(S[0], S[1], r_hub, g['z_crank'], g['t_crank'], NEW)
    ext.bodies.item(0).name = 'Crank'
    b.circle(K[0], K[1], r_end, g['z_crank'], g['t_crank'], JOIN)

    nx, ny = -s, c
    quad = [(S[0] + nx * r_end, S[1] + ny * r_end),
            (K[0] + nx * r_end, K[1] + ny * r_end),
            (K[0] - nx * r_end, K[1] - ny * r_end),
            (S[0] - nx * r_end, S[1] - ny * r_end)]
    b.polygons([quad], g['z_crank'], g['t_crank'], JOIN)

    pin_bottom = g['z_ring'] + 1.0
    b.circle(K[0], K[1], g['pin_d'] / 2.0, pin_bottom,
             g['z_crank'] + g['t_crank'] - pin_bottom, JOIN)

    b.circle(S[0], S[1], sv['spline_d'] / 2.0 + 0.1,
             g['z_crank'] - 1.0, g['t_crank'] + 2.0, CUT)
    return comp


# ===========================================================================
#  コマンド UI
# ===========================================================================
CUSTOM_KEYS = [
    ('cs_body_l', '本体長さ L', 40.0),
    ('cs_body_w', '本体幅 W', 20.0),
    ('cs_hole_pitch', '取付穴ピッチ', 49.5),
    ('cs_hole_d', '取付穴径', 4.3),
    ('cs_row_pitch', '穴の幅方向ピッチ(2穴時)', 10.0),
    ('cs_tab_span', 'タブ込み全長', 54.0),
    ('cs_shaft_off', '本体中心→軸オフセット', 10.0),
    ('cs_horn_seat', 'タブ上面→ホーン面の高さ', 8.0),
    ('cs_spline_d', 'スプライン外径', 5.9),
]


class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.isExecutedWhenPreEmpted = False
            cmd.setDialogInitialSize(420, 620)
            inputs = cmd.commandInputs

            ap = inputs.addValueInput('ap_d', '開口直径（全開時）', 'mm', val(60.0))
            ap.tooltip = ('全開にしたときの開口（真円）の直径です。'
                          'これに合わせて機構全体が拡縮されます。')
            nb = inputs.addIntegerSpinnerCommandInput('n_blade', '羽根の枚数',
                                                      4, 16, 1, 6)
            nb.tooltip = ('枚数が多いほど開口が真円に近づき、必要な羽根の'
                          '長さが短くなるため外形もコンパクトになります。'
                          '（外形/開口比 … 4枚で約3.3倍、16枚で約3.0倍）')

            st = inputs.addDropDownCommandInput(
                'state', 'モデル化する状態',
                adsk.core.DropDownStyles.TextListDropDownStyle)
            st.listItems.add('全開', False)
            st.listItems.add('中間', True)
            st.listItems.add('全閉', False)

            gs = inputs.addGroupCommandInput('g_servo', 'サーボモータ')
            si = gs.children
            dd = si.addDropDownCommandInput(
                'servo', '機種', adsk.core.DropDownStyles.TextListDropDownStyle)
            for i, (nm, _) in enumerate(SERVO_TABLE):
                dd.listItems.add(nm, i == 0)
            si.addValueInput('servo_sweep', 'サーボ動作角', 'deg',
                             adsk.core.ValueInput.createByString('45 deg'))
            cg = si.addGroupCommandInput('g_custom', 'カスタム寸法')
            cg.isVisible = False
            for key, label, default in CUSTOM_KEYS:
                cg.children.addValueInput(key, label, 'mm', val(default))
            hp = cg.children.addDropDownCommandInput(
                'cs_holes', '片側の取付穴数',
                adsk.core.DropDownStyles.TextListDropDownStyle)
            hp.listItems.add('1', False)
            hp.listItems.add('2', True)

            gt = inputs.addGroupCommandInput('g_thick', '板厚・はめあい')
            ti = gt.children
            ti.addValueInput('t_base', 'ベース板厚', 'mm', val(3.0))
            ti.addValueInput('t_blade', '羽根板厚', 'mm', val(1.2))
            ti.addValueInput('t_ring', 'ドライブリング板厚', 'mm', val(3.0))
            ti.addValueInput('t_cover', 'カバー板厚', 'mm', val(2.5))
            ti.addValueInput('t_crank', 'クランク板厚', 'mm', val(3.0))
            ti.addValueInput('pin_d', 'ピン径', 'mm', val(3.0))
            ti.addValueInput('clearance', 'クリアランス', 'mm', val(0.3))
            ti.addValueInput('over_close', '全閉時の重なり代', 'mm', val(0.6))
            ti.addFloatSpinnerCommandInput('blade_room', '羽根室の深さ（羽根板厚の倍数）',
                                           '', 1.5, 6.0, 0.5, 2.5)
            gt.isExpanded = False

            on_changed = InputChangedHandler()
            cmd.inputChanged.add(on_changed)
            _handlers.append(on_changed)
            on_exec = ExecuteHandler()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)
            on_destroy = DestroyHandler()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except:
            if _ui:
                _ui.messageBox('CommandCreated failed:\n{}'
                               .format(traceback.format_exc()))


class InputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            if args.input.id == 'servo':
                sel = args.input.selectedItem.name
                spec = None
                for nm, sp in SERVO_TABLE:
                    if nm == sel:
                        spec = sp
                        break
                grp = args.inputs.command.commandInputs.itemById('g_custom')
                grp.isVisible = (spec is None)
        except:
            if _ui:
                _ui.messageBox('InputChanged failed:\n{}'
                               .format(traceback.format_exc()))


class DestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        adsk.terminate()


class ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        global _component_mode
        try:
            inputs = args.command.commandInputs

            def v(idd):
                return inputs.itemById(idd).value / MM

            servo_name = inputs.itemById('servo').selectedItem.name
            spec = None
            for nm, sp in SERVO_TABLE:
                if nm == servo_name:
                    spec = sp
                    break
            if spec is None:
                spec = dict(
                    body_l=v('cs_body_l'), body_w=v('cs_body_w'),
                    hole_pitch=v('cs_hole_pitch'), hole_d=v('cs_hole_d'),
                    holes_per_side=int(inputs.itemById('cs_holes').selectedItem.name),
                    row_pitch=v('cs_row_pitch'), tab_span=v('cs_tab_span'),
                    shaft_off=v('cs_shaft_off'), horn_seat=v('cs_horn_seat'),
                    spline_d=v('cs_spline_d'))
                servo_name = 'カスタム'
            else:
                spec = dict(spec)

            state_map = {'全開': 0.0, '中間': 0.5, '全閉': 1.0}
            p = dict(
                ap_d=v('ap_d'),
                n_blade=inputs.itemById('n_blade').value,
                state=state_map[inputs.itemById('state').selectedItem.name],
                servo=spec,
                servo_sweep=math.degrees(inputs.itemById('servo_sweep').value),
                t_base=v('t_base'), t_blade=v('t_blade'), t_ring=v('t_ring'),
                t_cover=v('t_cover'), t_crank=v('t_crank'),
                pin_d=v('pin_d'), clearance=v('clearance'),
                over_close=v('over_close'),
                blade_room=inputs.itemById('blade_room').value,
            )
            if p['ap_d'] < 8.0:
                raise ValueError('開口直径は 8mm 以上にしてください。')

            g, warn = compute(p)

            app = adsk.core.Application.get()
            des = adsk.fusion.Design.cast(app.activeProduct)
            root = des.rootComponent
            _component_mode = True
            try:
                occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                occ.component.name = 'MechanicalIris D{:.0f}'.format(p['ap_d'])
                asm = occ.component
            except RuntimeError:
                # パーツ ドキュメントではルート コンポーネントへ直接作成する。
                _component_mode = False
                asm = root

            build_base(asm, g)
            build_blades(asm, g)
            build_ring(asm, g)
            build_cover(asm, g)
            build_crank(asm, g)

            m = []
            m.append('メカニカルアイリスを生成しました。')
            m.append('')
            m.append('  開口直径（全開）     : {:.1f} mm'.format(2 * g['a']))
            m.append('  羽根枚数             : {}'.format(g['N']))
            m.append('  外形直径             : {:.1f} mm'.format(2 * g['R_plate']))
            m.append('  全高（クランク上面） : {:.1f} mm'
                     .format(g['z_crank'] + g['t_crank']))
            m.append('  ピボット円半径 Rp    : {:.2f} mm'.format(g['Rp']))
            m.append('  羽根外周半径 Lb      : {:.2f} mm'.format(g['Lb']))
            m.append('  羽根の回転角         : {:.1f}°'.format(g['d_theta']))
            m.append('  リング回転角         : {:.1f}°'.format(g['d_phi']))
            m.append('')
            m.append('  サーボ               : {}'.format(servo_name))
            m.append('  サーボ動作角         : {:.0f}°'.format(g['sweep']))
            m.append('  サーボ軸の中心距離   : {:.1f} mm'.format(g['R_s']))
            m.append('  クランクアーム長     : {:.1f} mm'.format(g['L_c']))
            m.append('')
            m.append('※ 羽根は実機同様、同一平面上で互いに重なる配置です')
            m.append('   （CAD 上は干渉表示になります）。羽根は薄く出力し、')
            m.append('   羽根室の深さで滑らせてください。')
            if warn:
                m.append('')
                m.append('【注意】')
                for w in warn:
                    m.append('  ・' + w)
            _ui.messageBox('\n'.join(m), 'メカニカルアイリス')

        except ValueError as ex:
            _ui.messageBox(str(ex), 'メカニカルアイリス')
        except:
            if _ui:
                _ui.messageBox('生成に失敗しました:\n{}'
                               .format(traceback.format_exc()))


def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        if not adsk.fusion.Design.cast(_app.activeProduct):
            _ui.messageBox('デザイン環境（Design）で実行してください。')
            return
        cd = _ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, 'メカニカルアイリス',
            'パラメトリックなメカニカルアイリスを生成します')
        on_created = CommandCreatedHandler()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        adsk.autoTerminate(False)
    except:
        if _ui:
            _ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
