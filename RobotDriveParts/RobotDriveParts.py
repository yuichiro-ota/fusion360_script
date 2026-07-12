# -*- coding: utf-8 -*-
# ==========================================================================
#  RobotDriveParts.py  —  Fusion 360 スクリプト  【駆動系】
#
#    1. サイクロイド減速機 (ディスク + ピンリング + 偏心軸 + 出力ピン)
#    2. 遊星歯車機構     (太陽 + 遊星 + 内歯リング + キャリア)
#    3. はすば / ヘリンボーン歯車
#    4. ウォームギア     (ウォーム + ウォームホイール)
#    5. ラック & ピニオン
#    6. カム & フォロワ  (サイクロイド運動曲線)
#    7. 自在継手 (ユニバーサルジョイント)
#
#  ■ インストール
#    "RobotDriveParts" フォルダを作り、このファイルを入れる。
#    「ユーティリティ」→「アドイン」→「スクリプトとアドイン」→ 「+」でフォルダ指定 → 実行。
#
#  ※ Fusion の内部単位は cm / rad です。ValueCommandInput.value は cm・rad で返ります。
# ==========================================================================

import adsk.core
import adsk.fusion
import traceback
import math


# ---- パーツ ドキュメント互換レイヤ ----------------------------------------
# Fusion の「パーツ」ドキュメントは 1 コンポーネントしか持てないため
# occurrences.addNewComponent() が失敗する。その場合はルート コンポーネント内の
# 別ボディとして生成し、結合/切り取りを自パーツのボディだけに限定したうえで、
# 最後に本来の組立位置へ移動する。「アセンブリ」ドキュメントでは従来どおり
# パーツごとにコンポーネントを作る。
_parts = []
_cur = None
_multi = True


class _Part(object):
    """コンポーネント(アセンブリ) と ボディ群(パーツ) を同じ形で扱うラッパ"""

    def __init__(self, root, name, transform):
        global _multi
        object.__setattr__(self, 'transform', transform)
        object.__setattr__(self, 'label', name)
        if _multi:
            try:
                occ = root.occurrences.addNewComponent(transform)
                occ.component.name = name
                object.__setattr__(self, 'multi', True)
                object.__setattr__(self, 'comp', occ.component)
                object.__setattr__(self, 'base', 0)
                object.__setattr__(self, 'end', None)
                return
            except:
                _multi = False
        object.__setattr__(self, 'multi', False)
        object.__setattr__(self, 'comp', root)
        object.__setattr__(self, 'base', root.bRepBodies.count)
        object.__setattr__(self, 'end', None)   # 次のパーツ生成時に確定

    def __getattr__(self, k):
        return getattr(object.__getattribute__(self, 'comp'), k)

    def __setattr__(self, k, v):
        if k == 'name':
            object.__setattr__(self, 'label', v)
            if self.multi:
                self.comp.name = v
        else:
            object.__setattr__(self, k, v)

    def close(self):
        """このパーツのボディ範囲を確定する(以降に増えるボディは別パーツのもの)"""
        if not self.multi and self.end is None:
            object.__setattr__(self, 'end', self.comp.bRepBodies.count)

    def bodies(self):
        bs = self.comp.bRepBodies
        if self.multi:
            return [bs.item(i) for i in range(bs.count)]
        end = bs.count if self.end is None else self.end
        return [bs.item(i) for i in range(self.base, end)]


def _reset():
    global _parts, _cur, _multi
    _parts = []
    _cur = None
    _multi = True


def _new_part(root, name, transform=None):
    global _cur
    if _cur is not None:
        _cur.close()
    p = _Part(root, name, transform if transform else adsk.core.Matrix3D.create())
    _parts.append(p)
    _cur = p
    return p


def _scope(fi):
    """結合/切り取りの対象を、いま作っているパーツのボディだけに限定する"""
    if _cur is None or _cur.multi:
        return fi
    try:
        if fi.operation == adsk.fusion.FeatureOperations.NewBodyFeatureOperation:
            return fi
    except:
        return fi
    bodies = _cur.bodies()
    if bodies:
        try:
            fi.participantBodies = bodies
        except:
            pass
    return fi


def _place():
    """ボディに名前を付け、各パーツを組立位置へ移動する(パーツ ドキュメントのみ)"""
    if _multi:
        return
    ident = adsk.core.Matrix3D.create()
    for p in _parts:
        p.close()
        bodies = p.bodies()
        if not bodies:
            continue
        for i, b in enumerate(bodies):
            b.name = p.label if len(bodies) == 1 else '{0}_{1}'.format(p.label, i + 1)
        if p.transform.isEqualTo(ident):
            continue
        col = adsk.core.ObjectCollection.create()
        for b in bodies:
            col.add(b)
        mf = p.comp.features.moveFeatures
        try:
            mi = mf.createInput2(col)
            mi.defineAsFreeMove(p.transform)
        except:
            mi = mf.createInput(col, p.transform)
        mf.add(mi)


def _note():
    if _multi:
        return ''
    return ('\n\n※このドキュメントは「パーツ」のため 1 コンポーネントしか持てません。\n'
            '　各部品はルート内の別ボディとして生成し、組立位置に配置しました。\n'
            '　部品ごとにコンポーネントを分けたい場合は「アセンブリ」ドキュメントで実行してください。')

def _finish(msg=''):
    """組立位置への配置と完了メッセージ表示。build() の最後に呼ぶ"""
    _place()
    text = (msg + _note()).strip()
    if text and _ui:
        _ui.messageBox(text)



_app = None
_ui = None
_handlers = []

CMD_ID = 'robotDrivePartsGen'
CMD_NAME = 'ロボット駆動系ジェネレータ'
CMD_DESC = 'サイクロイド減速機 / 遊星歯車 / はすば / ウォーム / ラック / カム / 自在継手'


# ==========================================================================
#  共通ユーティリティ
# ==========================================================================
def design():
    d = adsk.fusion.Design.cast(_app.activeProduct)
    if not d:
        raise RuntimeError('デザインを開いた状態で実行してください。')
    return d


def new_comp(parent, name, tx=0.0, ty=0.0, tz=0.0):
    mat = adsk.core.Matrix3D.create()
    mat.translation = adsk.core.Vector3D.create(tx, ty, tz)
    p = parent.comp if isinstance(parent, _Part) else parent
    return _new_part(p, name, mat)


def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)


def VI(v):
    return adsk.core.ValueInput.createByReal(v)


def offset_plane(comp, base, dist):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(base, VI(dist))
    return comp.constructionPlanes.add(pin)


def extrude(comp, prof, dist, op, z0=0.0, participants=None):
    ein = comp.features.extrudeFeatures.createInput(prof, op)
    if abs(z0) > 1e-9:
        ein.startExtent = adsk.fusion.OffsetStartDefinition.create(VI(z0))
    ein.setDistanceExtent(False, VI(dist))
    if participants:
        ein.participantBodies = participants   # 明示指定が優先
    else:
        _scope(ein)
    return comp.features.extrudeFeatures.add(ein)


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def circle_sketch(comp, plane, radius, cx=0.0, cy=0.0):
    sk = comp.sketches.add(plane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(cx, cy), radius)
    return sk


def ring_prof(sk):
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        if p.profileLoops.count == 2:
            return p
    return sk.profiles.item(0)


class defer(object):
    """曲線を追加する間、スケッチの再計算(拘束解決 + プロファイル生成)を止める。

    Fusion は曲線を 1 本追加するたびに再計算するため、歯形やスプラインのように
    曲線数が多いスケッチではこれを止めるだけで生成時間が大きく縮む。
    """
    __slots__ = ('sk',)

    def __init__(self, sk):
        self.sk = sk

    def __enter__(self):
        self.sk.isComputeDeferred = True
        return self.sk

    def __exit__(self, *exc):
        self.sk.isComputeDeferred = False
        return False


def all_profs(sk):
    coll = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        coll.add(sk.profiles.item(i))
    return coll


def cut_holes(comp, body, centers, radius, z0, depth):
    """centers = [(x,y), ...] のピン穴/ボルト穴をまとめて開ける"""
    if radius <= 1e-6 or not centers:
        return
    sk = comp.sketches.add(comp.xYConstructionPlane)
    with defer(sk):
        for (cx, cy) in centers:
            sk.sketchCurves.sketchCircles.addByCenterRadius(P(cx, cy), radius)
    extrude(comp, all_profs(sk), depth + 0.2, CUT, z0 - 0.1, [body])


def bolt_circle(n, r, phase=0.0):
    return [(r * math.cos(phase + 2 * math.pi * i / n),
             r * math.sin(phase + 2 * math.pi * i / n)) for i in range(n)]


def spline(sk, pts_2d, z=0.0):
    coll = adsk.core.ObjectCollection.create()
    for (x, y) in pts_2d:
        coll.add(P(x, y, z))
    with defer(sk):
        return sk.sketchCurves.sketchFittedSplines.add(coll)


def closed_spline(sk, pts_2d):
    """始点=終点の閉じたスプライン(サイクロイド/カム輪郭用)"""
    coll = adsk.core.ObjectCollection.create()
    for (x, y) in pts_2d:
        coll.add(P(x, y))
    coll.add(P(pts_2d[0][0], pts_2d[0][1]))
    with defer(sk):
        s = sk.sketchCurves.sketchFittedSplines.add(coll)
        s.isClosed = True
    return s


def circ_pattern(comp, feats_or_bodies, count):
    coll = adsk.core.ObjectCollection.create()
    for f in feats_or_bodies:
        coll.add(f)
    pin = comp.features.circularPatternFeatures.createInput(
        coll, comp.zConstructionAxis)
    pin.quantity = VI(count)
    pin.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    return comp.features.circularPatternFeatures.add(pin)


# ==========================================================================
#  インボリュート歯形(全ギア共通)
# ==========================================================================
def inv(a):
    return math.tan(a) - a


def arc_sweep(p_start, p_end):
    """原点中心の円弧を p_start から p_end まで引くときの掃引角 [rad]。

    atan2 の差をそのまま使うと、円弧が ±180° の分岐をまたぐ位置(= 真後ろの歯)で
    符号が反転して ほぼ一周する円弧になってしまう。必ず (-π, π] に正規化する。
    """
    d = math.atan2(p_end.y, p_end.x) - math.atan2(p_start.y, p_start.x)
    while d > math.pi:
        d -= 2.0 * math.pi
    while d <= -math.pi:
        d += 2.0 * math.pi
    return d


def tooth_curves(sk, m, z, pa, add, ded, backlash, ang_off=0.0, scale=1.0):
    """
    歯1枚の閉輪郭を sk に描く(プロファイルは読まない)。
    add/ded : 歯末のたけ / 歯元のたけ [cm]
    backlash: 歯厚から差し引く量 [cm](外歯は歯を細く、内歯カッターは太く=負値)
    ang_off : 歯全体の回転オフセット [rad](はすば歯車のねじれ/歯の割り付け用)
    scale   : 半径方向の一様スケール(かさ歯車等、ここでは 1.0 固定)
    """
    rp = m * z / 2.0
    rb = rp * math.cos(pa)
    ra = rp + add
    rf = rp - ded
    if rf <= 0:
        raise RuntimeError('歯数が少なすぎます(歯元円が原点を超えます)。')

    half_t = math.pi / (2.0 * z) - backlash / (2.0 * rp)
    inv_pa = inv(pa)
    a_min = 0.0 if rf < rb else math.acos(min(rb / rf, 1.0))
    a_max = math.acos(min(rb / ra, 1.0))

    def pol(r, th):
        return P(scale * r * math.cos(th + ang_off),
                 scale * r * math.sin(th + ang_off))

    n = 12
    sa, sb = [], []
    for i in range(n + 1):
        a = a_min + (a_max - a_min) * i / n
        r = rb / math.cos(a)
        th = (inv(a) - inv_pa) - half_t
        sa.append(pol(r, th))
        sb.append(pol(r, -th))

    cv = sk.sketchCurves

    def fit(pts):
        c = adsk.core.ObjectCollection.create()
        for p in pts:
            c.add(p)
        cv.sketchFittedSplines.add(c)

    fit(sa)
    fit(sb)

    o = P(0, 0)
    cv.sketchArcs.addByCenterStartSweep(
        o, sa[-1], arc_sweep(sa[-1], sb[-1]))                   # 歯先円弧

    th_root = -(half_t + inv_pa)
    if rf < rb:
        pa_r = pol(rf, th_root)
        pb_r = pol(rf, -th_root)
        cv.sketchLines.addByTwoPoints(pa_r, sa[0])
        cv.sketchLines.addByTwoPoints(pb_r, sb[0])
    else:
        pa_r, pb_r = sa[0], sb[0]
    cv.sketchArcs.addByCenterStartSweep(
        o, pb_r, arc_sweep(pb_r, pa_r))                         # 歯元円弧


def draw_tooth(sk, m, z, pa, add, ded, backlash, ang_off=0.0, scale=1.0):
    """歯1枚を描いてプロファイルを返す(ロフト断面用)"""
    with defer(sk):
        tooth_curves(sk, m, z, pa, add, ded, backlash, ang_off, scale)
    return sk.profiles.item(0)


def draw_all_teeth(sk, m, z, pa, add, ded, backlash, phase=0.0):
    """z 枚の歯をまとめて 1 スケッチに描き、全プロファイルを返す。

    歯1枚 + 円形パターン(= 歯数ぶんのブーリアン)より、
    多プロファイルの押し出し 1 回のほうが圧倒的に速い。
    """
    with defer(sk):
        for i in range(z):
            tooth_curves(sk, m, z, pa, add, ded, backlash,
                         ang_off=phase + 2.0 * math.pi * i / z)
    return all_profs(sk)


def build_spur(comp, m, z, pa, thick, bore, backlash=0.0, add=None, ded=None,
               z0=0.0, helix=0.0, herringbone=False):
    """
    外歯車をコンポーネント comp 内に生成して本体ボディを返す。
    helix : ねじれ角 [rad](0 で平歯車)
    herringbone : True で上下逆ねじれのヘリンボーン
    """
    add = m if add is None else add
    ded = 1.25 * m if ded is None else ded
    rp = m * z / 2.0
    rf = rp - ded

    # --- 歯底円筒 ---
    sk = circle_sketch(comp, comp.xYConstructionPlane, rf)
    body = extrude(comp, sk.profiles.item(0), thick, NEW, z0).bodies.item(0)

    def add_teeth(h0, h, hx):
        """h0 から高さ h の区間に、ねじれ角 hx の歯を1枚ロフトして作る"""
        beta = h * math.tan(hx) / rp        # 区間全体のねじれ角
        n_sec = 4
        lin = comp.features.loftFeatures.createInput(JOIN)
        for i in range(n_sec + 1):
            zz = h0 + h * i / n_sec
            pl = offset_plane(comp, comp.xYConstructionPlane, zz)
            skt = comp.sketches.add(pl)
            prof = draw_tooth(skt, m, z, pa, add, ded, backlash,
                              ang_off=beta * i / n_sec)
            lin.loftSections.add(prof)
        lin.participantBodies = [body]
        return comp.features.loftFeatures.add(lin)

    if abs(helix) < 1e-9:
        # 平歯車: 全歯を 1 スケッチに描いて 1 回の押し出しで結合する
        skt = comp.sketches.add(comp.xYConstructionPlane)
        profs = draw_all_teeth(skt, m, z, pa, add, ded, backlash)
        extrude(comp, profs, thick, JOIN, z0, [body])
    elif herringbone:
        f1 = add_teeth(z0, thick / 2.0, helix)
        f2 = add_teeth(z0 + thick / 2.0, thick / 2.0, -helix)
        circ_pattern(comp, [f1, f2], z)
    else:
        f1 = add_teeth(z0, thick, helix)
        circ_pattern(comp, [f1], z)

    if bore > 1e-6:
        cut_holes(comp, body, [(0, 0)], bore / 2.0, z0, thick)
    return body


def build_ring_gear(comp, m, z, pa, thick, rim, backlash=0.0, z0=0.0):
    """内歯車(リングギア)。歯先 rp-m、歯元 rp+1.25m。"""
    rp = m * z / 2.0
    r_tip = rp - m                     # 内歯の歯先(最小内径)
    r_out = rp + 1.25 * m + rim

    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), r_out)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), r_tip)
    body = extrude(comp, ring_prof(sk), thick, NEW, z0).bodies.item(0)

    # 歯すきま = 外歯カッターの歯(歯先 rp+1.25m / 歯元 rp-m-余裕)を全周ぶん一括で切る
    skt = comp.sketches.add(comp.xYConstructionPlane)
    profs = draw_all_teeth(skt, m, z, pa, 1.25 * m, m + 0.05, -backlash)
    extrude(comp, profs, thick + 0.2, CUT, z0 - 0.1, [body])
    return body


# ==========================================================================
#  1. サイクロイド減速機
# ==========================================================================
def cycloid_profile(R, Rr, E, N, n_pts=None):
    """R:ピン円半径 Rr:ピン半径 E:偏心量 N:ピン数 → 減速比 = N-1

    山 1 つあたり 20 点あれば補間誤差は 1µm 未満。点数を増やしても精度は上がらず、
    スプラインとその後の押し出し/切り取りが重くなるだけなので上限を設ける。
    """
    if n_pts is None:
        n_pts = min(max(N * 20, 200), 400)
    pts = []
    for i in range(n_pts):
        t = 2 * math.pi * i / n_pts
        psi = math.atan2(math.sin((1 - N) * t),
                         (R / (E * N)) - math.cos((1 - N) * t))
        x = R * math.cos(t) - Rr * math.cos(t + psi) - E * math.cos(N * t)
        y = -R * math.sin(t) + Rr * math.sin(t + psi) + E * math.sin(N * t)
        pts.append((x, y))
    return pts


def build_cycloidal(pin_circle_r, n_pins, pin_r, ecc, disc_t,
                    n_out, out_pin_r, out_circle_r, bore, housing_rim):
    """減速比 = n_pins - 1(ディスク1枚、180°位相の2枚は生成後にコピー推奨)"""
    if ecc >= pin_r:
        raise RuntimeError('偏心量はピン半径より小さくしてください。')
    if out_circle_r + out_pin_r + ecc >= pin_circle_r - pin_r - 0.2:
        raise RuntimeError('出力ピン円が大きすぎます。小さくしてください。')

    root = design().rootComponent
    ratio = n_pins - 1
    parent = new_comp(root, 'CycloidalDrive 1:{}'.format(ratio))

    # ---- サイクロイドディスク(偏心位置に配置) ----
    dc = new_comp(parent, 'CycloidDisc', tx=ecc)
    sk = dc.sketches.add(dc.xYConstructionPlane)
    closed_spline(sk, cycloid_profile(pin_circle_r, pin_r, ecc, n_pins))
    disc = extrude(dc, sk.profiles.item(0), disc_t, NEW).bodies.item(0)
    # 出力ピン穴(穴径 = ピン径 + 2×偏心量)
    cut_holes(dc, disc, bolt_circle(n_out, out_circle_r),
              out_pin_r + ecc, 0.0, disc_t)
    # 中央の偏心ベアリング穴
    cut_holes(dc, disc, [(0, 0)], bore / 2.0, 0.0, disc_t)

    # ---- ピンリング(ハウジング) ----
    hc = new_comp(parent, 'PinRing_Housing')
    # 内径 = ピン円半径。溝円(中心 pin_circle_r / 半径 pin_r)が内壁をまたぐので
    # ピンの外側半分がリングに彫り込まれ、半円溝になる。
    r_in = pin_circle_r
    r_out = pin_circle_r + pin_r + housing_rim
    skh = hc.sketches.add(hc.xYConstructionPlane)
    skh.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), r_out)
    skh.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), r_in)
    hb = extrude(hc, ring_prof(skh), disc_t + 0.2, NEW, -0.1).bodies.item(0)
    # ピンを保持する半円溝(全周ぶんを 1 スケッチ → 1 回の切り取り)
    skp = hc.sketches.add(hc.xYConstructionPlane)
    with defer(skp):
        for (px, py) in bolt_circle(n_pins, pin_circle_r):
            skp.sketchCurves.sketchCircles.addByCenterRadius(P(px, py), pin_r)
    extrude(hc, all_profs(skp), disc_t + 0.4, CUT, -0.2, [hb])

    # ---- ローラーピン ----
    pc = new_comp(parent, 'Pins x{}'.format(n_pins))
    skpp = pc.sketches.add(pc.xYConstructionPlane)
    with defer(skpp):
        for (px, py) in bolt_circle(n_pins, pin_circle_r):
            skpp.sketchCurves.sketchCircles.addByCenterRadius(P(px, py), pin_r * 0.97)
    extrude(pc, all_profs(skpp), disc_t, NEW)

    # ---- 偏心入力軸 ----
    ec = new_comp(parent, 'EccentricShaft')
    sk1 = circle_sketch(ec, ec.xYConstructionPlane, bore / 2.0 - 0.01, cx=ecc)
    extrude(ec, sk1.profiles.item(0), disc_t, NEW)      # 偏心カム部
    sk2 = circle_sketch(ec, ec.xYConstructionPlane, bore / 2.0 * 0.6)
    extrude(ec, sk2.profiles.item(0), disc_t + 1.0, JOIN, -1.0)   # 同軸の軸部

    # ---- 出力キャリア(ピン付き) ----
    oc = new_comp(parent, 'OutputCarrier')
    sko = circle_sketch(oc, oc.xYConstructionPlane, r_in * 0.92)
    ob = extrude(oc, sko.profiles.item(0), 0.4, NEW, disc_t + 0.05).bodies.item(0)
    skop = oc.sketches.add(oc.xYConstructionPlane)
    for (x, y) in bolt_circle(n_out, out_circle_r):
        skop.sketchCurves.sketchCircles.addByCenterRadius(P(x, y), out_pin_r)
    profs = adsk.core.ObjectCollection.create()
    for i in range(skop.profiles.count):
        profs.add(skop.profiles.item(i))
    extrude(oc, profs, disc_t + 0.05, JOIN, 0.0, [ob])
    cut_holes(oc, ob, [(0, 0)], bore / 2.0 * 0.62, 0.0, disc_t + 0.5)

    _ui.messageBox(
        'サイクロイド減速機を生成しました。\n'
        '  減速比 : 1 : {}\n'
        '  ピン数 : {} / 偏心量 : {:.2f} mm\n'
        '  ディスク : 外径 約 {:.1f} mm\n\n'
        '※ 振動を打ち消すには CycloidDisc をコピーし、'
        '偏心を 180° 反転させた2枚重ねにしてください。'.format(
            ratio, n_pins, ecc * 10, (pin_circle_r + ecc) * 20))
    return parent


# ==========================================================================
#  2. 遊星歯車機構
# ==========================================================================
def build_planetary(m, zs, zp, n_planets, pa, thick, backlash, rim, bore):
    zr = zs + 2 * zp
    if (zs + zr) % n_planets != 0:
        raise RuntimeError(
            '遊星が等配置できません。(Zs+Zr)={} が遊星数 {} で割り切れる必要があります。\n'
            '太陽歯数か遊星歯数を調整してください。'.format(zs + zr, n_planets))
    carrier_r = m * (zs + zp) / 2.0
    # 隣接遊星の干渉チェック
    gap = 2 * carrier_r * math.sin(math.pi / n_planets)
    if gap <= m * (zp + 2) + 0.05:
        raise RuntimeError('遊星同士が干渉します。遊星数を減らしてください。')

    root = design().rootComponent
    ratio_fix_ring = 1.0 + zr / float(zs)      # リング固定・キャリア出力
    parent = new_comp(root, 'Planetary M{:.1f} Zs{} Zp{} Zr{}'.format(
        m * 10, zs, zp, zr))

    sun = new_comp(parent, 'Sun Z{}'.format(zs))
    build_spur(sun, m, zs, pa, thick, bore, backlash)

    for i in range(n_planets):
        th = 2 * math.pi * i / n_planets
        pc = new_comp(parent, 'Planet{} Z{}'.format(i + 1, zp),
                      tx=carrier_r * math.cos(th), ty=carrier_r * math.sin(th))
        build_spur(pc, m, zp, pa, thick, bore * 0.8, backlash)

    rc = new_comp(parent, 'Ring Z{}'.format(zr))
    build_ring_gear(rc, m, zr, pa, thick, rim, backlash)

    cc = new_comp(parent, 'Carrier')
    sk = circle_sketch(cc, cc.xYConstructionPlane, carrier_r + m * zp / 2.0 * 0.6)
    cb = extrude(cc, sk.profiles.item(0), 0.4, NEW, thick + 0.05).bodies.item(0)
    cut_holes(cc, cb, bolt_circle(n_planets, carrier_r), bore * 0.4, thick, 1.0)
    cut_holes(cc, cb, [(0, 0)], bore / 2.0 + 0.03, thick, 1.0)

    _ui.messageBox(
        '遊星歯車機構を生成しました。\n'
        '  太陽 Z={} / 遊星 Z={} ×{} / リング Z={}\n'
        '  減速比(リング固定・太陽入力・キャリア出力) = 1 : {:.3f}\n'
        '  減速比(キャリア固定・太陽入力・リング出力) = 1 : {:.3f} (逆転)'.format(
            zs, zp, n_planets, zr, ratio_fix_ring, zr / float(zs)))
    return parent


# ==========================================================================
#  3. はすば / ヘリンボーン歯車
# ==========================================================================
def build_helical(m, z, pa, helix, thick, bore, herringbone, backlash):
    root = design().rootComponent
    name = 'Herringbone' if herringbone else 'Helical'
    comp = new_comp(root, '{} M{:.1f} Z{} beta{:.0f}'.format(
        name, m * 10, z, math.degrees(helix)))
    build_spur(comp, m, z, pa, thick, bore, backlash,
               helix=helix, herringbone=herringbone)
    # 相手歯車は同じ諸元・ねじれ角を反転して生成してください
    return comp


# ==========================================================================
#  4. ウォームギア
# ==========================================================================
def helix_path(comp, radius, lead, length):
    """Z軸まわりのらせん(3Dスプライン)を作りパスを返す。始点は (radius, 0, 0)"""
    turns = length / lead
    n = max(int(turns * 16), 24)   # 1回転16点で補間誤差は1µm未満(点を増やすとスイープが重くなる)
    sk = comp.sketches.add(comp.xYConstructionPlane)
    coll = adsk.core.ObjectCollection.create()
    for i in range(n + 1):
        th = 2 * math.pi * turns * i / n
        coll.add(P(radius * math.cos(th), radius * math.sin(th),
                   lead * th / (2 * math.pi)))
    with defer(sk):
        curve = sk.sketchCurves.sketchFittedSplines.add(coll)
    return comp.features.createPath(curve, False)


def sweep_thread(comp, prof, path, op, participants=None):
    sin = comp.features.sweepFeatures.createInput(prof, path, op)
    sin.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
    if participants:
        sin.participantBodies = participants   # 明示指定が優先
    else:
        _scope(sin)
    return comp.features.sweepFeatures.add(sin)


def build_worm(m, starts, worm_pd, worm_len, pa, z_wheel, wheel_face, bore):
    """
    m        : 軸方向モジュール [cm]
    starts   : 条数
    worm_pd  : ウォームのピッチ円直径 [cm]
    z_wheel  : ホイール歯数 → 減速比 = z_wheel / starts
    """
    px = math.pi * m                  # 軸方向ピッチ
    lead = starts * px                # リード
    gamma = math.atan2(lead, math.pi * worm_pd)   # 進み角
    rp = worm_pd / 2.0
    add, ded = m, 1.2 * m
    r_root, r_tip = rp - ded, rp + add
    if r_root <= bore / 2.0 + 0.1:
        raise RuntimeError('ウォームのピッチ円直径が小さすぎます。')

    root = design().rootComponent
    ratio = z_wheel / float(starts)
    parent = new_comp(root, 'WormGearSet 1:{:.1f}'.format(ratio))

    # ---- ウォーム ----
    wc = new_comp(parent, 'Worm {}start'.format(starts))
    sk = circle_sketch(wc, wc.xYConstructionPlane, r_root)
    core = extrude(wc, sk.profiles.item(0), worm_len, NEW).bodies.item(0)

    # 軸断面のねじ山(台形)を XZ 平面に描く : スケッチ座標 (u,v) → 全体 (u, 0, v)
    wt = px / 4.0 - add * math.tan(pa)      # 歯先側の半幅
    wr = px / 4.0 + ded * math.tan(pa)      # 歯元側の半幅
    if wt <= 0.01:
        raise RuntimeError('圧力角が大きすぎて歯先が消えます。')
    skp = wc.sketches.add(wc.xZConstructionPlane)
    ln = skp.sketchCurves.sketchLines
    p1, p2 = P(r_root, -wr), P(r_root, wr)
    p3, p4 = P(r_tip, wt), P(r_tip, -wt)
    ln.addByTwoPoints(p1, p2)
    ln.addByTwoPoints(p2, p3)
    ln.addByTwoPoints(p3, p4)
    ln.addByTwoPoints(p4, p1)

    path = helix_path(wc, rp, lead, worm_len)
    sw = sweep_thread(wc, skp.profiles.item(0), path, JOIN, [core])
    if starts > 1:
        # 条数ぶん軸まわりに等分コピー(360/starts ずつ位相をずらす)
        circ_pattern(wc, [sw], starts)

    if bore > 1e-6:
        cut_holes(wc, core, [(0, 0)], bore / 2.0, -0.1, worm_len + 0.2)

    # ---- ウォームホイール(はすば歯車で近似) ----
    hc = new_comp(parent, 'WormWheel Z{}'.format(z_wheel),
                  tx=rp + m * z_wheel / 2.0)
    build_spur(hc, m, z_wheel, pa, wheel_face, bore, backlash=0.02,
               helix=gamma)

    _ui.messageBox(
        'ウォームギアを生成しました。\n'
        '  減速比 = 1 : {:.2f}  ({}条 / ホイール {}枚)\n'
        '  進み角 γ = {:.2f}°  (ホイールのねじれ角も同値)\n'
        '  軸間距離 = {:.2f} mm\n\n'
        '※ ホイールは「はすば歯車による近似」です(鼓形ではありません)。\n'
        '※ 進み角が約 5° 以下ならセルフロックが期待できます。'.format(
            ratio, starts, z_wheel, math.degrees(gamma),
            (rp + m * z_wheel / 2.0) * 10))
    return parent


# ==========================================================================
#  5. ラック & ピニオン
# ==========================================================================
def build_rack_pinion(m, z_pin, pa, rack_len, rack_h, thick, bore, backlash):
    root = design().rootComponent
    parent = new_comp(root, 'RackAndPinion M{:.1f}'.format(m * 10))

    rp = m * z_pin / 2.0
    pc = new_comp(parent, 'Pinion Z{}'.format(z_pin), ty=rp)
    build_spur(pc, m, z_pin, pa, thick, bore, backlash)

    # --- ラック(ピッチ線を y=0 に置く) ---
    rc = new_comp(parent, 'Rack')
    p = math.pi * m
    n_teeth = int(rack_len / p)
    add, ded = m, 1.25 * m
    wt = p / 4.0 - add * math.tan(pa)          # 歯先の半幅
    wr = p / 4.0 + ded * math.tan(pa)          # 歯元の半幅
    if wt <= 0.01:
        raise RuntimeError('モジュールに対して圧力角が大きすぎます。')

    sk = rc.sketches.add(rc.xYConstructionPlane)
    ln = sk.sketchCurves.sketchLines
    y_bot = -ded - rack_h
    y_top, y_root = add, -ded
    pts = [(0.0, y_bot), (0.0, y_root)]
    for i in range(n_teeth):
        c = (i + 0.5) * p
        pts += [(c - wr, y_root), (c - wt, y_top),
                (c + wt, y_top), (c + wr, y_root)]
    pts += [(n_teeth * p, y_root), (n_teeth * p, y_bot)]
    with defer(sk):
        for i in range(len(pts) - 1):
            ln.addByTwoPoints(P(*pts[i]), P(*pts[i + 1]))
        ln.addByTwoPoints(P(*pts[-1]), P(*pts[0]))
    extrude(rc, sk.profiles.item(0), thick, NEW)

    _ui.messageBox(
        'ラック&ピニオンを生成しました。\n'
        '  ピニオン Z={} / ピッチ円直径 {:.2f} mm\n'
        '  1回転あたりの送り量 = {:.2f} mm\n'
        '  ラック歯数 = {} (全長 {:.1f} mm)'.format(
            z_pin, 2 * rp * 10, math.pi * m * z_pin * 10, n_teeth, n_teeth * p * 10))
    return parent


# ==========================================================================
#  6. カム & フォロワ
# ==========================================================================
def circumradius(p0, p1, p2):
    """3点を通る円の (中心, 半径)。ほぼ一直線なら None"""
    (x0, y0), (x1, y1), (x2, y2) = p0, p1, p2
    d = 2.0 * (x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1))
    if abs(d) < 1e-12:
        return None
    s0 = x0 * x0 + y0 * y0
    s1 = x1 * x1 + y1 * y1
    s2 = x2 * x2 + y2 * y2
    cx = (s0 * (y1 - y2) + s1 * (y2 - y0) + s2 * (y0 - y1)) / d
    cy = (s0 * (x2 - x1) + s1 * (x0 - x2) + s2 * (x1 - x0)) / d
    return (cx, cy), math.hypot(x1 - cx, y1 - cy)


def cam_profile(pitch_r, roller_r, n=360):
    """ローラー中心の軌跡(ピッチ曲線)から、カムの実輪郭を作る。

    実輪郭 = ピッチ曲線を「法線方向」に roller_r だけ内側へオフセットしたもの。
    半径方向に単純に引くと、圧力角の分だけ形が狂う(上昇/下降の途中ほど誤差が出て、
    実際のリフトが指定値どおりにならない)。

    フォロワは +Y に置き、カムを CCW に回すと 上昇→休止→下降 の順に接触するよう、
    カム角 th の点を極角 (π/2 - th) に置く。
    """
    pitch = []
    for i in range(n):
        th = 2.0 * math.pi * i / n
        psi = math.pi / 2.0 - th
        R = pitch_r(th)
        pitch.append((R * math.cos(psi), R * math.sin(psi)))

    pts = []
    rho_min = None
    for i in range(n):
        x0, y0 = pitch[(i - 1) % n]
        x1, y1 = pitch[i]
        x2, y2 = pitch[(i + 1) % n]
        tx, ty = x2 - x0, y2 - y0          # 接線(中心差分)
        L = math.hypot(tx, ty)
        if L < 1e-12:
            raise RuntimeError('カム輪郭が退化しています。寸法を見直してください。')
        nx, ny = ty / L, -tx / L           # 法線
        if nx * x1 + ny * y1 > 0.0:        # 外向きなら反転して内向きに
            nx, ny = -nx, -ny

        cr = circumradius((x0, y0), (x1, y1), (x2, y2))
        if cr is not None:
            (cx, cy), rho = cr
            # 曲率中心が内側にある(= 外に凸)ところだけアンダーカットしうる
            if (cx - x1) * nx + (cy - y1) * ny > 0.0:
                if rho_min is None or rho < rho_min:
                    rho_min = rho

        px, py = x1 + nx * roller_r, y1 + ny * roller_r
        pts.append((px, py))

    if rho_min is not None and roller_r >= rho_min:
        raise RuntimeError(
            'ローラー半径 {:.1f}mm が、ピッチ曲線の最小曲率半径 {:.1f}mm 以上です。\n'
            'このままではカムがえぐれて(アンダーカット)、指定どおりのリフトになりません。\n'
            'ローラーを小さくするか、基礎円を大きくするか、リフトを減らしてください。'
            .format(roller_r * 10, rho_min * 10))
    return pts


def build_cam(base_r, lift, rise_deg, dwell_deg, fall_deg, thick, bore,
              roller_r, follower_len):
    """base_r : カム基礎円半径(カム輪郭の最小半径)
    ローラー中心の最小距離(プライム円) = base_r + roller_r"""
    rise = math.radians(rise_deg)
    dwell = math.radians(dwell_deg)
    fall = math.radians(fall_deg)
    if rise_deg + dwell_deg + fall_deg > 360.0:
        raise RuntimeError('上昇+休止+下降 の合計が 360° を超えています。')
    if rise <= 0 or fall <= 0:
        raise RuntimeError('上昇角と下降角は 0 より大きくしてください。')
    if bore / 2.0 >= base_r - 0.15:
        raise RuntimeError('軸穴が基礎円に対して大きすぎます(肉が残りません)。')

    prime = base_r + roller_r        # ローラー中心の最小距離

    def s_at(th):
        """変位 s(θ) [0..lift]。サイクロイド運動曲線(加速度が連続 = 高速でも滑らか)"""
        th = th % (2 * math.pi)
        if th < rise:
            b = th / rise
            return lift * (b - math.sin(2 * math.pi * b) / (2 * math.pi))
        th -= rise
        if th < dwell:
            return lift
        th -= dwell
        if th < fall:
            b = th / fall
            return lift * (1 - (b - math.sin(2 * math.pi * b) / (2 * math.pi)))
        return 0.0

    def ds_at(th):
        """ds/dθ(圧力角の計算用)"""
        th = th % (2 * math.pi)
        if th < rise:
            return lift / rise * (1 - math.cos(2 * math.pi * th / rise))
        th -= rise
        if th < dwell:
            return 0.0
        th -= dwell
        if th < fall:
            return -lift / fall * (1 - math.cos(2 * math.pi * th / fall))
        return 0.0

    root = design().rootComponent
    parent = new_comp(root, 'CamFollower lift{:.0f}mm'.format(lift * 10))

    n = 360      # 1°刻み。これ以上細かくしてもスプラインが重くなるだけ
    pts = cam_profile(lambda th: prime + s_at(th), roller_r, n)

    # 最大圧力角: tanα = (ds/dθ) / (プライム円 + s)
    pa_max = max(abs(math.atan2(ds_at(2 * math.pi * i / n),
                                prime + s_at(2 * math.pi * i / n)))
                 for i in range(n))

    cc = new_comp(parent, 'CamPlate')
    sk = cc.sketches.add(cc.xYConstructionPlane)
    closed_spline(sk, pts)
    cam = extrude(cc, sk.profiles.item(0), thick, NEW).bodies.item(0)
    cut_holes(cc, cam, [(0, 0)], bore / 2.0, 0.0, thick)

    # ローラーフォロワ(ローラー + アーム)。ローラーはカムに接した位置に置く
    fc = new_comp(parent, 'Follower', ty=prime)
    skr = circle_sketch(fc, fc.xYConstructionPlane, roller_r)
    extrude(fc, skr.profiles.item(0), thick, NEW)
    ska = fc.sketches.add(fc.xYConstructionPlane)
    ln = ska.sketchCurves.sketchLines
    w = roller_r * 0.6
    ln.addByTwoPoints(P(-w, 0), P(w, 0))
    ln.addByTwoPoints(P(w, 0), P(w, follower_len))
    ln.addByTwoPoints(P(w, follower_len), P(-w, follower_len))
    ln.addByTwoPoints(P(-w, follower_len), P(-w, 0))
    extrude(fc, ska.profiles.item(0), thick * 0.6, JOIN, thick)

    _ui.messageBox(
        'カム&フォロワを生成しました。\n'
        '  基礎円半径 {:.1f} mm (カム輪郭の最小半径) / リフト {:.1f} mm\n'
        '  プライム円半径 {:.1f} mm = 基礎円 + ローラー半径 {:.1f} mm\n'
        '    → ローラー中心は 原点から {:.1f} mm の位置(接触状態)に置いています。\n'
        '  線図: 上昇 {:.0f}° → 休止 {:.0f}° → 下降 {:.0f}° → 休止 {:.0f}°\n'
        '  運動曲線: サイクロイド(加速度連続)\n'
        '  最大圧力角 {:.1f}°  {}\n'
        '  カムを反時計回りに回すと 上昇→休止→下降 の順に動きます。\n'
        '  輪郭はピッチ曲線をローラー半径ぶん法線方向にオフセットした実輪郭です。'.format(
            base_r * 10, lift * 10, prime * 10, roller_r * 10, prime * 10,
            rise_deg, dwell_deg, fall_deg,
            360 - rise_deg - dwell_deg - fall_deg,
            math.degrees(pa_max),
            '(30°以下が目安。超えると横力が大きく、フォロワが渋くなります)'
            if math.degrees(pa_max) > 30.0 else '(良好)'))
    return parent


# ==========================================================================
#  7. 自在継手(ユニバーサルジョイント)
# ==========================================================================
def build_ujoint(shaft_d, yoke_gap, prong_t, prong_len, cross_d, hub_len, bore):
    """
    yoke_gap : 二又の内側間隔(クロス腕の長さに対応)
    cross_d  : クロス(十字)ピンの直径
    """
    root = design().rootComponent
    parent = new_comp(root, 'UniversalJoint d{:.0f}'.format(shaft_d * 10))
    r_hub = shaft_d / 2.0
    half = yoke_gap / 2.0
    if half + prong_t > r_hub * 2.5:
        pass

    def make_yoke(name, flip):
        c = new_comp(parent, name)
        # ハブ
        sk = circle_sketch(c, c.xYConstructionPlane, r_hub)
        body = extrude(c, sk.profiles.item(0), hub_len, NEW).bodies.item(0)
        # 二又の腕(左右)
        for sgn in (1, -1):
            ska = c.sketches.add(c.xYConstructionPlane)
            ln = ska.sketchCurves.sketchLines
            x0 = sgn * half
            x1 = sgn * (half + prong_t)
            lo, hi = min(x0, x1), max(x0, x1)
            ln.addByTwoPoints(P(lo, -r_hub), P(hi, -r_hub))
            ln.addByTwoPoints(P(hi, -r_hub), P(hi, r_hub))
            ln.addByTwoPoints(P(hi, r_hub), P(lo, r_hub))
            ln.addByTwoPoints(P(lo, r_hub), P(lo, -r_hub))
            extrude(c, ska.profiles.item(0), prong_len, JOIN,
                    hub_len, [body])
        # クロスピン穴(Y方向に貫通)
        skh = c.sketches.add(c.xZConstructionPlane)   # XZ平面 → Y方向に押し出し
        zc = hub_len + prong_len - r_hub * 0.6
        skh.sketchCurves.sketchCircles.addByCenterRadius(P(half + prong_t / 2.0, zc),
                                                         cross_d / 2.0)
        skh.sketchCurves.sketchCircles.addByCenterRadius(P(-(half + prong_t / 2.0), zc),
                                                         cross_d / 2.0)
        profs = adsk.core.ObjectCollection.create()
        for i in range(skh.profiles.count):
            profs.add(skh.profiles.item(i))
        ein = c.features.extrudeFeatures.createInput(profs, CUT)
        ein.setSymmetricExtent(VI(r_hub * 2), True)
        ein.participantBodies = [body]
        c.features.extrudeFeatures.add(ein)
        # 軸穴
        if bore > 1e-6:
            cut_holes(c, body, [(0, 0)], bore / 2.0, -0.1, hub_len + 0.2)
        return c

    make_yoke('Yoke_Input', False)
    make_yoke('Yoke_Output', True)

    # ---- クロス(十字ピン) ----
    cc = new_comp(parent, 'Cross')
    arm = half + prong_t / 2.0
    sk = cc.sketches.add(cc.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cross_d / 2.0)
    ein = cc.features.extrudeFeatures.createInput(sk.profiles.item(0), NEW)
    ein.setSymmetricExtent(VI(arm), True)
    b1 = cc.features.extrudeFeatures.add(_scope(ein)).bodies.item(0)
    sk2 = cc.sketches.add(cc.yZConstructionPlane)
    sk2.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0), cross_d / 2.0)
    ein2 = cc.features.extrudeFeatures.createInput(sk2.profiles.item(0), JOIN)
    ein2.setSymmetricExtent(VI(arm), True)
    ein2.participantBodies = [b1]
    cc.features.extrudeFeatures.add(ein2)

    _ui.messageBox(
        '自在継手を生成しました。\n'
        '  クロス腕長 {:.1f} mm / ピン径 {:.1f} mm\n\n'
        '※ 単一の自在継手は角速度が不等速になります。\n'
        '  等速にしたい場合は2個を対称に配置(ダブルカルダン)し、\n'
        '  両端ヨークの位相を揃えてください。'.format(arm * 10, cross_d * 10))
    return parent


# ==========================================================================
#  UI
# ==========================================================================
PARTS = [
    'サイクロイド減速機',
    '遊星歯車機構',
    'はすば / ヘリンボーン歯車',
    'ウォームギア',
    'ラック & ピニオン',
    'カム & フォロワ',
    '自在継手 (ユニバーサルジョイント)',
]


class CreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.setDialogInitialSize(420, 560)
            ins = cmd.commandInputs

            dd = ins.addDropDownCommandInput(
                'part', '部品', adsk.core.DropDownStyles.TextListDropDownStyle)
            for i, n in enumerate(PARTS):
                dd.listItems.add(n, i == 0)

            def S(v):
                return adsk.core.ValueInput.createByString(v)

            # --- 1. サイクロイド ---
            g = ins.addGroupCommandInput('g0', 'サイクロイド減速機')
            c = g.children
            c.addValueInput('cyPinR', 'ピン円半径', 'mm', S('30 mm'))
            c.addIntegerSpinnerCommandInput('cyNPins', 'ピン数 (減速比 = ピン数-1)', 6, 40, 1, 12)
            c.addValueInput('cyPinD', 'ピン直径', 'mm', S('6 mm'))
            c.addValueInput('cyEcc', '偏心量', 'mm', S('1.5 mm'))
            c.addValueInput('cyThk', 'ディスク厚', 'mm', S('8 mm'))
            c.addIntegerSpinnerCommandInput('cyNOut', '出力ピン数', 3, 12, 1, 6)
            c.addValueInput('cyOutD', '出力ピン直径', 'mm', S('5 mm'))
            c.addValueInput('cyOutR', '出力ピン円半径', 'mm', S('15 mm'))
            c.addValueInput('cyBore', '中央ベアリング穴径', 'mm', S('12 mm'))
            c.addValueInput('cyRim', 'ハウジング肉厚', 'mm', S('6 mm'))

            # --- 2. 遊星 ---
            g = ins.addGroupCommandInput('g1', '遊星歯車機構')
            c = g.children
            c.addValueInput('plM', 'モジュール', 'mm', S('1 mm'))
            c.addIntegerSpinnerCommandInput('plZs', '太陽歯数 Zs', 8, 60, 1, 12)
            c.addIntegerSpinnerCommandInput('plZp', '遊星歯数 Zp', 8, 60, 1, 18)
            c.addIntegerSpinnerCommandInput('plN', '遊星の数', 2, 6, 1, 3)
            c.addValueInput('plPA', '圧力角', 'deg', S('20 deg'))
            c.addValueInput('plThk', '歯幅', 'mm', S('8 mm'))
            c.addValueInput('plBL', 'バックラッシ', 'mm', S('0.15 mm'))
            c.addValueInput('plRim', 'リム肉厚', 'mm', S('5 mm'))
            c.addValueInput('plBore', '軸穴径', 'mm', S('5 mm'))
            g.isVisible = False

            # --- 3. はすば ---
            g = ins.addGroupCommandInput('g2', 'はすば / ヘリンボーン歯車')
            c = g.children
            c.addValueInput('hxM', 'モジュール', 'mm', S('1.5 mm'))
            c.addIntegerSpinnerCommandInput('hxZ', '歯数', 8, 150, 1, 24)
            c.addValueInput('hxPA', '圧力角', 'deg', S('20 deg'))
            c.addValueInput('hxBeta', 'ねじれ角', 'deg', S('20 deg'))
            c.addValueInput('hxThk', '歯幅', 'mm', S('12 mm'))
            c.addValueInput('hxBore', '軸穴径', 'mm', S('5 mm'))
            c.addValueInput('hxBL', 'バックラッシ', 'mm', S('0.1 mm'))
            c.addBoolValueInput('hxHB', 'ヘリンボーンにする', True, '', True)
            g.isVisible = False

            # --- 4. ウォーム ---
            g = ins.addGroupCommandInput('g3', 'ウォームギア')
            c = g.children
            c.addValueInput('wmM', '軸方向モジュール', 'mm', S('1.5 mm'))
            c.addIntegerSpinnerCommandInput('wmStart', '条数', 1, 4, 1, 1)
            c.addValueInput('wmPD', 'ウォームのピッチ円直径', 'mm', S('14 mm'))
            c.addValueInput('wmLen', 'ウォーム長さ', 'mm', S('30 mm'))
            c.addIntegerSpinnerCommandInput('wmZ', 'ホイール歯数', 10, 100, 1, 30)
            c.addValueInput('wmFace', 'ホイール歯幅', 'mm', S('10 mm'))
            c.addValueInput('wmPA', '圧力角', 'deg', S('20 deg'))
            c.addValueInput('wmBore', '軸穴径', 'mm', S('5 mm'))
            g.isVisible = False

            # --- 5. ラック ---
            g = ins.addGroupCommandInput('g4', 'ラック & ピニオン')
            c = g.children
            c.addValueInput('rkM', 'モジュール', 'mm', S('1.5 mm'))
            c.addIntegerSpinnerCommandInput('rkZ', 'ピニオン歯数', 8, 60, 1, 18)
            c.addValueInput('rkPA', '圧力角', 'deg', S('20 deg'))
            c.addValueInput('rkLen', 'ラック全長', 'mm', S('120 mm'))
            c.addValueInput('rkH', 'ラック背肉厚', 'mm', S('8 mm'))
            c.addValueInput('rkThk', '歯幅', 'mm', S('10 mm'))
            c.addValueInput('rkBore', 'ピニオン軸穴径', 'mm', S('5 mm'))
            c.addValueInput('rkBL', 'バックラッシ', 'mm', S('0.1 mm'))
            g.isVisible = False

            # --- 6. カム ---
            g = ins.addGroupCommandInput('g5', 'カム & フォロワ')
            c = g.children
            c.addValueInput('cmBase', '基礎円半径', 'mm', S('20 mm'))
            c.addValueInput('cmLift', 'リフト量', 'mm', S('10 mm'))
            c.addValueInput('cmRise', '上昇角', 'deg', S('120 deg'))
            c.addValueInput('cmDwell', '上死点の休止角', 'deg', S('60 deg'))
            c.addValueInput('cmFall', '下降角', 'deg', S('120 deg'))
            c.addValueInput('cmThk', 'カム板厚', 'mm', S('8 mm'))
            c.addValueInput('cmBore', '軸穴径', 'mm', S('8 mm'))
            c.addValueInput('cmRoller', 'ローラー半径', 'mm', S('5 mm'))
            c.addValueInput('cmArm', 'フォロワアーム長', 'mm', S('30 mm'))
            g.isVisible = False

            # --- 7. 自在継手 ---
            g = ins.addGroupCommandInput('g6', '自在継手')
            c = g.children
            c.addValueInput('ujD', 'ハブ外径', 'mm', S('20 mm'))
            c.addValueInput('ujGap', '二又の内側間隔', 'mm', S('12 mm'))
            c.addValueInput('ujPt', '腕の厚み', 'mm', S('4 mm'))
            c.addValueInput('ujPl', '腕の長さ', 'mm', S('16 mm'))
            c.addValueInput('ujCross', 'クロスピン直径', 'mm', S('4 mm'))
            c.addValueInput('ujHub', 'ハブ長さ', 'mm', S('15 mm'))
            c.addValueInput('ujBore', '軸穴径', 'mm', S('6 mm'))
            g.isVisible = False

            h1 = ChangedHandler()
            cmd.inputChanged.add(h1)
            _handlers.append(h1)
            h2 = ExecHandler()
            cmd.execute.add(h2)
            _handlers.append(h2)
            h3 = DestroyHandler()
            cmd.destroy.add(h3)
            _handlers.append(h3)
        except Exception:
            _ui.messageBox('UI作成に失敗:\n' + traceback.format_exc())


class ChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            if args.input.id != 'part':
                return
            idx = PARTS.index(args.input.selectedItem.name)
            for i in range(len(PARTS)):
                args.inputs.itemById('g{}'.format(i)).isVisible = (i == idx)
        except Exception:
            _ui.messageBox(traceback.format_exc())


class ExecHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            ins = args.command.commandInputs
            v = lambda k: ins.itemById(k).value
            idx = PARTS.index(ins.itemById('part').selectedItem.name)
            _reset()

            if idx == 0:
                build_cycloidal(v('cyPinR'), v('cyNPins'), v('cyPinD') / 2.0,
                                v('cyEcc'), v('cyThk'), v('cyNOut'),
                                v('cyOutD') / 2.0, v('cyOutR'), v('cyBore'),
                                v('cyRim'))
            elif idx == 1:
                build_planetary(v('plM'), v('plZs'), v('plZp'), v('plN'),
                                v('plPA'), v('plThk'), v('plBL'), v('plRim'),
                                v('plBore'))
            elif idx == 2:
                build_helical(v('hxM'), v('hxZ'), v('hxPA'), v('hxBeta'),
                              v('hxThk'), v('hxBore'), v('hxHB'), v('hxBL'))
            elif idx == 3:
                build_worm(v('wmM'), v('wmStart'), v('wmPD'), v('wmLen'),
                           v('wmPA'), v('wmZ'), v('wmFace'), v('wmBore'))
            elif idx == 4:
                build_rack_pinion(v('rkM'), v('rkZ'), v('rkPA'), v('rkLen'),
                                  v('rkH'), v('rkThk'), v('rkBore'), v('rkBL'))
            elif idx == 5:
                build_cam(v('cmBase'), v('cmLift'), math.degrees(v('cmRise')),
                          math.degrees(v('cmDwell')), math.degrees(v('cmFall')),
                          v('cmThk'), v('cmBore'), v('cmRoller'), v('cmArm'))
            else:
                build_ujoint(v('ujD'), v('ujGap'), v('ujPt'), v('ujPl'),
                             v('ujCross'), v('ujHub'), v('ujBore'))

            _finish()
            _app.activeViewport.fit()
        except Exception as e:
            _ui.messageBox('生成に失敗しました:\n{}\n\n{}'.format(
                e, traceback.format_exc()))


class DestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            adsk.terminate()
        except Exception:
            pass


def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        d = _ui.commandDefinitions.itemById(CMD_ID)
        if d:
            d.deleteMe()
        d = _ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC)
        h = CreatedHandler()
        d.commandCreated.add(h)
        _handlers.append(h)
        d.execute()
        adsk.autoTerminate(False)
    except Exception:
        if _ui:
            _ui.messageBox('起動に失敗:\n' + traceback.format_exc())
