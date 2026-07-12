# -*- coding: utf-8 -*-
# ==========================================================================
#  RobotDiffJoint.py  —  Fusion 360 スクリプト
#  【ベルト駆動・差動2自由度 ロボット関節(ディファレンシャル手首/肩)】
#
#  画像のような機構を一式生成します:
#
#      出力フランジ(クロスローラ軸受)
#            ▲ Z (ロール軸)
#            │   ┌── 出力ベベルギア
#      ┌─────┴─────┐
#      │  ヨーク    │        ← X軸まわりに傾く(ピッチ)
#   ◄──┤ ◣     ◢  ├──►  X (入力軸 / ピッチ軸)
#      │サイド サイド│
#      └───────────┘
#     大プーリ      大プーリ
#        ║            ║   ← タイミングベルト
#      モータ1      モータ2
#
#  ■ 運動学(2モータ → 2自由度)
#      ピッチ φ = (θ1 + θ2) / (2i)
#      ロール ψ = (Zs/Zo) × (θ1 - θ2) / (2i)
#        i = ベルト減速比 = 関節プーリ歯数 / モータプーリ歯数
#        Zs = サイドギア歯数, Zo = 出力ベベル歯数
#      → 両モータ同方向 = ピッチ / 逆方向 = ロール
#
#  ■ インストール
#    "RobotDiffJoint" フォルダを作りこのファイルを入れる。
#    「ユーティリティ」→「アドイン」→「スクリプトとアドイン」→「+」→ 実行。
# ==========================================================================

import adsk.core
import adsk.fusion
import traceback
import math

_app = None
_ui = None
_handlers = []

CMD_ID = 'robotDiffJointGen'
CMD_NAME = '差動2自由度関節ジェネレータ'
CMD_DESC = 'ベルト駆動ディファレンシャル関節(ベベル×3 + プーリ + ヨーク + フレーム)を一式生成'

MM = 0.1   # mm → cm(Fusion 内部単位)

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


# ==========================================================================
#  ユーティリティ
# ==========================================================================
def design():
    d = adsk.fusion.Design.cast(_app.activeProduct)
    if not d:
        raise RuntimeError('デザインを開いた状態で実行してください。')
    return d


def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)


def V(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def VI(v):
    return adsk.core.ValueInput.createByReal(v)


# ==========================================================================
#  パーツ ドキュメント互換レイヤ
#  「パーツ」ドキュメントは 1 コンポーネントしか持てないため
#  occurrences.addNewComponent() が失敗する。その場合はルート コンポーネント内の
#  別ボディとして生成し、最後に本来の組立位置(回転込み)へ移動する。
#  「アセンブリ」ドキュメントでは従来どおり部品ごとにコンポーネントを作る。
# ==========================================================================
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
            except Exception:
                _multi = False        # パーツ ドキュメント → 以降ボディで作る
        object.__setattr__(self, 'multi', False)
        object.__setattr__(self, 'comp', root)
        object.__setattr__(self, 'base', root.bRepBodies.count)
        object.__setattr__(self, 'end', None)

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


def _new_part(parent, name, transform):
    global _cur
    if _cur is not None:
        _cur.close()
    root = parent.comp if isinstance(parent, _Part) else parent
    p = _Part(root, name, transform)
    _parts.append(p)
    _cur = p
    return p


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
        except Exception:
            mi = mf.createInput(col, p.transform)
        mf.add(mi)


def _hide_helpers():
    """生成に使ったスケッチと構築面を隠す。ソリッドには影響しないが、
    出したままだと歯形の線と点がビューを埋め尽くし、表示も重くなる。

    ConstructionPlane.isVisible は読み取り専用(可視かどうかの結果)なので、
    表示の切り替えは isLightBulbOn で行う。
    """
    for p in _parts:
        c = p.comp
        for i in range(c.sketches.count):
            sk = c.sketches.item(i)
            try:
                sk.isLightBulbOn = False
            except Exception:
                pass
        for i in range(c.constructionPlanes.count):
            pl = c.constructionPlanes.item(i)
            try:
                pl.isLightBulbOn = False
            except Exception:
                pass


def _note():
    if _multi:
        return ''
    return ('\n\n※このドキュメントは「パーツ」のため 1 コンポーネントしか持てません。\n'
            '　各部品はルート内の別ボディとして生成し、組立位置に配置しました。\n'
            '　部品ごとにコンポーネントを分けたい場合は「アセンブリ」ドキュメントで'
            '実行してください。')


def comp_at(parent, name, origin, xa, ya, za):
    """ローカル座標系を (xa, ya, za) に向けた部品(コンポーネント/ボディ群)を作る"""
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(P(*origin), V(*xa), V(*ya), V(*za))
    return _new_part(parent, name, mat)


def comp_plain(parent, name):
    return comp_at(parent, name, (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1))


class defer(object):
    """曲線を追加する間、スケッチの再計算(拘束解決 + プロファイル生成)を止める。

    Fusion は曲線を 1 本追加するたびに再計算するため、歯形のように曲線数が多い
    スケッチではこれを止めるだけで生成時間が大きく縮む。
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


def extrude(comp, prof, dist, op, z0=0.0, participants=None):
    ein = comp.features.extrudeFeatures.createInput(prof, op)
    if abs(z0) > 1e-9:
        ein.startExtent = adsk.fusion.OffsetStartDefinition.create(VI(z0))
    ein.setDistanceExtent(False, VI(dist))
    if participants:
        ein.participantBodies = participants
    return comp.features.extrudeFeatures.add(ein)


def all_profiles(sk):
    c = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        c.add(sk.profiles.item(i))
    return c


def ring_prof(sk):
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        if p.profileLoops.count == 2:
            return p
    return sk.profiles.item(0)


def circle(comp, plane, r, cx=0.0, cy=0.0):
    sk = comp.sketches.add(plane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(cx, cy), r)
    return sk


def rect(sk, x0, y0, x1, y1):
    """XY 平面スケッチ用の矩形(スケッチ座標 = モデルの X, Y)"""
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P(x0, y0), P(x1, y1))


# --------------------------------------------------------------------------
#  YZ 平面スケッチ (X 方向に押し出す板・穴)
#
#  Fusion の YZ 構築平面は、スケッチ座標軸が モデルの (Y, Z) の順に並んでいない
#  (X軸 = モデルZ / Y軸 = モデルY)。P(y, z) をそのまま渡すと Y と Z が入れ替わり、
#  板やモータ穴が 90° 回った位置に出る。以下は必ず API の変換を通して描く。
# --------------------------------------------------------------------------
def yz_sketch(comp):
    return comp.sketches.add(comp.yZConstructionPlane)


def PYZ(sk, y, z):
    """モデルの (Y, Z) → そのスケッチ平面上の点"""
    return sk.modelToSketchSpace(adsk.core.Point3D.create(0.0, y, z))


def yz_nx(sk):
    """スケッチ平面の法線が モデル +X 向きなら +1、-X 向きなら -1"""
    return 1.0 if sk.transform.asArray()[2] >= 0.0 else -1.0


def rect_yz(sk, y0, z0, y1, z1):
    sk.sketchCurves.sketchLines.addTwoPointRectangle(PYZ(sk, y0, z0),
                                                     PYZ(sk, y1, z1))


def circle_yz(sk, y, z, r):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(PYZ(sk, y, z), r)


def slot_yz(sk, y, z, half_len, r):
    """Z 方向に伸びた長穴(ベルトのテンション調整方向)。

    円弧は掃引角ではなく通過点で決める。YZ 平面はスケッチ座標系の向きが
    直感と違うので、角度の符号に頼ると弧が逆向き(内側)に出る。
    """
    cv = sk.sketchCurves
    zb, zt = z - half_len, z + half_len
    cv.sketchArcs.addByThreePoints(PYZ(sk, y - r, zb), PYZ(sk, y, zb - r),
                                   PYZ(sk, y + r, zb))
    cv.sketchArcs.addByThreePoints(PYZ(sk, y + r, zt), PYZ(sk, y, zt + r),
                                   PYZ(sk, y - r, zt))
    cv.sketchLines.addByTwoPoints(PYZ(sk, y - r, zb), PYZ(sk, y - r, zt))
    cv.sketchLines.addByTwoPoints(PYZ(sk, y + r, zt), PYZ(sk, y + r, zb))


def extrude_x(comp, sk, prof, x0, length, op, participants=None):
    """YZ 平面のスケッチを X 方向へ押し出す。
    x0 = 開始位置(モデル X) / length = +X 方向の長さ(負なら -X 方向)"""
    n = yz_nx(sk)
    ein = comp.features.extrudeFeatures.createInput(prof, op)
    ein.startExtent = adsk.fusion.OffsetStartDefinition.create(VI(x0 * n))
    ein.setDistanceExtent(False, VI(length * n))
    if participants:
        ein.participantBodies = participants
    return comp.features.extrudeFeatures.add(ein)


def cut_holes(comp, body, centers, r, z0, depth, plane=None):
    if r <= 1e-6 or not centers:
        return
    sk = comp.sketches.add(plane or comp.xYConstructionPlane)
    with defer(sk):
        for (cx, cy) in centers:
            sk.sketchCurves.sketchCircles.addByCenterRadius(P(cx, cy), r)
    extrude(comp, all_profiles(sk), depth + 0.2, CUT, z0 - 0.1, [body])


def circ_pattern(comp, feats, n):
    coll = adsk.core.ObjectCollection.create()
    for f in feats:
        coll.add(f)
    pin = comp.features.circularPatternFeatures.createInput(
        coll, comp.zConstructionAxis)
    pin.quantity = VI(n)
    pin.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    return comp.features.circularPatternFeatures.add(pin)


def bolt_circle(n, r, phase=0.0):
    return [(r * math.cos(phase + 2 * math.pi * i / n),
             r * math.sin(phase + 2 * math.pi * i / n)) for i in range(n)]


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


# ==========================================================================
#  ベベルギア(すぐばかさ歯車・軸角90°)
#  ローカル座標: 軸 = +Z / 背面(大端) = z=0 / 円錐頂点 = z=za / 歯は上へ
# ==========================================================================
def bevel_geom(m, z, z_mate, face):
    delta = math.atan2(z, z_mate)          # ピッチ円錐角
    rp = m * z / 2.0
    Re = rp / math.sin(delta)              # 外側円錐距離
    if face > 0.45 * Re:
        raise RuntimeError(
            'ベベルの歯幅が大きすぎます。円錐距離 {:.1f}mm の 1/3 以下に'
            'してください。'.format(Re * 10))
    k = (Re - face) / Re
    za = rp / math.tan(delta)              # 背面から円錐頂点までの高さ
    h = za * (1.0 - k)                     # 歯車の軸方向厚み
    return dict(delta=delta, rp=rp, Re=Re, k=k, za=za, h=h)


def build_bevel(comp, m, z, z_mate, pa, face, bore, backlash=0.0,
                hub_d=0.0, hub_len=0.0):
    """Tredgold 近似(背円錐上の相当平歯車)で歯形を作り、頂点へ相似縮小ロフト"""
    G = bevel_geom(m, z, z_mate, face)
    delta, rp, k, za, h = G['delta'], G['rp'], G['k'], G['za'], G['h']

    zv = z / math.cos(delta)               # 相当歯数
    rv = m * zv / 2.0
    rb = rv * math.cos(pa)
    ra_v = rv + m
    rf_v = rv - 1.45 * m
    inv_pa = inv(pa)
    # バックラッシ分だけ歯厚を薄くする(左右で半分ずつ)
    half_t = math.pi / (2.0 * zv) - backlash / (2.0 * rv)
    if half_t <= 0:
        raise RuntimeError('バックラッシが大きすぎます。歯が消えます。')
    a_min = 0.0 if rf_v < rb else math.acos(min(rb / rf_v, 1.0))
    a_max = math.acos(min(rb / ra_v, 1.0))

    def mapped(r_e, th_e, s, off=0.0):
        dx = r_e * math.cos(th_e) - rv
        dy = r_e * math.sin(th_e)
        rho = rp + dx
        phi = dy / rp + off
        return P(s * rho * math.cos(phi), s * rho * math.sin(phi))

    th_r0 = -(half_t + inv_pa)              # 歯元点の(相当平歯車での)角度
    pitch_ang = 2.0 * math.pi / z           # 歯のピッチ角
    # 歯元点の実際の角度半幅。ピッチ角の半分を超えると隣の歯とくっついてしまう
    _pr = mapped(rf_v, -th_r0, 1.0)
    phi_root = math.atan2(_pr.y, _pr.x)
    gap_ang = pitch_ang - 2.0 * phi_root    # 歯と歯の間(歯底)の角度

    def tooth_chain(cv, s, off):
        """歯1枚のフランクと歯先を描き、歯元の2点 (歯元-, 歯元+) を返す"""
        n = 12
        sa, sb = [], []
        for i in range(n + 1):
            a = a_min + (a_max - a_min) * i / n
            r_e = rb / math.cos(a)
            th = (inv(a) - inv_pa) - half_t
            sa.append(mapped(r_e, th, s, off))
            sb.append(mapped(r_e, -th, s, off))

        for pts in (sa, sb):
            c = adsk.core.ObjectCollection.create()
            for p in pts:
                c.add(p)
            cv.sketchFittedSplines.add(c)

        o = P(0, 0)
        cv.sketchArcs.addByCenterStartSweep(       # 歯先円弧
            o, sa[-1], arc_sweep(sa[-1], sb[-1]))

        if rf_v < rb:
            par = mapped(rf_v, th_r0, s, off)
            pbr = mapped(rf_v, -th_r0, s, off)
            cv.sketchLines.addByTwoPoints(par, sa[0])
            cv.sketchLines.addByTwoPoints(pbr, sb[0])
        else:
            par, pbr = sa[0], sb[0]
        return par, pbr

    def draw_outline(sk, s):
        """歯車断面まるごと(全歯 + 歯底)を 1 本の閉ループとして描く。

        歯1枚をロフトして円形パターンすると歯数ぶんのブーリアンが走るが、
        この外形ならロフト 1 回で歯まで含めた本体ができる。
        """
        with defer(sk):
            cv = sk.sketchCurves
            o = P(0, 0)
            for i in range(z):
                off = i * pitch_ang
                _, pbr = tooth_chain(cv, s, off)
                # 歯底円弧: この歯の歯元(+側) → 次の歯の歯元(-側)
                cv.sketchArcs.addByCenterStartSweep(o, pbr, gap_ang)
        return sk.profiles.item(0)

    def draw_tooth(sk, s):
        """歯1枚だけの閉輪郭(歯元同士が重なる極端な歯数比のとき用)"""
        with defer(sk):
            cv = sk.sketchCurves
            par, pbr = tooth_chain(cv, s, 0.0)
            cv.sketchArcs.addByCenterStartSweep(   # 歯の下を通る歯元円弧
                P(0, 0), pbr, arc_sweep(pbr, par))
        return sk.profiles.item(0)

    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, VI(h))
    top = comp.constructionPlanes.add(pin)

    if gap_ang > 1e-3:
        # 断面外形を大端 → 小端(相似縮小)へロフト = 本体 + 歯が一発でできる
        sk1 = comp.sketches.add(comp.xYConstructionPlane)
        sk2 = comp.sketches.add(top)
        lin = comp.features.loftFeatures.createInput(NEW)
        lin.loftSections.add(draw_outline(sk1, 1.0))
        lin.loftSections.add(draw_outline(sk2, k))
        body = comp.features.loftFeatures.add(lin).bodies.item(0)
    else:
        # 歯数比が極端で歯元が隣とくっつく場合は従来どおり
        # 歯底円錐 + 歯1枚のロフト + 円形パターンで作る
        r_cone = rp - 1.25 * m
        sk1 = circle(comp, comp.xYConstructionPlane, r_cone)
        sk2 = circle(comp, top, r_cone * k)
        lin = comp.features.loftFeatures.createInput(NEW)
        lin.loftSections.add(sk1.profiles.item(0))
        lin.loftSections.add(sk2.profiles.item(0))
        body = comp.features.loftFeatures.add(lin).bodies.item(0)

        skt1 = comp.sketches.add(comp.xYConstructionPlane)
        skt2 = comp.sketches.add(top)
        lin = comp.features.loftFeatures.createInput(JOIN)
        lin.loftSections.add(draw_tooth(skt1, 1.0))
        lin.loftSections.add(draw_tooth(skt2, k))
        lin.participantBodies = [body]
        circ_pattern(comp, [comp.features.loftFeatures.add(lin)], z)

    # ハブ(ローカル -Z 方向 = 組立後は軸受側へ伸びる)
    if hub_d > 1e-6 and hub_len > 1e-6:
        skh = circle(comp, comp.xYConstructionPlane, hub_d / 2.0)
        extrude(comp, skh.profiles.item(0), hub_len, JOIN, -hub_len, [body])

    if bore > 1e-6:
        cut_holes(comp, body, [(0, 0)], bore / 2.0, -hub_len - 0.2,
                  h + hub_len + 0.4)
    return body, G


# ==========================================================================
#  タイミングプーリ(GT2 近似)
#  ローカル座標: 軸 = +Z / 歯部 z = 0..width
# ==========================================================================
def pulley_pd(z, pitch):
    return z * pitch / math.pi


def build_pulley(comp, z, pitch, width, bore, flange=True, hub_d=0.0, hub_len=0.0):
    pld = 0.0254           # GT2 の PLD (0.254 mm)
    r_out = pulley_pd(z, pitch) / 2.0 - pld
    if r_out <= bore / 2.0 + 0.05:
        raise RuntimeError('プーリ歯数が少なすぎるか軸穴が大きすぎます。')

    sk = circle(comp, comp.xYConstructionPlane, r_out)
    body = extrude(comp, sk.profiles.item(0), width, NEW).bodies.item(0)

    # 歯溝は全周ぶんを 1 スケッチに描いて一括で切る
    # (溝1つ + 円形パターンだと歯数ぶんのブーリアンになり、Z が大きいと極端に遅い)
    gr = pitch * 0.25
    go = pitch * 0.10
    skg = comp.sketches.add(comp.xYConstructionPlane)
    with defer(skg):
        for (gx, gy) in bolt_circle(z, r_out - go):
            skg.sketchCurves.sketchCircles.addByCenterRadius(P(gx, gy), gr)
    extrude(comp, all_profiles(skg), width, CUT, 0.0, [body])

    if flange:
        fr = r_out + pitch * 0.55
        ft = 0.1
        for z0 in (-ft, width):
            skf = circle(comp, comp.xYConstructionPlane, fr)
            extrude(comp, skf.profiles.item(0), ft, JOIN, z0, [body])

    if hub_d > 1e-6 and hub_len > 1e-6:
        skh = circle(comp, comp.xYConstructionPlane, hub_d / 2.0)
        extrude(comp, skh.profiles.item(0), hub_len, JOIN, width, [body])

    cut_holes(comp, body, [(0, 0)], bore / 2.0, -0.3, width + hub_len + 0.6)
    return body, r_out


# ==========================================================================
#  ベルト長
# ==========================================================================
GT2_STOCK = [100, 110, 122, 140, 152, 158, 160, 170, 176, 180, 188, 200, 220,
             240, 250, 260, 280, 300, 320, 340, 350, 360, 380, 400, 420, 450,
             480, 500, 530, 560, 600, 610, 640, 670, 700, 760, 800, 850, 900,
             1000, 1100, 1200]


def belt_len_mm(z1, z2, pitch_mm, C_mm):
    d1 = z1 * pitch_mm / math.pi
    d2 = z2 * pitch_mm / math.pi
    return 2 * C_mm + math.pi * (d1 + d2) / 2.0 + (d2 - d1) ** 2 / (4.0 * C_mm)


def center_from_belt(z1, z2, pitch_mm, L):
    C = L / 4.0
    d1 = z1 * pitch_mm / math.pi
    d2 = z2 * pitch_mm / math.pi
    for _ in range(80):
        f = belt_len_mm(z1, z2, pitch_mm, C) - L
        df = 2.0 - (d2 - d1) ** 2 / (4.0 * C * C)
        C -= f / df
        if C < 1.0:
            C = 1.0
    return C


# ==========================================================================
#  モータ(NEMA)プリセット [mm]
# ==========================================================================
MOTORS = {
    'NEMA17 (42mm)': dict(bx=31.0, by=31.0, bolt=3.4, boss=22.5, shaft=5.0,
                          size=42.3, length=40.0),
    'NEMA14 (35mm)': dict(bx=26.0, by=26.0, bolt=3.4, boss=22.0, shaft=5.0,
                          size=35.2, length=34.0),
    'NEMA23 (57mm)': dict(bx=47.14, by=47.14, bolt=5.4, boss=38.4, shaft=6.35,
                          size=56.4, length=51.0),
}


# ==========================================================================
#  差動関節アセンブリ 本体
# ==========================================================================
def build_diff_joint(
        # ベベル
        bm, zs, zo, pa, face, backlash,
        # プーリ / ベルト
        zj, zmp, pitch, pw, center,
        # 軸受
        in_brg_od, in_brg_id, in_brg_w,          # 入力軸(サイドギア)用
        yk_brg_od, yk_brg_id, yk_brg_w,          # ヨーク旋回用(フレーム側)
        out_brg_od, out_brg_id, out_brg_w,       # 出力(ロール)用
        # 構造
        yoke_t, yoke_w, frame_t, gap, flange_bolt_n, flange_bolt_d,
        motor_key, slot_len, make_motor=True):
    """
    bm  : ベベルのモジュール
    zs  : サイドギア歯数(左右共通) / zo : 出力ベベル歯数
    zj  : 関節側プーリ歯数 / zmp : モータ側プーリ歯数
    center : ベルト軸間距離
    """
    root = design().rootComponent
    mot = MOTORS[motor_key]
    _reset()

    # ---------- 幾何の解決 ----------
    Gs = bevel_geom(bm, zs, zo, face)     # サイドギア
    Go = bevel_geom(bm, zo, zs, face)     # 出力ベベル
    za_s, h_s = Gs['za'], Gs['h']
    za_o, h_o = Go['za'], Go['h']

    x_arm_in = za_s + gap                       # ヨーク腕の内面
    x_arm_out = x_arm_in + yoke_t               # ヨーク腕の外面
    x_frame = x_arm_out                         # フレーム板の内面
    x_frame_out = x_frame + frame_t             # フレーム板の外面
    # 旋回ボスは軸受を抜けてフレーム板を貫通する長さが要る
    yk_boss_len = max(yk_brg_w, frame_t)
    x_boss_end = x_arm_out + yk_boss_len        # ヨーク旋回ボスの端
    x_pulley = max(x_boss_end, x_frame_out) + gap   # プーリ内面(フレーム板の外)
    x_shaft_end = x_pulley + pw + gap           # 入力軸の先端(プーリを貫通させる)
    x_motor_face = x_pulley + pw + gap          # モータ前面
    spacer = x_motor_face - x_frame_out         # モータ取付に必要なスペーサ長

    r_j = pulley_pd(zj, pitch) / 2.0
    r_m = pulley_pd(zmp, pitch) / 2.0
    if center <= r_j + r_m + 0.2:
        raise RuntimeError('軸間距離が短すぎます。プーリ同士が干渉します。')

    z_yoke_top = za_o + gap                     # ヨーク天板の下面
    z_out_seat = z_yoke_top                     # 出力軸受の座面
    z_flange = z_yoke_top + out_brg_w + 0.2     # 出力フランジ下面

    i_belt = zj / float(zmp)
    ratio_roll = zs / float(zo)

    parent = comp_plain(root, 'DiffJoint Zs{}-Zo{} i{:.1f}'.format(zs, zo, i_belt))

    # ---------- サイドギア (左右) ----------
    # ローカル +Z(頂点方向)を グローバル ±X に向け、頂点を原点に一致させる
    # → 背面(大端)は x = ∓za_s に来る
    for sgn, nm in ((-1, 'L'), (+1, 'R')):
        if sgn < 0:
            org = (-za_s, 0, 0)
            xa, ya, za_v = (0, 1, 0), (0, 0, 1), (1, 0, 0)
        else:
            org = (+za_s, 0, 0)
            xa, ya, za_v = (0, 1, 0), (0, 0, -1), (-1, 0, 0)
        c = comp_at(parent, 'SideGear_{} Z{}'.format(nm, zs), org, xa, ya, za_v)
        # ハブ = 入力軸そのもの。軸受の内径ぴったりに作り、腕の軸受 → 旋回ボスの中 →
        # フレーム板 → 大プーリ を貫通させる(以前は軸受より太く、プーリにも届いて
        # いなかったのでプーリが宙に浮いていた)
        build_bevel(c, bm, zs, zo, pa, face, in_brg_id * 0.5, backlash,
                    hub_d=in_brg_id, hub_len=x_shaft_end - za_s)

    # ---------- 出力ベベル ----------
    # ハブ = 出力軸。出力軸受の内径ぴったりで、上端はフランジの下面でぴたりと止める
    # (以前はフランジを突き抜けていた)。上端面にフランジ締結用のねじ穴を彫る。
    c_out = comp_at(parent, 'OutputBevel Z{}'.format(zo),
                    (0, 0, za_o), (1, 0, 0), (0, -1, 0), (0, 0, -1))
    out_hub_len = max(z_flange - za_o, 0.5)     # 大端からフランジ下面まで
    out_bcr = out_brg_id / 2.0 * 0.62           # フランジのボルト円(ハブの中に収める)
    ob_body, _ = build_bevel(c_out, bm, zo, zs, pa, face, out_brg_id * 0.35, backlash,
                             hub_d=out_brg_id, hub_len=out_hub_len)
    # ハブ上端面 = ローカル -Z 側。下穴(ねじ込み用)を深さ 2.5×d で彫る
    cut_holes(c_out, ob_body, bolt_circle(flange_bolt_n, out_bcr),
              flange_bolt_d * 0.42, -out_hub_len, flange_bolt_d * 2.5)

    # ---------- 大プーリ(関節側)左右 ----------
    for sgn, nm in ((-1, 'L'), (+1, 'R')):
        if sgn < 0:
            org = (-(x_pulley + pw), 0, 0)
            xa, ya, za_v = (0, 1, 0), (0, 0, 1), (1, 0, 0)
        else:
            org = ((x_pulley + pw), 0, 0)
            xa, ya, za_v = (0, 1, 0), (0, 0, -1), (-1, 0, 0)
        c = comp_at(parent, 'JointPulley_{} Z{}'.format(nm, zj), org, xa, ya, za_v)
        build_pulley(c, zj, pitch, pw, in_brg_id)

    # ---------- モータプーリ 左右 ----------
    for sgn, nm in ((-1, 'L'), (+1, 'R')):
        if sgn < 0:
            org = (-(x_pulley + pw), 0, -center)
            xa, ya, za_v = (0, 1, 0), (0, 0, 1), (1, 0, 0)
        else:
            org = ((x_pulley + pw), 0, -center)
            xa, ya, za_v = (0, 1, 0), (0, 0, -1), (-1, 0, 0)
        c = comp_at(parent, 'MotorPulley_{} Z{}'.format(nm, zmp), org, xa, ya, za_v)
        build_pulley(c, zmp, pitch, pw, mot['shaft'] * MM + 0.02)

    # ---------- ヨーク(差動キャリア) ----------
    yc = comp_plain(parent, 'Yoke')
    hw = yoke_w / 2.0
    z_bot = -(Gs['rp'] + bm * 2)                 # 腕の下端
    arm_body = None
    for sgn in (-1, +1):
        # YZ 平面にスケッチ → X 方向に押し出し
        sk = yz_sketch(yc)
        rect_yz(sk, -hw, z_bot, hw, z_yoke_top + yoke_t)
        x0 = x_arm_in if sgn > 0 else -x_arm_out
        f = extrude_x(yc, sk, sk.profiles.item(0), x0, yoke_t,
                      NEW if arm_body is None else JOIN,
                      None if arm_body is None else [arm_body])
        if arm_body is None:
            arm_body = f.bodies.item(0)

    # 天板(左右の腕をつなぐ)
    skt = yc.sketches.add(yc.xYConstructionPlane)
    rect(skt, -x_arm_out, -hw, x_arm_out, hw)
    extrude(yc, skt.profiles.item(0), yoke_t, JOIN, z_yoke_top, [arm_body])

    # 旋回ボス(フレーム軸受に入る)。軸受を抜けて板を貫通する長さにする
    for sgn in (-1, +1):
        sk = yz_sketch(yc)
        circle_yz(sk, 0, 0, yk_brg_id / 2.0)
        x0 = x_arm_out if sgn > 0 else -x_boss_end
        extrude_x(yc, sk, sk.profiles.item(0), x0, yk_boss_len, JOIN, [arm_body])

    # 腕の入力軸受座(X方向にザグリ)+ 貫通穴
    sk = yz_sketch(yc)
    circle_yz(sk, 0, 0, in_brg_od / 2.0)
    extrude_x(yc, sk, sk.profiles.item(0), -x_boss_end - 0.1,
              2 * x_boss_end + 0.2, CUT, [arm_body])

    # 天板の出力軸受座 + 貫通
    cut_holes(yc, arm_body, [(0, 0)], out_brg_od / 2.0,
              z_yoke_top, out_brg_w if out_brg_w < yoke_t else yoke_t)
    cut_holes(yc, arm_body, [(0, 0)], out_brg_id / 2.0 + 0.05,
              z_yoke_top - 0.1, yoke_t + 0.4)

    # 軽量化窓(腕の下側)。入力軸受の座を食わないよう、軸受穴の下に収まるときだけ開ける
    w_top = -(in_brg_od / 2.0 + 0.3)      # 軸受穴の下に肉を 3mm 残す
    w_bot = z_bot + 0.3                   # 腕の下端にも 3mm 残す
    w_r = min((w_top - w_bot) / 2.0, hw * 0.55)
    if w_r >= 0.25:
        sk = yz_sketch(yc)
        circle_yz(sk, 0, (w_top + w_bot) / 2.0, w_r)
        extrude_x(yc, sk, sk.profiles.item(0), -x_boss_end - 0.1,
                  2 * x_boss_end + 0.2, CUT, [arm_body])

    # ---------- 出力フランジ ----------
    # 出力ベベルのハブ上端にボルト止めする。ボルト円はハブ(φ out_brg_id)の内側に置く
    # (以前はハブの外側に出ていて、締結先が無かった)
    fc = comp_plain(parent, 'OutputFlange')
    flange_t = 0.5
    fr = out_brg_od / 2.0 + flange_bolt_d * 1.6
    sk = circle(fc, fc.xYConstructionPlane, fr)
    fb = extrude(fc, sk.profiles.item(0), flange_t, NEW, z_flange).bodies.item(0)
    cut_holes(fc, fb, bolt_circle(flange_bolt_n, out_bcr), flange_bolt_d / 2.0,
              z_flange, flange_t)
    cut_holes(fc, fb, [(0, 0)], out_brg_id / 2.0 * 0.35, z_flange, flange_t)

    # ---------- フレーム側板(左右) ----------
    frame_top = z_yoke_top + yoke_t + 1.0
    plate_half_y = max(hw + 1.0, mot['size'] * MM / 2.0 + mot['bolt'] * MM)
    plate_bot = -center - plate_half_y

    for sgn, nm in ((-1, 'L'), (+1, 'R')):
        c = comp_plain(parent, 'FramePlate_{}'.format(nm))
        x0 = x_frame if sgn > 0 else -(x_frame + frame_t)
        sk = yz_sketch(c)
        rect_yz(sk, -plate_half_y, plate_bot, plate_half_y, frame_top)
        pb = extrude_x(c, sk, sk.profiles.item(0), x0, frame_t, NEW).bodies.item(0)

        # ヨーク旋回軸受の座(YZ 面に円 → X 方向カット)
        sk = yz_sketch(c)
        circle_yz(sk, 0, 0, yk_brg_od / 2.0)
        extrude_x(c, sk, sk.profiles.item(0), x0 - 0.1, frame_t + 0.2, CUT, [pb])

        # モータのボス逃げ + ボルト長穴(長穴は Z 方向 = ベルトを張る向き)
        skm = yz_sketch(c)
        with defer(skm):
            circle_yz(skm, 0, -center, mot['boss'] * MM / 2.0 + 0.03)
            bx = mot['bx'] * MM / 2.0
            for sz in (1, -1):
                for sy in (1, -1):
                    slot_yz(skm, sy * bx, -center + sz * bx, slot_len / 2.0,
                            mot['bolt'] * MM / 2.0)
        extrude_x(c, skm, all_profiles(skm), x0 - 0.1, frame_t + 0.2, CUT, [pb])

        # 角のフィレット
        try:
            edges = adsk.core.ObjectCollection.create()
            for e in pb.edges:
                g = e.geometry
                if g.objectType == adsk.core.Line3D.classType():
                    v = g.startPoint.vectorTo(g.endPoint)
                    if abs(v.y) < 1e-6 and abs(v.z) < 1e-6:
                        edges.add(e)
            if edges.count:
                fin = c.features.filletFeatures.createInput()
                fin.addConstantRadiusEdgeSet(edges, VI(0.5), True)
                c.features.filletFeatures.add(fin)
        except Exception:
            pass

    # ---------- モータ外形(参考) ----------
    # モータはフレーム板の外面にスペーサで浮かせて留め、軸は内向き。
    # プーリはモータ前面と板の間に入り、関節プーリと同じ面に並ぶ。
    if make_motor:
        hs = mot['size'] * MM / 2.0
        ml = mot['length'] * MM
        shaft_len = pw + 2.0 * gap
        for sgn, nm in ((-1, 'L'), (+1, 'R')):
            mc = comp_plain(parent, 'MotorRef_{}'.format(nm))
            xf = sgn * x_motor_face                 # モータ前面(軸が出ている面)
            # 胴体は前面から外側へ
            skb = yz_sketch(mc)
            rect_yz(skb, -hs, -center - hs, hs, -center + hs)
            mb = extrude_x(mc, skb, skb.profiles.item(0),
                           xf if sgn > 0 else xf - ml, ml, NEW).bodies.item(0)
            # 軸は前面から内側(フレーム板の方)へ。途中でプーリを貫く
            sks = yz_sketch(mc)
            circle_yz(sks, 0, -center, mot['shaft'] * MM / 2.0)
            extrude_x(mc, sks, sks.profiles.item(0),
                      xf - shaft_len if sgn > 0 else xf, shaft_len, JOIN, [mb])

    # ---------- 組立位置への配置(パーツ ドキュメントのとき) ----------
    _place()
    _hide_helpers()

    # ---------- レポート ----------
    pitch_mm = pitch * 10.0
    L = belt_len_mm(zmp, zj, pitch_mm, center * 10.0)
    cand = sorted(GT2_STOCK, key=lambda s: abs(s - L))[:3]
    lines = ['   {:>4.0f} mm ({:>3.0f}歯) → 軸間 {:.2f} mm ({:+.1f} mm)'.format(
        s, s / pitch_mm, center_from_belt(zmp, zj, pitch_mm, s),
        center_from_belt(zmp, zj, pitch_mm, s) - center * 10.0) for s in cand]

    _ui.messageBox((
        '差動2自由度関節を生成しました。\n'
        '═══════════════════════════\n'
        '【ベベル差動】\n'
        '  サイドギア Z={} (円錐角 {:.1f}°) ×2\n'
        '  出力ベベル Z={} (円錐角 {:.1f}°)\n'
        '  モジュール {:.2f} mm / 歯幅 {:.1f} mm\n\n'
        '【ベルト】\n'
        '  モータ Z{} → 関節 Z{}  減速比 i = {:.2f}\n'
        '  軸間 {:.1f} mm / 必要ベルト長 {:.1f} mm\n'
        '  近い標準長:\n{}\n\n'
        '【運動学】θ1,θ2 = 左右モータ角\n'
        '  ピッチ φ = (θ1 + θ2) / {:.2f}\n'
        '  ロール ψ = (θ1 - θ2) × {:.4f}\n'
        '  逆解:\n'
        '    θ1 = {:.2f}×φ + {:.2f}×ψ\n'
        '    θ2 = {:.2f}×φ - {:.2f}×ψ\n'
        '  → 両モータ同方向 = ピッチ / 逆方向 = ロール\n\n'
        '【組立】\n'
        '  ヨーク旋回軸受: OD{:.0f}×ID{:.0f}×W{:.0f} ×2\n'
        '  入力軸受      : OD{:.0f}×ID{:.0f}×W{:.0f} ×2\n'
        '  出力軸受      : OD{:.0f}×ID{:.0f}×W{:.0f} ×1\n'
        '  入力軸は サイドギアと一体 (φ{:.1f} × 突き出し {:.1f} mm)。\n'
        '    腕の軸受 → ヨーク旋回ボスの中 → フレーム板 → 大プーリ を貫通します。\n'
        '  モータはフレーム板の外面に <スペーサ {:.1f} mm> を挟んで固定。\n'
        '    モータプーリはモータ前面と板の間に入り、関節プーリと同一面に並びます。\n'
        '    軸の突き出しは {:.1f} mm 以上必要です。\n'
        '  出力フランジは 出力ベベルのハブ上端に M{:.0f} ×{} 本で締結。\n'
        '  全幅 約 {:.0f} mm (モータ含まず) / 出力面高さ 約 {:.0f} mm\n'
        '═══════════════════════════\n'
        '※ 3枚のベベルは円錐頂点が原点で一致しています。\n'
        '  かみ合い位相は「回転」で微調整し、干渉コマンドで確認してください。'.format(
            zs, math.degrees(Gs['delta']), zo, math.degrees(Go['delta']),
            bm * 10, face * 10,
            zmp, zj, i_belt, center * 10, L, '\n'.join(lines),
            2 * i_belt, ratio_roll / (2 * i_belt),
            i_belt, i_belt / ratio_roll, i_belt, i_belt / ratio_roll,
            yk_brg_od * 10, yk_brg_id * 10, yk_brg_w * 10,
            in_brg_od * 10, in_brg_id * 10, in_brg_w * 10,
            out_brg_od * 10, out_brg_id * 10, out_brg_w * 10,
            in_brg_id * 10, (x_shaft_end - za_s) * 10,
            spacer * 10, (pw + 2 * gap) * 10,
            flange_bolt_d * 10, flange_bolt_n,
            2 * (x_pulley + pw) * 10, (z_flange + 0.5) * 10) + _note()))
    return parent


# ==========================================================================
#  UI
# ==========================================================================
class CreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.setDialogInitialSize(440, 620)
            ins = cmd.commandInputs

            def S(v):
                return adsk.core.ValueInput.createByString(v)

            ins.addTextBoxCommandInput(
                'info', '', '<b>ベルト駆動 差動2自由度関節</b><br>'
                '両モータ同方向 → ピッチ / 逆方向 → ロール', 2, True)

            g = ins.addGroupCommandInput('gb', '① ベベル差動')
            c = g.children
            c.addValueInput('bm', 'モジュール', 'mm', S('1.5 mm'))
            c.addIntegerSpinnerCommandInput('zs', 'サイドギア歯数 Zs', 10, 60, 1, 20)
            c.addIntegerSpinnerCommandInput('zo', '出力ベベル歯数 Zo', 10, 60, 1, 20)
            c.addValueInput('pa', '圧力角', 'deg', S('20 deg'))
            c.addValueInput('face', '歯幅', 'mm', S('6 mm'))
            c.addValueInput('bl', 'バックラッシ', 'mm', S('0.15 mm'))

            g = ins.addGroupCommandInput('gp', '② ベルト / プーリ')
            c = g.children
            c.addIntegerSpinnerCommandInput('zj', '関節プーリ歯数', 20, 150, 1, 60)
            c.addIntegerSpinnerCommandInput('zmp', 'モータプーリ歯数', 10, 60, 1, 20)
            c.addValueInput('pitch', 'ベルトピッチ', 'mm', S('2 mm'))
            c.addValueInput('pw', 'プーリ幅', 'mm', S('7 mm'))
            c.addValueInput('center', '軸間距離', 'mm', S('90 mm'))
            c.addValueInput('slot', 'テンション長穴の長さ', 'mm', S('8 mm'))

            g = ins.addGroupCommandInput('gbr', '③ 軸受')
            c = g.children
            c.addValueInput('ibOD', '入力軸受 外径', 'mm', S('19 mm'))
            c.addValueInput('ibID', '入力軸受 内径', 'mm', S('8 mm'))
            c.addValueInput('ibW', '入力軸受 幅', 'mm', S('6 mm'))
            c.addValueInput('ykOD', 'ヨーク旋回軸受 外径', 'mm', S('32 mm'))
            c.addValueInput('ykID', 'ヨーク旋回軸受 内径', 'mm', S('25 mm'))
            c.addValueInput('ykW', 'ヨーク旋回軸受 幅', 'mm', S('4 mm'))
            c.addValueInput('obOD', '出力軸受 外径', 'mm', S('42 mm'))
            c.addValueInput('obID', '出力軸受 内径', 'mm', S('30 mm'))
            c.addValueInput('obW', '出力軸受 幅', 'mm', S('7 mm'))

            g = ins.addGroupCommandInput('gs', '④ 構造 / モータ')
            c = g.children
            md = c.addDropDownCommandInput(
                'motor', 'モータ', adsk.core.DropDownStyles.TextListDropDownStyle)
            for i, k in enumerate(MOTORS.keys()):
                md.listItems.add(k, i == 0)
            c.addValueInput('yt', 'ヨーク板厚', 'mm', S('8 mm'))
            c.addValueInput('yw', 'ヨーク幅 (奥行)', 'mm', S('50 mm'))
            c.addValueInput('ft', 'フレーム板厚', 'mm', S('6 mm'))
            c.addValueInput('gap', '各部のすきま', 'mm', S('1.5 mm'))
            c.addIntegerSpinnerCommandInput('fbn', 'フランジ穴数', 3, 12, 1, 6)
            c.addValueInput('fbd', 'フランジ穴径', 'mm', S('3.4 mm'))
            c.addBoolValueInput('mref', 'モータ外形を参考生成', True, '', True)

            h1 = ExecHandler()
            cmd.execute.add(h1)
            _handlers.append(h1)
            h2 = DestroyHandler()
            cmd.destroy.add(h2)
            _handlers.append(h2)
        except Exception:
            _ui.messageBox('UI作成に失敗:\n' + traceback.format_exc())


class ExecHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            ins = args.command.commandInputs
            v = lambda k: ins.itemById(k).value
            build_diff_joint(
                v('bm'), v('zs'), v('zo'), v('pa'), v('face'), v('bl'),
                v('zj'), v('zmp'), v('pitch'), v('pw'), v('center'),
                v('ibOD'), v('ibID'), v('ibW'),
                v('ykOD'), v('ykID'), v('ykW'),
                v('obOD'), v('obID'), v('obW'),
                v('yt'), v('yw'), v('ft'), v('gap'),
                v('fbn'), v('fbd'),
                ins.itemById('motor').selectedItem.name, v('slot'), v('mref'))
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
