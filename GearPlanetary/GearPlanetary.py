# -*- coding: utf-8 -*-
"""GearPlanetary - 遊星歯車セットジェネレータ
太陽歯車・遊星歯車(複数)・内歯リングギアを噛み合い位置に一括生成します。
リング歯数 = 太陽歯数 + 2 x 遊星歯数 で自動決定。
"""
import adsk.core, adsk.fusion, traceback, math


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

CMD_ID = 'jpGearPlanetary'
CMD_NAME = '遊星歯車ジェネレータ'
CMD_DESC = '太陽・遊星・リングの遊星歯車セットを生成します'

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def _v(inputs, iid):
    return adsk.core.ValueCommandInput.cast(inputs.itemById(iid)).value

def _i(inputs, iid):
    return adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById(iid)).value

def _pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)

def _pcol(pts):
    c = adsk.core.ObjectCollection.create()
    for p in pts:
        c.add(_pt(p[0], p[1]))
    return c

def _extrude(comp, prof, dist, op):
    exts = comp.features.extrudeFeatures
    ei = exts.createInput(prof, op)
    ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(dist))
    return exts.add(_scope(ei))

def _pattern(comp, feats, nq):
    col = adsk.core.ObjectCollection.create()
    for f in feats:
        col.add(f)
    cps = comp.features.circularPatternFeatures
    ci = cps.createInput(col, comp.zConstructionAxis)
    ci.quantity = adsk.core.ValueInput.createByString(str(int(nq)))
    ci.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    return cps.add(ci)

def _circle(sk, r, cx=0.0, cy=0.0):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(cx, cy), r)

def _ring_profile(sk):
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        if p.profileLoops.count == 2:
            return p
    return sk.profiles.item(0)


def draw_tooth(sk, m, z, alpha, backlash=0.0, rot=0.0, ra=None, rf=None):
    r = m * z / 2.0
    rb = r * math.cos(alpha)
    if ra is None:
        ra = r + m
    if rf is None:
        rf = r - 1.25 * m
    inva = math.tan(alpha) - alpha
    th0 = math.pi / (2.0 * z) + inva - backlash / (2.0 * r)
    r0 = max(rb, rf)

    def flank(sign, n=12):
        pts = []
        for i in range(n + 1):
            rr = r0 + (ra - r0) * i / float(n)
            aa = math.acos(max(-1.0, min(1.0, rb / rr)))
            inv = math.tan(aa) - aa
            th = sign * (th0 - inv) + rot
            pts.append((rr * math.cos(th), rr * math.sin(th)))
        return pts

    lo = flank(-1.0)
    up = flank(1.0)
    cur = sk.sketchCurves
    slo = cur.sketchFittedSplines.add(_pcol(lo))
    sup = cur.sketchFittedSplines.add(_pcol(up))
    cur.sketchArcs.addByThreePoints(
        slo.endSketchPoint, _pt(ra * math.cos(rot), ra * math.sin(rot)), sup.endSketchPoint)
    a0 = math.acos(max(-1.0, min(1.0, rb / r0)))
    ang = th0 - (math.tan(a0) - a0)
    mid = _pt(rf * math.cos(rot), rf * math.sin(rot))
    if rf < r0 - 1e-7:
        pl = _pt(rf * math.cos(rot - ang), rf * math.sin(rot - ang))
        pu = _pt(rf * math.cos(rot + ang), rf * math.sin(rot + ang))
        l1 = cur.sketchLines.addByTwoPoints(pl, slo.startSketchPoint)
        l2 = cur.sketchLines.addByTwoPoints(sup.startSketchPoint, pu)
        cur.sketchArcs.addByThreePoints(l2.endSketchPoint, mid, l1.startSketchPoint)
    else:
        cur.sketchArcs.addByThreePoints(sup.startSketchPoint, mid, slo.startSketchPoint)


def make_external_gear(comp, m, z, alpha, bl, w, rot, bore):
    r = m * z / 2.0
    sk = comp.sketches.add(comp.xYConstructionPlane)
    _circle(sk, r - 1.25 * m)
    _extrude(comp, sk.profiles.item(0), w, NEW)
    sk1 = comp.sketches.add(comp.xYConstructionPlane)
    draw_tooth(sk1, m, z, alpha, bl, rot=rot)
    tf = _extrude(comp, sk1.profiles.item(0), w, JOIN)
    _pattern(comp, [tf], z)
    if bore > 1e-6:
        skb = comp.sketches.add(comp.xYConstructionPlane)
        _circle(skb, bore / 2.0)
        _extrude(comp, skb.profiles.item(0), w, CUT)


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    m = _v(inputs, 'module')
    zs = _i(inputs, 'sun')
    zp = _i(inputs, 'planet')
    k = _i(inputs, 'count')
    alpha = _v(inputs, 'pa')
    w = _v(inputs, 'width')
    bs = _v(inputs, 'bores')
    bp = _v(inputs, 'borep')
    rim = _v(inputs, 'rim')
    bl = _v(inputs, 'backlash')

    zr = zs + 2 * zp
    if (zs + zr) % k != 0:
        _ui.messageBox('注意: (太陽歯数+リング歯数) が遊星数で割り切れないため、\n'
                       '等間隔配置では正しく組めません。歯数を調整してください。\n'
                       '(zs+zr)={0}, 遊星数={1}'.format(zs + zr, k))

    a = m * (zs + zp) / 2.0    # 太陽-遊星の軸間距離
    root = des.rootComponent

    # ---- 太陽歯車 ----
    sun = _new_part(root, '太陽歯車 z{0}'.format(zs))
    make_external_gear(sun, m, zs, alpha, bl, w, 0.0, bs)

    # ---- 遊星歯車 ----
    psi0 = None
    for i in range(k):
        phi = 2.0 * math.pi * i / k
        f = (phi * zs / (2.0 * math.pi)) % 1.0
        psi = phi + math.pi - math.pi / zp - f * 2.0 * math.pi / zp
        if i == 0:
            psi0 = psi
        tr = adsk.core.Matrix3D.create()
        tr.translation = adsk.core.Vector3D.create(a * math.cos(phi), a * math.sin(phi), 0)
        po = _new_part(root, '遊星歯車{0} z{1}'.format(i + 1, zp), tr)
        make_external_gear(po, m, zp, alpha, bl, w, psi, bp)

    # ---- リングギア(内歯) ----
    rr = m * zr / 2.0
    pitch_p = 2.0 * math.pi / zp
    e = (psi0 + pitch_p / 2.0) % pitch_p - pitch_p / 2.0   # 遊星の外向き歯の位相
    rot_r = e * zp / float(zr)
    ring = _new_part(root, 'リングギア z{0}'.format(zr))
    ra_i = rr - m
    rf_i = rr + 1.25 * m
    sk = ring.sketches.add(ring.xYConstructionPlane)
    _circle(sk, rf_i + rim)
    _circle(sk, ra_i)
    _extrude(ring, _ring_profile(sk), w, NEW)
    sk1 = ring.sketches.add(ring.xYConstructionPlane)
    draw_tooth(sk1, m, zr, alpha, backlash=-bl, rot=rot_r, ra=rf_i, rf=ra_i - 0.15 * m)
    cf = _extrude(ring, sk1.profiles.item(0), w, CUT)
    _pattern(ring, [cf], zr)

    ratio = 1.0 + float(zr) / zs
    _finish('生成完了\nリング歯数 zr = {0}\n'
            '減速比(太陽入力・キャリア出力・リング固定) = {1:.3f} : 1\n'
            '※遊星の位相が僅かにずれる場合は各遊星を微回転させてください'.format(zr, ratio))


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addValueInput('module', 'モジュール', 'mm', vi.createByString('1.5 mm'))
    inputs.addIntegerSpinnerCommandInput('sun', '太陽歯数', 6, 200, 1, 12)
    inputs.addIntegerSpinnerCommandInput('planet', '遊星歯数', 6, 200, 1, 18)
    inputs.addIntegerSpinnerCommandInput('count', '遊星の個数', 2, 8, 1, 3)
    inputs.addValueInput('pa', '圧力角', 'deg', vi.createByString('20 deg'))
    inputs.addValueInput('width', '歯幅', 'mm', vi.createByString('8 mm'))
    inputs.addValueInput('bores', '太陽の軸穴径(0=なし)', 'mm', vi.createByString('4 mm'))
    inputs.addValueInput('borep', '遊星の軸穴径(0=なし)', 'mm', vi.createByString('3 mm'))
    inputs.addValueInput('rim', 'リングのリム厚', 'mm', vi.createByString('4 mm'))
    inputs.addValueInput('backlash', 'バックラッシ(歯厚減少量)', 'mm', vi.createByString('0.15 mm'))


class _Exec(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            build(adsk.core.CommandEventArgs.cast(args).command.commandInputs)
        except:
            if _ui:
                _ui.messageBox('失敗しました:\n{}'.format(traceback.format_exc()))

class _Destroy(adsk.core.CommandEventHandler):
    def notify(self, args):
        adsk.terminate()

class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = adsk.core.CommandCreatedEventArgs.cast(args).command
            make_inputs(cmd.commandInputs)
            h1 = _Exec(); cmd.execute.add(h1); _handlers.append(h1)
            h2 = _Destroy(); cmd.destroy.add(h2); _handlers.append(h2)
        except:
            if _ui:
                _ui.messageBox('失敗しました:\n{}'.format(traceback.format_exc()))

def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface
        cd = _ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = _ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC)
        h = _Created(); cd.commandCreated.add(h); _handlers.append(h)
        cd.execute()
        adsk.autoTerminate(False)
    except:
        if _ui:
            _ui.messageBox('失敗しました:\n{}'.format(traceback.format_exc()))
