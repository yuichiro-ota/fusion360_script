# -*- coding: utf-8 -*-
"""GearWorm - ウォームギアジェネレータ
ウォーム(ねじ歯車)とウォームホイールを噛み合い位置に生成します。
ウォーム軸はY方向・ホイール軸はZ方向で、軸間距離に自動配置されます。
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

CMD_ID = 'jpGearWorm'
CMD_NAME = 'ウォームギアジェネレータ'
CMD_DESC = 'ウォームとウォームホイールを生成します'

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def _v(inputs, iid):
    return adsk.core.ValueCommandInput.cast(inputs.itemById(iid)).value

def _i(inputs, iid):
    return adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById(iid)).value

def _bool(inputs, iid):
    return adsk.core.BoolValueCommandInput.cast(inputs.itemById(iid)).value

def _pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)

def _pcol2(pts):
    c = adsk.core.ObjectCollection.create()
    for p in pts:
        c.add(_pt(p[0], p[1]))
    return c

def _extrude(comp, prof, dist, op):
    exts = comp.features.extrudeFeatures
    ei = exts.createInput(prof, op)
    ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(dist))
    return exts.add(_scope(ei))

def _plane(comp, offset):
    pls = comp.constructionPlanes
    pi = pls.createInput()
    pi.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(offset))
    return pls.add(pi)

def _pattern(comp, feats, nq):
    col = adsk.core.ObjectCollection.create()
    for f in feats:
        col.add(f)
    cps = comp.features.circularPatternFeatures
    ci = cps.createInput(col, comp.zConstructionAxis)
    ci.quantity = adsk.core.ValueInput.createByString(str(int(nq)))
    ci.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    return cps.add(ci)


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
    slo = cur.sketchFittedSplines.add(_pcol2(lo))
    sup = cur.sketchFittedSplines.add(_pcol2(up))
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


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    m = _v(inputs, 'module')
    starts = _i(inputs, 'starts')
    d1 = _v(inputs, 'wormd')
    ln = _v(inputs, 'wlen')
    z2 = _i(inputs, 'teeth')
    alpha = _v(inputs, 'pa')
    w2 = _v(inputs, 'width')
    bore1 = _v(inputs, 'bore1')
    bore2 = _v(inputs, 'bore2')
    bl = _v(inputs, 'backlash')
    throat = _bool(inputs, 'throat')

    r1 = d1 / 2.0
    r2 = m * z2 / 2.0
    a = r1 + r2                                # 軸間距離
    lead = math.pi * m * starts                # リード
    gamma = math.atan2(lead, math.pi * d1)     # 進み角

    root = des.rootComponent

    # ---------- ウォーム (Z軸方向に作りY軸方向へ回転配置) ----------
    tr = adsk.core.Matrix3D.create()
    tr.setToRotation(math.pi / 2.0, adsk.core.Vector3D.create(1, 0, 0), _pt(0, 0, 0))
    tr.translation = adsk.core.Vector3D.create(a, ln / 2.0, w2 / 2.0)
    worm = _new_part(root, 'ウォーム m{0} 条数{1}'.format(round(m * 10, 3), starts), tr)

    # 芯円筒
    skc = worm.sketches.add(worm.xYConstructionPlane)
    skc.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), r1 - 1.25 * m)
    _extrude(worm, skc.profiles.item(0), ln, NEW)

    # ねじ山: らせんスプラインに沿ってスイープ
    turns = ln / lead
    npt = max(24, int(turns * 24))
    ha = m
    hf = 1.25 * m
    wt = math.pi * m / 4.0 - ha * math.tan(alpha) - bl / 2.0
    wr = math.pi * m / 4.0 + hf * math.tan(alpha) - bl / 2.0
    for s in range(starts):
        phase = 2.0 * math.pi * s / starts
        skh = worm.sketches.add(worm.xYConstructionPlane)
        col = adsk.core.ObjectCollection.create()
        for i in range(npt + 1):
            t = 2.0 * math.pi * turns * i / float(npt)
            col.add(_pt(r1 * math.cos(t + phase), r1 * math.sin(t + phase), lead * t / (2.0 * math.pi)))
        helix = skh.sketchCurves.sketchFittedSplines.add(col)
        # プロファイル平面(経路始点で経路に垂直)
        pls = worm.constructionPlanes
        pin = pls.createInput()
        pin.setByDistanceOnPath(helix, adsk.core.ValueInput.createByReal(0.0))
        ppl = pls.add(pin)
        skp = worm.sketches.add(ppl)

        def mp(rr, zz):
            p = skp.modelToSketchSpace(_pt(rr * math.cos(phase), rr * math.sin(phase), zz))
            p.z = 0
            return p

        lines = skp.sketchCurves.sketchLines
        pA = mp(r1 - hf - 0.05 * m, -wr)
        pB = mp(r1 + ha, -wt)
        pC = mp(r1 + ha, wt)
        pD = mp(r1 - hf - 0.05 * m, wr)
        l1 = lines.addByTwoPoints(pA, pB)
        l2 = lines.addByTwoPoints(l1.endSketchPoint, pC)
        l3 = lines.addByTwoPoints(l2.endSketchPoint, pD)
        lines.addByTwoPoints(l3.endSketchPoint, l1.startSketchPoint)

        path = worm.features.createPath(helix)
        sweeps = worm.features.sweepFeatures
        si = sweeps.createInput(skp.profiles.item(0), path, JOIN)
        si.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
        sweeps.add(_scope(si))

    if bore1 > 1e-6:
        skb = worm.sketches.add(worm.xYConstructionPlane)
        skb.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), bore1 / 2.0)
        _extrude(worm, skb.profiles.item(0), ln, CUT)

    # ---------- ウォームホイール (はすば歯車 + 任意で喉部) ----------
    wheel = _new_part(root, 'ウォームホイール m{0} z{1}'.format(round(m * 10, 3), z2))

    ra2 = r2 + m + (0.4 * m if throat else 0.0)
    rf2 = r2 - 1.25 * m
    skw = wheel.sketches.add(wheel.xYConstructionPlane)
    skw.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), rf2)
    _extrude(wheel, skw.profiles.item(0), w2, NEW)

    twist = w2 * math.tan(gamma) / r2
    sk1 = wheel.sketches.add(wheel.xYConstructionPlane)
    draw_tooth(sk1, m, z2, alpha, bl, rot=0.0, ra=ra2, rf=rf2)
    pl2 = _plane(wheel, w2)
    sk2 = wheel.sketches.add(pl2)
    draw_tooth(sk2, m, z2, alpha, bl, rot=twist, ra=ra2, rf=rf2)
    lofts = wheel.features.loftFeatures
    li = lofts.createInput(JOIN)
    li.loftSections.add(sk1.profiles.item(0))
    li.loftSections.add(sk2.profiles.item(0))
    tf = lofts.add(_scope(li))
    _pattern(wheel, [tf], z2)

    if throat:
        # ウォームを包み込む喉部を回転カット
        rc = r1 - m
        skt = wheel.sketches.add(wheel.xZConstructionPlane)
        ctr = skt.modelToSketchSpace(_pt(a, 0.0, w2 / 2.0))
        ctr.z = 0
        skt.sketchCurves.sketchCircles.addByCenterRadius(ctr, rc)
        revs = wheel.features.revolveFeatures
        ri = revs.createInput(skt.profiles.item(0), wheel.zConstructionAxis, CUT)
        ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(math.pi * 2))
        revs.add(_scope(ri))

    if bore2 > 1e-6:
        skb2 = wheel.sketches.add(wheel.xYConstructionPlane)
        skb2.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), bore2 / 2.0)
        _extrude(wheel, skb2.profiles.item(0), w2, CUT)

    _finish('生成完了\n軸間距離 a = {:.3f} mm\n進み角 γ = {:.2f}°\n減速比 = {}:{}'.format(
        a * 10, math.degrees(gamma), z2, starts))


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addValueInput('module', 'モジュール(軸直角)', 'mm', vi.createByString('2 mm'))
    inputs.addIntegerSpinnerCommandInput('starts', 'ウォーム条数', 1, 6, 1, 1)
    inputs.addValueInput('wormd', 'ウォームピッチ円径', 'mm', vi.createByString('16 mm'))
    inputs.addValueInput('wlen', 'ウォーム長さ', 'mm', vi.createByString('30 mm'))
    inputs.addIntegerSpinnerCommandInput('teeth', 'ホイール歯数', 10, 300, 1, 30)
    inputs.addValueInput('pa', '圧力角', 'deg', vi.createByString('20 deg'))
    inputs.addValueInput('width', 'ホイール歯幅', 'mm', vi.createByString('10 mm'))
    inputs.addValueInput('bore1', 'ウォーム軸穴径(0=なし)', 'mm', vi.createByString('5 mm'))
    inputs.addValueInput('bore2', 'ホイール軸穴径(0=なし)', 'mm', vi.createByString('6 mm'))
    inputs.addValueInput('backlash', 'バックラッシ(歯厚減少量)', 'mm', vi.createByString('0.15 mm'))
    inputs.addBoolValueInput('throat', '喉付きホイール(鼓形)にする', True, '', True)


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
