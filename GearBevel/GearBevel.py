# -*- coding: utf-8 -*-
"""GearBevel - すぐばかさ歯車ジェネレータ (Tredgold相当歯車近似)
軸角90度のかさ歯車を生成します。相手歯数を指定すると円すい角が決まります。
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

CMD_ID = 'jpGearBevel'
CMD_NAME = 'かさ歯車ジェネレータ'
CMD_DESC = 'すぐばかさ歯車を生成します(軸角90度)'

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

def _new_comp(name):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    return _new_part(des.rootComponent, name)

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


def bevel_tooth_sketch(sk, m, zv, alpha, backlash, r_act, cosd, scale):
    """相当平歯車(歯数zv)の歯形を実ピッチ半径r_actへ配置し、scale倍(頂点=原点)で描画"""
    rv = m * zv / 2.0
    rb = rv * math.cos(alpha)
    ra = rv + m
    rf = rv - 1.25 * m
    inva = math.tan(alpha) - alpha
    th0 = math.pi / (2.0 * zv) + inva - backlash / (2.0 * rv)
    r0 = max(rb, rf)

    def xf(px, py):
        return ((r_act + (px - rv) * cosd) * scale, py * scale)

    def flank(sign, n=10):
        pts = []
        for i in range(n + 1):
            rr = r0 + (ra - r0) * i / float(n)
            aa = math.acos(max(-1.0, min(1.0, rb / rr)))
            inv = math.tan(aa) - aa
            th = sign * (th0 - inv)
            pts.append(xf(rr * math.cos(th), rr * math.sin(th)))
        return pts

    lo = flank(-1.0)
    up = flank(1.0)
    cur = sk.sketchCurves
    slo = cur.sketchFittedSplines.add(_pcol(lo))
    sup = cur.sketchFittedSplines.add(_pcol(up))
    tm = xf(ra, 0.0)
    cur.sketchArcs.addByThreePoints(slo.endSketchPoint, _pt(tm[0], tm[1]), sup.endSketchPoint)
    a0 = math.acos(max(-1.0, min(1.0, rb / r0)))
    ang = th0 - (math.tan(a0) - a0)
    rl = xf(rf * math.cos(-ang), rf * math.sin(-ang))
    ru = xf(rf * math.cos(ang), rf * math.sin(ang))
    rm = xf(rf, 0.0)
    if rf < r0 - 1e-7:
        l1 = cur.sketchLines.addByTwoPoints(_pt(rl[0], rl[1]), slo.startSketchPoint)
        l2 = cur.sketchLines.addByTwoPoints(sup.startSketchPoint, _pt(ru[0], ru[1]))
        cur.sketchArcs.addByThreePoints(l2.endSketchPoint, _pt(rm[0], rm[1]), l1.startSketchPoint)
    else:
        cur.sketchArcs.addByThreePoints(sup.startSketchPoint, _pt(rm[0], rm[1]), slo.startSketchPoint)


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    m = _v(inputs, 'module')
    z1 = _i(inputs, 'teeth')
    z2 = _i(inputs, 'mate')
    alpha = _v(inputs, 'pa')
    b = _v(inputs, 'face')
    bore = _v(inputs, 'bore')
    bl = _v(inputs, 'backlash')

    delta = math.atan2(float(z1), float(z2))   # ピッチ円すい角(軸角90度)
    r = m * z1 / 2.0
    Re = r / math.sin(delta)                   # 外端円すい距離
    if b > Re / 3.0:
        _ui.messageBox('歯幅が円すい距離の1/3を超えています。b={:.2f}mm に縮小します'.format(Re / 3.0 * 10))
        b = Re / 3.0
    k = (Re - b) / Re
    cosd = math.cos(delta)
    zv = z1 / cosd                              # 相当歯数
    z_o = Re * cosd                             # 外端断面のZ位置(頂点=原点)
    z_i = z_o * k

    comp = _new_comp('かさ歯車 m{0} z{1}/z{2}'.format(round(m * 10, 3), z1, z2))

    # 歯底円すいブランク (回転体)
    rf_o = r - 1.25 * m * cosd
    skr = comp.sketches.add(comp.xZConstructionPlane)
    mps = [(0.0, z_i), (0.0, z_o), (rf_o, z_o), (rf_o * k, z_i)]
    lines = skr.sketchCurves.sketchLines
    prev = None
    first = None
    for i in range(len(mps)):
        a = skr.modelToSketchSpace(_pt(mps[i][0], 0.0, mps[i][1]))
        a.z = 0
        bpt = skr.modelToSketchSpace(_pt(mps[(i + 1) % len(mps)][0], 0.0, mps[(i + 1) % len(mps)][1]))
        bpt.z = 0
        if prev is None:
            ln = lines.addByTwoPoints(a, bpt)
            first = ln
        elif i < len(mps) - 1:
            ln = lines.addByTwoPoints(prev.endSketchPoint, bpt)
        else:
            ln = lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
        prev = ln
    revs = comp.features.revolveFeatures
    ri = revs.createInput(skr.profiles.item(0), comp.zConstructionAxis, NEW)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(math.pi * 2))
    revs.add(ri)

    # 歯: 外端断面と内端断面(頂点に向けて縮小)のロフト
    pl_o = _plane(comp, z_o)
    pl_i = _plane(comp, z_i)
    sk_o = comp.sketches.add(pl_o)
    sk_i = comp.sketches.add(pl_i)
    bevel_tooth_sketch(sk_o, m, zv, alpha, bl, r, cosd, 1.0)
    bevel_tooth_sketch(sk_i, m, zv, alpha, bl, r, cosd, k)
    lofts = comp.features.loftFeatures
    li = lofts.createInput(JOIN)
    li.loftSections.add(sk_o.profiles.item(0))
    li.loftSections.add(sk_i.profiles.item(0))
    tf = lofts.add(_scope(li))
    _pattern(comp, [tf], z1)

    # 軸穴
    if bore > 1e-6:
        skb = comp.sketches.add(pl_o)
        skb.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), bore / 2.0)
        exts = comp.features.extrudeFeatures
        ei = exts.createInput(skb.profiles.item(0), CUT)
        ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-(z_o - z_i + 0.5)))
        exts.add(_scope(ei))

    _finish()


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addValueInput('module', 'モジュール(外端)', 'mm', vi.createByString('2 mm'))
    inputs.addIntegerSpinnerCommandInput('teeth', '歯数(この歯車)', 6, 200, 1, 20)
    inputs.addIntegerSpinnerCommandInput('mate', '相手歯数(円すい角決定)', 6, 200, 1, 20)
    inputs.addValueInput('pa', '圧力角', 'deg', vi.createByString('20 deg'))
    inputs.addValueInput('face', '歯幅(円すい距離方向)', 'mm', vi.createByString('8 mm'))
    inputs.addValueInput('bore', '軸穴径(0=なし)', 'mm', vi.createByString('5 mm'))
    inputs.addValueInput('backlash', 'バックラッシ(歯厚減少量)', 'mm', vi.createByString('0.1 mm'))


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
