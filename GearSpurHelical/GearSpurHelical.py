# -*- coding: utf-8 -*-
"""GearSpurHelical - 平歯車 / はすば歯車 / 内歯車ジェネレータ
モジュール・歯数などを入力するだけでインボリュート歯車ボディを生成します。
Fusion 360 スクリプト (ユーティリティ > アドイン > スクリプト から実行)
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

CMD_ID = 'jpGearSpurHelical'
CMD_NAME = '平/はすば/内歯車ジェネレータ'
CMD_DESC = 'インボリュート歯車を生成します'

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


# ---------- 共通ヘルパ ----------
def _v(inputs, iid):
    return adsk.core.ValueCommandInput.cast(inputs.itemById(iid)).value

def _i(inputs, iid):
    return adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById(iid)).value

def _bool(inputs, iid):
    return adsk.core.BoolValueCommandInput.cast(inputs.itemById(iid)).value

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

def _pattern(comp, feats, n):
    col = adsk.core.ObjectCollection.create()
    for f in feats:
        col.add(f)
    cps = comp.features.circularPatternFeatures
    ci = cps.createInput(col, comp.zConstructionAxis)
    ci.quantity = adsk.core.ValueInput.createByString(str(int(n)))
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
    """1歯分の閉じたインボリュート輪郭を描く (単位: cm, 角度: rad)"""
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


# ---------- 生成本体 ----------
def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    m = _v(inputs, 'module')
    z = _i(inputs, 'teeth')
    alpha = _v(inputs, 'pa')
    beta = _v(inputs, 'helix')
    w = _v(inputs, 'width')
    bore = _v(inputs, 'bore')
    bl = _v(inputs, 'backlash')
    internal = _bool(inputs, 'internal')
    rim = _v(inputs, 'rim')

    r = m * z / 2.0
    comp = _new_comp('{0} m{1} z{2}'.format(
        '内歯車' if internal else ('はすば歯車' if abs(beta) > 1e-6 else '平歯車'),
        round(m * 10, 3), z))

    if internal:
        # リング本体 (外径 = 歯底円 + リム)
        ra_i = r - m            # 内歯の歯先(内側)
        rf_i = r + 1.25 * m     # 内歯の歯底(外側)
        sk = comp.sketches.add(comp.xYConstructionPlane)
        _circle(sk, rf_i + rim)
        _circle(sk, ra_i)
        _extrude(comp, _ring_profile(sk), w, NEW)
        # 歯溝を外歯形状でカット
        sk1 = comp.sketches.add(comp.xYConstructionPlane)
        draw_tooth(sk1, m, z, alpha, backlash=-bl, ra=rf_i, rf=ra_i - 0.15 * m)
        cf = _extrude(comp, sk1.profiles.item(0), w, CUT)
        _pattern(comp, [cf], z)
        _finish()
        return

    ra = r + m
    rf = r - 1.25 * m
    # 歯底円筒
    sk = comp.sketches.add(comp.xYConstructionPlane)
    _circle(sk, rf)
    _extrude(comp, sk.profiles.item(0), w, NEW)
    # 歯
    sk1 = comp.sketches.add(comp.xYConstructionPlane)
    draw_tooth(sk1, m, z, alpha, bl)
    if abs(beta) < 1e-6:
        tf = _extrude(comp, sk1.profiles.item(0), w, JOIN)
    else:
        twist = w * math.tan(beta) / r
        pl = _plane(comp, w)
        sk2 = comp.sketches.add(pl)
        draw_tooth(sk2, m, z, alpha, bl, rot=twist)
        lofts = comp.features.loftFeatures
        li = lofts.createInput(JOIN)
        li.loftSections.add(sk1.profiles.item(0))
        li.loftSections.add(sk2.profiles.item(0))
        tf = lofts.add(_scope(li))
    _pattern(comp, [tf], z)
    # 軸穴
    if bore > 1e-6:
        skb = comp.sketches.add(comp.xYConstructionPlane)
        _circle(skb, bore / 2.0)
        _extrude(comp, skb.profiles.item(0), w, CUT)

    _finish()


# ---------- ダイアログ ----------
def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addValueInput('module', 'モジュール', 'mm', vi.createByString('2 mm'))
    inputs.addIntegerSpinnerCommandInput('teeth', '歯数', 6, 300, 1, 20)
    inputs.addValueInput('pa', '圧力角', 'deg', vi.createByString('20 deg'))
    inputs.addValueInput('helix', 'ねじれ角(0=平歯車)', 'deg', vi.createByString('0 deg'))
    inputs.addValueInput('width', '歯幅', 'mm', vi.createByString('10 mm'))
    inputs.addValueInput('bore', '軸穴径(0=なし)', 'mm', vi.createByString('5 mm'))
    inputs.addValueInput('backlash', 'バックラッシ(歯厚減少量)', 'mm', vi.createByString('0.1 mm'))
    inputs.addBoolValueInput('internal', '内歯車として生成', True, '', False)
    inputs.addValueInput('rim', 'リム厚(内歯車のみ)', 'mm', vi.createByString('3 mm'))


# ---------- 定型イベント処理 ----------
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
