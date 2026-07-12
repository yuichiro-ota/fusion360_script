# -*- coding: utf-8 -*-
"""Sprocket - ローラーチェーン用スプロケットジェネレータ
チェーンピッチ・ローラー径・歯数から近似歯形のスプロケットを生成します。
例: #25(6.35/3.30), #35(9.525/5.08), #40(12.7/7.92), 自転車(12.7/7.75)
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

CMD_ID = 'jpSprocket'
CMD_NAME = 'スプロケットジェネレータ'
CMD_DESC = 'ローラーチェーン用スプロケットを生成します'

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation


def _v(inputs, iid):
    return adsk.core.ValueCommandInput.cast(inputs.itemById(iid)).value

def _i(inputs, iid):
    return adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById(iid)).value

def _pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)

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


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    p = _v(inputs, 'pitch')
    dr = _v(inputs, 'roller')
    z = _i(inputs, 'teeth')
    t = _v(inputs, 'thick')
    bore = _v(inputs, 'bore')
    hub = _v(inputs, 'hub')
    hubd = _v(inputs, 'hubd')

    R = p / (2.0 * math.sin(math.pi / z))            # ピッチ円半径
    OD = p * (0.6 + 1.0 / math.tan(math.pi / z))     # 外径(ANSI簡易式)
    seat_r = 0.505 * dr + 0.003                       # ローラー座半径(ANSIクリアランス込み)

    root = des.rootComponent
    comp = _new_part(root, 'スプロケット {0}T p{1}mm'.format(z, round(p * 10, 3)))

    # ブランク
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), OD / 2.0)
    _extrude(comp, sk.profiles.item(0), t, NEW)

    # ローラー座を円形カット
    sks = comp.sketches.add(comp.xYConstructionPlane)
    sks.sketchCurves.sketchCircles.addByCenterRadius(_pt(R, 0), seat_r)
    cf = _extrude(comp, sks.profiles.item(0), t, CUT)
    _pattern(comp, [cf], z)

    # ハブ
    total = t
    if hub > 1e-6:
        pls = comp.constructionPlanes
        pi = pls.createInput()
        pi.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(t))
        pl = pls.add(pi)
        skh = comp.sketches.add(pl)
        skh.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), hubd / 2.0)
        _extrude(comp, skh.profiles.item(0), hub, JOIN)
        total += hub

    # 軸穴
    if bore > 1e-6:
        skb = comp.sketches.add(comp.xYConstructionPlane)
        skb.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), bore / 2.0)
        _extrude(comp, skb.profiles.item(0), total, CUT)

    _finish('生成完了\nピッチ円直径 = {:.3f} mm\n外径 = {:.3f} mm'.format(R * 20, OD * 10))


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addValueInput('pitch', 'チェーンピッチ', 'mm', vi.createByString('12.7 mm'))
    inputs.addValueInput('roller', 'ローラー径', 'mm', vi.createByString('7.92 mm'))
    inputs.addIntegerSpinnerCommandInput('teeth', '歯数', 8, 150, 1, 16)
    inputs.addValueInput('thick', '歯板厚', 'mm', vi.createByString('7 mm'))
    inputs.addValueInput('bore', '軸穴径(0=なし)', 'mm', vi.createByString('10 mm'))
    inputs.addValueInput('hub', 'ハブ高さ(0=なし)', 'mm', vi.createByString('0 mm'))
    inputs.addValueInput('hubd', 'ハブ直径', 'mm', vi.createByString('25 mm'))


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
