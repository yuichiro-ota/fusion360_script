# -*- coding: utf-8 -*-
"""GearRack - ラックギアジェネレータ
モジュール・歯数・歯幅を入力するだけでラックのボディを生成します。
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

CMD_ID = 'jpGearRack'
CMD_NAME = 'ラックギアジェネレータ'
CMD_DESC = 'ラック(直線歯)を生成します'

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def _v(inputs, iid):
    return adsk.core.ValueCommandInput.cast(inputs.itemById(iid)).value

def _i(inputs, iid):
    return adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById(iid)).value

def _pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)

def _new_comp(name):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    return _new_part(des.rootComponent, name)


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    m = _v(inputs, 'module')
    n = _i(inputs, 'teeth')
    alpha = _v(inputs, 'pa')
    w = _v(inputs, 'width')
    hb = _v(inputs, 'base')
    bl = _v(inputs, 'backlash')

    p = math.pi * m          # ピッチ
    ha = m                   # 歯末のたけ
    hf = 1.25 * m            # 歯元のたけ
    y_root = hb              # 歯底の高さ
    y_tip = hb + ha + hf     # 歯先の高さ
    L = n * p

    # 歯厚 (ピッチ線上) = p/2 - バックラッシ
    half_p = (p / 2.0 - bl) / 2.0            # ピッチ線での半歯厚
    wt = half_p - ha * math.tan(alpha)       # 歯先での半歯厚
    wr = half_p + hf * math.tan(alpha)       # 歯底での半歯厚

    comp = _new_comp('ラック m{0} x{1}歯'.format(round(m * 10, 3), n))
    sk = comp.sketches.add(comp.xYConstructionPlane)
    pts = [(0.0, 0.0), (0.0, y_root)]
    for i in range(n):
        xc = p / 2.0 + i * p
        pts.append((xc - wr, y_root))
        pts.append((xc - wt, y_tip))
        pts.append((xc + wt, y_tip))
        pts.append((xc + wr, y_root))
    pts.append((L, y_root))
    pts.append((L, 0.0))

    lines = sk.sketchCurves.sketchLines
    first = None
    prev = None
    for i in range(len(pts) - 1):
        a = _pt(*pts[i]); b = _pt(*pts[i + 1])
        if prev is None:
            ln = lines.addByTwoPoints(a, b)
            first = ln
        else:
            ln = lines.addByTwoPoints(prev.endSketchPoint, b)
        prev = ln
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)

    ext = comp.features.extrudeFeatures
    ei = ext.createInput(sk.profiles.item(0), NEW)
    ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(w))
    ext.add(_scope(ei))

    _finish()


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addValueInput('module', 'モジュール', 'mm', vi.createByString('2 mm'))
    inputs.addIntegerSpinnerCommandInput('teeth', '歯数(長さ=歯数xπm)', 2, 500, 1, 20)
    inputs.addValueInput('pa', '圧力角', 'deg', vi.createByString('20 deg'))
    inputs.addValueInput('width', '歯幅', 'mm', vi.createByString('10 mm'))
    inputs.addValueInput('base', '土台の高さ(歯底まで)', 'mm', vi.createByString('5 mm'))
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
