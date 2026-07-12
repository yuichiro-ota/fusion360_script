# -*- coding: utf-8 -*-
"""TimingPulleyGT2 - GT2タイミングプーリージェネレータ
歯数・ベルト幅・軸穴径を入力するだけでGT2(2mmピッチ)プーリーを生成します。
3Dプリンタ・CNC工作向けの近似歯形です。
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

CMD_ID = 'jpTimingPulleyGT2'
CMD_NAME = 'GT2プーリージェネレータ'
CMD_DESC = 'GT2タイミングプーリーを生成します'

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def _v(inputs, iid):
    return adsk.core.ValueCommandInput.cast(inputs.itemById(iid)).value

def _i(inputs, iid):
    return adsk.core.IntegerSpinnerCommandInput.cast(inputs.itemById(iid)).value

def _bool(inputs, iid):
    return adsk.core.BoolValueCommandInput.cast(inputs.itemById(iid)).value

def _pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)

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


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    z = _i(inputs, 'teeth')
    w = _v(inputs, 'width')
    bore = _v(inputs, 'bore')
    flange = _bool(inputs, 'flange')
    hub = _v(inputs, 'hub')          # ハブ高さ(0=なし)
    hubd = _v(inputs, 'hubd')

    pitch = 0.2                       # GT2 = 2mm ピッチ (cm)
    PLD = 0.0254                      # ピッチライン差 0.254mm
    pd = pitch * z / math.pi          # ピッチ円直径
    odr = pd / 2.0 - PLD              # 外半径
    tf = 0.1                          # フランジ厚 1mm
    rfl = odr + 0.15                  # フランジ半径 (+1.5mm)
    groove_r = 0.075                  # 溝アール 0.75mm (近似)

    root = des.rootComponent
    comp = _new_part(root, 'GT2プーリー {0}T'.format(z))

    z0 = 0.0
    # 下フランジ
    if flange:
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), rfl)
        _extrude(comp, sk.profiles.item(0), tf, NEW)
        z0 = tf
    # 本体
    plb = _plane(comp, z0)
    skb = comp.sketches.add(plb)
    skb.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), odr)
    _extrude(comp, skb.profiles.item(0), w, NEW if not flange else adsk.fusion.FeatureOperations.JoinFeatureOperation)
    # 上フランジ
    if flange:
        plt = _plane(comp, z0 + w)
        skt = comp.sketches.add(plt)
        skt.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), rfl)
        _extrude(comp, skt.profiles.item(0), tf, adsk.fusion.FeatureOperations.JoinFeatureOperation)
    # ハブ
    top = z0 + w + (tf if flange else 0.0)
    if hub > 1e-6:
        plh = _plane(comp, top)
        skh = comp.sketches.add(plh)
        skh.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), hubd / 2.0)
        _extrude(comp, skh.profiles.item(0), hub, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        top += hub
    # 歯溝カット
    plg = _plane(comp, z0)
    skg = comp.sketches.add(plg)
    skg.sketchCurves.sketchCircles.addByCenterRadius(_pt(odr, 0), groove_r)
    gf = _extrude(comp, skg.profiles.item(0), w, CUT)
    _pattern(comp, [gf], z)
    # 軸穴
    if bore > 1e-6:
        skx = comp.sketches.add(comp.xYConstructionPlane)
        skx.sketchCurves.sketchCircles.addByCenterRadius(_pt(0, 0), bore / 2.0)
        _extrude(comp, skx.profiles.item(0), top, CUT)

    _finish('生成完了\nピッチ円直径 = {:.3f} mm\n外径 = {:.3f} mm'.format(pd * 10, odr * 20))


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addIntegerSpinnerCommandInput('teeth', '歯数', 10, 200, 1, 20)
    inputs.addValueInput('width', 'ベルト溝幅', 'mm', vi.createByString('7 mm'))
    inputs.addValueInput('bore', '軸穴径(0=なし)', 'mm', vi.createByString('5 mm'))
    inputs.addBoolValueInput('flange', 'フランジ付き', True, '', True)
    inputs.addValueInput('hub', 'ハブ高さ(0=なし)', 'mm', vi.createByString('6 mm'))
    inputs.addValueInput('hubd', 'ハブ直径', 'mm', vi.createByString('14 mm'))


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
