# -*- coding: utf-8 -*-
"""CycloidalDrive - サイクロイダルドライブ(サイクロイド減速機)ジェネレータ
サイクロイドディスク・ピンハウジング・偏心カムシャフト・出力フランジを
組立位置に一括生成します。減速比 = ピン数 - 1。
"""
import adsk.core, adsk.fusion, traceback, math

_app = None
_ui = None
_handlers = []

CMD_ID = 'jpCycloidalDrive'
CMD_NAME = 'サイクロイダルドライブジェネレータ'
CMD_DESC = 'サイクロイド減速機一式を生成します'

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


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

def _circle(sk, r, cx=0.0, cy=0.0):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(cx, cy), r)

def _ring_profile(sk):
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        if p.profileLoops.count == 2:
            return p
    return sk.profiles.item(0)


def cycloid_point(R, Rr, E, N, t):
    """サイクロイドディスクの外形点(標準的なエピトロコイド等距離曲線)"""
    psi = math.atan2(math.sin((1 - N) * t), (R / (E * N)) - math.cos((1 - N) * t))
    x = R * math.cos(t) - Rr * math.cos(t + psi) - E * math.cos(N * t)
    y = -R * math.sin(t) + Rr * math.sin(t + psi) + E * math.sin(N * t)
    return (x, y)


def build_disc(comp, R, Rr, E, N, t, bearing_d, n_out, out_pin_d, r_out, clearance):
    npt = max(240, N * 30)
    col = adsk.core.ObjectCollection.create()
    first = None
    for i in range(npt):
        tt = 2.0 * math.pi * i / float(npt)
        x, y = cycloid_point(R, Rr + clearance, E, N, tt)
        p = _pt(x, y)
        if i == 0:
            first = p
        col.add(p)
    col.add(first)  # 始点に戻して閉じる
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchFittedSplines.add(col)
    _extrude(comp, sk.profiles.item(0), t, NEW)
    # 中心の偏心ベアリング穴
    if bearing_d > 1e-6:
        skb = comp.sketches.add(comp.xYConstructionPlane)
        _circle(skb, bearing_d / 2.0)
        _extrude(comp, skb.profiles.item(0), t, CUT)
    # 出力ピン穴 (径 = 出力ピン径 + 2E)
    if n_out > 0 and out_pin_d > 1e-6:
        sko = comp.sketches.add(comp.xYConstructionPlane)
        _circle(sko, (out_pin_d + 2.0 * E) / 2.0, r_out, 0.0)
        cf = _extrude(comp, sko.profiles.item(0), t, CUT)
        _pattern(comp, [cf], n_out)


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    N = _i(inputs, 'pins')
    R = _v(inputs, 'pinR')
    Rr = _v(inputs, 'pinr')
    E = _v(inputs, 'ecc')
    t = _v(inputs, 'thick')
    bearing_d = _v(inputs, 'bearing')
    n_out = _i(inputs, 'nout')
    out_pin_d = _v(inputs, 'outd')
    r_out = _v(inputs, 'outR')
    two = _bool(inputs, 'two')
    ds = _v(inputs, 'shaft')
    margin = _v(inputs, 'margin')
    clearance = _v(inputs, 'clr')

    if E * N >= R:
        _ui.messageBox('警告: 偏心量が大きすぎます (E×N < R を推奨)。歯形に尖りが出る可能性があります。')
    if R - Rr - E <= r_out + (out_pin_d + 2 * E) / 2.0:
        _ui.messageBox('警告: 出力穴がディスク外形に近すぎる可能性があります。')

    root = des.rootComponent
    n_disc = 2 if two else 1
    t_total = t * n_disc

    # ---- サイクロイドディスク(偏心位置に配置) ----
    tr1 = adsk.core.Matrix3D.create()
    tr1.translation = adsk.core.Vector3D.create(E, 0, 0)
    d1 = _new_part(root, 'サイクロイドディスク1 ({0}枚歯)'.format(N - 1), tr1)
    build_disc(d1, R, Rr, E, N, t, bearing_d, n_out, out_pin_d, r_out, clearance)

    if two:
        tr2 = adsk.core.Matrix3D.create()
        tr2.setToRotation(math.pi, adsk.core.Vector3D.create(0, 0, 1), _pt(0, 0, 0))
        tr2.translation = adsk.core.Vector3D.create(-E, 0, t)
        d2 = _new_part(root, 'サイクロイドディスク2 ({0}枚歯)'.format(N - 1), tr2)
        build_disc(d2, R, Rr, E, N, t, bearing_d, n_out, out_pin_d, r_out, clearance)

    # ---- ピンハウジング ----
    hz = _new_part(root, 'ピンハウジング ({0}ピン)'.format(N))
    skh = hz.sketches.add(hz.xYConstructionPlane)
    _circle(skh, R + Rr + margin)
    _circle(skh, R)
    _extrude(hz, _ring_profile(skh), t_total, NEW)
    skp = hz.sketches.add(hz.xYConstructionPlane)
    _circle(skp, Rr, R, 0.0)
    pf = _extrude(hz, skp.profiles.item(0), t_total, JOIN)
    _pattern(hz, [pf], N)

    # ---- 偏心カムシャフト ----
    cam = _new_part(root, '偏心カムシャフト')
    pl0 = _plane(cam, -1.0)
    sks = cam.sketches.add(pl0)
    _circle(sks, ds / 2.0)
    _extrude(cam, sks.profiles.item(0), t_total + 2.0, NEW)
    skc1 = cam.sketches.add(cam.xYConstructionPlane)
    _circle(skc1, bearing_d / 2.0 - 0.02, E, 0.0)   # ベアリング内輪相当(印刷なら要調整)
    _extrude(cam, skc1.profiles.item(0), t, JOIN)
    if two:
        plc = _plane(cam, t)
        skc2 = cam.sketches.add(plc)
        _circle(skc2, bearing_d / 2.0 - 0.02, -E, 0.0)
        _extrude(cam, skc2.profiles.item(0), t, JOIN)

    # ---- 出力フランジ ----
    if n_out > 0 and out_pin_d > 1e-6:
        trf = adsk.core.Matrix3D.create()
        trf.translation = adsk.core.Vector3D.create(0, 0, t_total)
        fl = _new_part(root, '出力フランジ', trf)
        skf = fl.sketches.add(fl.xYConstructionPlane)
        _circle(skf, r_out + out_pin_d)
        _extrude(fl, skf.profiles.item(0), 0.3, NEW)
        skfp = fl.sketches.add(fl.xYConstructionPlane)
        _circle(skfp, out_pin_d / 2.0, r_out, 0.0)
        ff = _extrude(fl, skfp.profiles.item(0), -t_total, JOIN)
        _pattern(fl, [ff], n_out)
        skfb = fl.sketches.add(fl.xYConstructionPlane)
        _circle(skfb, ds / 2.0 + 0.05)
        _extrude(fl, skfb.profiles.item(0), 0.3, CUT)

    _finish('生成完了\n減速比 = {0}:1 (入力:カム, 出力:フランジ, ハウジング固定)\n'
            'ディスク歯数 = {1}'.format(N - 1, N - 1))


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addIntegerSpinnerCommandInput('pins', 'ピン数 N (減速比=N-1)', 4, 60, 1, 11)
    inputs.addValueInput('pinR', 'ピン配置円半径 R', 'mm', vi.createByString('25 mm'))
    inputs.addValueInput('pinr', 'ピン半径 Rr', 'mm', vi.createByString('2.5 mm'))
    inputs.addValueInput('ecc', '偏心量 E', 'mm', vi.createByString('1 mm'))
    inputs.addValueInput('thick', 'ディスク厚', 'mm', vi.createByString('5 mm'))
    inputs.addValueInput('bearing', 'ディスク中心穴径(ベアリング外径)', 'mm', vi.createByString('16 mm'))
    inputs.addIntegerSpinnerCommandInput('nout', '出力ピン本数', 0, 12, 1, 4)
    inputs.addValueInput('outd', '出力ピン径', 'mm', vi.createByString('5 mm'))
    inputs.addValueInput('outR', '出力ピン配置円半径', 'mm', vi.createByString('13 mm'))
    inputs.addBoolValueInput('two', 'ディスク2枚(180°位相)にする', True, '', True)
    inputs.addValueInput('shaft', '入力シャフト径', 'mm', vi.createByString('6 mm'))
    inputs.addValueInput('margin', 'ハウジング外周マージン', 'mm', vi.createByString('4 mm'))
    inputs.addValueInput('clr', '歯面クリアランス(印刷用)', 'mm', vi.createByString('0.1 mm'))


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
