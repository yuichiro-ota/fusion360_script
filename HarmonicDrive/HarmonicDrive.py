# -*- coding: utf-8 -*-
"""HarmonicDrive - ハーモニックドライブ(波動歯車装置)ジェネレータ
フレクスプライン(カップ+外歯)・サーキュラスプライン(内歯リング)・
ウェーブジェネレータ(楕円カム)の3点を生成します。
減速比 = フレクスプライン歯数 / 2。
※ フレクスプラインは無変形状態で生成されます(弾性変形は表現されません)。
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

CMD_ID = 'jpHarmonicDrive'
CMD_NAME = 'ハーモニックドライブジェネレータ'
CMD_DESC = '波動歯車装置一式を生成します'

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

    def flank(sign, n=10):
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


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    m = _v(inputs, 'module')
    zf = _i(inputs, 'teeth')
    alpha = _v(inputs, 'pa')
    wt = _v(inputs, 'wall')
    H = _v(inputs, 'height')
    ht = _v(inputs, 'band')
    tb = _v(inputs, 'base')
    bh = _v(inputs, 'boreb')
    bw = _v(inputs, 'borew')
    rim = _v(inputs, 'rim')
    bl = _v(inputs, 'backlash')

    if zf % 2 != 0:
        zf += 1
        _ui.messageBox('フレクスプライン歯数は偶数が必要なため {0} に変更しました'.format(zf))
    zc = zf + 2
    rf_pitch = m * zf / 2.0
    rr_f = rf_pitch - 1.25 * m       # フレクスプライン歯底半径 = 壁外径
    wall_in = rr_f - wt
    rc = m * zc / 2.0

    root = des.rootComponent

    # ---- フレクスプライン (カップ) ----
    fx = _new_part(root, 'フレクスプライン z{0}'.format(zf))
    sk = fx.sketches.add(fx.xYConstructionPlane)
    _circle(sk, rr_f)
    _circle(sk, wall_in)
    _extrude(fx, _ring_profile(sk), H, NEW)
    skb = fx.sketches.add(fx.xYConstructionPlane)
    _circle(skb, rr_f)
    _extrude(fx, skb.profiles.item(0), tb, JOIN)
    if bh > 1e-6:
        skh = fx.sketches.add(fx.xYConstructionPlane)
        _circle(skh, bh / 2.0)
        _extrude(fx, skh.profiles.item(0), tb, CUT)
    # 上端の歯帯
    plt = _plane(fx, H - ht)
    skt = fx.sketches.add(plt)
    draw_tooth(skt, m, zf, alpha, bl)
    tf = _extrude(fx, skt.profiles.item(0), ht, JOIN)
    _pattern(fx, [tf], zf)

    # ---- サーキュラスプライン (内歯リング) ----
    cs = _new_part(root, 'サーキュラスプライン z{0}'.format(zc))
    ra_i = rc - m
    rf_i = rc + 1.25 * m
    plc = _plane(cs, H - ht)
    skc = cs.sketches.add(plc)
    _circle(skc, rf_i + rim)
    _circle(skc, ra_i)
    _extrude(cs, _ring_profile(skc), ht, NEW)
    skcut = cs.sketches.add(plc)
    draw_tooth(skcut, m, zc, alpha, backlash=-bl, ra=rf_i, rf=ra_i - 0.15 * m)
    cf = _extrude(cs, skcut.profiles.item(0), ht, CUT)
    _pattern(cs, [cf], zc)

    # ---- ウェーブジェネレータ (楕円カム) ----
    wg = _new_part(root, 'ウェーブジェネレータ')
    defl = m  # 半径方向のたわみ量 ≒ モジュール
    aa = wall_in + defl
    bb = wall_in - defl
    plw = _plane(wg, H - ht)
    skw = wg.sketches.add(plw)
    skw.sketchCurves.sketchEllipses.add(_pt(0, 0), _pt(aa, 0), _pt(0, bb))
    _extrude(wg, skw.profiles.item(0), ht, NEW)
    if bw > 1e-6:
        skwb = wg.sketches.add(plw)
        _circle(skwb, bw / 2.0)
        _extrude(wg, skwb.profiles.item(0), ht, CUT)

    _finish('生成完了\nサーキュラスプライン歯数 = {0}\n'
            '減速比 = {1}:1 (WG入力・FS出力・CS固定)\n'
            '※フレクスプラインは無変形状態です。実動作には薄肉壁の弾性変形が必要で、\n'
            '  3Dプリントの場合はTPU等の柔軟材や壁厚調整をご検討ください。'.format(zc, zf // 2))


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addValueInput('module', 'モジュール', 'mm', vi.createByString('0.5 mm'))
    inputs.addIntegerSpinnerCommandInput('teeth', 'フレクスプライン歯数(偶数)', 30, 400, 2, 100)
    inputs.addValueInput('pa', '圧力角', 'deg', vi.createByString('20 deg'))
    inputs.addValueInput('wall', 'カップ壁厚', 'mm', vi.createByString('1.2 mm'))
    inputs.addValueInput('height', 'カップ高さ', 'mm', vi.createByString('20 mm'))
    inputs.addValueInput('band', '歯帯の幅(上端から)', 'mm', vi.createByString('6 mm'))
    inputs.addValueInput('base', 'カップ底厚', 'mm', vi.createByString('2 mm'))
    inputs.addValueInput('boreb', 'カップ底の穴径(0=なし)', 'mm', vi.createByString('5 mm'))
    inputs.addValueInput('borew', 'ウェーブジェネレータ軸穴径', 'mm', vi.createByString('5 mm'))
    inputs.addValueInput('rim', 'サーキュラスプラインのリム厚', 'mm', vi.createByString('4 mm'))
    inputs.addValueInput('backlash', 'バックラッシ(歯厚減少量)', 'mm', vi.createByString('0.05 mm'))


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
