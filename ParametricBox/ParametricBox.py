# -*- coding: utf-8 -*-
"""ParametricBox - パラメトリックケースジェネレータ
外形寸法を入れるだけで、ネジボス付きの箱と皿ネジ用フタを生成します。
電子工作のエンクロージャなどに。M3ネジ想定(下穴2.5mm/バカ穴3.4mm)。
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

CMD_ID = 'jpParametricBox'
CMD_NAME = 'パラメトリックケースジェネレータ'
CMD_DESC = 'ネジボス付きの箱とフタを生成します'

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def _v(inputs, iid):
    return adsk.core.ValueCommandInput.cast(inputs.itemById(iid)).value

def _pt(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)

def _extrude(comp, prof, dist, op):
    exts = comp.features.extrudeFeatures
    ei = exts.createInput(prof, op)
    ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(dist))
    return exts.add(_scope(ei))

def _rect(sk, L, W, cx=0.0, cy=0.0):
    lines = sk.sketchCurves.sketchLines
    return lines.addTwoPointRectangle(_pt(cx - L / 2, cy - W / 2), _pt(cx + L / 2, cy + W / 2))

def _circle(sk, r, cx=0.0, cy=0.0):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(cx, cy), r)

def _plane(comp, offset):
    pls = comp.constructionPlanes
    pi = pls.createInput()
    pi.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(offset))
    return pls.add(pi)


def build(inputs):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    if not des:
        raise Exception('Fusionデザインをアクティブにしてください')
    _reset()
    L = _v(inputs, 'len')
    W = _v(inputs, 'wid')
    H = _v(inputs, 'hei')       # 外形全高(フタ含む)
    wall = _v(inputs, 'wall')
    floor_t = _v(inputs, 'floor')
    lid_t = _v(inputs, 'lid')
    boss_d = _v(inputs, 'bossd')
    pilot = _v(inputs, 'pilot')
    clear = _v(inputs, 'clear')

    Hb = H - lid_t              # 箱本体の高さ
    br = boss_d / 2.0
    off = wall + br             # ボス中心のコーナーからのオフセット
    bx = L / 2.0 - off
    by = W / 2.0 - off

    root = des.rootComponent

    # ---- 箱本体 ----
    box = _new_part(root, 'ケース本体 {0}x{1}x{2}'.format(
        round(L * 10, 1), round(W * 10, 1), round(H * 10, 1)))
    sk = box.sketches.add(box.xYConstructionPlane)
    _rect(sk, L, W)
    ext = _extrude(box, sk.profiles.item(0), Hb, NEW)
    body = ext.bodies.item(0)

    # シェル(上面を開口)
    top_face = None
    zmax = -1e9
    for i in range(body.faces.count):
        f = body.faces.item(i)
        pz = f.pointOnFace.z
        if pz > zmax:
            zmax = pz
            top_face = f
    faces = adsk.core.ObjectCollection.create()
    faces.add(top_face)
    shells = box.features.shellFeatures
    si = shells.createInput(faces, False)
    si.insideThickness = adsk.core.ValueInput.createByReal(wall)
    shells.add(si)

    # 底厚が壁厚と異なる場合の底上げ
    if floor_t > wall + 1e-6:
        pl = _plane(box, wall)
        skf = box.sketches.add(pl)
        _rect(skf, L - 2 * wall, W - 2 * wall)
        _extrude(box, skf.profiles.item(0), floor_t - wall, JOIN)

    # コーナーのネジボス
    for sx in (1, -1):
        for sy in (1, -1):
            skb = box.sketches.add(box.xYConstructionPlane)
            _circle(skb, br, sx * bx, sy * by)
            _extrude(box, skb.profiles.item(0), Hb, JOIN)
            # 下穴 (上から深さ 10mm)
            plt = _plane(box, Hb)
            skp = box.sketches.add(plt)
            _circle(skp, pilot / 2.0, sx * bx, sy * by)
            _extrude(box, skp.profiles.item(0), -min(1.0, Hb - floor_t), CUT)

    # ---- フタ (箱の隣に配置) ----
    tr = adsk.core.Matrix3D.create()
    tr.translation = adsk.core.Vector3D.create(L + 2.0, 0, 0)
    lid = _new_part(root, 'フタ', tr)
    skl = lid.sketches.add(lid.xYConstructionPlane)
    _rect(skl, L, W)
    _extrude(lid, skl.profiles.item(0), lid_t, NEW)
    # 内側リブ(位置決め用の落とし込み)
    pll = _plane(lid, 0.0)
    skr = lid.sketches.add(lid.xYConstructionPlane)
    _rect(skr, L - 2 * wall - 0.04, W - 2 * wall - 0.04)
    _extrude(lid, skr.profiles.item(0), -0.15, JOIN)
    # ネジのバカ穴
    for sx in (1, -1):
        for sy in (1, -1):
            skh = lid.sketches.add(lid.xYConstructionPlane)
            _circle(skh, clear / 2.0, sx * bx, sy * by)
            _extrude(lid, skh.profiles.item(0), lid_t, CUT)
            # リブとボスの干渉逃げ
            skv = lid.sketches.add(lid.xYConstructionPlane)
            _circle(skv, br + 0.03, sx * bx, sy * by)
            _extrude(lid, skv.profiles.item(0), -0.15, CUT)

    _finish('生成完了\n内寸: {0} x {1} x {2} mm\nM3タッピングネジ4本でフタを固定できます'.format(
        round((L - 2 * wall) * 10, 1), round((W - 2 * wall) * 10, 1),
        round((Hb - floor_t) * 10, 1)))


def make_inputs(inputs):
    vi = adsk.core.ValueInput
    inputs.addValueInput('len', '外形 長さ(X)', 'mm', vi.createByString('80 mm'))
    inputs.addValueInput('wid', '外形 幅(Y)', 'mm', vi.createByString('50 mm'))
    inputs.addValueInput('hei', '外形 高さ(フタ込み)', 'mm', vi.createByString('30 mm'))
    inputs.addValueInput('wall', '壁厚', 'mm', vi.createByString('2 mm'))
    inputs.addValueInput('floor', '底厚', 'mm', vi.createByString('2 mm'))
    inputs.addValueInput('lid', 'フタ厚', 'mm', vi.createByString('2.5 mm'))
    inputs.addValueInput('bossd', 'ネジボス直径', 'mm', vi.createByString('7 mm'))
    inputs.addValueInput('pilot', 'ネジ下穴径(M3=2.5)', 'mm', vi.createByString('2.5 mm'))
    inputs.addValueInput('clear', 'フタのバカ穴径(M3=3.4)', 'mm', vi.createByString('3.4 mm'))


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
