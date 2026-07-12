# -*- coding: utf-8 -*-
# ==========================================================================
#  RobotLinkages.py  —  Fusion 360 スクリプト
#
#  ロボットの脚・歩行機構によく使うリンク機構をパラメータ指定で自動生成します。
#    1. テオ・ヤンセン機構(Strandbeest の脚)
#    2. ヘッケンリンク機構(Hoecken: 近似直線運動)
#    3. チェビシェフリンク機構(Chebyshev: 近似直線運動)
#    4. 平行リンク機構(ロボット膝用パンタグラフ)
#
#  指定した入力角(クランク角)の姿勢でリンク板を計算し、
#  各リンクを個別コンポーネント(丸端バー/三角プレート+ピン穴)として、
#  Z方向に1枚ずつ積層した状態で生成します。
#  そのまま3Dプリント・レーザーカットするか、生成後に回転ジョイントで
#  組み立ててモーションスタディに使えます。
#
#  ■ インストール方法
#    1. "RobotLinkages" という名前のフォルダを作りこのファイルを入れる
#       (フォルダ名とファイル名を一致させる)
#    2. Fusion 360「ユーティリティ」→「アドイン」→「スクリプトとアドイン」
#    3. スクリプトタブの「+」でフォルダを指定して「実行」
# ==========================================================================

import adsk.core
import adsk.fusion
import traceback
import math


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

CMD_ID = 'robotLinkageGenerator'
CMD_NAME = 'リンク機構ジェネレータ'
CMD_DESC = 'ヤンセン / ヘッケン / チェビシェフ / 平行リンク(膝) をパラメータ指定で生成'

CLEARANCE = 0.02   # 積層時のリンク間クリアランス [cm] (0.2mm)


# --------------------------------------------------------------------------
# 幾何ユーティリティ
# --------------------------------------------------------------------------
def pt(p, z=0.0):
    return adsk.core.Point3D.create(p[0], p[1], z)


def circle_intersect(c1, r1, c2, r2, pick):
    """2円の交点を求め、pick(候補リスト)で1つ選ぶ。解なしはエラー。"""
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    if d > r1 + r2 or d < abs(r1 - r2) or d == 0:
        raise RuntimeError(
            'この入力角ではリンクが組み立てられません。入力角を変えてください。')
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = math.sqrt(max(r1 * r1 - a * a, 0.0))
    mx, my = c1[0] + a * dx / d, c1[1] + a * dy / d
    px, py = -dy / d * h, dx / d * h
    return pick([(mx + px, my + py), (mx - px, my - py)])


def get_design():
    design = adsk.fusion.Design.cast(_app.activeProduct)
    if not design:
        raise RuntimeError('デザインを開いた状態で実行してください。')
    return design


def new_component(parent, name):
    p = parent.comp if isinstance(parent, _Part) else parent
    return _new_part(p, name)


def add_bar_profile(sketch, p1, p2, half_w):
    """p1-p2 間の丸端バー外形(閉ループ)をスケッチに描く"""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        raise RuntimeError('長さゼロのリンクは作れません。')
    ux, uy = dx / length, dy / length
    nx, ny = -uy, ux
    a1s = pt((p1[0] + nx * half_w, p1[1] + ny * half_w))
    a2s = pt((p2[0] - nx * half_w, p2[1] - ny * half_w))
    a1e = pt((p1[0] - nx * half_w, p1[1] - ny * half_w))
    a2e = pt((p2[0] + nx * half_w, p2[1] + ny * half_w))
    arcs = sketch.sketchCurves.sketchArcs
    lines = sketch.sketchCurves.sketchLines
    arcs.addByCenterStartSweep(pt(p1), a1s, math.pi)   # p1側キャップ
    arcs.addByCenterStartSweep(pt(p2), a2s, math.pi)   # p2側キャップ
    lines.addByTwoPoints(a1e, a2s)
    lines.addByTwoPoints(a2e, a1s)


def make_link(comp, segments, holes, half_w, thick, z0, hole_r):
    """
    segments : [(p1, p2), ...]  1本ならバー、複数なら結合して板(三角プレート等)
    holes    : ピン穴を開ける点のリスト
    z0       : このリンクを置く高さ(積層レイヤ)
    """
    ext_feats = comp.features.extrudeFeatures
    body = None
    for (p1, p2) in segments:
        sk = comp.sketches.add(comp.xYConstructionPlane)
        add_bar_profile(sk, p1, p2, half_w)
        op = (adsk.fusion.FeatureOperations.NewBodyFeatureOperation if body is None
              else adsk.fusion.FeatureOperations.JoinFeatureOperation)
        ein = ext_feats.createInput(sk.profiles.item(0), op)
        ein.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(z0))
        ein.setDistanceExtent(False, adsk.core.ValueInput.createByReal(thick))
        if body is not None:
            ein.participantBodies = [body]
        feat = ext_feats.add(ein)
        if body is None:
            body = feat.bodies.item(0)

    if hole_r > 1e-6 and holes:
        sk = comp.sketches.add(comp.xYConstructionPlane)
        for p in holes:
            sk.sketchCurves.sketchCircles.addByCenterRadius(pt(p), hole_r)
        profs = adsk.core.ObjectCollection.create()
        for idx in range(sk.profiles.count):
            profs.add(sk.profiles.item(idx))
        ein = ext_feats.createInput(
            profs, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ein.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(z0 - 0.05))
        ein.setDistanceExtent(False,
                              adsk.core.ValueInput.createByReal(thick + 0.1))
        ein.participantBodies = [body]
        ext_feats.add(ein)
    return body


def build_pieces(mech_name, pieces, half_w, thick, hole_r):
    """pieces = [(名前, segments, holes), ...] をレイヤ積層で生成"""
    design = get_design()
    parent = new_component(design.rootComponent, mech_name)
    step = thick + CLEARANCE
    for layer, (name, segments, holes) in enumerate(pieces):
        comp = new_component(parent, name)
        make_link(comp, segments, holes, half_w, thick, layer * step, hole_r)
    return parent


# --------------------------------------------------------------------------
# 1. テオ・ヤンセン機構
# --------------------------------------------------------------------------
# 標準の "Holy Numbers" [mm]
JANSEN = dict(a=38.0, b=41.5, c=39.3, d=40.1, e=55.8, f=39.4, g=36.7,
              h=65.7, i=49.0, j=50.0, k=61.9, l=7.8, m=15.0)


def build_jansen(scale, crank_rad, half_w, thick, hole_r):
    """scale=1.0 で標準寸法(クランク15mm・脚全高 約110mm)"""
    s = 0.1 * scale                       # mm → cm × 倍率
    J = {kk: vv * s for kk, vv in JANSEN.items()}

    O = (0.0, 0.0)                                       # クランク軸(固定)
    F = (-J['a'], -J['l'])                               # 固定ピボット
    C = (J['m'] * math.cos(crank_rad), J['m'] * math.sin(crank_rad))

    upper = lambda ps: max(ps, key=lambda p: p[1])
    lower = lambda ps: min(ps, key=lambda p: p[1])
    left = lambda ps: min(ps, key=lambda p: p[0])

    B1 = circle_intersect(F, J['b'], C, J['j'], upper)   # 上部ノード
    B2 = circle_intersect(F, J['c'], C, J['k'], lower)   # 下部ノード
    B3 = circle_intersect(F, J['d'], B1, J['e'], left)   # 上三角の左頂点
    B4 = circle_intersect(B3, J['f'], B2, J['g'], left)  # 下三角の上頂点
    B5 = circle_intersect(B4, J['h'], B2, J['i'], lower) # 足先

    pieces = [
        ('Frame_a-l',        [(O, F)],                     [O, F]),
        ('Crank_m',          [(O, C)],                     [O, C]),
        ('Link_j',           [(C, B1)],                    [C, B1]),
        ('Link_k',           [(C, B2)],                    [C, B2]),
        ('Link_c',           [(F, B2)],                    [F, B2]),
        ('TriUpper_b-d-e',   [(F, B1), (B1, B3), (B3, F)], [F, B1, B3]),
        ('Link_f',           [(B3, B4)],                   [B3, B4]),
        ('TriLower_g-h-i',   [(B2, B4), (B4, B5), (B5, B2)], [B2, B4]),
    ]
    build_pieces('JansenLeg x{:.2f}'.format(scale), pieces, half_w, thick, hole_r)


# --------------------------------------------------------------------------
# 2. ヘッケンリンク機構(近似直線運動)
# --------------------------------------------------------------------------
def build_hoecken(a_cm, crank_rad, half_w, thick, hole_r):
    """
    比率: フレーム2a / クランクa / カプラ2.5a+延長2.5a / ロッカー2.5a
    先端Cはクランク角 約90°〜270° の間で高さ4a のほぼ直線を描く。
    """
    O2 = (0.0, 0.0)
    O4 = (2.0 * a_cm, 0.0)
    A = (a_cm * math.cos(crank_rad), a_cm * math.sin(crank_rad))
    upper = lambda ps: max(ps, key=lambda p: p[1])
    B = circle_intersect(A, 2.5 * a_cm, O4, 2.5 * a_cm, upper)
    C = (2 * B[0] - A[0], 2 * B[1] - A[1])               # 軌道点(ABの延長)

    pieces = [
        ('Frame',        [(O2, O4)], [O2, O4]),
        ('Crank',        [(O2, A)],  [O2, A]),
        ('Rocker',       [(O4, B)],  [O4, B]),
        ('Coupler_AC',   [(A, C)],   [A, B, C]),          # 中間穴Bで ロッカーと結合
    ]
    build_pieces('HoeckenLinkage', pieces, half_w, thick, hole_r)


# --------------------------------------------------------------------------
# 3. チェビシェフリンク機構(近似直線運動)
# --------------------------------------------------------------------------
def build_chebyshev(a_cm, input_rad, half_w, thick, hole_r):
    """
    比率: フレーム4a / 左右リンク5a / カプラ2a(交差配置)
    カプラ中点Pが高さ4a のほぼ直線を描く。入力角の有効範囲は約40°〜100°。
    """
    O2 = (0.0, 0.0)
    O4 = (4.0 * a_cm, 0.0)
    A = (5.0 * a_cm * math.cos(input_rad), 5.0 * a_cm * math.sin(input_rad))
    lower = lambda ps: min(ps, key=lambda p: p[1])       # 交差配置の枝
    B = circle_intersect(A, 2.0 * a_cm, O4, 5.0 * a_cm, lower)
    P = ((A[0] + B[0]) / 2.0, (A[1] + B[1]) / 2.0)       # 軌道点(中点)

    pieces = [
        ('Frame',      [(O2, O4)], [O2, O4]),
        ('Link_L',     [(O2, A)],  [O2, A]),
        ('Link_R',     [(O4, B)],  [O4, B]),
        ('Coupler_AB', [(A, B)],   [A, B, P]),            # 中央穴Pが軌道点
    ]
    build_pieces('ChebyshevLinkage', pieces, half_w, thick, hole_r)


# --------------------------------------------------------------------------
# 4. 平行リンク機構(ロボット膝)
# --------------------------------------------------------------------------
def build_knee(link_len, pivot_gap, knee_rad, shank_len, half_w, thick, hole_r):
    """
    腰側プレートの2ピボットから等長リンク2本を平行に伸ばし、
    すね側プレートの姿勢を保ったまま揺動する平行四辺形リンク。
    knee_rad は鉛直下向きからの振り角。
    """
    P1 = (0.0, 0.0)
    P2 = (0.0, -pivot_gap)
    ux, uy = math.sin(knee_rad), -math.cos(knee_rad)     # リンク方向
    Q1 = (P1[0] + link_len * ux, P1[1] + link_len * uy)
    Q2 = (P2[0] + link_len * ux, P2[1] + link_len * uy)
    T = (Q2[0], Q2[1] - shank_len)                        # すね先端(姿勢は常に鉛直)

    pieces = [
        ('HipPlate',   [(P1, P2)], [P1, P2]),
        ('LinkUpper',  [(P1, Q1)], [P1, Q1]),
        ('LinkLower',  [(P2, Q2)], [P2, Q2]),
        ('ShankPlate', [(Q1, T)],  [Q1, Q2]),             # Q1-Q2-T は一直線
    ]
    build_pieces('ParallelKnee', pieces, half_w, thick, hole_r)


# --------------------------------------------------------------------------
# コマンドUI
# --------------------------------------------------------------------------
class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.setDialogInitialSize(400, 460)
            inputs = cmd.commandInputs

            dd = inputs.addDropDownCommandInput(
                'mechType', '機構の種類',
                adsk.core.DropDownStyles.TextListDropDownStyle)
            dd.listItems.add('テオ・ヤンセン機構', True)
            dd.listItems.add('ヘッケンリンク機構', False)
            dd.listItems.add('チェビシェフリンク機構', False)
            dd.listItems.add('平行リンク機構 (膝)', False)

            # --- 共通 ---
            cg = inputs.addGroupCommandInput('commonGroup', '共通パラメータ')
            ci = cg.children
            ci.addValueInput('linkThk', 'リンク板厚', 'mm',
                             adsk.core.ValueInput.createByString('3 mm'))
            ci.addValueInput('linkW', 'リンク幅', 'mm',
                             adsk.core.ValueInput.createByString('8 mm'))
            ci.addValueInput('holeDia', 'ピン穴径 (0で穴なし)', 'mm',
                             adsk.core.ValueInput.createByString('3 mm'))

            # --- ヤンセン ---
            jg = inputs.addGroupCommandInput('jansenGroup', 'ヤンセン機構パラメータ')
            ji = jg.children
            ji.addValueInput('jScale', '倍率 (1.0=クランク半径15mm)', '',
                             adsk.core.ValueInput.createByString('1.0'))
            ji.addValueInput('jAngle', 'クランク角', 'deg',
                             adsk.core.ValueInput.createByString('0 deg'))

            # --- ヘッケン ---
            hg = inputs.addGroupCommandInput('hoeckenGroup', 'ヘッケンリンクパラメータ')
            hi = hg.children
            hi.addValueInput('hUnit', '基準長 a (クランク半径)', 'mm',
                             adsk.core.ValueInput.createByString('15 mm'))
            hi.addValueInput('hAngle', 'クランク角 (直線区間: 90〜270°)', 'deg',
                             adsk.core.ValueInput.createByString('180 deg'))
            hg.isVisible = False

            # --- チェビシェフ ---
            tg = inputs.addGroupCommandInput('chebyGroup', 'チェビシェフリンクパラメータ')
            ti = tg.children
            ti.addValueInput('cUnit', '基準長 a (フレーム=4a)', 'mm',
                             adsk.core.ValueInput.createByString('15 mm'))
            ti.addValueInput('cAngle', '入力角 (有効範囲: 約40〜100°)', 'deg',
                             adsk.core.ValueInput.createByString('70 deg'))
            tg.isVisible = False

            # --- 膝 ---
            kg = inputs.addGroupCommandInput('kneeGroup', '平行リンク(膝)パラメータ')
            ki = kg.children
            ki.addValueInput('kLen', 'リンク長', 'mm',
                             adsk.core.ValueInput.createByString('60 mm'))
            ki.addValueInput('kGap', 'ピボット間隔', 'mm',
                             adsk.core.ValueInput.createByString('20 mm'))
            ki.addValueInput('kAngle', '膝振り角 (鉛直から)', 'deg',
                             adsk.core.ValueInput.createByString('20 deg'))
            ki.addValueInput('kShank', 'すね長さ', 'mm',
                             adsk.core.ValueInput.createByString('80 mm'))
            kg.isVisible = False

            on_changed = InputChangedHandler()
            cmd.inputChanged.add(on_changed)
            _handlers.append(on_changed)

            on_exec = CommandExecuteHandler()
            cmd.execute.add(on_exec)
            _handlers.append(on_exec)

            on_destroy = CommandDestroyHandler()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)
        except Exception:
            _ui.messageBox('UI作成に失敗:\n{}'.format(traceback.format_exc()))


class InputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            if args.input.id != 'mechType':
                return
            inputs = args.inputs
            sel = args.input.selectedItem.name
            inputs.itemById('jansenGroup').isVisible = sel.startswith('テオ')
            inputs.itemById('hoeckenGroup').isVisible = sel.startswith('ヘッケン')
            inputs.itemById('chebyGroup').isVisible = sel.startswith('チェビシェフ')
            inputs.itemById('kneeGroup').isVisible = sel.startswith('平行')
        except Exception:
            _ui.messageBox(traceback.format_exc())


class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            sel = inputs.itemById('mechType').selectedItem.name
            _reset()

            # ValueCommandInput.value は内部単位(cm / rad)
            thick = inputs.itemById('linkThk').value
            half_w = inputs.itemById('linkW').value / 2.0
            hole_r = inputs.itemById('holeDia').value / 2.0

            if sel.startswith('テオ'):
                build_jansen(
                    inputs.itemById('jScale').value,
                    inputs.itemById('jAngle').value,
                    half_w, thick, hole_r)
            elif sel.startswith('ヘッケン'):
                build_hoecken(
                    inputs.itemById('hUnit').value,
                    inputs.itemById('hAngle').value,
                    half_w, thick, hole_r)
            elif sel.startswith('チェビシェフ'):
                build_chebyshev(
                    inputs.itemById('cUnit').value,
                    inputs.itemById('cAngle').value,
                    half_w, thick, hole_r)
            else:
                build_knee(
                    inputs.itemById('kLen').value,
                    inputs.itemById('kGap').value,
                    inputs.itemById('kAngle').value,
                    inputs.itemById('kShank').value,
                    half_w, thick, hole_r)

            _finish()
            _app.activeViewport.fit()
        except Exception:
            _ui.messageBox('生成に失敗しました:\n{}'.format(traceback.format_exc()))


class CommandDestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            adsk.terminate()
        except Exception:
            pass


# --------------------------------------------------------------------------
# エントリポイント
# --------------------------------------------------------------------------
def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        cmd_def = _ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        cmd_def = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_DESC)

        on_created = CommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        cmd_def.execute()
        adsk.autoTerminate(False)
    except Exception:
        if _ui:
            _ui.messageBox('起動に失敗:\n{}'.format(traceback.format_exc()))
