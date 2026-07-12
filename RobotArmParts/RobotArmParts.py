# -*- coding: utf-8 -*-
# ==========================================================================
#  RobotArmParts.py  —  Fusion 360 スクリプト
#
#  ロボットアームによく使う3種類の部品をパラメータ指定で自動生成します。
#    1. 平歯車(インボリュート歯形)
#    2. ベベルギア(すぐばかさ歯車・軸角90°) … 差動(ディファレンシャル)機構用
#    3. タイミングプーリ(GT2系の近似形状)
#    4. 深溝ボールベアリング(簡易モデル)
#
#  ■ 差動機構の作り方
#    サイドギア(例: Z16, 相手Z10)を2個、ピニオン(Z10, 相手Z16)を2個生成し、
#    軸角90°で十字に組むと自動車のデフと同じ差動機構になります。
#    1:1の差動ならサイドギア・ピニオンとも同歯数(相手歯数=自歯数)でOKです。
#
#  ■ インストール方法
#    1. Fusion 360 で「ユーティリティ」タブ →「アドイン」→「スクリプトとアドイン」
#    2. 「スクリプト」タブの「+」を押し、このファイルを含むフォルダを指定
#       (フォルダ名とファイル名を同じ "RobotArmParts" にしてください)
#    3. 一覧から選んで「実行」
#
#  ■ 使い方
#    実行するとダイアログが開くので、部品の種類を選びパラメータを入力してOK。
#    新しいコンポーネントとしてルート直下に生成されます。
#    生成後もタイムラインの各フィーチャを編集すれば寸法変更できます。
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
_handlers = []          # ハンドラをGCから守るための参照保持

CMD_ID = 'robotArmPartsGenerator'
CMD_NAME = 'ロボットアーム部品ジェネレータ'
CMD_DESC = '平歯車 / タイミングプーリ / ボールベアリング をパラメータ指定で生成します'


# --------------------------------------------------------------------------
# 補助関数
# --------------------------------------------------------------------------
def get_design():
    design = adsk.fusion.Design.cast(_app.activeProduct)
    if not design:
        raise RuntimeError('デザイン(Fusionドキュメント)を開いた状態で実行してください。')
    return design


def new_component(name):
    """ルート直下に新しいコンポーネントを作って返す

    パーツ ドキュメントではコンポーネントを追加できないため、
    ルート内のボディとして生成する (_Part が差異を吸収する)。
    """
    design = get_design()
    return _new_part(design.rootComponent, name)


def involute_point(base_r, alpha):
    """圧力角 alpha [rad] に対応するインボリュート上の点 (r, theta)"""
    r = base_r / math.cos(alpha)
    theta = math.tan(alpha) - alpha      # インボリュート関数 inv(alpha)
    return r, theta


def pol(r, theta, z=0.0):
    return adsk.core.Point3D.create(r * math.cos(theta), r * math.sin(theta), z)


def arc_sweep(p_start, p_end):
    """原点中心の円弧を p_start から p_end まで引くときの掃引角 [rad]。

    atan2 の差をそのまま使うと、円弧が ±180° の分岐をまたぐ位置(= 真後ろの歯)で
    符号が反転して ほぼ一周する円弧になってしまう。必ず (-π, π] に正規化する。
    """
    d = math.atan2(p_end.y, p_end.x) - math.atan2(p_start.y, p_start.x)
    while d > math.pi:
        d -= 2.0 * math.pi
    while d <= -math.pi:
        d += 2.0 * math.pi
    return d


def ring_profile(sketch):
    """2重円スケッチからリング(ループ2本)のプロファイルを返す"""
    for i in range(sketch.profiles.count):
        p = sketch.profiles.item(i)
        if p.profileLoops.count == 2:
            return p
    return sketch.profiles.item(0)


class defer(object):
    """曲線を追加する間、スケッチの再計算(拘束解決 + プロファイル生成)を止める。

    Fusion は曲線を 1 本追加するたびに再計算するため、歯形のように曲線数が多い
    スケッチではこれを止めるだけで生成時間が大きく縮む。
    """
    __slots__ = ('sk',)

    def __init__(self, sk):
        self.sk = sk

    def __enter__(self):
        self.sk.isComputeDeferred = True
        return self.sk

    def __exit__(self, *exc):
        self.sk.isComputeDeferred = False
        return False


def all_profiles(sketch):
    coll = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        coll.add(sketch.profiles.item(i))
    return coll


def cut_bore(comp, radius_cm, span_cm):
    """原点中心の貫通穴(対称カット)"""
    if radius_cm <= 0:
        return
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0), radius_cm)
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures.createInput(
        prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
    ext.setSymmetricExtent(adsk.core.ValueInput.createByReal(span_cm * 2.5), True)
    comp.features.extrudeFeatures.add(_scope(ext))


# --------------------------------------------------------------------------
# 1. 平歯車(インボリュート)
# --------------------------------------------------------------------------
def build_spur_gear(module_cm, num_teeth, pressure_angle, thickness_cm, bore_cm):
    """
    module_cm      : モジュール [cm]
    num_teeth      : 歯数
    pressure_angle : 圧力角 [rad]
    thickness_cm   : 歯幅 [cm]
    bore_cm        : 軸穴径 [cm]
    """
    comp = new_component('SpurGear M{:.1f} Z{}'.format(module_cm * 10, num_teeth))

    m = module_cm
    z = num_teeth
    rp = m * z / 2.0                          # 基準円半径
    rb = rp * math.cos(pressure_angle)        # 基礎円半径
    ra = rp + m                               # 歯先円半径
    rf = rp - 1.25 * m                        # 歯底円半径
    if rf <= bore_cm / 2.0:
        raise RuntimeError('軸穴が歯底円より大きすぎます。')

    # --- 歯底円の円筒(ベース) ---
    sk_base = comp.sketches.add(comp.xYConstructionPlane)
    sk_base.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0), rf)
    ext_in = comp.features.extrudeFeatures.createInput(
        sk_base.profiles.item(0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext_in.setDistanceExtent(False, adsk.core.ValueInput.createByReal(thickness_cm))
    comp.features.extrudeFeatures.add(_scope(ext_in))

    # --- 歯の輪郭(全歯を1スケッチに描く) ---
    sk_tooth = comp.sketches.add(comp.xYConstructionPlane)
    curves = sk_tooth.sketchCurves

    half_tooth = math.pi / (2.0 * z)                       # 基準円上の歯厚の半角
    inv_pa = math.tan(pressure_angle) - pressure_angle

    # インボリュート開始点(歯底が基礎円より内側なら基礎円から)
    alpha_min = 0.0 if rf < rb else math.acos(rb / rf)
    alpha_max = math.acos(rb / ra)

    n_pts = 12
    flank = []    # (半径, 基準歯における角度) — 歯ごとに回転させて使う
    for i in range(n_pts + 1):
        a = alpha_min + (alpha_max - alpha_min) * i / n_pts
        r, inv_a = involute_point(rb, a)
        flank.append((r, (inv_a - inv_pa) - half_tooth))   # 基準円上で -half_tooth

    origin = adsk.core.Point3D.create(0, 0, 0)

    def tooth_curves(off):
        """off [rad] だけ回した位置に歯1枚の閉輪郭を描く"""
        side_a = [pol(r, th + off) for (r, th) in flank]    # -theta 側フランク
        side_b = [pol(r, -th + off) for (r, th) in flank]   # +theta 側フランク

        for points in (side_a, side_b):
            coll = adsk.core.ObjectCollection.create()
            for p in points:
                coll.add(p)
            curves.sketchFittedSplines.add(coll)

        # 歯先の円弧(A端 → B端)
        curves.sketchArcs.addByCenterStartSweep(
            origin, side_a[-1], arc_sweep(side_a[-1], side_b[-1]))

        # 歯底側の接続
        if rf < rb:
            # 半径方向の直線で基礎円から歯底円まで降ろす
            pa_root = pol(rf, math.atan2(side_a[0].y, side_a[0].x))
            pb_root = pol(rf, math.atan2(side_b[0].y, side_b[0].x))
            curves.sketchLines.addByTwoPoints(pa_root, side_a[0])
            curves.sketchLines.addByTwoPoints(pb_root, side_b[0])
            curves.sketchArcs.addByCenterStartSweep(
                origin, pb_root, arc_sweep(pb_root, pa_root))   # 歯の中心を通って閉じる
        else:
            curves.sketchArcs.addByCenterStartSweep(
                origin, side_b[0], arc_sweep(side_b[0], side_a[0]))

    with defer(sk_tooth):
        for i in range(z):
            tooth_curves(2.0 * math.pi * i / z)

    # 全歯を 1 回の押し出しで結合する(歯1枚 + 円形パターンは歯数ぶんの
    # ブーリアン演算になり、歯数が増えるほど極端に遅くなる)
    ext_in = comp.features.extrudeFeatures.createInput(
        all_profiles(sk_tooth),
        adsk.fusion.FeatureOperations.JoinFeatureOperation)
    ext_in.setDistanceExtent(False, adsk.core.ValueInput.createByReal(thickness_cm))
    comp.features.extrudeFeatures.add(_scope(ext_in))

    # --- 軸穴 ---
    cut_bore(comp, bore_cm / 2.0, thickness_cm)
    return comp


# --------------------------------------------------------------------------
# 2. ベベルギア(すぐばかさ歯車・軸角90°)— 差動機構用
# --------------------------------------------------------------------------
def build_bevel_gear(module_cm, num_teeth, mate_teeth, pressure_angle,
                     face_width_cm, bore_cm):
    """
    module_cm      : 大端モジュール [cm]
    num_teeth      : この歯車の歯数
    mate_teeth     : かみ合う相手の歯数(軸角90°としてピッチ円錐角を決定)
    pressure_angle : 圧力角 [rad]
    face_width_cm  : 歯幅(円錐距離方向)[cm]
    bore_cm        : 軸穴径 [cm]

    Tredgold近似(背円錐上の相当平歯車)でインボリュート歯形を作り、
    円錐頂点に向けて相似縮小した断面へロフトして歯を生成します。
    """
    m = module_cm
    z = num_teeth
    delta = math.atan2(num_teeth, mate_teeth)     # ピッチ円錐角
    rp = m * z / 2.0                               # 大端の基準円半径
    Re = rp / math.sin(delta)                      # 外側円錐距離
    if face_width_cm > 0.45 * Re:
        raise RuntimeError(
            '歯幅が大きすぎます。円錐距離 {:.1f}mm の 1/3 程度以下にしてください。'.format(Re * 10))

    k = (Re - face_width_cm) / Re                  # 小端側の縮小率
    za = rp / math.tan(delta)                      # 円錐頂点の高さ(背面基準)
    height = za * (1.0 - k)                        # 歯車の軸方向厚み

    comp = new_component(
        'BevelGear M{:.1f} Z{} (mate Z{})'.format(m * 10, z, mate_teeth))
    origin = adsk.core.Point3D.create(0, 0, 0)

    # --- 相当平歯車(Tredgold) ---
    zv = z / math.cos(delta)                       # 相当歯数
    rv = m * zv / 2.0
    rb = rv * math.cos(pressure_angle)
    ra_v = rv + m
    rf_v = rv - 1.45 * m                           # 本体との結合を保証するため少し深め
    inv_pa = math.tan(pressure_angle) - pressure_angle
    half_tooth = math.pi / (2.0 * zv)
    alpha_min = 0.0 if rf_v < rb else math.acos(rb / rf_v)
    alpha_max = math.acos(rb / ra_v)

    def mapped(r_e, th_e, scale):
        """相当平歯車座標 → 実歯車座標(背円錐の展開を戻す)+ 頂点方向スケール"""
        dx = r_e * math.cos(th_e) - rv             # 半径方向オフセット
        dy = r_e * math.sin(th_e)                  # 接線方向オフセット
        rho = rp + dx
        phi = dy / rp
        return adsk.core.Point3D.create(
            scale * rho * math.cos(phi), scale * rho * math.sin(phi), 0)

    def draw_tooth(sketch, scale):
        curves = sketch.sketchCurves
        n_pts = 12
        side_a, side_b = [], []
        for i in range(n_pts + 1):
            a = alpha_min + (alpha_max - alpha_min) * i / n_pts
            r_e, inv_a = involute_point(rb, a)
            th = (inv_a - inv_pa) - half_tooth
            side_a.append(mapped(r_e, th, scale))
            side_b.append(mapped(r_e, -th, scale))

        def fitted(points):
            coll = adsk.core.ObjectCollection.create()
            for p in points:
                coll.add(p)
            curves.sketchFittedSplines.add(coll)

        with defer(sketch):
            fitted(side_a)
            fitted(side_b)

            # 歯先の円弧
            th_a = math.atan2(side_a[-1].y, side_a[-1].x)
            th_b = math.atan2(side_b[-1].y, side_b[-1].x)
            curves.sketchArcs.addByCenterStartSweep(origin, side_a[-1], th_b - th_a)

            # 歯底側の接続
            th_r0 = -(half_tooth + inv_pa)
            if rf_v < rb:
                pa_r = mapped(rf_v, th_r0, scale)
                pb_r = mapped(rf_v, -th_r0, scale)
                curves.sketchLines.addByTwoPoints(pa_r, side_a[0])
                curves.sketchLines.addByTwoPoints(pb_r, side_b[0])
            else:
                pa_r, pb_r = side_a[0], side_b[0]
            tha = math.atan2(pa_r.y, pa_r.x)
            thb = math.atan2(pb_r.y, pb_r.x)
            curves.sketchArcs.addByCenterStartSweep(origin, pb_r, tha - thb)
        return sketch.profiles.item(0)

    # --- 小端側のオフセット平面 ---
    planes = comp.constructionPlanes
    pin = planes.createInput()
    pin.setByOffset(comp.xYConstructionPlane,
                    adsk.core.ValueInput.createByReal(height))
    top_plane = planes.add(pin)

    # --- 本体(歯底円錐台)を2円のロフトで作成 ---
    r_cone = rp - 1.25 * m
    sk1 = comp.sketches.add(comp.xYConstructionPlane)
    sk1.sketchCurves.sketchCircles.addByCenterRadius(origin, r_cone)
    sk2 = comp.sketches.add(top_plane)
    sk2.sketchCurves.sketchCircles.addByCenterRadius(origin, r_cone * k)
    loft_in = comp.features.loftFeatures.createInput(
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    loft_in.loftSections.add(sk1.profiles.item(0))
    loft_in.loftSections.add(sk2.profiles.item(0))
    comp.features.loftFeatures.add(_scope(loft_in))

    # --- 歯1枚を大端→小端の断面でロフト ---
    skt1 = comp.sketches.add(comp.xYConstructionPlane)
    prof1 = draw_tooth(skt1, 1.0)
    skt2 = comp.sketches.add(top_plane)
    prof2 = draw_tooth(skt2, k)
    loft_in = comp.features.loftFeatures.createInput(
        adsk.fusion.FeatureOperations.JoinFeatureOperation)
    loft_in.loftSections.add(prof1)
    loft_in.loftSections.add(prof2)
    tooth_feat = comp.features.loftFeatures.add(_scope(loft_in))

    # --- 歯を円形パターン ---
    feats = adsk.core.ObjectCollection.create()
    feats.add(tooth_feat)
    pat_in = comp.features.circularPatternFeatures.createInput(
        feats, comp.zConstructionAxis)
    pat_in.quantity = adsk.core.ValueInput.createByReal(z)
    pat_in.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    comp.features.circularPatternFeatures.add(pat_in)

    # --- 軸穴 ---
    cut_bore(comp, bore_cm / 2.0, height + 1.0)
    return comp


# --------------------------------------------------------------------------
# 3. タイミングプーリ(GT2系の近似)
# --------------------------------------------------------------------------
def build_timing_pulley(num_teeth, pitch_cm, width_cm, bore_cm, with_flange):
    """
    num_teeth : 歯数
    pitch_cm  : ベルトピッチ [cm](GT2 = 0.2cm)
    width_cm  : 歯部の幅 [cm]
    bore_cm   : 軸穴径 [cm]
    ※ 歯溝は円弧による近似形状です(GT2正規プロファイルではありません)
    """
    comp = new_component('Pulley GT{:.0f} T{}'.format(pitch_cm * 10, num_teeth))

    pd = num_teeth * pitch_cm / math.pi        # ピッチ円直径
    pld = 0.0254                                # GT2のPLD(ベルトピッチライン差)0.254mm
    r_out = pd / 2.0 - pld                      # 外径半径

    if r_out <= bore_cm / 2.0 + 0.05:
        raise RuntimeError('歯数が少なすぎるか軸穴が大きすぎます。')

    # --- 本体円筒 ---
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0), r_out)
    ext_in = comp.features.extrudeFeatures.createInput(
        sk.profiles.item(0),
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext_in.setDistanceExtent(False, adsk.core.ValueInput.createByReal(width_cm))
    comp.features.extrudeFeatures.add(_scope(ext_in))

    # --- 歯溝(円弧カット)を全周ぶん 1 スケッチに描いて一括で切る ---
    groove_r = pitch_cm * 0.25                 # 溝の円弧半径
    groove_off = pitch_cm * 0.10               # 外径からの食い込み調整
    gr = r_out - groove_off
    sk_g = comp.sketches.add(comp.xYConstructionPlane)
    with defer(sk_g):
        for i in range(num_teeth):
            th = 2.0 * math.pi * i / num_teeth
            sk_g.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(gr * math.cos(th), gr * math.sin(th), 0),
                groove_r)
    ext_in = comp.features.extrudeFeatures.createInput(
        all_profiles(sk_g),
        adsk.fusion.FeatureOperations.CutFeatureOperation)
    ext_in.setDistanceExtent(False, adsk.core.ValueInput.createByReal(width_cm))
    comp.features.extrudeFeatures.add(_scope(ext_in))

    # --- フランジ(両端の鍔) ---
    if with_flange:
        flange_r = r_out + pitch_cm * 0.5
        flange_t = 0.1                          # 1mm
        for start, dist in ((-flange_t, flange_t), (width_cm, flange_t)):
            sk_f = comp.sketches.add(comp.xYConstructionPlane)
            sk_f.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(0, 0, 0), flange_r)
            ext_in = comp.features.extrudeFeatures.createInput(
                sk_f.profiles.item(0),
                adsk.fusion.FeatureOperations.JoinFeatureOperation)
            ext_in.startExtent = adsk.fusion.OffsetStartDefinition.create(
                adsk.core.ValueInput.createByReal(start))
            ext_in.setDistanceExtent(False, adsk.core.ValueInput.createByReal(dist))
            comp.features.extrudeFeatures.add(_scope(ext_in))

    # --- 軸穴 ---
    cut_bore(comp, bore_cm / 2.0, width_cm + 0.4)
    return comp


# --------------------------------------------------------------------------
# 3. 深溝ボールベアリング(簡易)
# --------------------------------------------------------------------------
def build_ball_bearing(od_cm, id_cm, width_cm, num_balls):
    """
    od_cm     : 外径 [cm]
    id_cm     : 内径 [cm]
    width_cm  : 幅 [cm]
    num_balls : ボール数
    例) 608ベアリング: 外径22mm 内径8mm 幅7mm ボール7個
    """
    if od_cm <= id_cm + 0.2:
        raise RuntimeError('外径と内径の差が小さすぎます。')

    comp = new_component('Bearing {:.0f}x{:.0f}x{:.0f}'.format(
        od_cm * 10, id_cm * 10, width_cm * 10))

    r_o = od_cm / 2.0
    r_i = id_cm / 2.0
    r_c = (r_o + r_i) / 2.0                    # ボール中心の半径
    ball_r = (od_cm - id_cm) * 0.14            # ボール半径(直径 = 差の28%)
    gap = ball_r * 0.55                        # 軌道面までの隙間

    origin = adsk.core.Point3D.create(0, 0, 0)

    def ring(r_in, r_out):
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.sketchCurves.sketchCircles.addByCenterRadius(origin, r_in)
        sk.sketchCurves.sketchCircles.addByCenterRadius(origin, r_out)
        ext_in = comp.features.extrudeFeatures.createInput(
            ring_profile(sk),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        ext_in.setSymmetricExtent(
            adsk.core.ValueInput.createByReal(width_cm), True)
        return comp.features.extrudeFeatures.add(_scope(ext_in)).bodies.item(0)

    outer_body = ring(r_c + gap, r_o)          # 外輪
    inner_body = ring(r_i, r_c - gap)          # 内輪

    # --- 軌道溝: ボール断面をトーラス回転させて両輪からカット ---
    sk_t = comp.sketches.add(comp.xZConstructionPlane)
    sk_t.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(r_c, 0, 0), ball_r * 1.03)
    rev_in = comp.features.revolveFeatures.createInput(
        sk_t.profiles.item(0), comp.zConstructionAxis,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    rev_in.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
    torus_body = comp.features.revolveFeatures.add(_scope(rev_in)).bodies.item(0)

    tools = adsk.core.ObjectCollection.create()
    tools.add(torus_body)
    cmb = comp.features.combineFeatures.createInput(outer_body, tools)
    cmb.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    cmb.isKeepToolBodies = True
    comp.features.combineFeatures.add(cmb)

    tools2 = adsk.core.ObjectCollection.create()
    tools2.add(torus_body)
    cmb2 = comp.features.combineFeatures.createInput(inner_body, tools2)
    cmb2.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    cmb2.isKeepToolBodies = False              # ここでトーラスを消費
    comp.features.combineFeatures.add(cmb2)

    # --- ボール1個(半円を回転)→ 円形パターン ---
    sk_b = comp.sketches.add(comp.xZConstructionPlane)
    c = adsk.core.Point3D.create(r_c, 0, 0)
    p_top = adsk.core.Point3D.create(r_c, ball_r, 0)
    arc = sk_b.sketchCurves.sketchArcs.addByCenterStartSweep(c, p_top, math.pi)
    axis_line = sk_b.sketchCurves.sketchLines.addByTwoPoints(
        arc.startSketchPoint.geometry, arc.endSketchPoint.geometry)
    rev_in = comp.features.revolveFeatures.createInput(
        sk_b.profiles.item(0), axis_line,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    rev_in.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
    ball_body = comp.features.revolveFeatures.add(_scope(rev_in)).bodies.item(0)

    bodies = adsk.core.ObjectCollection.create()
    bodies.add(ball_body)
    pat_in = comp.features.circularPatternFeatures.createInput(
        bodies, comp.zConstructionAxis)
    pat_in.quantity = adsk.core.ValueInput.createByReal(num_balls)
    pat_in.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    comp.features.circularPatternFeatures.add(pat_in)
    return comp


# --------------------------------------------------------------------------
# コマンドUI
# --------------------------------------------------------------------------
class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.setDialogInitialSize(380, 420)
            inputs = cmd.commandInputs

            dd = inputs.addDropDownCommandInput(
                'partType', '部品の種類',
                adsk.core.DropDownStyles.TextListDropDownStyle)
            dd.listItems.add('平歯車', True)
            dd.listItems.add('ベベルギア (差動用)', False)
            dd.listItems.add('タイミングプーリ (GT2近似)', False)
            dd.listItems.add('ボールベアリング', False)

            # --- 歯車 ---
            g = inputs.addGroupCommandInput('gearGroup', '歯車パラメータ')
            gi = g.children
            gi.addValueInput('gearModule', 'モジュール', 'mm',
                             adsk.core.ValueInput.createByString('1.5 mm'))
            gi.addIntegerSpinnerCommandInput('gearTeeth', '歯数', 6, 200, 1, 24)
            gi.addValueInput('gearPA', '圧力角', 'deg',
                             adsk.core.ValueInput.createByString('20 deg'))
            gi.addValueInput('gearThk', '歯幅', 'mm',
                             adsk.core.ValueInput.createByString('8 mm'))
            gi.addValueInput('gearBore', '軸穴径', 'mm',
                             adsk.core.ValueInput.createByString('5 mm'))

            # --- ベベルギア ---
            v = inputs.addGroupCommandInput('bevelGroup', 'ベベルギアパラメータ')
            vi = v.children
            vi.addValueInput('bevModule', 'モジュール(大端)', 'mm',
                             adsk.core.ValueInput.createByString('1.5 mm'))
            vi.addIntegerSpinnerCommandInput('bevTeeth', '歯数', 8, 100, 1, 16)
            vi.addIntegerSpinnerCommandInput('bevMate', '相手歯数', 8, 100, 1, 16)
            vi.addValueInput('bevPA', '圧力角', 'deg',
                             adsk.core.ValueInput.createByString('20 deg'))
            vi.addValueInput('bevFW', '歯幅', 'mm',
                             adsk.core.ValueInput.createByString('5 mm'))
            vi.addValueInput('bevBore', '軸穴径', 'mm',
                             adsk.core.ValueInput.createByString('4 mm'))
            v.isVisible = False

            # --- プーリ ---
            p = inputs.addGroupCommandInput('pulleyGroup', 'プーリパラメータ')
            pi = p.children
            pi.addIntegerSpinnerCommandInput('pulTeeth', '歯数', 10, 120, 1, 20)
            pi.addValueInput('pulPitch', 'ベルトピッチ', 'mm',
                             adsk.core.ValueInput.createByString('2 mm'))
            pi.addValueInput('pulWidth', '歯部の幅', 'mm',
                             adsk.core.ValueInput.createByString('7 mm'))
            pi.addValueInput('pulBore', '軸穴径', 'mm',
                             adsk.core.ValueInput.createByString('5 mm'))
            pi.addBoolValueInput('pulFlange', 'フランジ(鍔)を付ける', True, '', True)
            p.isVisible = False

            # --- ベアリング ---
            b = inputs.addGroupCommandInput('bearingGroup', 'ベアリングパラメータ')
            bi = b.children
            bi.addValueInput('brgOD', '外径', 'mm',
                             adsk.core.ValueInput.createByString('22 mm'))
            bi.addValueInput('brgID', '内径', 'mm',
                             adsk.core.ValueInput.createByString('8 mm'))
            bi.addValueInput('brgW', '幅', 'mm',
                             adsk.core.ValueInput.createByString('7 mm'))
            bi.addIntegerSpinnerCommandInput('brgBalls', 'ボール数', 4, 30, 1, 7)
            b.isVisible = False

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
            inputs = args.inputs
            if args.input.id != 'partType':
                return
            sel = args.input.selectedItem.name
            inputs.itemById('gearGroup').isVisible = (sel == '平歯車')
            inputs.itemById('bevelGroup').isVisible = sel.startswith('ベベル')
            inputs.itemById('pulleyGroup').isVisible = sel.startswith('タイミング')
            inputs.itemById('bearingGroup').isVisible = (sel == 'ボールベアリング')
        except Exception:
            _ui.messageBox(traceback.format_exc())


class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            sel = inputs.itemById('partType').selectedItem.name
            _reset()

            # ValueCommandInput.value は内部単位(cm / rad)で返る
            if sel == '平歯車':
                build_spur_gear(
                    inputs.itemById('gearModule').value,
                    inputs.itemById('gearTeeth').value,
                    inputs.itemById('gearPA').value,
                    inputs.itemById('gearThk').value,
                    inputs.itemById('gearBore').value)
            elif sel.startswith('ベベル'):
                build_bevel_gear(
                    inputs.itemById('bevModule').value,
                    inputs.itemById('bevTeeth').value,
                    inputs.itemById('bevMate').value,
                    inputs.itemById('bevPA').value,
                    inputs.itemById('bevFW').value,
                    inputs.itemById('bevBore').value)
            elif sel.startswith('タイミング'):
                build_timing_pulley(
                    inputs.itemById('pulTeeth').value,
                    inputs.itemById('pulPitch').value,
                    inputs.itemById('pulWidth').value,
                    inputs.itemById('pulBore').value,
                    inputs.itemById('pulFlange').value)
            else:
                build_ball_bearing(
                    inputs.itemById('brgOD').value,
                    inputs.itemById('brgID').value,
                    inputs.itemById('brgW').value,
                    inputs.itemById('brgBalls').value)

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
        adsk.autoTerminate(False)   # ダイアログが閉じるまでスクリプトを維持
    except Exception:
        if _ui:
            _ui.messageBox('起動に失敗:\n{}'.format(traceback.format_exc()))
