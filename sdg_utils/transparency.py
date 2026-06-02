"""把场景内所有 mesh 临时变为完全不透明的工具。

背景
----
Replicator 的深度 annotator 会让光线穿透 opacity < 1 的材质,导致玻璃 / 电脑屏幕 /
窗户等"看起来透明"的 mesh 在 RGB 中能看到表面,深度图却记录的是它们后方物体的距
离 → RGB 与深度像素级不对齐。

由于 USD 场景中可能存在各式各样的 shader (UsdPreviewSurface / OmniGlass /
OmniPBR / OmniSurface / 第三方 MDL …),靠关键字 / 启发式去精确识别"哪些 mesh
是透明的"非常容易漏判 (典型例子:电脑屏幕、显示器、镜子等)。本模块采取**完全不
做透明判定、对所有传入 mesh 一律执行不透明化**的策略,采集深度时绝对杜绝光线穿
透;采集结束立刻 `restore` 回去,不污染下一帧的 RGB。

核心 API
--------
- `make_all_meshes_opaque(stage, mesh_paths) -> OpacityOverride`
- `restore_meshes(stage, override)`
- `OpacityOverride`: 缓存原始状态的轻量数据类。

详细思路与流程见 `docs/transparent_and_far_filter.md`。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional

from loguru import logger
from pxr import Sdf, Usd, UsdShade


_OPAQUE_OVERRIDE_MATERIAL_PATH = "/World/_OpaqueOverrideMat"

_PREVIEW_SURFACE_OPACITY_INPUTS = ("opacity",)
_MDL_OPACITY_ENABLE_INPUTS = ("enable_opacity",)
_MDL_OPACITY_VALUE_INPUTS = ("opacity_constant", "cutout_opacity", "opacity")
# OmniGlass 这类 shader 改 input 也无法 100% 不透明,统一走 binding override 兜底
_FORCE_OVERRIDE_SHADER_KEYWORDS = ("glass", "translucent")


@dataclass
class _AttributeBackup:
    """单个 USD 属性的原始值备份。"""
    attr: Usd.Attribute
    had_value: bool
    value: Any = None


@dataclass
class _BindingBackup:
    """单个 mesh 的 material binding 原始状态备份。"""
    prim: Usd.Prim
    had_direct_binding: bool
    original_material_path: Optional[Sdf.Path] = None


@dataclass
class OpacityOverride:
    """`make_all_meshes_opaque` 的返回值;`restore_meshes` 会按字段顺序将所有改动
    反向写回。"""
    attribute_backups: List[_AttributeBackup] = field(default_factory=list)
    binding_backups: List[_BindingBackup] = field(default_factory=list)
    opaque_override_path: Optional[Sdf.Path] = None

    def empty(self) -> bool:
        return not self.attribute_backups and not self.binding_backups


# =============================================================================
# 内部工具
# =============================================================================

def _get_bound_shader(prim: Usd.Prim) -> Optional[UsdShade.Shader]:
    """拿到 prim 绑定材质的 surface shader (若不存在则返回 None)。"""
    binding_api = UsdShade.MaterialBindingAPI(prim)
    material, _rel = binding_api.ComputeBoundMaterial()
    if not material:
        return None

    surface_output = material.GetSurfaceOutput()
    if surface_output:
        connected = surface_output.GetConnectedSource()
        if connected:
            source_api = connected[0]
            source_prim = source_api.GetPrim()
            if source_prim and source_prim.IsValid():
                return UsdShade.Shader(source_prim)

    # 兜底:遍历 material 的 children Shader
    for child in material.GetPrim().GetChildren():
        if child.IsA(UsdShade.Shader):
            return UsdShade.Shader(child)
    return None


def _shader_id(shader: Optional[UsdShade.Shader]) -> str:
    """获取 shader 的 id (UsdPreviewSurface 用 ShaderId,MDL 用 SourceAsset 路径)。

    返回小写,以便后续做包含匹配。
    """
    if shader is None:
        return ""
    sid = ""
    id_attr = shader.GetIdAttr()
    if id_attr and id_attr.HasValue():
        try:
            sid = id_attr.Get() or ""
        except Exception:
            sid = ""
    if not sid:
        try:
            asset_attr = shader.GetPrim().GetAttribute("info:mdl:sourceAsset")
            if asset_attr and asset_attr.HasValue():
                sid = str(asset_attr.Get()) or ""
        except Exception:
            pass
    if not sid:
        sid = shader.GetPrim().GetName() or ""
    return str(sid).lower()


def _shader_needs_binding_override(shader: Optional[UsdShade.Shader]) -> bool:
    """OmniGlass 这类 shader 即便把 cutout_opacity 设为 1 也不会 100% 不透明,
    统一让它走 binding override 兜底。"""
    if shader is None:
        return False
    sid = _shader_id(shader)
    return any(kw in sid for kw in _FORCE_OVERRIDE_SHADER_KEYWORDS)


def _backup_and_set_attr(attr: Usd.Attribute,
                         new_value: Any,
                         backups: List[_AttributeBackup]) -> bool:
    """缓存属性当前值并设置新值。返回是否成功修改。"""
    if not attr:
        return False
    had_value = attr.HasValue()
    old_value = None
    if had_value:
        try:
            old_value = attr.Get()
        except Exception:
            had_value = False
    try:
        attr.Set(new_value)
    except Exception as e:
        logger.warning(f"transparency: 修改 {attr.GetPath()} 失败: {e}")
        return False
    backups.append(_AttributeBackup(attr=attr, had_value=had_value, value=old_value))
    return True


def _force_shader_opaque(shader: UsdShade.Shader,
                         backups: List[_AttributeBackup]) -> bool:
    """尝试把一个 shader 强制改为不透明 (PreviewSurface 与各种 MDL 通吃)。

    返回是否对该 shader 做了任何修改。
    """
    modified_any = False

    # PreviewSurface: opacity = 1, opacityThreshold = 0
    inp = shader.GetInput("opacity")
    if inp:
        modified_any |= _backup_and_set_attr(inp.GetAttr(), 1.0, backups)
    inp = shader.GetInput("opacityThreshold")
    if inp:
        modified_any |= _backup_and_set_attr(inp.GetAttr(), 0.0, backups)

    # MDL: enable_opacity = False (关闭透明通道)
    for name in _MDL_OPACITY_ENABLE_INPUTS:
        inp = shader.GetInput(name)
        if inp:
            modified_any |= _backup_and_set_attr(inp.GetAttr(), False, backups)

    # MDL: opacity_constant / cutout_opacity = 1
    for name in _MDL_OPACITY_VALUE_INPUTS:
        if name == "opacity":
            continue  # 已在 PreviewSurface 段处理过
        inp = shader.GetInput(name)
        if inp:
            modified_any |= _backup_and_set_attr(inp.GetAttr(), 1.0, backups)

    return modified_any


def _ensure_opaque_override(stage: Usd.Stage) -> Optional[UsdShade.Material]:
    """创建 (或复用) 一个全局共享的不透明 PreviewSurface,用于 binding override 兜底。"""
    sdf_path = Sdf.Path(_OPAQUE_OVERRIDE_MATERIAL_PATH)
    existing = stage.GetPrimAtPath(sdf_path)
    if existing and existing.IsValid() and existing.IsA(UsdShade.Material):
        return UsdShade.Material(existing)

    try:
        mat = UsdShade.Material.Define(stage, sdf_path)
        shader_path = sdf_path.AppendChild("Shader")
        shader = UsdShade.Shader.Define(stage, shader_path)
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.6, 0.6, 0.6))
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat
    except Exception as e:
        logger.warning(f"transparency: 创建 opaque override material 失败: {e}")
        return None


def _binding_override_to_opaque(prim: Usd.Prim,
                                opaque_mat: UsdShade.Material,
                                backups: List[_BindingBackup]) -> bool:
    """把 prim 的 material binding 临时切到 opaque override material。

    Returns:
        是否成功完成 override (失败时不会向 backups 追加任何项)。
    """
    binding_api = UsdShade.MaterialBindingAPI(prim)

    had_direct = False
    orig_path: Optional[Sdf.Path] = None
    try:
        direct = binding_api.GetDirectBinding()
        if direct.GetMaterial():
            had_direct = True
            orig_path = direct.GetMaterialPath()
    except Exception:
        pass

    if not had_direct:
        # 没有直接绑定但可能有继承绑定;记录继承材质路径以便 restore 时尝试还原
        try:
            current_material, _ = binding_api.ComputeBoundMaterial()
            if current_material:
                orig_path = current_material.GetPath()
        except Exception:
            orig_path = None

    try:
        binding_api.Bind(opaque_mat)
    except Exception as e:
        logger.warning(f"transparency: bind opaque 到 {prim.GetPath()} 失败: {e}")
        return False

    backups.append(_BindingBackup(
        prim=prim,
        had_direct_binding=had_direct,
        original_material_path=orig_path,
    ))
    return True


# =============================================================================
# 对外 API
# =============================================================================

def make_all_meshes_opaque(stage: Usd.Stage,
                           mesh_paths: List[Any]) -> OpacityOverride:
    """把传入的所有 mesh 临时变为完全不透明。

    策略 (按 mesh 独立决策):
      a. 修改其绑定 shader 的 opacity 类输入 (PreviewSurface / MDL)。
      b. 兜底:无可用 shader、shader 是 OmniGlass 等需强制 override 的类型,
         或策略 a 没能改动任何属性时,把 mesh 的 material binding 临时切到全
         局共享的不透明 PreviewSurface。

    必须配合 `restore_meshes` 使用,否则场景被永久修改。
    """
    override = OpacityOverride()
    if not mesh_paths:
        return override

    opaque_mat: Optional[UsdShade.Material] = None  # 懒创建

    binding_override_count = 0
    shader_modified_count = 0

    for path in mesh_paths:
        prim = stage.GetPrimAtPath(Sdf.Path(str(path)))
        if not prim or not prim.IsValid():
            continue

        shader = _get_bound_shader(prim)

        used_strategy_a = False
        if shader is not None and not _shader_needs_binding_override(shader):
            used_strategy_a = _force_shader_opaque(shader, override.attribute_backups)
            if used_strategy_a:
                shader_modified_count += 1

        if not used_strategy_a:
            if opaque_mat is None:
                opaque_mat = _ensure_opaque_override(stage)
                if opaque_mat is not None:
                    override.opaque_override_path = opaque_mat.GetPath()
            if opaque_mat is None:
                logger.warning(f"transparency: 无法创建 opaque override, 跳过 {path}")
                continue
            if _binding_override_to_opaque(prim, opaque_mat, override.binding_backups):
                binding_override_count += 1

    logger.info(
        "make_all_meshes_opaque: shader 改 opacity 的 mesh {} 个 (备份属性 {} 个), "
        "binding override 兜底 {} 个",
        shader_modified_count,
        len(override.attribute_backups),
        binding_override_count,
    )
    return override


def restore_meshes(stage: Usd.Stage, override: OpacityOverride) -> None:
    """把 `make_all_meshes_opaque` 做的修改全部恢复。"""
    if override is None or override.empty():
        return

    # 恢复 binding override
    for backup in override.binding_backups:
        prim = backup.prim
        if not prim or not prim.IsValid():
            continue
        binding_api = UsdShade.MaterialBindingAPI(prim)
        try:
            binding_api.UnbindAllBindings()
        except Exception as e:
            logger.warning(f"restore: UnbindAllBindings 失败 {prim.GetPath()}: {e}")

        # 仅当 mesh 原本有"直接绑定"时才显式 rebind;原本是"继承绑定"的话 unbind 后
        # 自然还会落回继承,不要画蛇添足。
        if backup.had_direct_binding and backup.original_material_path:
            orig_prim = stage.GetPrimAtPath(backup.original_material_path)
            if orig_prim and orig_prim.IsValid():
                try:
                    binding_api.Bind(UsdShade.Material(orig_prim))
                except Exception as e:
                    logger.warning(f"restore: 重新绑定原材质失败 {prim.GetPath()}: {e}")

    # 恢复 shader 属性 (倒序,避免重复 attr 后写覆盖前写)
    for backup in reversed(override.attribute_backups):
        attr = backup.attr
        if not attr:
            continue
        try:
            if backup.had_value:
                attr.Set(backup.value)
            else:
                attr.Clear()
        except Exception as e:
            logger.warning(f"restore: 恢复属性 {attr.GetPath()} 失败: {e}")

    logger.info(
        "restore_meshes: 已恢复 {} 个属性, {} 个 binding",
        len(override.attribute_backups),
        len(override.binding_backups),
    )


# =============================================================================
# 兼容老接口 (保留若干别名,后续可逐步移除)
# =============================================================================

#: 历史名,等价于 ``OpacityOverride``。
TransparentMeshOverride = OpacityOverride


def make_transparent_meshes_opaque(stage: Usd.Stage,
                                   mesh_paths: List[Any]) -> OpacityOverride:
    """兼容旧接口,等价于 `make_all_meshes_opaque`。"""
    return make_all_meshes_opaque(stage, mesh_paths)


def restore_transparent_meshes(stage: Usd.Stage,
                               override: OpacityOverride) -> None:
    """兼容旧接口,等价于 `restore_meshes`。"""
    restore_meshes(stage, override)
