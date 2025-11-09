import importlib
import pkgutil
from pathlib import Path
from types import ModuleType

def discover_widgets():
    """widgets ディレクトリ内のウィジェットを自動発見し、WIDGET_METAを登録"""
    widget_dir = Path(__file__).parent
    registry = {}

    # 🚫 モーダルなど登録不要のファイルを除外
    skip_modules = {"widget_select_view", "widget_remove_view", "__init__"}

    for modinfo in pkgutil.iter_modules([str(widget_dir)]):
        name = modinfo.name
        if name in skip_modules:
            continue

        try:
            module: ModuleType = importlib.import_module(f"widgets.{name}")
        except Exception as e:
            print(f"[WIDGET_LOAD_ERROR] {name}: {e}")
            continue

        # --- メタ情報を持つもののみ登録 ---
        if not hasattr(module, "WIDGET_META"):
            continue

        meta = getattr(module, "WIDGET_META")
        widget_cls = None

        # --- クラス検出ロジック ---
        for attr in dir(module):
            obj = getattr(module, attr)
            if isinstance(obj, type) and not attr.startswith("_"):
                if meta["title"].replace(" ", "").lower() in attr.lower():
                    widget_cls = obj
                    break
                elif widget_cls is None and "widget" in str(obj).lower():
                    widget_cls = obj

        if widget_cls is None:
            print(f"[WIDGET_WARN] クラス未検出: {name}")
            continue

        # --- メタ情報登録 ---
        registry[meta["id"]] = {
            **meta,  # category, description, order も展開
            "class": widget_cls,
            "module": module,
        }

    return registry


# 🔥 グローバルレジストリ
WIDGET_REGISTRY = discover_widgets()
