"""Name -> class registries used as factories throughout dynabarn.

Built-in implementations register themselves with a decorator. Third-party
packages can add new ones without touching this repo by declaring a Python
entry point in the registry's group, e.g. in their pyproject.toml::

    [project.entry-points."dynabarn_core.backends"]
    isaac_lab = "my_pkg.backend:IsaacLabBackend"
"""
from __future__ import annotations

from importlib.metadata import entry_points
from typing import Callable, Dict, Generic, Iterator, TypeVar

T = TypeVar("T")


class RegistryError(KeyError):
    pass


class Registry(Generic[T]):
    def __init__(self, kind: str, entry_point_group: str | None = None) -> None:
        self.kind = kind
        self.group = entry_point_group
        self._items: Dict[str, type[T]] = {}
        self._entry_points_loaded = False

    def register(self, name: str) -> Callable[[type[T]], type[T]]:
        def deco(cls: type[T]) -> type[T]:
            if name in self._items and self._items[name] is not cls:
                raise RegistryError(f"{self.kind} '{name}' is already registered")
            self._items[name] = cls
            setattr(cls, "registry_name", name)
            return cls

        return deco

    def _load_entry_points(self) -> None:
        if self._entry_points_loaded or not self.group:
            return
        self._entry_points_loaded = True
        for ep in entry_points(group=self.group):
            if ep.name not in self._items:
                cls = ep.load()
                self._items[ep.name] = cls
                setattr(cls, "registry_name", ep.name)

    def get(self, name: str) -> type[T]:
        if name not in self._items:
            self._load_entry_points()
        try:
            return self._items[name]
        except KeyError:
            raise RegistryError(
                f"unknown {self.kind} '{name}'. Available: {', '.join(self.names()) or 'none'}"
            ) from None

    def create(self, name: str, *args, **kwargs) -> T:
        """Factory: instantiate the class registered under ``name``."""
        return self.get(name)(*args, **kwargs)

    def names(self) -> list[str]:
        self._load_entry_points()
        return sorted(self._items)

    def __contains__(self, name: str) -> bool:
        self._load_entry_points()
        return name in self._items

    def __iter__(self) -> Iterator[str]:
        return iter(self.names())
