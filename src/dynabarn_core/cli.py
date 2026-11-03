"""Command line interface: ``dynabarn <command> ...`` (or ``python -m dynabarn_core``)."""
from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

from .assets import AssetLibrary
from .backends import BACKENDS, Artifacts, BackendError, create_backend
from .core import TRAJECTORIES, Scenario, VelocityCommand
from .generators import GENERATORS, create_generator, generator_from_preset, list_presets, load_preset


def _kv(items: list[str]) -> dict[str, str]:
    out = {}
    for item in items:
        if "=" not in item:
            raise SystemExit(f"expected key=value, got {item!r}")
        k, v = item.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def cmd_list(args, assets: AssetLibrary) -> int:
    what = args.what
    if what in ("backends", "all"):
        print("backends:")
        for name in BACKENDS.names():
            caps = BACKENDS.get(name).capabilities
            mode = "in-process" if caps.in_process else "external"
            print(f"  {name:<10} {mode:<11} sensors: {', '.join(sorted(caps.sensors))}")
    if what in ("generators", "all"):
        print("generators:")
        for name in GENERATORS.names():
            doc = (GENERATORS.get(name).__doc__ or "").strip().splitlines()[0]
            print(f"  {name:<10} {doc}")
    if what in ("presets", "all"):
        print("presets:")
        for name in list_presets():
            print(f"  {name:<18} {load_preset(name)['description']}")
    if what in ("trajectories", "all"):
        print("trajectories:", ", ".join(TRAJECTORIES.names()))
    if what in ("assets", "all"):
        print("asset folders:", ", ".join(map(str, assets.search_paths)) or "(none)")
        for kind in ("robot", "obstacle", "environment"):
            for aid in assets.ids(kind):
                a = assets.get(aid)
                reps = ", ".join(sorted(a.representations)) or "-"
                print(f"  {kind:<12} {aid:<16} backends: {reps:<14} {a.description}")
    return 0


def cmd_assets_copy(args, assets: AssetLibrary) -> int:
    import shutil

    import yaml

    src = assets.source(args.asset_id)
    if src is None:
        raise ValueError(f"asset '{args.asset_id}' was defined in code and has no folder to copy")
    new_id = args.new_id or args.asset_id
    dest = Path(args.dest) / new_id
    if dest.exists():
        raise ValueError(f"{dest} already exists")
    shutil.copytree(src, dest)
    if new_id != args.asset_id:
        manifest = dest / "asset.yaml"
        data = yaml.safe_load(manifest.read_text())
        data["id"] = new_id
        manifest.write_text(yaml.safe_dump(data, sort_keys=False))
    print(f"copied '{args.asset_id}' to {dest}")
    what = "a new asset" if new_id != args.asset_id else "an override of the built-in asset"
    print(f"Edit {dest / 'asset.yaml'}; it is {what} '{new_id}'. Use it with:")
    print(f"  dynabarn --assets {Path(args.dest)} ...     or     export DYNABARN_ASSET_PATH={Path(args.dest).resolve()}")
    return 0


def cmd_import_world(args, assets: AssetLibrary) -> int:
    from .importers import WORLD_IMPORTERS, find_world_folders, import_world, write_environment

    targets: list[Path] = []
    for raw in args.paths:
        p = Path(raw)
        if p.is_dir() and not any(_can(p, n) for n in WORLD_IMPORTERS.names()):
            found = find_world_folders(p)
            if not found:
                raise ValueError(f"{p}: not a world and contains no world folders")
            targets += found
        else:
            targets.append(p)
    if args.id and len(targets) != 1:
        raise ValueError("--id needs exactly one world")
    failed = 0
    for t in targets:
        try:
            res = import_world(t, importer=args.importer, asset_id=args.id)
        except (ValueError, LookupError) as e:
            failed += 1
            print(f"FAILED {t}: {e}")
            continue
        folder = write_environment(res, args.output)
        env = res.environment
        print(f"{env.id:<36} {len(env.static_objects()):5d} objects  {len(env.spawns)} spawns  "
              f"{len(res.warnings)} warnings  -> {folder}")
        for w in res.warnings[: None if args.verbose else 3]:
            print(f"    warning: {w}")
        if len(res.warnings) > 3 and not args.verbose:
            print(f"    ... {len(res.warnings) - 3} more (use -v)")
    print(f"imported {len(targets) - failed} of {len(targets)} world(s). Use them with: dynabarn --assets {args.output} ...")
    return 1 if failed else 0


def _can(path: Path, importer: str) -> bool:
    from .importers import WORLD_IMPORTERS

    return WORLD_IMPORTERS.get(importer).can_import(path)


def cmd_options(args, assets: AssetLibrary) -> int:
    import dataclasses

    cfg = GENERATORS.get(args.generator).Config
    for f in dataclasses.fields(cfg):
        default = f.default if f.default is not dataclasses.MISSING else "?"
        print(f"  {f.name:<16} = {default!r}")
    return 0


def cmd_generate(args, assets: AssetLibrary) -> int:
    if args.preset:
        if args.generator:
            raise SystemExit("give either a generator or --preset, not both")
        gen = generator_from_preset(args.preset, assets, **_kv(args.set))
    elif args.generator:
        gen = create_generator(args.generator, assets, **_kv(args.set))
    else:
        raise SystemExit("give a generator (lanes, random) or --preset NAME")
    scenario = gen.build(seed=args.seed, name=args.name)
    path = scenario.save(args.output)
    print(f"wrote {path} ({len(scenario.obstacles)} obstacles)")
    return 0


def cmd_export(args, assets: AssetLibrary) -> int:
    scenario = Scenario.load(args.scenario)
    art = create_backend(args.backend, assets).export(scenario, args.output, clean=args.clean)
    print(f"wrote {art.scene}")
    return 0


def cmd_run(args, assets: AssetLibrary) -> int:
    scenario = Scenario.load(args.scenario)
    backend = create_backend(args.backend, assets)
    out = Path(args.output or tempfile.mkdtemp(prefix=f"dynabarn_{backend.name}_"))
    art: Artifacts = backend.export(scenario, out)
    if not backend.capabilities.in_process:
        print(" ".join(backend.launch_command(art)))
        return 0
    cmd = VelocityCommand(args.linear, args.angular)
    hits: dict[str, float] = {}
    with backend.create_simulation(art, control_hz=args.rate) as sim:
        obs = sim.reset()
        while obs.time < args.duration:
            obs = sim.step(cmd)
            for c in obs.contacts:
                hits.setdefault(c.other, obs.time)
            if obs.pose.distance_to(scenario.goal) < args.goal_tolerance:
                print(f"goal reached at t={obs.time:.2f}s")
                break
    p = obs.pose
    scan_min = min(obs.scan.ranges) if obs.scan else float("nan")
    print(f"t={obs.time:.2f}s pose=({p.x:.2f}, {p.y:.2f}, {p.yaw:.2f}) "
          f"v={obs.velocity.linear:.3f} closest_scan={scan_min:.2f}")
    print(f"collisions: {len(hits)}" + "".join(f"\n  {k} at t={t:.2f}s" for k, t in sorted(hits.items(), key=lambda kv: kv[1])))
    return 0


def cmd_view(args, assets: AssetLibrary) -> int:
    scenario = Scenario.load(args.scenario)
    backend = create_backend(args.backend, assets)
    out = Path(args.output or tempfile.mkdtemp(prefix=f"dynabarn_{backend.name}_"))
    art = backend.export(scenario, out)
    print("Up/Down: speed  Left/Right: turn  Space: stop  Backspace: reset  (close window to quit)")
    backend.view(art, control_hz=args.rate, linear=args.linear, angular=args.angular,
                 follow=args.follow, duration=args.duration)
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="dynabarn", description="DynaBARN scenario toolkit")
    p.add_argument("--assets", action="append", default=[], metavar="DIR",
                   help="extra asset folder, searched before the defaults (repeatable)")
    sub = p.add_subparsers(dest="command", required=True)

    s = sub.add_parser("list", help="list backends, generators, trajectories and assets")
    s.add_argument("what", nargs="?", default="all",
                   choices=["all", "backends", "generators", "presets", "trajectories", "assets"])
    s.set_defaults(func=cmd_list)

    s = sub.add_parser("assets", help="work with assets")
    asub = s.add_subparsers(dest="assets_command", required=True)
    a = asub.add_parser("copy", help="copy an asset into your own folder to edit it")
    a.add_argument("asset_id")
    a.add_argument("dest", help="your asset folder, e.g. ./my_assets")
    a.add_argument("--as", dest="new_id", help="give the copy a new id instead of overriding the original")
    a.set_defaults(func=cmd_assets_copy)

    s = sub.add_parser("import-world", help="import worlds from other tools (SDF, world-collection) as assets")
    s.add_argument("paths", nargs="+", help="world file/folder(s), or a folder containing world folders")
    s.add_argument("-o", "--output", required=True, help="asset folder to write into")
    s.add_argument("--importer", help="force an importer (default: detected)")
    s.add_argument("--id", help="asset id (single world only)")
    s.add_argument("-v", "--verbose", action="store_true", help="show all warnings")
    s.set_defaults(func=cmd_import_world)

    s = sub.add_parser("options", help="show a generator's options and defaults")
    s.add_argument("generator")
    s.set_defaults(func=cmd_options)

    s = sub.add_parser("generate", help="generate a scenario file")
    s.add_argument("generator", nargs="?")
    s.add_argument("-p", "--preset", help="start from a preset (see `dynabarn list presets`)")
    s.add_argument("-s", "--set", action="append", default=[], metavar="KEY=VALUE")
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--name")
    s.add_argument("-o", "--output", required=True)
    s.set_defaults(func=cmd_generate)

    s = sub.add_parser("export", help="export a scenario for a simulator")
    s.add_argument("scenario")
    s.add_argument("-b", "--backend", required=True)
    s.add_argument("-o", "--output", required=True)
    s.add_argument("--clean", action="store_true", help="delete the output folder first")
    s.set_defaults(func=cmd_export)

    s = sub.add_parser("run", help="headless smoke run with a constant velocity command")
    s.add_argument("scenario")
    s.add_argument("-b", "--backend", default="mujoco")
    s.add_argument("-o", "--output")
    s.add_argument("--duration", type=float, default=20.0)
    s.add_argument("--linear", type=float, default=0.2)
    s.add_argument("--angular", type=float, default=0.0)
    s.add_argument("--rate", type=float, default=20.0, help="control rate in Hz")
    s.add_argument("--goal-tolerance", type=float, default=0.3)
    s.set_defaults(func=cmd_run)

    s = sub.add_parser("view", help="open the scenario in an interactive viewer (drive with arrow keys)")
    s.add_argument("scenario")
    s.add_argument("-b", "--backend", default="mujoco")
    s.add_argument("-o", "--output")
    s.add_argument("--linear", type=float, default=0.0, help="initial forward speed")
    s.add_argument("--angular", type=float, default=0.0, help="initial turn rate")
    s.add_argument("--follow", action="store_true", help="camera follows the robot")
    s.add_argument("--duration", type=float, help="close after this many simulated seconds")
    s.add_argument("--rate", type=float, default=20.0, help="control rate in Hz")
    s.set_defaults(func=cmd_view)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args, AssetLibrary(args.assets))
    except (BackendError, LookupError, ValueError, TypeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
