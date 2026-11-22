#!/usr/bin/env python3
"""Soft Actor-Critic learner for the DynaBARN II learned planner.

Actors (actor_loop.sh) run Gazebo episodes with the latest policy and drop them into
<run>/buffer/*.npz. This process loads new episodes as they appear, trains on a GPU, and keeps
exporting the actor to <run>/policy/policy_latest.npz (numpy, so actors need no PyTorch).
Off-policy learning means actors may use a policy a few versions old without harm.

Outputs in <run>/:
  policy/policy_latest.npz        newest actor, replaced atomically
  policy/policy_v<N>.npz          snapshots every --snapshot_every exports, for later evaluation
  checkpoint.pt                   full learner state; restarting with the same --run resumes
  progress.csv                    one row per export: episodes, success and collision rates, losses
  STOP                            create this file to stop the learner and all actors

Usage: learner.py --run /shared/dynabarn2_rl/run1 [--device cuda]
Needs: numpy, torch. The dynabarn2_learned package is found relative to this file.
"""
import argparse
import csv
import glob
import os
import sys
import time
from collections import deque

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from dynabarn2_learned import core  # noqa: E402

GOAL = (-9.0, 0.0)  # world frame, as in run_trial.launch.py


class Actor(nn.Module):
    def __init__(self, hidden):
        super().__init__()
        dims = [core.OBS_DIM] + hidden
        self.body = nn.ModuleList(nn.Linear(a, b) for a, b in zip(dims[:-1], dims[1:]))
        self.mu = nn.Linear(hidden[-1], core.ACT_DIM)
        self.ls = nn.Linear(hidden[-1], core.ACT_DIM)

    def forward(self, obs):
        h = obs
        for layer in self.body:
            h = F.relu(layer(h))
        mu, log_std = self.mu(h), self.ls(h).clamp(-5.0, 2.0)
        std = log_std.exp()
        u = mu + std * torch.randn_like(mu)
        a = torch.tanh(u)
        logp = (-0.5 * ((u - mu) / std) ** 2 - log_std - 0.5 * np.log(2 * np.pi)).sum(-1)
        logp -= (2 * (np.log(2) - u - F.softplus(-2 * u))).sum(-1)
        return a, logp

    def export(self, path, version):
        arrays = {'version': np.int64(version)}
        for i, layer in enumerate(self.body):
            arrays[f'w{i}'] = layer.weight.detach().cpu().numpy().T.astype(np.float32)
            arrays[f'b{i}'] = layer.bias.detach().cpu().numpy().astype(np.float32)
        for name, layer in (('mu', self.mu), ('ls', self.ls)):
            arrays[f'{name}_w'] = layer.weight.detach().cpu().numpy().T.astype(np.float32)
            arrays[f'{name}_b'] = layer.bias.detach().cpu().numpy().astype(np.float32)
        tmp = path + '.tmp.npz'
        np.savez(tmp, **arrays)
        os.replace(tmp, path)


class Critic(nn.Module):
    def __init__(self, hidden):
        super().__init__()
        def q():
            dims = [core.OBS_DIM + core.ACT_DIM] + hidden
            layers = []
            for a, b in zip(dims[:-1], dims[1:]):
                layers += [nn.Linear(a, b), nn.ReLU()]
            return nn.Sequential(*layers, nn.Linear(hidden[-1], 1))
        self.q1, self.q2 = q(), q()

    def forward(self, obs, act):
        x = torch.cat([obs, act], -1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)


class Replay:
    def __init__(self, capacity):
        self.cap, self.n, self.i = capacity, 0, 0
        self.obs = np.zeros((capacity, core.OBS_DIM), np.float32)
        self.nxt = np.zeros((capacity, core.OBS_DIM), np.float32)
        self.act = np.zeros((capacity, core.ACT_DIM), np.float32)
        self.rew = np.zeros(capacity, np.float32)
        self.done = np.zeros(capacity, np.float32)

    def add(self, obs, act, rew, nxt, done):
        for k in range(len(act)):
            j = self.i
            self.obs[j], self.act[j], self.rew[j], self.nxt[j], self.done[j] = \
                obs[k], act[k], rew[k], nxt[k], done[k]
            self.i = (self.i + 1) % self.cap
            self.n = min(self.n + 1, self.cap)

    def sample(self, batch, device):
        idx = np.random.randint(0, self.n, batch)
        t = lambda x: torch.as_tensor(x[idx], device=device)  # noqa: E731
        return t(self.obs), t(self.act), t(self.rew), t(self.nxt), t(self.done)


def load_episode(path):
    with np.load(path) as d:
        status = str(d['status'])
        if status == 'hit_at_start':
            return None
        obs, act = d['obs'], d['act']
        if len(obs) != len(act) + 1 or len(act) == 0:
            return None
        rew, terminal = core.rewards(d['pos'], GOAL, d['min_scan'], status)
        done = np.zeros(len(act), np.float32)
        done[-1] = float(terminal)
        return dict(obs=obs[:-1], nxt=obs[1:], act=act, rew=rew, done=done, status=status,
                    reactive=str(d['reactive']), version=int(d['policy_version']))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', required=True, help='shared run directory')
    ap.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    ap.add_argument('--hidden', type=int, nargs='+', default=[512, 512, 256])
    ap.add_argument('--batch', type=int, default=512)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--gamma', type=float, default=0.99)
    ap.add_argument('--tau', type=float, default=0.005)
    ap.add_argument('--replay', type=int, default=2_000_000)
    ap.add_argument('--warmup', type=int, default=20_000, help='transitions before training starts')
    ap.add_argument('--utd', type=float, default=1.0, help='max gradient updates per collected transition')
    ap.add_argument('--export_every', type=int, default=1000, help='updates between policy exports')
    ap.add_argument('--snapshot_every', type=int, default=20, help='exports between kept snapshots')
    ap.add_argument('--max_updates', type=int, default=3_000_000)
    args = ap.parse_args()

    run = os.path.abspath(args.run)
    buf_dir, pol_dir = os.path.join(run, 'buffer'), os.path.join(run, 'policy')
    os.makedirs(buf_dir, exist_ok=True)
    os.makedirs(pol_dir, exist_ok=True)
    dev = torch.device(args.device)
    torch.manual_seed(0)
    np.random.seed(0)

    actor, critic = Actor(args.hidden).to(dev), Critic(args.hidden).to(dev)
    target = Critic(args.hidden).to(dev)
    target.load_state_dict(critic.state_dict())
    log_alpha = torch.zeros(1, requires_grad=True, device=dev)
    opt_a = torch.optim.Adam(actor.parameters(), lr=args.lr)
    opt_c = torch.optim.Adam(critic.parameters(), lr=args.lr)
    opt_al = torch.optim.Adam([log_alpha], lr=args.lr)
    target_entropy = -float(core.ACT_DIM)
    updates, version = 0, 0

    ckpt_path = os.path.join(run, 'checkpoint.pt')
    if os.path.exists(ckpt_path):
        ck = torch.load(ckpt_path, map_location=dev)
        actor.load_state_dict(ck['actor']); critic.load_state_dict(ck['critic'])
        target.load_state_dict(ck['target']); log_alpha.data[:] = ck['log_alpha']
        opt_a.load_state_dict(ck['opt_a']); opt_c.load_state_dict(ck['opt_c'])
        opt_al.load_state_dict(ck['opt_al'])
        updates, version = ck['updates'], ck['version']
        print(f'Resumed at update {updates}, policy version {version}', flush=True)

    replay = Replay(args.replay)
    seen = set()
    recent = deque(maxlen=300)   # (status, reactive) of the newest episodes
    episodes = transitions = 0
    progress = os.path.join(run, 'progress.csv')
    new_log = not os.path.exists(progress)
    log = open(progress, 'a', newline='')
    writer = csv.writer(log)
    if new_log:
        writer.writerow(['wall_time', 'version', 'updates', 'episodes', 'transitions', 'recent_success',
                         'recent_collision', 'success_off', 'success_yield', 'success_block',
                         'critic_loss', 'actor_loss', 'alpha', 'mean_episode_return'])
    returns = deque(maxlen=300)
    lc = la = 0.0
    t0 = time.time()

    def ingest():
        nonlocal episodes, transitions
        new = 0
        for path in sorted(glob.glob(os.path.join(buf_dir, '*.npz'))):
            if path in seen:
                continue
            seen.add(path)
            try:
                ep = load_episode(path)
            except Exception as e:  # half-written or corrupt file
                print(f'skip {path}: {e}', flush=True)
                continue
            if ep is None:
                continue
            replay.add(ep['obs'], ep['act'], ep['rew'], ep['nxt'], ep['done'])
            recent.append((ep['status'], ep['reactive']))
            returns.append(float(ep['rew'].sum()))
            episodes += 1
            transitions += len(ep['act'])
            new += len(ep['act'])
        return new

    def rate(status, mode=None):
        sel = [s for s, m in recent if mode is None or m == mode]
        return round(sum(s == status for s in sel) / len(sel), 3) if sel else ''

    def export():
        nonlocal version
        version += 1
        actor.export(os.path.join(pol_dir, 'policy_latest.npz'), version)
        if version % args.snapshot_every == 0:
            actor.export(os.path.join(pol_dir, f'policy_v{version}.npz'), version)
        torch.save(dict(actor=actor.state_dict(), critic=critic.state_dict(), target=target.state_dict(),
                        log_alpha=log_alpha.data, opt_a=opt_a.state_dict(), opt_c=opt_c.state_dict(),
                        opt_al=opt_al.state_dict(), updates=updates, version=version), ckpt_path + '.tmp')
        os.replace(ckpt_path + '.tmp', ckpt_path)
        writer.writerow([round(time.time() - t0), version, updates, episodes, transitions, rate('succeeded'),
                         rate('collided'), rate('succeeded', 'off'), rate('succeeded', 'yield'),
                         rate('succeeded', 'block'), round(lc, 4), round(la, 4),
                         round(float(log_alpha.exp()), 4),
                         round(float(np.mean(returns)), 2) if returns else ''])
        log.flush()
        print(f'v{version} updates {updates} episodes {episodes} transitions {transitions} '
              f'success {rate("succeeded")} collision {rate("collided")}', flush=True)

    budget = 0.0
    last_scan = 0.0
    while updates < args.max_updates and not os.path.exists(os.path.join(run, 'STOP')):
        if time.time() - last_scan > 10.0 or budget < 1:
            budget += args.utd * ingest()
            last_scan = time.time()
        if replay.n < args.warmup or budget < 1:
            time.sleep(5.0)
            continue
        obs, act, rew, nxt, done = replay.sample(args.batch, dev)
        alpha = log_alpha.exp().detach()
        with torch.no_grad():
            a2, logp2 = actor(nxt)
            q1t, q2t = target(nxt, a2)
            y = rew + args.gamma * (1 - done) * (torch.min(q1t, q2t) - alpha * logp2)
        q1, q2 = critic(obs, act)
        loss_c = F.mse_loss(q1, y) + F.mse_loss(q2, y)
        opt_c.zero_grad(); loss_c.backward(); opt_c.step()

        a, logp = actor(obs)
        q1p, q2p = critic(obs, a)
        loss_a = (alpha * logp - torch.min(q1p, q2p)).mean()
        opt_a.zero_grad(); loss_a.backward(); opt_a.step()

        loss_al = -(log_alpha * (logp.detach() + target_entropy)).mean()
        opt_al.zero_grad(); loss_al.backward(); opt_al.step()

        with torch.no_grad():
            for p, pt in zip(critic.parameters(), target.parameters()):
                pt.mul_(1 - args.tau).add_(args.tau * p)
        lc, la = loss_c.item(), loss_a.item()
        updates += 1
        budget -= 1
        if updates % args.export_every == 0:
            export()
    if updates % args.export_every:
        export()
    print('Learner stopped', flush=True)


if __name__ == '__main__':
    main()
