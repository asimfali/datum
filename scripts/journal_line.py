#!/usr/bin/env python3
"""Append one line to status/journal.jsonl from verified sources.

Every emitted field is derived from a source rather than taken from the
command line: the model from the agent frontmatter, checked against the
session; worker timings and tokens from the opencode API; commit times and
the diff from git; rev/supersedes from the journal itself. A disagreement
between a source and the request aborts the write with exit code 2.

The journal always lives in the main worktree. Its path comes from
`git rev-parse --git-common-dir`, so a run inside a linked worktree appends
to the main journal and never creates a worktree-local one.
"""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def refuse(reason):
    print(f"journal_line: {reason}", file=sys.stderr)
    sys.exit(2)


def git(*args):
    try:
        return subprocess.run(
            ["git", *args],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except FileNotFoundError:
        refuse("git not found")
    except subprocess.CalledProcessError as err:
        refuse(f"git {' '.join(args)}: {err.stderr.strip() or err.returncode}")


def journal_path():
    """Return the journal in the main worktree; never create a new one."""
    common = Path(git("rev-parse", "--path-format=absolute", "--git-common-dir").strip())
    path = common.parent / "status" / "journal.jsonl"
    if not path.is_file():
        refuse(f"journal not found: {path}")
    return path


def agent_model(agent):
    """Return (model string, source reference) from the agent frontmatter."""
    path = ROOT / ".opencode" / "agent" / f"{agent}.md"
    if "/" in agent or not path.is_file():
        refuse(f"no agent file .opencode/agent/{agent}.md")
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0].strip() != "---":
        refuse(f".opencode/agent/{agent}.md has no frontmatter")
    for number, line in enumerate(lines[1:], start=2):
        if line.strip() == "---":
            break
        if line.startswith("model:"):
            value = line.split(":", 1)[1].strip().strip("'\"")
            if not value:
                break
            return value, f".opencode/agent/{agent}.md:{number}"
    refuse(f".opencode/agent/{agent}.md has no model: line")


def session_info(session):
    """Return the session payload from the local opencode API."""
    try:
        result = subprocess.run(
            ["opencode", "api", "get", f"/api/session/{session}"],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        refuse("opencode not found")
    if result.returncode != 0:
        refuse(f"session {session} not found: {result.stderr.strip() or 'api error'}")
    try:
        body = json.loads(result.stdout)
    except json.JSONDecodeError:
        refuse(f"session {session}: api returned no JSON")
    if "_tag" in body:
        refuse(f"session {session}: {body.get('message', body['_tag'])}")
    if "data" not in body:
        refuse(f"session {session}: api payload has no data")
    return body["data"]


def model_string(model):
    provider = model.get("providerID")
    ident = model.get("id")
    if not provider or not ident:
        refuse("session has no model provider or id")
    variant = model.get("variant")
    return f"{provider}/{ident}#{variant}" if variant else f"{provider}/{ident}"


def resolve_commit(rev, what):
    if not git("cat-file", "-t", rev).strip():
        refuse(f"{what} {rev} is not a commit")
    return git("rev-parse", "--verify", f"{rev}^{{commit}}").strip()


def read_journal(path):
    records = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            refuse(f"{path}:{number} is not JSON")
    return records


def diff_line(base, candidate):
    parts = []
    for row in git("diff", "--numstat", f"{base}..{candidate}").splitlines():
        added, deleted, path = row.split("\t", 2)
        if added == "-" or deleted == "-":
            parts.append(f"{path} (binary)")
        else:
            parts.append(f"{path} +{added}/-{deleted}")
    return ", ".join(parts)


def main():
    parser = argparse.ArgumentParser(
        description="Append one verified line to status/journal.jsonl."
    )
    parser.add_argument("--task", required=True)
    parser.add_argument("--agent", required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--base", required=True)
    parser.add_argument("--candidate", required=True)
    parser.add_argument("--class", dest="klass")
    parser.add_argument("--brief", help="brief file; its sha256 becomes brief_sha")
    parser.add_argument("--branch")
    parser.add_argument("--outcome", default="pending")
    parser.add_argument("--check-wall", type=int)
    parser.add_argument("--manual")
    parser.add_argument("--win-exe-sha256")
    parser.add_argument("--architect-time-source")
    parser.add_argument("--author-time", type=int)
    parser.add_argument("--author-touch")
    parser.add_argument("--note")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    # 1. The agent must declare a model.
    expected_model, model_source = agent_model(args.agent)

    # 4. The session must exist.
    session = session_info(args.session)

    # 2. The session model must be the agent model.
    actual_model = model_string(session.get("model") or {})
    if actual_model != expected_model:
        refuse(
            f"session model {actual_model} does not match agent model "
            f"{expected_model} ({model_source})"
        )

    # 3. The session agent must be the requested agent.
    actual_agent = session.get("agent")
    if actual_agent != args.agent:
        refuse(f"session agent {actual_agent} is not --agent {args.agent}")

    # 5. The candidate must be a commit.
    candidate = resolve_commit(args.candidate, "candidate")

    # 6. The base must be an ancestor of the candidate.
    result = subprocess.run(
        ["git", "merge-base", "--is-ancestor", args.base, candidate],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        refuse(f"base {args.base} is not an ancestor of {candidate}")

    path = journal_path()
    records = read_journal(path)

    # Existing lines carry no rev: the implicit rev is the line's ordinal
    # within its task. The highest rev is the line that stands.
    prior = [rec for rec in records if rec.get("task") == args.task]
    rev = len(prior) + 1
    supersedes = rev - 1 if prior else None

    # 7. Refuse to re-record the decision that already stands.
    if (
        prior
        and prior[-1].get("candidate_sha") == candidate
        and prior[-1].get("outcome") == args.outcome
    ):
        refuse(
            f"task {args.task} already has {args.outcome} for {candidate[:12]} "
            "as its effective record"
        )

    time = session.get("time") or {}
    created, idle = time.get("created"), time.get("idle")
    if not created or not idle:
        refuse(f"session {args.session} has no created/idle timestamps")

    tokens = session.get("tokens") or {}
    cache = tokens.get("cache") or {}

    line = {"task": args.task}
    if args.klass:
        line["class"] = args.klass
    if args.brief:
        brief = Path(args.brief)
        if not brief.is_file():
            refuse(f"brief not found: {brief}")
        line["brief_sha"] = hashlib.sha256(brief.read_bytes()).hexdigest()
    line.update(
        {
            "base_sha": git("rev-parse", "--verify", f"{args.base}^{{commit}}").strip(),
            "candidate_sha": candidate,
        }
    )
    if args.branch:
        line["branch"] = args.branch
    line.update(
        {
            "agent": args.agent,
            "model": actual_model,
            "model_source": model_source,
            "session": args.session,
            "worker_wall_s": round((idle - created) / 1000),
            "t_worker_start": created // 1000,
            "t_worker_end": idle // 1000,
            "candidate_ct": int(git("log", "-1", "--format=%ct", candidate).strip()),
            "diff": diff_line(args.base, candidate),
            "usage": {
                "input": tokens.get("input", 0),
                "output": tokens.get("output", 0),
                "reasoning": tokens.get("reasoning", 0),
                "cache_read": cache.get("read", 0),
                "cache_write": cache.get("write", 0),
            },
        }
    )
    if args.check_wall is not None:
        line["check"] = {"wall_s": args.check_wall}
    line.update({"rev": rev, "supersedes": supersedes, "outcome": args.outcome})
    if args.manual:
        line["manual"] = args.manual
    if args.win_exe_sha256:
        line["win_exe_sha256"] = args.win_exe_sha256
    line.update({"time_source": "derived", "cost": None, "cost_source": "proxy reports 0"})
    if args.architect_time_source:
        line["architect_time_source"] = args.architect_time_source
    if args.author_time is not None:
        line["author_time"] = args.author_time
        line["author_time_source"] = "estimate"
    if args.author_touch:
        line["author_touch"] = args.author_touch
    if args.note:
        line["note"] = args.note

    text = json.dumps(line, ensure_ascii=False, separators=(",", ":"))
    if args.dry_run:
        print(text)
        return
    with path.open("a", encoding="utf-8") as handle:
        handle.write(text + "\n")


if __name__ == "__main__":
    main()
