#!/usr/bin/env python3
"""Сводка по планам из `status/`: карта, «на чём остановились», итоги.

    status_summary.py <режим> [план] [--dir status] [--date=auto|mtime|git]

Режимы:
    overview      все планы: прогресс, следующий незакрытый пункт, дата
    next <план>   последний начатый этап и его незакрытые пункты (или «завершён»)
    plan <план>   незакрытые пункты по этапам, закрытые — только заголовками
    done <план>   тексты выполненных пунктов
    full <план>   план целиком (escape hatch)

Формат плана, критерии плана и режимы — `status-memory.md` скилла `status-memory`
в конфиге opencode (вне репозитория; этот файл — копия для самодостаточности Datum).
Кэш `status/summary.md` пересобирается при любом вызове, но перезаписывается
только если изменился: повторное чтение не трогает файл.
"""
import argparse
import os
import pathlib
import re
import subprocess
import sys
import tempfile
from datetime import datetime

NAME = "status_summary"
MODES = ("overview", "next", "plan", "done", "full")
STAGE_LEVELS = (2, 3)
CHECKBOX_RE = re.compile(r"^([ \t]*)([-*+])[ \t]+\[([ xX])\][ \t]*(.*)$")
CHECKBOX_ANY_RE = re.compile(r"^[ \t]*[-*+][ \t]+\[[ xX]\]")
HEADING_RE = re.compile(r"^(#{1,6})[ \t]+(.+)$")
FENCE_RE = re.compile(r"^\s*(`{3,}|~{3,})")


class PlanError(Exception):
    """Ошибка, которую показываем пользователю без трейсбека."""


def indent_depth(indent):
    """Глубина вложенности пункта: 0 — пункт этапа, 1 — один уровень вложенности."""
    width = sum(4 if ch == "\t" else 1 for ch in indent)
    return width // 2


def parse_plan(path):
    """Разбирает markdown-план: этапы, валидные чекбоксы и нестандартные строки."""
    stages = []
    warnings = []
    current = None
    first_heading = None
    in_fence = False
    fence = ""
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise PlanError(f"не читается {path}: {exc}") from exc
    text = text.lstrip("\ufeff")

    for no, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        if in_fence:
            if re.match(r"^\s*" + re.escape(fence) + r"{3,}\s*$", line):
                in_fence = False
                continue
            if CHECKBOX_ANY_RE.match(line):
                warnings.append((no, "чекбокс в код-блоке"))
            continue
        fence_match = FENCE_RE.match(line)
        if fence_match:
            in_fence = True
            fence = fence_match.group(1)[0]
            continue

        heading = HEADING_RE.match(line)
        if heading:
            level = len(heading.group(1))
            title = heading.group(2).strip()
            if first_heading is None:
                first_heading = stripped
            if level in STAGE_LEVELS:
                current = {"level": level, "title": title, "items": []}
                stages.append(current)
            continue

        # Чекбокс в цитате: снимаем `>`-префиксы и проверяем, начинается ли строка с пункта.
        if stripped.startswith(">"):
            if CHECKBOX_ANY_RE.match(re.sub(r"^(?:>[ \t]?)+", "", stripped)):
                warnings.append((no, "чекбокс в цитате"))
            continue
        if stripped.startswith("|"):
            if any(CHECKBOX_ANY_RE.match(part.strip()) for part in stripped.split("|")):
                warnings.append((no, "чекбокс в таблице"))
            continue

        box = CHECKBOX_RE.match(line)
        if not box:
            continue
        depth = indent_depth(box.group(1))
        if depth > 1:
            warnings.append((no, "вложенность глубже 1 уровня"))
            continue
        if current is None:
            current = {"level": 2, "title": "(без этапа)", "items": []}
            stages.append(current)
        current["items"].append({
            "text": box.group(4).strip(),
            "done": box.group(3).lower() == "x",
            "depth": depth,
            "line": no,
        })

    total = sum(len(stage["items"]) for stage in stages)
    closed = sum(1 for stage in stages for item in stage["items"] if item["done"])
    return {
        "path": path,
        "name": path.stem,
        "stages": stages,
        "warnings": warnings,
        "total": total,
        "closed": closed,
        "is_plan": (first_heading or "").startswith("# План:") or total > 0,
    }


def progress(plan):
    """Строка прогресса: `4/8 (50%)`; проценты усекаются, как в спецификации."""
    total, closed = plan["total"], plan["closed"]
    pct = int(closed * 100 / total) if total else 0
    return f"{closed}/{total} ({pct}%)"


def one_line(text, limit=110):
    """Текст пункта в одну строку для карты планов."""
    flat = " ".join(text.split())
    return flat if len(flat) <= limit else flat[: limit - 1].rstrip() + "…"


def open_items(plan):
    return [(stage, item) for stage in plan["stages"] for item in stage["items"] if not item["done"]]


def next_stage(plan):
    """Последний начатый этап: нижний этап, в котором уже закрыт хотя бы один пункт.

    Если он весь закрыт — первый следующий за ним этап с открытыми пунктами; если
    начатых этапов нет — первый этап с открытыми пунктами.
    """
    started = [index for index, stage in enumerate(plan["stages"])
               if any(item["done"] for item in stage["items"])]
    if started:
        for stage in plan["stages"][started[-1]:]:
            if any(not item["done"] for item in stage["items"]):
                return stage
    for stage in plan["stages"]:
        if any(not item["done"] for item in stage["items"]):
            return stage
    return None


def git_date(path):
    """Дата последнего коммита, затронувшего файл; None, если git не ответил."""
    try:
        result = subprocess.run(
            ["git", "log", "-1", "--format=%cs", "--", path.name],
            cwd=str(path.parent),
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    value = result.stdout.strip()
    if result.returncode != 0 or not re.fullmatch(r"\d{4}-\d{2}-\d{2}", value):
        return None
    return value


def mtime_date(path):
    return datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d")


def plan_date(path, mode):
    if mode != "mtime":
        date = git_date(path)
        if date:
            return date
    return mtime_date(path)


def warn(message):
    print(message, file=sys.stderr)


def emit_warnings(plan):
    for no, reason in plan["warnings"]:
        warn(f"{plan['path'].name}:{no}: нестандартная строка ({reason}) — пропущен")


def stage_header(stage):
    return "#" * stage["level"] + " " + stage["title"]


def render_items(items):
    lines = []
    for item in items:
        prefix = "  " if item["depth"] else ""
        mark = "x" if item["done"] else " "
        lines.append(f"{prefix}- [{mark}] {item['text']}")
    return lines


def render_overview(plans, skipped, date_mode, directory):
    lines = []
    if not plans:
        lines.append(f"В `{directory}/` планов нет.")
    else:
        lines.append(f"# Планы в `{directory}/` ({len(plans)})")
        for plan in plans:
            lines.append("")
            lines.append(f"## {plan['name']} — {progress(plan)}")
            opening = open_items(plan)
            if not plan["total"]:
                lines.append("- дальше: в плане нет чекбоксов")
            elif not opening:
                lines.append("- дальше: план завершён")
            else:
                stage, item = opening[0]
                where = "" if stage["title"] == "(без этапа)" else f"{stage['title']} → "
                lines.append(f"- дальше: {where}{one_line(item['text'])}")
            lines.append(f"- изменён: {plan_date(plan['path'], date_mode)}")
    if skipped:
        names = ", ".join(f"`{name}`" for name in skipped)
        lines.append("")
        lines.append(f"Не планы (нет `# План:` и чекбоксов): {names}. "
                     f"Уберите из `{directory}/` или добавьте чекбоксы.")
    return lines


def render_next(plan):
    lines = [f"# {plan['name']} — {progress(plan)}"]
    total, closed = plan["total"], plan["closed"]
    if not total:
        lines.append("")
        lines.append("В плане нет валидных чекбоксов — этапов нет.")
        return lines
    if closed == total:
        lines.append("")
        lines.append("План завершён: все пункты закрыты. В архив переносит агент или человек.")
        return lines
    stage = next_stage(plan)
    opened = [item for item in stage["items"] if not item["done"]]
    lines.append("")
    lines.append(f"{stage_header(stage)} ({len(stage['items']) - len(opened)}/{len(stage['items'])} закрыто)")
    lines.extend(render_items(opened))
    lines.append("")
    lines.append("Закрытые пункты — режим `done`.")
    return lines


def render_plan(plan):
    lines = [f"# {plan['name']} — {progress(plan)}"]
    if not plan["total"]:
        lines.append("")
        lines.append("В плане нет валидных чекбоксов — этапов с пунктами нет.")
        return lines
    for stage in plan["stages"]:
        if not stage["items"]:
            continue
        opened = [item for item in stage["items"] if not item["done"]]
        lines.append("")
        if not opened:
            lines.append(f"{stage_header(stage)} — закрыт ({len(stage['items'])}/{len(stage['items'])})")
            continue
        lines.append(f"{stage_header(stage)} — "
                     f"{len(stage['items']) - len(opened)}/{len(stage['items'])} закрыто")
        lines.extend(render_items(opened))
    if not open_items(plan):
        lines.append("")
        lines.append("План завершён: незакрытых пунктов нет.")
    return lines


def render_done(plan):
    lines = [f"# {plan['name']} — выполненные пункты, {progress(plan)}"]
    done_stages = 0
    for stage in plan["stages"]:
        done = [item for item in stage["items"] if item["done"]]
        if not done:
            continue
        done_stages += 1
        lines.append("")
        lines.append(f"{stage_header(stage)} ({len(done)}/{len(stage['items'])})")
        lines.extend(render_items(done))
    if not done_stages:
        lines.append("")
        lines.append("Выполненных пунктов нет.")
    return lines


def render_full(path):
    text = path.read_text(encoding="utf-8", errors="replace")
    return text.rstrip("\n").splitlines()


def cell(text):
    """Ячейка markdown-таблицы: без переводов строк и без ломающих `|`."""
    flat = " ".join(str(text).split()).replace("|", "\\|")
    return flat if len(flat) <= 80 else flat[:79].rstrip() + "…"


def write_summary(directory, plans, date_mode):
    """Пересобирает `<dir>/summary.md`: только если содержимое изменилось.

    Шапка — не часы, а максимальная дата «Изменён» по планам: сводка меняется
    вместе с планами, а не от каждого чтения, поэтому её можно держать в git.
    """
    rows, dates = [], []
    for plan in plans:
        opening = open_items(plan)
        if not opening:
            stage_name = "завершён"
        else:
            stage, _item = opening[0]
            stage_name = stage["title"]
        date = plan_date(plan["path"], date_mode)
        dates.append(date)
        rows.append(f"| {cell(plan['name'])} | {cell(progress(plan))} | "
                    f"{cell(stage_name)} | {date} |")
    if dates:
        title = f"# Сводка планов (по планам на {max(dates)})"
    else:
        title = "# Сводка планов"
    lines = [title, "", "| План | Прогресс | Следующий этап | Изменён |",
             "|---|---|---|---|"] + rows
    content = "\n".join(lines) + "\n"
    target = directory / "summary.md"
    try:
        if target.read_text(encoding="utf-8") == content:
            return
    except (OSError, UnicodeDecodeError):
        pass
    fd, tmp = tempfile.mkstemp(prefix=".summary-", suffix=".tmp", dir=str(directory))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(content)
        os.chmod(tmp, 0o644)
        os.replace(tmp, target)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def scan_plans(directory):
    """Планы папки (по имени) и имена md-файлов, которые планами не являются."""
    plans, skipped = [], []
    for path in sorted(directory.glob("*.md"), key=lambda p: p.name.lower()):
        if path.name.startswith(".") or path.name == "summary.md":
            continue
        plan = parse_plan(path)
        if plan["is_plan"]:
            plans.append(plan)
        else:
            skipped.append(path.name)
    return plans, skipped


def resolve_plan(directory, argument):
    name = argument.strip()
    if not name:
        raise PlanError("пустое имя плана")
    if os.path.basename(name) != name:
        raise PlanError(f"план указывается именем в `{directory}/`, без пути: {argument}")
    filename = name if name.endswith(".md") else name + ".md"
    path = directory / filename
    if not path.is_file():
        available = sorted(p.name for p in directory.glob("*.md") if p.name != "summary.md")
        hint = ", ".join(available) if available else "планов нет"
        raise PlanError(f"план не найден: {argument} (в `{directory}/`: {hint})")
    return parse_plan(path)


def build_parser():
    parser = argparse.ArgumentParser(
        prog=NAME,
        description="Сводка по планам из status/: карта, «на чём остановились», итоги.",
    )
    parser.add_argument("mode", help="overview | next | plan | done | full")
    parser.add_argument("plan", nargs="?", help="имя плана: NAME или NAME.md")
    parser.add_argument("--dir", default="status", help="папка планов (по умолчанию status)")
    parser.add_argument("--date", default="auto", choices=("auto", "mtime", "git"),
                        help="источник даты изменения (по умолчанию auto)")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    directory = pathlib.Path(args.dir)
    if not directory.is_dir():
        warn(f"{NAME}: папка планов не найдена: {args.dir} (создайте её: mkdir -p {args.dir})")
        return 1

    plans, skipped = scan_plans(directory)
    write_summary(directory, plans, args.date)

    mode = args.mode
    if mode not in MODES:
        warn(f"{NAME}: неизвестный режим: {mode} (доступны: {', '.join(MODES)})")
        return 2

    if mode == "overview":
        for plan in plans:
            emit_warnings(plan)
        output = render_overview(plans, skipped, args.date, args.dir)
    else:
        if not args.plan:
            warn(f"{NAME}: режиму {mode} нужен аргумент <план>")
            return 2
        try:
            plan = resolve_plan(directory, args.plan)
        except PlanError as exc:
            warn(f"{NAME}: {exc}")
            return 1
        if not plan["is_plan"]:
            warn(f"{NAME}: {plan['path'].name} не похож на план "
                 "(нет `# План:` и чекбоксов) — читайте его как обычный файл")
        emit_warnings(plan)
        if mode == "next":
            output = render_next(plan)
        elif mode == "plan":
            output = render_plan(plan)
        elif mode == "done":
            output = render_done(plan)
        else:
            output = render_full(plan["path"])

    print("\n".join(output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
