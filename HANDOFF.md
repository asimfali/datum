# HANDOFF — текущая задача

> Решения — `architecture/decisions.md`, планы — `status/`.

## Задача

Пилот оркестратора (v2.3, переписка с agent-kit): маршрутизация «лёгкое — лёгкой модели» при авторе в петле. Три задачи приняты с первого кандидата: 01 docs (README-синхронизация, worker-light 155 с), 02 view (полигон на дуге 1–2 стороны, worker 701 с), 03 геометрия (`sheet.path` по имени группы — D301, worker 1666 с); обе Windows-ручные проверки автором подтверждены. Журнал `status/journal.jsonl` (10 строк) пишет `scripts/journal_line.py` — модель из файла агента, времена/токены из opencode api, rev/supersedes, отказ писать при расхождении. Идёт: один общий PR всех правок (`pilot/orchestrator-batch`, слит из трёх веток + tooling), после CI — влив автором.

История пилота (ветки, замеры, аудиты) — в `status/journal.jsonl` и письмах agent-kit; ветки `pilot/01-readme-sync`, `pilot/02-polygon-arc-sides` поглощены батчем.

Разбиение архитектуры — [план](status/architecture-split.md). Далее ревью PR.

Библиотечная штамповка 0.5.6: план влит (PR #275), шаги в `status/stamp-component.md`. Тестовые модели автора: `C:\develop\datum-test\Детали\` (копировать эталоны туда же для ручной проверки).

## Следующий шаг

- Досмотреть PR батча пилота (CI → влив автором → `win-build` release).
- Второй режим D289 — рез самим штампом.
- Отложено: рост длительности Linux CI (3 мин → 5:06), `ROADMAP.md`; в пилоте зафиксирован аргумент за профили проверок (216 с код против 56 с docs).

## Проверки

- Батч: `CI=true scripts/check.sh` зелёный на каждом кандидате и на слитой ветке (см. `status/journal.jsonl`); Windows `mirror` собран для 02 и 03, exe сохранён под SHA.
- Команды в `AGENTS.md`: «проверка», «Обнови», «PR», «релиз».

## Не трогать

Untracked файлы автора: `crates/datum-ops/tests/stamp_spike.rs`, `examples/stamp-leaf.json`, его правка `status/summary.md`.
