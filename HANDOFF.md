# HANDOFF — текущая задача

> Память между сессиями: только текущая задача. Потолок — 2k токенов (`ctx-cost --cap 2000 HANDOFF.md`). Долгие решения — `ARCHITECTURE.md` (D*), планы и паузы — `status/`, среда и агенты — `.opencode/README.md`, глобальные правила — `~/.config/opencode/AGENTS.md`. Секретов здесь нет.

## Задача

**D264, лофт-фланец, кусочек 2 влит (#262, squash `63cd5f5`), идёт доводка.** Одна линия A → две ломаные линии B: три треугольные фасеты и два цилиндрических гиба, гибы сходятся в общей вершине B (уточнение автора 2026-11-17; прежняя формулировка «2 фасеты + 1 гиб» ошибочна). Контракт и план — `status/loft-02-bend.md`.

## Состояние

- Строитель тела собирает полный набор граней (средние, крылья, стенки, цилиндры с дугами, ступени). `crates/datum-ops/src/loft.rs`: `two_bend_geom`/`BendSkin` — точки обеих кож, `two_bend_body` — сборка.
- `Body::validate()` ловит non-manifold — нечетный цикл ориентации у пар стенка/ступень; тест строителя под `#[ignore]`.
- Оси гибов `a0→vertex`/`a1→vertex`; у offset-кожи гребень ≠ сдвинутая вершина: крылья подрезаются гребнем (точка R = ridge ∩ дальнее ребро), у апекса появляется полоска.
- `Op::Loft` отвечает прежней заглушкой — строитель в коде, не подключён.
- Кусочек 1 влит (#260, squash `916832d`): трапеция, толщины Left/Right/Middle, `unfold_side`, эталон `tests/models/loft-01-line.json`.
- PR #251 (`feat/transition`) закрыт как superseded (2026-11-18): 2200 строк недостижимого `Op::Transition` (веер фасет, разгрузочные секторы) дали бы вторую реализацию рядом с `Op::Loft`, а номер D265 там занят другим решением; ветка на origin — справочник.

## Следующий шаг

1. Non-manifold: диагностика `DATUM_DEBUG_EDGES=1 cargo test -p datum-ops two_bend_body_tests -- --nocapture` (печатает все грани и DUP-пары рёбер; врезка перед `// Assemble.` в `two_bend_body`).
2. Подключить `two_bend_body` в `Op::Loft` вместо заглушки, снять `#[ignore]`.
3. `unfold_side` — ожидание 4 линии сгибов; эталон `tests/models/loft-02-*.json` + bless, `TESTERS.md`, Windows-сборка.
4. Из PR #251 перенести при «квадрат → круг»: `verify`-проверки разметки (плоскости фасет, оси/касательные гибов, станции торцов, «ничего над плоскостью верхнего профиля»), разгрузочные секторы в вершинах, шов ребром с `gap` как у `Rolled`.

## Паузы

- Штамп-компонент (ROADMAP 0.5.5, #248) — на паузе, 3 вопроса ждут автора: `status/stamp-component.md`.
- Лепесток: профиль не подтверждён, отложен (планировка point в операции уже реализована).

## Файлы

- `crates/datum-ops/src/loft.rs` — геометрия и строитель кусочка 2; `lib.rs` — `Op::Loft`, `Op::Component`.
- `crates/datum-ops/tests/loft.rs` — эталонные случаи шагов 1–2; `tests/models/loft-01-line.json`, `tests/model-hashes.json`.
- `status/loft-01-trapezoid.md`, `status/loft-02-bend.md` — планы кусочков.
- Локально не закоммичено: `crates/datum-ops/tests/stamp_spike.rs`, `examples/stamp-leaf.json` (заготовки штампа).

## Проверки

- `f0ec94f`: `cargo test -p datum-ops --lib` — 5 passed, 1 ignored; `--test loft` — 7 passed. Операция лофта — прежняя заглушка.
- `scripts/check.sh` сверяет потолки `AGENTS.md`/`HANDOFF.md` (`ctx-cost --cap 2000`, ставится `install.sh` из agent-kit), если `ctx-cost` в PATH.
- Потолки на этой правке: `AGENTS.md` 1983, `HANDOFF.md` 1267 — ok.
- Справочные точки: `scripts/check.sh` exit 0 на `63cd5f5` и `96d8c49`.
