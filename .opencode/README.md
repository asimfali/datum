# .opencode — агенты и ключевые слова

Меню под задачу: агент выбирает модель и уровень усилия, команда — агента. Автор сам решает, что
запускать; здесь только перечень.

## Агенты

| агент | модель | когда |
|---|---|---|
| `architect-light` | `cliproxy/glm-5.3-flash#low` | только разговор: `edit: deny`, ничего не меняет |
| `architect` | `cliproxy/glm-5.3-flash#high` | обычный план, решения, брифы |
| `architect-max` | `cliproxy/glm-5.3-flash#max` | сложная задача: варианты, критерии, спорные места |
| `worker-light` | `cliproxy/deepseek-v4.1-flash#low` | механические правки: тексты, переименования, документация |
| `worker` | `cliproxy/deepseek-v4.1-flash#high` | реализация по брифу |
| `worker-max` | `cliproxy/deepseek-v4-pro#max` | тяжёлые участки: сложная геометрия, отладка |
| `reviewer` | `cliproxy/deepseek-v4.1-flash-200k#high` | ревью diff (окно 200k) |
| `reviewer-pro` | `cliproxy/deepseek-v4-pro#high` | тяжёлое ревью, только по прямой просьбе |

`architect-max`, `architect-light`, `worker-max`, `worker-light` — тонкие варианты: их тело отсылает к
`architect.md`/`worker.md`, чтобы правила проекта жили в одном месте.

## Ключевые слова

| команда | агент | что делает |
|---|---|---|
| `/обсуждаем` | `architect-light` | обсуждение, правок нет |
| `/план` | `architect` | план и обсуждение, код не пишется |
| `/план-макс` | `architect-max` | то же для сложной задачи |
| `/делаем` | `worker` | реализация в текущей сессии |
| `/делаем-лайт` | `worker-light` | механические правки |
| `/делаем-макс` | `worker-max` | тяжёлая реализация |

В команде `agent:` выбирает агента, `subagent: false` оставляет работу в текущей сессии (видно ход,
можно прервать). `opencode run "/план"` применяет только текст шаблона — агент и модель из команды
подставляются в TUI (и в `/api/session/{id}/command`), поэтому при скриптовании передавай
`--agent`/`--model` явно.

## Что должно быть на машине вне репозитория

Варианты `#low/#high/#max` не берутся из воздуха: их даёт `capabilities.reasoning.effort` в
глобальном `~/.config/opencode/opencode.json`, а пропускает — лестница `thinking.levels` в конфиге
прокси. Обе лестницы должны совпадать, иначе клиент покажет вариант, а прокси ответит
`level "..." not supported, valid levels: ...`.

`thinking.levels` применяется только рестартом сервиса прокси (watcher перечитывает `headers`, но не
лестницу), клиентский конфиг — без рестарта.

Глобальный конфиг (V2-форма; ключ подставь свой, в репозиторий он не попадает):

```jsonc
"providers": {
  "cliproxy": {
    "package": "@opencode/ai/providers/openai-compatible",
    "settings": { "baseURL": "http://127.0.0.1:8317/v1", "apiKey": "<proxy key>" },
    "models": {
      "glm-5.3-flash": {
        "capabilities": {
          "reasoning": { "supported": true, "effort": ["low", "high", "max"] }
        },
        "limit": { "context": 300000, "output": 32000 }
      }
    }
  }
}
```

Внимание: варианты работают только в V2-блоке `providers` (множественное число). В V1-блоке
`provider` у модели другая форма `variants` (объект, не массив), и массив там молча выбрасывает весь
провайдер — все модели отвечают `Model unavailable`.
